"""UWB ranging driver for the two cart anchors."""

import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial


class UwbRanging(Node):
    def __init__(self):
        super().__init__('uwb_ranging')

        # Ports (Pi 5 hardware UARTs; see hardware/wiring.md section 6).
        self.declare_parameter('right_port', '/dev/ttyAMA0')
        self.declare_parameter('left_port', '/dev/ttyAMA2')
        self.declare_parameter('baud_rate', 115200)

        # Network. All three modules share network_id (and cpin, if set).
        self.declare_parameter('tag_address', 'TAG00001')
        self.declare_parameter('network_id', 'TROLLEYX')
        self.declare_parameter('cpin', '')            # empty -> no encryption set
        self.declare_parameter('right_address', 'ANCHOR_R')
        self.declare_parameter('left_address', 'ANCHOR_L')
        self.declare_parameter('right_cal_cm', 0)     # AT+CAL offset, per anchor
        self.declare_parameter('left_cal_cm', 0)

        # Topics and frames.
        self.declare_parameter('right_topic', 'uwb/right')
        self.declare_parameter('left_topic', 'uwb/left')
        self.declare_parameter('right_frame', 'uwb_right')
        self.declare_parameter('left_frame', 'uwb_left')

        # Timing and range limits.
        self.declare_parameter('poll_period_s', 0.05)  # pause after each full cycle
        self.declare_parameter('reply_timeout_s', 0.30)
        self.declare_parameter('payload', 'P')         # sent in ANCHOR_SEND
        self.declare_parameter('range_min_m', 0.10)
        self.declare_parameter('range_max_m', 50.0)
        self.declare_parameter('field_of_view_rad', 0.5)

        # Push AT config to each anchor at startup. RYUW122 saves config to
        # flash, so set this False after the first successful run.
        self.declare_parameter('configure_on_start', True)

        g = lambda n: self.get_parameter(n).value
        self.baud = int(g('baud_rate'))
        self.tag_address = g('tag_address')
        self.network_id = g('network_id')
        self.cpin = g('cpin')
        self.payload = g('payload')
        self.poll_period = float(g('poll_period_s'))
        self.reply_timeout = float(g('reply_timeout_s'))
        self.range_min = float(g('range_min_m'))
        self.range_max = float(g('range_max_m'))
        self.fov = float(g('field_of_view_rad'))
        configure = bool(g('configure_on_start'))

        self.anchors = [
            self._make_anchor('right', g('right_port'), g('right_address'),
                              int(g('right_cal_cm')), g('right_topic'), g('right_frame')),
            self._make_anchor('left', g('left_port'), g('left_address'),
                              int(g('left_cal_cm')), g('left_topic'), g('left_frame')),
        ]

        if configure:
            for a in self.anchors:
                self._configure(a)

        self._stop = False
        self.worker = threading.Thread(target=self._loop, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f"UWB ranging active. anchors -> tag '{self.tag_address}' "
            f"on network '{self.network_id}'")

    def _make_anchor(self, name, port, address, cal_cm, topic, frame):
        ser = None
        try:
            ser = serial.Serial(port, self.baud, timeout=self.reply_timeout)
            self.get_logger().info(f"{name} anchor on {port}")
        except Exception as e:
            self.get_logger().error(f"{name} anchor: cannot open {port}: {e}")
        return {
            'name': name, 'port': port, 'address': address, 'cal': cal_cm,
            'frame': frame, 'serial': ser,
            'pub': self.create_publisher(Range, topic, 10),
        }

    def _send_expect_ok(self, ser, cmd, timeout=0.5):
        if ser is None:
            return False
        try:
            ser.reset_input_buffer()
            ser.write((cmd + "\r\n").encode())
            deadline = time.time() + timeout
            while time.time() < deadline:
                line = ser.readline().decode(errors='ignore').strip()
                if line.startswith('+OK') or line == 'OK':
                    return True
                if 'ERR' in line:
                    return False
            return False
        except Exception:
            return False

    def _configure(self, a):
        cmds = [
            "AT+MODE=1",                       # 1 = ANCHOR
            f"AT+NETWORKID={self.network_id}",
            f"AT+ADDRESS={a['address']}",
        ]
        if self.cpin:
            cmds.append(f"AT+CPIN={self.cpin}")
        if a['cal'] != 0:
            cmds.append(f"AT+CAL={a['cal']}")
        for c in cmds:
            if not self._send_expect_ok(a['serial'], c):
                self.get_logger().warn(f"{a['name']}: '{c}' gave no +OK")

    def _parse_distance_m(self, line):
        # +ANCHOR_RCV=<addr>,<len>,<data>,<DISTANCE cm>,<RSSI>
        body = line.split('=', 1)[1]
        parts = [p.strip() for p in body.split(',')]
        for p in parts:
            low = p.lower()
            if 'cm' in low:
                try:
                    return float(low.replace('cm', '').strip()) / 100.0
                except ValueError:
                    return None
        if len(parts) >= 4:                    # fallback: 4th field is distance
            try:
                return float(parts[3].split()[0]) / 100.0
            except (ValueError, IndexError):
                return None
        return None

    def range_once(self, a):
        ser = a['serial']
        if ser is None:
            return None
        try:
            ser.reset_input_buffer()
            cmd = f"AT+ANCHOR_SEND={self.tag_address},{len(self.payload)},{self.payload}"
            ser.write((cmd + "\r\n").encode())
            deadline = time.time() + self.reply_timeout
            while time.time() < deadline:
                line = ser.readline().decode(errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('+ANCHOR_RCV='):
                    return self._parse_distance_m(line)
                # ignore +OK, echoes, and error lines here
            return None
        except Exception as e:
            self.get_logger().debug(f"{a['name']} range error: {e}")
            return None

    def _publish(self, a, dist_m):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = a['frame']
        msg.radiation_type = Range.ULTRASOUND   # no UWB type; ULTRASOUND is the placeholder
        msg.field_of_view = self.fov
        msg.min_range = self.range_min
        msg.max_range = self.range_max
        msg.range = float(dist_m)
        a['pub'].publish(msg)

    def _loop(self):
        while rclpy.ok() and not self._stop:
            for a in self.anchors:            # right, then left: time-multiplex
                d = self.range_once(a)
                if d is not None:
                    self._publish(a, d)
            time.sleep(self.poll_period)

    def close(self):
        self._stop = True
        for a in self.anchors:
            if a['serial'] is not None and a['serial'].is_open:
                a['serial'].close()


def main(args=None):
    rclpy.init(args=args)
    node = UwbRanging()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
