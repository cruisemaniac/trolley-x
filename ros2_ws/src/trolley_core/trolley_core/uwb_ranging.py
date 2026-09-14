"""UWB ranging driver (REYAX RYUW122, two cart anchors).

Two RYUW122 modules run as ANCHORs on the cart, one per front pillar. The
operator carries one RYUW122 as the TAG. The anchor is the side that starts a
range and reports the distance, so this node drives both anchors and reads the
two distances to the tag.

Both anchors share one network and one tag, so they must NOT transmit at the
same time. One worker thread ranges them in turn (right, then left) to keep the
two exchanges apart. Each range publishes a sensor_msgs/Range on its own topic.

v2: the module also reports RSSI (signal strength) in the reply. This is now
parsed and published as std_msgs/Float32 on <topic>/rssi, so signal strength can
be logged against distance (path-loss, and NLOS / body-blocking detection).

RYUW122 exchange (see the REYAX AT command guide):
  send:  AT+ANCHOR_SEND=<tag address>,<len>,<payload>
  reply: +ANCHOR_RCV=<tag address>,<len>,<payload>,<DISTANCE cm>,<RSSI>
"""

import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import serial


class UwbRanging(Node):
    def __init__(self):
        super().__init__('uwb_ranging')

        self.declare_parameter('right_port', '/dev/ttyAMA0')
        self.declare_parameter('left_port', '/dev/ttyAMA2')
        self.declare_parameter('baud_rate', 115200)

        self.declare_parameter('tag_address', 'TAG00001')
        self.declare_parameter('network_id', 'TROLLEYX')
        self.declare_parameter('cpin', '')
        self.declare_parameter('right_address', 'ANCHOR_R')
        self.declare_parameter('left_address', 'ANCHOR_L')
        self.declare_parameter('right_cal_cm', 0)
        self.declare_parameter('left_cal_cm', 0)

        self.declare_parameter('right_topic', 'uwb/right')
        self.declare_parameter('left_topic', 'uwb/left')
        self.declare_parameter('right_frame', 'uwb_right')
        self.declare_parameter('left_frame', 'uwb_left')

        self.declare_parameter('poll_period_s', 0.05)
        self.declare_parameter('reply_timeout_s', 0.30)
        self.declare_parameter('payload', 'P')
        self.declare_parameter('range_min_m', 0.10)
        self.declare_parameter('range_max_m', 50.0)
        self.declare_parameter('field_of_view_rad', 0.5)

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
            f"UWB ranging v2 active (range + RSSI). anchors -> tag "
            f"'{self.tag_address}' on network '{self.network_id}'")

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
            'rssi_pub': self.create_publisher(Float32, topic + '/rssi', 10),
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
            "AT+MODE=1",
            f"AT+NETWORKID={self.network_id}",
            f"AT+ADDRESS={a['address']}",
        ]
        if self.cpin:
            cmds.append(f"AT+CPIN={self.cpin}")
        if a['cal'] != 0:
            cmds.append(f"AT+CAL={a['cal']}")
        for c in cmds:
            ok = self._send_expect_ok(a['serial'], c)
            if not ok:
                time.sleep(0.3)
                ok = self._send_expect_ok(a['serial'], c)
            if not ok:
                self.get_logger().warn(f"{a['name']}: '{c}' gave no +OK")
            if c.startswith('AT+MODE'):
                time.sleep(0.3)

    def _parse_reply(self, line):
        """Return (distance_m, rssi) from +ANCHOR_RCV. rssi is None if absent."""
        body = line.split('=', 1)[1]
        parts = [p.strip() for p in body.split(',')]
        dist_m = None
        rssi = None
        # distance: field containing 'cm', else the 4th field
        for p in parts:
            if 'cm' in p.lower():
                try:
                    dist_m = float(p.lower().replace('cm', '').strip()) / 100.0
                except ValueError:
                    dist_m = None
                break
        if dist_m is None and len(parts) >= 4:
            try:
                dist_m = float(parts[3].split()[0]) / 100.0
            except (ValueError, IndexError):
                dist_m = None
        # RSSI: the last numeric field (often negative dBm)
        if len(parts) >= 5:
            try:
                rssi = float(parts[4].split()[0])
            except (ValueError, IndexError):
                rssi = None
        return dist_m, rssi

    def range_once(self, a):
        ser = a['serial']
        if ser is None:
            return None, None
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
                    return self._parse_reply(line)
            return None, None
        except Exception as e:
            self.get_logger().debug(f"{a['name']} range error: {e}")
            return None, None

    def _publish(self, a, dist_m, rssi):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = a['frame']
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov
        msg.min_range = self.range_min
        msg.max_range = self.range_max
        msg.range = float(dist_m)
        a['pub'].publish(msg)
        if rssi is not None:
            a['rssi_pub'].publish(Float32(data=float(rssi)))

    def _loop(self):
        while rclpy.ok() and not self._stop:
            for a in self.anchors:
                d, rssi = self.range_once(a)
                if d is not None:
                    self._publish(a, d, rssi)
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
