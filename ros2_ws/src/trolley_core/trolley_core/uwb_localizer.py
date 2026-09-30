"""UWB + odometry EKF localizer for the operator tag.

Two cart anchors range the operator's tag. This node fuses the two range streams
with the cart odometry into a smooth estimate of the tag position, in the fixed
odom frame, using a constant-velocity model for the walking operator. It then
publishes the tag in the base_link frame for the follow controller, which reads
it as range + bearing (the "Kalman follow").

State (odom frame): x = [px, py, vx, vy]   (tag position + velocity).

Measurement (per anchor): h(x) = || tag - anchor_odom ||, a range. The two
ranges are time-multiplexed, so each is its own scalar EKF update. H is the unit
line-of-sight row. An innovation gate rejects outliers.

TWO-ANCHOR AMBIGUITY. Two ranges fix the tag only up to a front/back mirror
across the anchor line. Range-only data cannot resolve it, and at close range
noise flips the estimate behind the cart (bearing jumps to +/-180 deg). Two
measures resolve it here:
  1. Initialisation solves the two-range circle intersection and picks the
     FRONT solution (the operator is ahead of the cart).
  2. A forward half-plane constraint reflects the state back to the front if it
     ever crosses behind the anchor line. This uses the follow-application prior
     (the operator is always in front).

Publishes:
  /follow/target     geometry_msgs/PointStamped (base_link)  fused tag, for follow
  /uwb/tag_odom      geometry_msgs/PointStamped (odom)        for RViz / evaluation
  /uwb/innovation    std_msgs/Float32  signed range innovation of accepted updates
  /uwb/nis           std_msgs/Float32  normalised innovation squared (consistency)
  /uwb/reject_count  std_msgs/Float32  cumulative count of gate-rejected ranges

Geometry: anchor_x (forward offset of both anchors from base_link) and
anchor_baseline (left-right separation). A wider baseline sharpens the bearing.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from trolley_core.diagnostics import Diagnostics, OK, WARN, ERROR


class UwbLocalizer(Node):
    def __init__(self):
        super().__init__('uwb_localizer')

        self.declare_parameter('anchor_x', 0.30)          # forward offset of the anchors (m)
        self.declare_parameter('anchor_baseline', 0.24)   # left-right separation (m)
        self.declare_parameter('left_topic', '/uwb/left')
        self.declare_parameter('right_topic', '/uwb/right')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('target_topic', '/follow/target')
        self.declare_parameter('odom_target_topic', '/uwb/tag_odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('range_noise_std', 0.05)   # m, from measured UWB std
        self.declare_parameter('process_accel_std', 0.6)  # m/s^2, operator manoeuvre
        self.declare_parameter('max_operator_speed', 2.0)  # m/s clamp on the tag velocity
        self.declare_parameter('min_valid_m', 0.20)
        self.declare_parameter('max_valid_m', 30.0)
        self.declare_parameter('innovation_gate_m', 0.4)  # reject |z-h| bigger than this; 0 = off
        self.declare_parameter('init_stale_s', 0.5)       # both ranges must be this fresh to init
        self.declare_parameter('starve_timeout_s', 0.5)   # no accepted range this long -> drop filter (no open-loop coast)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('publish_debug', True)

        g = lambda n: self.get_parameter(n).value
        self.anchor_x = float(g('anchor_x'))
        half = float(g('anchor_baseline')) / 2.0
        self.mounts = {'left': (self.anchor_x, +half), 'right': (self.anchor_x, -half)}
        self.base_frame = g('base_frame')
        self.odom_frame = g('odom_frame')
        self.R = float(g('range_noise_std')) ** 2
        self.q = float(g('process_accel_std')) ** 2
        self.vmax = float(g('max_operator_speed'))
        self.min_valid = float(g('min_valid_m'))
        self.max_valid = float(g('max_valid_m'))
        self.gate = float(g('innovation_gate_m'))
        self.baseline = float(g('anchor_baseline'))
        self.consist_margin = 0.05   # m; reject |rL-rR| beyond baseline+this (~0.29 m).
                                     # real front-follow |rL-rR| stays under 0.15 m; a
                                     # larger gap is NLOS/glitch. Was 0.15 (0.39 m) - too
                                     # loose: a 0.31 m glitch drove a false pivot.
        self.init_stale = float(g('init_stale_s'))
        self.starve_timeout = float(g('starve_timeout_s'))
        self.debug = bool(g('publish_debug'))

        self.x = None
        self.P = None
        self.t = None
        self.cart = None
        self.last_r = {}          # name -> (range, stamp)
        self.reject_count = 0.0
        self.last_accept = None   # stamp of the last ACCEPTED (fused) range

        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 20)
        self.create_subscription(Range, g('left_topic'),
                                 lambda m: self.on_range('left', m), 20)
        self.create_subscription(Range, g('right_topic'),
                                 lambda m: self.on_range('right', m), 20)
        self.tgt_pub = self.create_publisher(PointStamped, g('target_topic'), 10)
        self.odom_tgt_pub = self.create_publisher(PointStamped, g('odom_target_topic'), 10)
        self.innov_pub = self.create_publisher(Float32, '/uwb/innovation', 10)
        self.nis_pub = self.create_publisher(Float32, '/uwb/nis', 10)
        self.reject_pub = self.create_publisher(Float32, '/uwb/reject_count', 10)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self.on_timer)

        self._diag_last_rejects = 0.0
        self.diag = Diagnostics(self)
        self.diag.add('tracking', self._diag_tracking)
        self.diag.add('anchor left', lambda: self._diag_anchor('left'))
        self.diag.add('anchor right', lambda: self._diag_anchor('right'))
        self.get_logger().info("UWB localizer (EKF v3: two-anchor init + front constraint) active.")

    # ---- odom + geometry ----
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.cart = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

    def _anchor_odom(self, name):
        xc, yc, th = self.cart
        mx, my = self.mounts[name]
        return np.array([xc + mx * math.cos(th) - my * math.sin(th),
                         yc + mx * math.sin(th) + my * math.cos(th)])

    def _anchor_line(self):
        """Anchor-midpoint m (odom) and the cart forward unit vector n."""
        xc, yc, th = self.cart
        n = np.array([math.cos(th), math.sin(th)])
        m = np.array([xc + self.anchor_x * n[0], yc + self.anchor_x * n[1]])
        return m, n

    def _constrain_front(self):
        """Reflect the state to the front half-plane if it crossed behind the
        anchor line (two-anchor mirror-ambiguity resolution)."""
        if self.x is None or self.cart is None:
            return
        m, n = self._anchor_line()
        s = (self.x[0] - m[0]) * n[0] + (self.x[1] - m[1]) * n[1]   # forward distance
        if s < 0.0:
            self.x[0] -= 2.0 * s * n[0]
            self.x[1] -= 2.0 * s * n[1]
            vdot = self.x[2] * n[0] + self.x[3] * n[1]
            self.x[2] -= 2.0 * vdot * n[0]
            self.x[3] -= 2.0 * vdot * n[1]

    def _clamp_velocity(self):
        sp = math.hypot(self.x[2], self.x[3])
        if sp > self.vmax and sp > 1e-6:
            k = self.vmax / sp
            self.x[2] *= k
            self.x[3] *= k

    # ---- init / predict / update ----
    def _try_init(self, now):
        """Two-range circle intersection; seed the FRONT solution."""
        fresh = {}
        for nm, (r, ts) in self.last_r.items():
            if (now - ts).nanoseconds / 1e9 < self.init_stale:
                fresh[nm] = r
        if len(fresh) < 2 or self.cart is None:
            return
        aL = self._anchor_odom('left'); aR = self._anchor_odom('right')
        rL = fresh['left']; rR = fresh['right']
        if abs(rL - rR) > self.baseline + self.consist_margin:
            return                              # impossible pair -> do not seed
        d = np.linalg.norm(aR - aL)
        if d < 1e-3:
            return
        a = (rL * rL - rR * rR + d * d) / (2.0 * d)
        h2 = rL * rL - a * a
        h = math.sqrt(h2) if h2 > 0.0 else 0.0
        ex = (aR - aL) / d
        pm = aL + a * ex
        perp = np.array([-ex[1], ex[0]])
        s1 = pm + h * perp
        s2 = pm - h * perp
        m, n = self._anchor_line()
        p = s1 if np.dot(s1 - m, n) >= np.dot(s2 - m, n) else s2   # front solution
        self.x = np.array([p[0], p[1], 0.0, 0.0])
        self.P = np.diag([0.25, 0.25, 1.0, 1.0]).astype(float)
        self.t = now
        self.last_accept = now
        self.get_logger().info("EKF seeded from two-anchor intersection (front).")

    def _predict(self, now):
        if self.x is None or self.t is None:
            self.t = now
            return
        dt = (now - self.t).nanoseconds / 1e9
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        self.x = F @ self.x
        self._clamp_velocity()
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = self.q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                               [0, dt4 / 4, 0, dt3 / 2],
                               [dt3 / 2, 0, dt2, 0],
                               [0, dt3 / 2, 0, dt2]], float)
        self.P = F @ self.P @ F.T + Q
        self.t = now

    def on_range(self, name, msg):
        if self.cart is None:
            return
        r = float(msg.range)
        if not (self.min_valid < r < self.max_valid):
            return
        now = self.get_clock().now()
        self.last_r[name] = (r, now)

        if self.x is None:
            self._try_init(now)
            return

        # two-anchor consistency: |rL - rR| cannot exceed the baseline (triangle
        # inequality). A violation means one anchor is NLOS/glitching -> reject.
        other = 'right' if name == 'left' else 'left'
        if other in self.last_r:
            r_o, t_o = self.last_r[other]
            if (now - t_o).nanoseconds / 1e9 < 0.3 and abs(r - r_o) > self.baseline + self.consist_margin:
                self.reject_count += 1.0
                if self.debug:
                    self.reject_pub.publish(Float32(data=self.reject_count))
                return

        self._predict(now)
        anc = self._anchor_odom(name)
        dx, dy = self.x[0] - anc[0], self.x[1] - anc[1]
        h = math.hypot(dx, dy)
        if h < 1e-3:
            return
        yk = r - h                                   # innovation
        H = np.array([[dx / h, dy / h, 0.0, 0.0]], float)
        S = float(H @ self.P @ H.T) + self.R
        if self.gate > 0.0 and abs(yk) > self.gate:
            self.reject_count += 1.0
            if self.debug:
                self.reject_pub.publish(Float32(data=self.reject_count))
            return
        K = (self.P @ H.T) / S
        self.x = self.x + K[:, 0] * yk
        self.P = (np.eye(4) - K @ H) @ self.P
        self._clamp_velocity()
        self._constrain_front()
        self.last_accept = now
        if self.debug:
            self.innov_pub.publish(Float32(data=float(yk)))
            self.nis_pub.publish(Float32(data=float(yk * yk / S)))

    # ---- diagnostics ----
    def _diag_tracking(self):
        rejects = self.reject_count - self._diag_last_rejects
        self._diag_last_rejects = self.reject_count
        values = {'rejects_total': int(self.reject_count), 'rejects_last_s': int(rejects)}
        if self.cart is None:
            return ERROR, 'no odometry received', values
        if self.x is None:
            return WARN, 'no fix: waiting for a consistent range from both anchors', values
        if self.last_accept is not None:
            values['accept_age_s'] = f'{(self.get_clock().now() - self.last_accept).nanoseconds / 1e9:.2f}'
        values['target_range_m'] = f'{math.hypot(self.x[0] - self.cart[0], self.x[1] - self.cart[1]):.2f}'
        if rejects >= 3:
            return WARN, f'rejecting ranges ({int(rejects)} in the last second)', values
        return OK, 'tracking', values

    def _diag_anchor(self, name):
        if name not in self.last_r:
            return WARN, 'no valid range received', {}
        r, stamp = self.last_r[name]
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        values = {'range_m': f'{r:.2f}', 'range_age_s': f'{age:.2f}'}
        if age > self.init_stale:
            return WARN, f'no valid range for {age:.1f} s', values
        return OK, 'ranging', values

    # ---- output ----
    def on_timer(self):
        if self.x is None or self.cart is None:
            return
        now = self.get_clock().now()
        # Starvation guard: if no range has been ACCEPTED for a while, the filter
        # is coasting open-loop (range runs away). Drop it so we stop publishing a
        # target -> the controller holds instead of chasing a phantom. The next
        # consistent two-range pair reseeds it via _try_init.
        if self.last_accept is not None and \
           (now - self.last_accept).nanoseconds / 1e9 > self.starve_timeout:
            self.x = None
            self.get_logger().warn("UWB starved: no accepted range -> reset (hold).")
            return
        self._predict(now)
        self._constrain_front()
        px, py = float(self.x[0]), float(self.x[1])

        po = PointStamped()
        po.header.stamp = now.to_msg()
        po.header.frame_id = self.odom_frame
        po.point.x, po.point.y = px, py
        self.odom_tgt_pub.publish(po)

        xc, yc, th = self.cart
        c, s = math.cos(th), math.sin(th)
        pb = PointStamped()
        pb.header.stamp = now.to_msg()
        pb.header.frame_id = self.base_frame
        pb.point.x = (px - xc) * c + (py - yc) * s
        pb.point.y = -(px - xc) * s + (py - yc) * c
        self.tgt_pub.publish(pb)


def main(args=None):
    rclpy.init(args=args)
    node = UwbLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
