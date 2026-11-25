import json
import time

import rclpy
from control_msgs.msg import ChannelNormalized, NormalizedControls
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from pius_control.calibration import CRITICAL, load_config, names, unverified, warn_lines
from pius_control.waypoint_controller import THROTTLE_TABLE, WaypointController


class WaypointControllerNode(Node):
    """
    Turns the predicted waypoints (/inference/waypoints) into AUTO commands on /cmd/normalized/auto at 50 Hz:
    steering by Pure Pursuit, throttle / brake by a speed PID on the wheel speed (waypoint_controller.py).
    Between predictions the waypoints are re-projected with the wheel speed and the IMU yaw rate.

    Nothing is published (so mux refuses or leaves AUTO and stops the car) when the waypoints or the speed are
    stale, or while a parameter needed for safe AUTO is still provisional (config `unverified`, see
    calibration.py) unless controller.allow_unverified is true. /control/status (JSON, 10 Hz) always tells why.
    """

    def __init__(self):
        super().__init__('waypoint_controller_node')
        cfg = load_config()
        c = dict(cfg.get('controller', {}))
        veh = cfg.get('vehicle', {})
        c['wheelbase_m'] = float(veh.get('wheelbase_m', 1.5))
        c['steer_table'] = veh.get('steer_table', [[-1.0, 23.8], [0.0, 0.0], [1.0, -23.8]])
        self.yaw_sign = float(veh.get('yaw_sign', 1))
        self.c = c
        self.config_error = None
        try:
            self.ctl = WaypointController(c)
        except Exception as e:          # e.g. a steer_table that is not monotonic: never drive with it
            self.config_error = f"設定の誤り（vehicle.steer_table・controller.throttle_table など）: {e}"
            self.get_logger().error(f"【自動運転の制御】{self.config_error}。AUTO 用の操作量は出しません。")
            c_safe = dict(c, steer_table=[[-1.0, 1.0], [1.0, -1.0]], throttle_table=THROTTLE_TABLE)
            self.ctl = WaypointController(c_safe)
        self.verbose = int(c.get('verbose', 1))

        items = unverified(cfg)
        self.unverified = names(items)
        self.blocking = [n for n in self.unverified if n in CRITICAL]
        self.allow_unverified = bool(c.get('allow_unverified', False))
        if items:
            for line in warn_lines(items):
                self.get_logger().warning(line)
        if self.blocking and not self.allow_unverified:
            self.get_logger().error("【自動運転の制御】AUTO に必須のパラメータが仮の値のため、AUTO 用の操作量を出しません: "
                                    + "、".join(self.blocking) + "（実測して config.yaml の unverified から消してください）")
        elif self.blocking:
            self.get_logger().error("【自動運転の制御】controller.allow_unverified が true のため、仮の値のまま操作量を出します。"
                                    "車輪を浮かせた試験以外では使わないでください。")

        self.waypoints_t = None
        self.speed = None
        self.speed_t = None
        self.direction = 1
        self.yaw_rate = 0.0
        self.mode = None
        self.last_out = {}
        self.last_reason = None

        self.pub = self.create_publisher(NormalizedControls, c.get('output_topic', '/cmd/normalized/auto'), 10)
        self.status_pub = self.create_publisher(String, c.get('status_topic', '/control/status'), 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.create_subscription(String, c.get('input_topic_waypoints', '/inference/waypoints'), self._on_waypoints, 10)
        self.create_subscription(String, c.get('input_topic_wheel', '/wheel/status'), self._on_wheel, 10)
        self.create_subscription(Imu, c.get('input_topic_imu', '/imu/data'), self._on_imu, 50)
        self.create_subscription(String, c.get('input_topic_mode', '/mux/mode'), self._on_mode, 10)
        self.create_timer(1.0 / float(c.get('rate_hz', 50.0)), self._step)
        self.create_timer(0.1, self._publish_status)
        self.create_timer(1.0, self._publish_health)

    @staticmethod
    def _now():
        return time.time()

    def _on_waypoints(self, msg):
        try:
            d = json.loads(msg.data)
            points = [tuple(p) for p in d["points"]]
        except Exception as e:
            self.get_logger().error(f"waypoint を読めません: {e}", throttle_duration_sec=5.0)
            return
        stamp = float(d.get("stamp") or self._now())
        self.ctl.set_waypoints(stamp, points)
        self.waypoints_t = self._now()

    def _on_wheel(self, msg):
        d = json.loads(msg.data)
        if d.get("speed") is None:
            return
        self.speed = float(d["speed"])
        self.direction = -1 if d.get("direction") == -1 else 1
        self.speed_t = self._now()

    def _on_imu(self, msg):
        self.yaw_rate = self.yaw_sign * msg.angular_velocity.z

    def _on_mode(self, msg):
        if msg.data != self.mode and msg.data != "auto":
            self.ctl.reset()
        self.mode = msg.data

    def _not_ready(self, now):
        if self.config_error:
            return self.config_error
        if self.blocking and not self.allow_unverified:
            return "AUTO に必須のパラメータが仮の値（" + "、".join(self.blocking) + "）"
        if self.waypoints_t is None or now - self.waypoints_t > self.c.get('waypoint_timeout_sec', 0.4):
            return "waypoint が届いていない（モデル未読込・カメラ停止・推論ノード停止）"
        if self.speed_t is None or now - self.speed_t > self.c.get('speed_timeout_sec', 0.5):
            return "車速が分からない（車速センサー・外周の設定を確認）"
        return None

    def _step(self):
        now = self._now()
        if self.speed is not None and self.speed_t is not None and now - self.speed_t < 1.0:
            self.ctl.update_odometry(now, self.direction * self.speed, self.yaw_rate)
        reason = self._not_ready(now)
        if reason is None:
            out = self.ctl.step(now, self.speed)
            if not out["ok"]:
                reason = out["reason"]
            self.last_out = out
        else:
            self.ctl.reset()
            self.last_out = {"path": [[round(x, 3), round(y, 3)] for x, y in self.ctl.current_path()]}
        if reason != self.last_reason:
            if reason:
                self.get_logger().warning(f"【自動運転の制御】操作量を出していません: {reason}")
            elif self.last_reason is not None:
                self.get_logger().info("【自動運転の制御】操作量の送信を始めました。")
            self.last_reason = reason
        if reason:
            return             # mux refuses / leaves AUTO and stops the car
        out = self.last_out
        msg = NormalizedControls()
        msg.data = [ChannelNormalized(channel=0, normalized=float(out["steer"])),
                    ChannelNormalized(channel=1, normalized=float(out["throttle"])),
                    ChannelNormalized(channel=2, normalized=float(out["brake"]))]
        self.pub.publish(msg)

    def _publish_status(self):
        st = dict(self.last_out)
        st.update(ready=self.last_reason is None, reason=self.last_reason or "", mode=self.mode,
                  unverified=self.unverified, blocking=self.blocking if not self.allow_unverified else [],
                  allow_unverified=self.allow_unverified, speed=self.speed)
        msg = String()
        msg.data = json.dumps(st)
        self.status_pub.publish(msg)

    def _publish_health(self):
        msg = String()
        if self.last_reason is None:
            o = self.last_out
            msg.data = (f"control:OK:目標 {o.get('v_target', 0) * 3.6:.1f} km/h・操舵 {o.get('delta_deg', 0):+.0f}°"
                        + ("・仮の値で動作中" if self.blocking else ""))
        elif self.blocking and not self.allow_unverified:
            msg.data = "control:BLOCKED:AUTO に必須のパラメータが仮の値（" + "、".join(self.blocking) + "）"
        else:
            msg.data = f"control:WAITING:{self.last_reason}"
        self.health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
