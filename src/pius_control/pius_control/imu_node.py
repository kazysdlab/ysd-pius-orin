import json
import math
import os
import time

import rclpy
import serial
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import String

from .witmotion import WitMotionParser

GYRO_RANGE_DPS = 500.0  # WTGAHRS3 gyro full scale; attitude becomes unreliable when exceeded


def euler_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    """ZYX Euler angles in degrees -> (x, y, z, w)."""
    r, p, y = (math.radians(a) / 2.0 for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuNode(Node):
    """
    WitMotion WTGAHRS3 (IMU + GNSS) driver.

    Publishes sensor_msgs/Imu, sensor_msgs/NavSatFix and a JSON summary for the dashboard and
    data recording. The sensor is optional: when it is unplugged the node keeps running,
    reports RECONNECTING and reopens the port automatically.
    """

    def __init__(self):
        super().__init__('imu_node')

        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        cfg = {}
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f).get('imu', {})
        else:
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        self.port = cfg.get('port', '/host_dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.baudrate = int(cfg.get('baudrate', 9600))
        self.frame_id = cfg.get('frame_id', 'imu_link')
        self.gps_frame_id = cfg.get('gps_frame_id', 'gps_link')
        self.data_timeout = float(cfg.get('data_timeout_sec', 1.0))
        reconnect_interval = float(cfg.get('reconnect_interval', 1.0))
        self.verbose = int(cfg.get('verbose', 1))

        self.pub_imu = self.create_publisher(Imu, cfg.get('imu_topic', '/imu/data'), 10)
        self.pub_fix = self.create_publisher(NavSatFix, cfg.get('gps_topic', '/gps/fix'), 10)
        self.pub_status = self.create_publisher(String, cfg.get('status_topic', '/imu/status'), 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.notice_pub = self.create_publisher(String, '/system/notice', 10)

        self.ser = None
        self.parser = WitMotionParser()
        self.latest = {}
        self.last_packet_time = 0.0
        self.last_saturation_time = 0.0
        self.last_saturation_notice = 0.0
        self.angle_count = 0
        self.rate = 0.0
        self.rate_start = time.monotonic()
        self.status = "RECONNECTING"
        self.status_detail = "起動中"
        self.failed_attempts = 0
        self.ever_connected = False
        self.no_data_notified = False

        self._connect()
        self.read_timer = self.create_timer(0.02, self._read)
        self.reconnect_timer = self.create_timer(reconnect_interval, self._reconnect)
        self.status_timer = self.create_timer(0.1, self._publish_status)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()

    # --- notices / health ---
    def _notice(self, level, text):
        msg = String()
        msg.data = f"{level}|imu|{text}"
        self.notice_pub.publish(msg)

    def _gps_fixed(self):
        dop = self.latest.get("dop")
        pos = self.latest.get("position")
        return bool(dop and pos and dop["sats"] >= 4 and (pos["lat"] != 0.0 or pos["lon"] != 0.0))

    def _publish_health(self):
        now = time.monotonic()
        if now - self.rate_start >= 1.0:
            self.rate = self.angle_count / (now - self.rate_start)
            self.angle_count, self.rate_start = 0, now

        stale = time.monotonic() - self.last_packet_time > self.data_timeout
        if self.ser is not None and stale and not self.no_data_notified:
            self.no_data_notified = True
            self.status = "NO_DATA"
            self.status_detail = f"{self.data_timeout:.0f}秒以上データが届いていません（ボーレート {self.baudrate} を確認）"
            self.get_logger().warning(f"【IMU】{self.status_detail}")
            self._notice("WARN", f"IMUからデータが届いていません（ボーレート設定 {self.baudrate} を確認してください）")

        if self.status == "OK":
            dop = self.latest.get("dop", {})
            gps = f"GPS測位中（衛星{dop.get('sats', 0)}）" if self._gps_fixed() else f"GPS未測位（衛星{dop.get('sats', 0)}）"
            self.status_detail = f"{self.rate:.0f}Hz・{gps}"

        msg = String()
        msg.data = f"imu:{self.status}:{self.status_detail}"
        self.health_pub.publish(msg)

    # --- serial port ---
    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.failed_attempts += 1
            self.status, self.status_detail = "RECONNECTING", f"ポートを開けません: {self.port}"
            if self.failed_attempts == 1 or self.failed_attempts % 30 == 0:
                self.get_logger().warning(f"【IMU】ポート {self.port} を開けません。センサーの接続を確認してください。再接続を試行中です... ({e})")
            return
        self.parser = WitMotionParser()
        self.failed_attempts = 0
        self.last_packet_time = time.monotonic()
        self.no_data_notified = False
        self.angle_count, self.rate_start = 0, time.monotonic()
        self.status, self.status_detail = "NO_DATA", "データ待ち"
        text = "IMUに再接続しました" if self.ever_connected else "IMUのポートを開きました"
        self.get_logger().info(f"【IMU】{text}: {self.port} ({self.baudrate}bps)")
        self._notice("INFO", text)
        self.ever_connected = True

    def _disconnect(self, error):
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.latest = {}
        self.status, self.status_detail = "RECONNECTING", f"接続が切れました: {error}"
        self.get_logger().error(f"【IMU】接続が切れました: {error}。再接続を試みます。")
        self._notice("WARN", "IMUとの接続が切れました。再接続を試みています（走行には影響しません）")
        self._publish_health()

    def _reconnect(self):
        if self.ser is None:
            self._connect()

    def _read(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(4096)
        except (serial.SerialException, OSError) as e:
            self._disconnect(e)
            return
        if not data:
            return
        packets = self.parser.feed(data)
        if not packets:
            return
        self.last_packet_time = time.monotonic()
        self.no_data_notified = False
        if self.status != "OK":
            self.status = "OK"
            self.get_logger().info("【IMU】データの受信を開始しました。")
        for kind, fields in packets:
            self.latest[kind] = fields
            if kind == "gyro":
                self._check_saturation(fields["gyro"])
            elif kind == "angle":
                self.angle_count += 1
                self._publish_imu()
            elif kind == "position":
                self._publish_fix()

    def _check_saturation(self, gyro):
        if max(abs(v) for v in gyro) < GYRO_RANGE_DPS * 0.98:
            return
        now = time.monotonic()
        self.last_saturation_time = now
        if now - self.last_saturation_notice > 10.0:
            self.last_saturation_notice = now
            text = "角速度が測定範囲（±500°/s）に達しました。姿勢・加速度の値がしばらく不正確になる可能性があります"
            self.get_logger().warning(f"【IMU】{text}")
            self._notice("WARN", text)

    # --- ROS messages ---
    def _publish_imu(self):
        acc = self.latest.get("acc")
        gyro = self.latest.get("gyro")
        euler = self.latest["angle"]["euler"]
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        x, y, z, w = euler_to_quaternion(*euler)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
        if gyro:
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (math.radians(v) for v in gyro["gyro"])
        else:
            msg.angular_velocity_covariance[0] = -1.0
        if acc:
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc["acc"]
        else:
            msg.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(msg)

    def _publish_fix(self):
        pos = self.latest["position"]
        gps = self.latest.get("gps", {})
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.gps_frame_id
        msg.status.status = NavSatStatus.STATUS_FIX if self._gps_fixed() else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = pos["lat"]
        msg.longitude = pos["lon"]
        msg.altitude = gps.get("alt", 0.0)
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(msg)

    def _publish_status(self):
        if self.status != "OK":
            return
        acc = self.latest.get("acc", {})
        pos = self.latest.get("position", {})
        gps = self.latest.get("gps", {})
        dop = self.latest.get("dop", {})
        status = {
            "euler": list(self.latest.get("angle", {}).get("euler", [])),
            "acc": list(acc.get("acc", [])),
            "gyro": list(self.latest.get("gyro", {}).get("gyro", [])),
            "temp": acc.get("temp"),
            "gyro_saturated": time.monotonic() - self.last_saturation_time < 1.0,
            "gps": {
                "fix": self._gps_fixed(),
                "lat": pos.get("lat"),
                "lon": pos.get("lon"),
                "alt": gps.get("alt"),
                "speed": gps.get("speed"),
                "heading": gps.get("heading"),
                "sats": dop.get("sats"),
                "hdop": dop.get("hdop"),
            },
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def close(self):
        if self.ser is not None:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
