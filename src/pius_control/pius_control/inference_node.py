import json
import os
import time

import numpy as np
import rclpy
import torch
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from pius_control.waypoint_model import load_checkpoint, to_tensor


class InferenceNode(Node):
    """
    Runs the waypoint model (waypoint_model.py, trained with train_waypoint.py) on every camera image and
    publishes the predicted path on /inference/waypoints (JSON: rear-axle frame, x forward / y left [m],
    one point every `spacing` metres of travelled distance).

    The vehicle-side controller (Pure Pursuit + speed PID) that turns waypoints into commands is not
    implemented yet, so this node never publishes /cmd/normalized/auto: AUTO stays unavailable (mux refuses
    it without fresh AUTO commands), which is the safe state.
    """

    def __init__(self):
        super().__init__('inference_node')

        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        default_model_path = "/home/orin/dev/ws/colcon_ws/models/waypoint.pth"
        cfg = {}
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = (yaml.safe_load(f) or {}).get('inference', {})
        else:
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        self.input_topic_image = cfg.get('input_topic_image', '/camera/image_raw')
        self.output_topic = cfg.get('output_topic_waypoints', '/inference/waypoints')
        default_model_path = cfg.get('model_path', default_model_path)
        self.verbose = int(cfg.get('verbose', 1))

        self.declare_parameter('model_path', default_model_path)
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = None
        self.model_error = ""
        self.image_size = None
        self.last_ms = None
        if not os.path.exists(self.model_path):
            self.model_error = "モデルファイルがありません"
            self.get_logger().warning(f"【モデル未検出】モデルファイル {self.model_path} が見つかりません。")
        else:
            try:
                model, ckpt = load_checkpoint(self.model_path, map_location='cpu')
                self.model = model.to(self.device).eval()
                if self.device.type == 'cuda':
                    self.model.half()
                self.image_size = tuple(model.config["image_size"])
                metrics = ckpt.get("metrics", {})
                score = f"（検証 ADE {metrics['ade']:.2f} m）" if "ade" in metrics else ""
                self.get_logger().info(f"【モデル読込】waypoint モデルを読み込みました: {self.model_path}{score}"
                                       f"、使用デバイス: {str(self.device).upper()}")
            except Exception as e:
                self.model_error = f"読み込みに失敗: {e}"
                self.get_logger().error(f"【読込エラー】{self.model_path}: {e}")
        self.get_logger().warning("【制御未実装】waypoint を操作量に変える制御ノードがまだないため、AUTO 用の操作量は送信しません"
                                  "（AUTO モードには切り替わりません）。")

        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()
        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.sub_image = self.create_subscription(Image, self.input_topic_image, self._image_callback,
                                                  rclpy.qos.qos_profile_sensor_data)

    def _publish_health(self):
        msg = String()
        name = os.path.basename(self.model_path)
        if self.model is not None:
            speed = f"・{self.last_ms:.0f} ms" if self.last_ms is not None else ""
            msg.data = f"inference:OK:{name}（waypoint 予測のみ{speed}）"
        else:
            msg.data = f"inference:NO_MODEL:{name}（{self.model_error}）"
        self.health_pub.publish(msg)

    @staticmethod
    def _to_numpy(msg: Image) -> np.ndarray:
        h, w, c = msg.height, msg.width, 3
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if int(msg.step) == w * c:
            return buf.reshape((h, w, c))
        return buf.reshape((h, int(msg.step)))[:, : w * c].reshape((h, w, c))

    def _image_callback(self, msg: Image):
        if self.model is None:
            return
        frame = self._to_numpy(msg)
        if frame.shape[:2] != self.image_size:
            self.get_logger().error(f"画像サイズ {frame.shape[1]}×{frame.shape[0]} がモデルの入力"
                                    f"（{self.image_size[1]}×{self.image_size[0]}）と違います。config.yaml の camera.resize を確認してください。",
                                    throttle_duration_sec=10.0)
            return
        t0 = time.perf_counter()
        try:
            x = to_tensor(frame).to(self.device)
            if self.device.type == 'cuda':
                x = x.half()
            with torch.no_grad():
                waypoints, _ = self.model(x)
            points = waypoints[0].float().cpu().numpy()
        except Exception as e:
            self.get_logger().error(f"推論に失敗しました: {e}", throttle_duration_sec=5.0)
            return
        self.last_ms = (time.perf_counter() - t0) * 1000.0

        out = String()
        out.data = json.dumps({
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame": "rear_axle",
            "spacing": self.model.config["spacing"],
            "points": [[round(float(px), 3), round(float(py), 3)] for px, py in points],
        })
        self.pub.publish(out)
        if self.verbose >= 2:
            self.get_logger().info(f"[Inference] {self.last_ms:.1f} ms -> {out.data}")


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
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
