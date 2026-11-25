import rclpy
from rclpy.node import Node
from control_msgs.msg import NormalizedControls, ChannelNormalized
from std_msgs.msg import String
import yaml
import os
import time


def notice(pub, level, source, text):
    """Publish a human-readable notification for the dashboard: 'LEVEL|source|text'."""
    msg = String()
    msg.data = f"{level}|{source}|{text}"
    pub.publish(msg)


class MuxNode(Node):
    """
    Single owner of the driving mode.

    /cmd/mode accepts "manual", "auto" or "toggle". The current mode is published on /mux/mode.
    AUTO is only entered/kept while both the controller (manual stream) and the inference
    stream are alive, so a crashed inference node or a disconnected controller can never leave
    the vehicle driving autonomously without a way to take over.
    """

    def __init__(self):
        super().__init__('mux_node')

        # Load configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        cfg = {}
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        else:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f).get('mux', {})
        self.input_topic_manual = cfg.get('input_topic_manual', '/cmd/normalized/manual')
        self.input_topic_auto = cfg.get('input_topic_auto', '/cmd/normalized/auto')
        self.input_topic_mode = cfg.get('input_topic_mode', '/cmd/mode')
        self.output_topic = cfg.get('output_topic', '/cmd/normalized')
        self.status_topic = cfg.get('output_topic_status', '/mux/mode')
        self.default_mode = cfg.get('default_mode', 'manual')
        self.source_timeout = float(cfg.get('source_timeout_sec', 0.5))
        self.verbose = int(cfg.get('verbose', 1))

        self.current_mode = "manual"
        # default_mode "auto" is honored as soon as both streams are alive
        self.pending_auto = (self.default_mode == "auto")
        self.last_manual_time = 0.0
        self.last_auto_time = 0.0

        # Subscriptions
        self.sub_mode = self.create_subscription(String, self.input_topic_mode, self._mode_callback, 10)
        self.sub_manual = self.create_subscription(NormalizedControls, self.input_topic_manual, self._manual_callback, 10)
        self.sub_auto = self.create_subscription(NormalizedControls, self.input_topic_auto, self._auto_callback, 10)

        # Publishers
        self.pub = self.create_publisher(NormalizedControls, self.output_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_notice = self.create_publisher(String, '/system/notice', 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)

        self.watchdog_timer = self.create_timer(0.05, self._watchdog)
        self.status_timer = self.create_timer(0.2, self._publish_status)
        self.health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"【MUX】入力セレクタノードを初期化しました。(現在の制御ソース: MANUAL, 起動時の指定: {self.default_mode.upper()}, "
            f"入力途絶の判定: {self.source_timeout}秒)")

    # --- stream freshness ---
    def _fresh(self, last_time):
        return last_time > 0.0 and (time.time() - last_time) < self.source_timeout

    def _auto_blockers(self):
        reasons = []
        if not self._fresh(self.last_manual_time):
            reasons.append("コントローラー入力が途絶えています")
        if not self._fresh(self.last_auto_time):
            reasons.append("自動運転の操作量が届いていません（モデル未読込・カメラ停止・推論/制御ノード停止・車速不明・"
                           "仮の値のパラメータのいずれか。ダッシュボードの「自動運転の制御」を確認）")
        return reasons

    # --- mode handling ---
    def _set_mode(self, mode, reason=""):
        if self.current_mode == mode:
            return
        self.current_mode = mode
        suffix = f"（{reason}）" if reason else ""
        self.get_logger().info(f"【MUX】アクティブな制御ソースを {mode.upper()} に切り替えました。{suffix}")
        notice(self.pub_notice, "INFO", "mux", f"走行モードを {mode.upper()} に切り替えました{suffix}")
        self._publish_status()

    def _request_auto(self):
        blockers = self._auto_blockers()
        if blockers:
            text = "AUTOへの切替を拒否しました: " + " / ".join(blockers)
            self.get_logger().warning(f"【MUX】{text}")
            notice(self.pub_notice, "WARN", "mux", text)
            return
        self._set_mode("auto")

    def _mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if mode == "toggle":
            mode = "auto" if self.current_mode == "manual" else "manual"
        if mode == "auto":
            self.pending_auto = False
            self._request_auto()
        elif mode == "manual":
            self.pending_auto = False
            self._set_mode("manual")
        else:
            self.get_logger().warning(f"MUX received invalid mode: {mode}")

    def _publish_stop(self):
        msg = NormalizedControls()
        msg.data = [
            ChannelNormalized(channel=0, normalized=0.0),
            ChannelNormalized(channel=1, normalized=0.0),
            ChannelNormalized(channel=2, normalized=1.0),
        ]
        self.pub.publish(msg)

    def _watchdog(self):
        if self.pending_auto and not self._auto_blockers():
            self.pending_auto = False
            self._set_mode("auto", "起動時の設定 default_mode: auto")
            return

        if self.current_mode == "auto":
            blockers = self._auto_blockers()
            if blockers:
                text = "AUTO走行を中断し、停止してMANUALに戻しました: " + " / ".join(blockers)
                self.get_logger().error(f"【MUX安全停止】{text}")
                notice(self.pub_notice, "ERROR", "mux", text)
                self._publish_stop()
                self._set_mode("manual", "安全停止")

    # --- streams ---
    def _manual_callback(self, msg: NormalizedControls):
        self.last_manual_time = time.time()
        if self.current_mode == "manual":
            self.pub.publish(msg)
            if self.verbose >= 2:
                self.get_logger().info("MUX: Publishing manual controls.")

    def _auto_callback(self, msg: NormalizedControls):
        self.last_auto_time = time.time()
        if self.current_mode == "auto":
            self.pub.publish(msg)
            if self.verbose >= 2:
                self.get_logger().info("MUX: Publishing auto controls.")

    # --- status ---
    def _publish_status(self):
        msg = String()
        msg.data = self.current_mode
        self.pub_status.publish(msg)

    def _publish_health(self):
        msg = String()
        msg.data = f"mux:OK:{self.current_mode}"
        self.health_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MuxNode()
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
