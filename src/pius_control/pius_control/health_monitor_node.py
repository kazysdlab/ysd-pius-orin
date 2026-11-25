import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

from pius_control.calibration import load_config, unverified, warn_lines

class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor_node')
        
        # 状態保持用辞書
        self.status = {
            'camera': 'WAITING',      # WAITING, OK, DUMMY, ERROR
            'gamepad': 'WAITING',     # WAITING, OK, DUMMY, ERROR
            'serial': 'WAITING',      # WAITING, OK, DRY_RUN, ERROR
            'inference': 'WAITING',   # WAITING, OK, NO_MODEL
            'imu': 'WAITING',         # WAITING, OK, NO_DATA, RECONNECTING（任意のセンサー）
            'control': 'WAITING',     # WAITING, OK, BLOCKED（自動運転の制御）
        }
        self.control_detail = ''
        self.model_name = "N/A"
        
        # サブスクリプション
        self.sub_health = self.create_subscription(String, '/system/health', self._health_callback, 10)
        
        # 起動から15秒後にまとめを表示するタイマー
        self.report_timer = self.create_timer(15.0, self._report_summary)
        
        self.get_logger().info("【接続監視】システムの接続状態を診断中 (15秒)...")

    def _health_callback(self, msg: String):
        try:
            parts = msg.data.split(':')
            key = parts[0]
            val = parts[1]
            
            if key in self.status:
                self.status[key] = val
                if key == 'inference' and len(parts) > 2:
                    self.model_name = parts[2]
                if key == 'control':
                    self.control_detail = ':'.join(parts[2:])
        except Exception:
            pass

    def _report_summary(self):
        # タイマーを停止（1回限りの表示）
        self.report_timer.cancel()
        
        # ステータス表示マッピング
        # (ステータス文字列, 補足説明)
        status_map = {
            'OK':       ('🟢 接続正常', '正常に稼働しています。'),
            'DUMMY':    ('🟡 デバッグ起動', 'ダミー（模擬データ）で動作中。'),
            'DRY_RUN':  ('⚠️ DRY RUN起動', 'シリアルポート未接続（実機送信なし）で動作中。'),
            'NO_MODEL': ('⚠️ モデル未検出', '学習済みモデルがありません。手動運転のみ利用可能。'),
            'WAITING':  ('🔴 未接続/エラー', '初期化タイムアウト。ノードが起動していない可能性があります。'),
            'RECONNECTING': ('🔴 再接続中', '接続を確認してください。自動で再接続を試みています。'),
            'NO_DATA':  ('⚠️ データなし', 'ポートは開けましたがデータが届いていません。ボーレート設定を確認してください。'),
            'BLOCKED':  ('⚠️ 仮の値で停止中', 'AUTO に必須のパラメータが仮の値のため、AUTO 用の操作量を出しません。'),
        }
        
        cam_status, cam_desc = status_map.get(self.status['camera'], ('🔴 未接続', 'タイムアウト'))
        pad_status, pad_desc = status_map.get(self.status['gamepad'], ('🔴 未接続', 'タイムアウト'))
        ser_status, ser_desc = status_map.get(self.status['serial'], ('🔴 未接続', 'タイムアウト'))
        inf_status, inf_desc = status_map.get(self.status['inference'], ('🔴 未接続', 'タイムアウト'))
        imu_status, imu_desc = status_map.get(self.status['imu'], ('🔴 未接続', 'タイムアウト'))
        if self.status['imu'] != 'OK':
            imu_desc += '（任意のセンサーのため、走行・記録には影響しません）'
        
        if self.status['inference'] == 'OK':
            inf_status = f"🟢 読込完了 ({self.model_name})"
            inf_desc = "自動走行のための推論準備が整っています。"

        log = self.get_logger().info
        log("="*60)
        log("          【 PIUSシステム 接続健全性サマリー 】")
        log("="*60)
        log(f"  ● CSIカメラ接続   : {cam_status:<20} | {cam_desc}")
        log(f"  ● コントローラー : {pad_status:<20} | {pad_desc}")
        log(f"  ● 車両シリアル   : {ser_status:<20} | {ser_desc}")
        log(f"  ● AI推論モデル   : {inf_status:<20} | {inf_desc}")
        log(f"  ● IMU/GPS        : {imu_status:<20} | {imu_desc}")
        ctl_status, _ = status_map.get(self.status['control'], ('🔴 未接続', 'タイムアウト'))
        log(f"  ● 自動運転の制御 : {ctl_status:<20} | {self.control_detail or '—'}")
        log("="*60)
        items = unverified(load_config())
        if items:
            for line in warn_lines(items):
                log(line)
            log("  （一覧: python3 tools/check_calibration.py。実測したら config.yaml の unverified から名前を消す）")
            log("="*60)
        
        # 総合判定
        all_ok = (self.status['camera'] in ['OK', 'DUMMY']) and \
                 (self.status['gamepad'] in ['OK', 'DUMMY']) and \
                 (self.status['serial'] == 'OK') and \
                 (self.status['inference'] == 'OK')
                 
        if all_ok:
            log("  🎉 [システム診断: ALL GREEN] すべての接続が確認されました！")
            log("  👉 コントローラーの【 SHAREボタン 】を押すと、自動運転を開始します。")
        else:
            warnings = []
            if self.status['serial'] == 'DRY_RUN':
                warnings.append("・シリアル送信が動作していません。実機を動かす場合はシリアル接続を確認してください。")
            if self.status['gamepad'] in ('WAITING', 'RECONNECTING'):
                warnings.append("・コントローラーが未接続です。有線接続またはBluetoothペアリングを確認してください。")
            if self.status['inference'] == 'NO_MODEL':
                warnings.append("・自動運転モデルがありません。自動走行をする場合はモデルを models/ に配置してください。")
            
            log("  ⚠️ [システム診断: 一部機能制限あり]")
            for w in warnings:
                log(f"    {w}")
        log("  🖥  現在の状態はWebダッシュボード (http://localhost:8080 または http://<Tailscale の IP>:8080。ローカルネットワークからは開けません) で常時確認できます。")
        log("="*60)

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
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
