#!/bin/bash
# 受信プログラム（receiver_.py）を Raspberry Pi に送り、起動時に自動で動くサービスとして設置する（Jetson で実行）
#   ./tools/pi/install_receiver.sh                       # 既定の宛先 ais-pius@10.10.10.2（LAN ケーブルの直結）
#   ./tools/pi/install_receiver.sh ais-pius@<PiのIP>      # Wi-Fi 経由など、別の宛先
# Raspberry Pi の sudo のパスワードを 1 回聞かれます。何度実行しても同じ結果になります（更新にも使えます）。
set -e
PI="${1:-ais-pius@10.10.10.2}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
echo "送り先: $PI"
ssh "$PI" 'cd /home/ais-pius && [ -f receiver_.py ] && cp -p receiver_.py receiver_.py.prev; true'
scp "$ROOT/receiver_.py" "$ROOT/tools/pi/pius-receiver.service" "$PI:/home/ais-pius/"
ssh -t "$PI" 'set -e
  sudo cp /home/ais-pius/pius-receiver.service /etc/systemd/system/pius-receiver.service
  sudo systemctl daemon-reload
  sudo systemctl enable --now pigpiod
  sudo systemctl enable pius-receiver
  sudo systemctl restart pius-receiver
  sleep 3
  systemctl --no-pager --lines=5 status pius-receiver'
echo
echo "設置しました。受信プログラムは Raspberry Pi の起動時に自動で動き、SSH が切れても止まりません。"
echo "  様子を見る:   ssh $PI 'journalctl -u pius-receiver -f'"
echo "  止める:       ssh -t $PI 'sudo systemctl stop pius-receiver'   （止めた後も安全な値を出し続けます）"
echo "  元に戻す:     receiver_.py.prev に前の版を残しています"
