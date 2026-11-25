#!/bin/bash
# 車速のリアルタイム表示とログ記録（走行システムの起動中に、ホストの別の端末で実行）
# ブラウザで http://<JetsonのIP>:8888 を開く。Ctrl + C で終了。ログは data/speed_logs/ に保存。
# 名前を付けて保存: ./tools/speed_live.sh --name B_left → speed_<日時>_B_left.jsonl
if ! docker ps --format '{{.Names}}' | grep -qx roscar2; then
    echo "コンテナ roscar2 が起動していません。先に ./start_system.sh で走行システムを起動してください。"
    exit 1
fi
ARGS=""
[ $# -gt 0 ] && ARGS=$(printf ' %q' "$@")
docker exec -it -e TZ=Asia/Tokyo roscar2 bash -c "source /opt/ros/humble/setup.bash && \
    source /home/orin/dev/ws/colcon_ws/install/setup.bash && \
    python3 -u /home/orin/dev/ws/colcon_ws/tools/speed_live.py${ARGS}"
