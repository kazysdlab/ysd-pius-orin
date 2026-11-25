#!/bin/bash

# スクリプトの場所を基準にプロジェクトルートへ移動
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=============================================="
echo "      PIUS自律走行システム 一括起動スクリプト"
echo "=============================================="

# 1. シリアルポートの権限付与
echo "[1/3] シリアルポート (/dev/ttyTHS1) の権限を付与します..."
echo "※ sudoパスワードの入力を求められる場合があります。"
sudo chmod 777 /dev/ttyTHS1
if [ $? -eq 0 ]; then
    echo "-> シリアルポートの権限設定に成功しました。"
else
    echo "-> [警告] 権限の付与に失敗しました。シリアル送信はDRY RUNモードで起動します。"
fi

echo ""

# 2. Dockerコンテナの起動
echo "[2/3] Dockerコンテナの起動状態を確認しています..."
docker compose up -d
if [ $? -eq 0 ]; then
    echo "-> コンテナがバックグラウンドで起動しました。"
else
    echo "-> [エラー] Dockerコンテナの起動に失敗しました。"
    exit 1
fi

echo "コンテナの初期化を待っています (3秒)..."
sleep 3
echo ""

# 3. コンテナ内でROS 2 launchを実行
echo "[3/3] コンテナ内で ROS 2 システム (pius_launch.py) を実行します..."
echo "=========================================================================="
echo "※ 終了（安全停止）するには、このターミナルで 【 Ctrl + C 】 を押してください。"
echo "=========================================================================="
sleep 1

# 引数の処理（モデル指定）
if [ -n "$1" ]; then
    MODEL_ARG="$1"
    # 拡張子 .pth が付いていない場合は自動で付与
    if [[ ! "$MODEL_ARG" == *.pth ]]; then
        MODEL_ARG="${MODEL_ARG}.pth"
    fi
    
    # パスが / から始まらない（相対パス・ファイル名のみ）場合は、コンテナ内の標準モデルディレクトリと結合
    if [[ ! "$MODEL_ARG" == /* ]]; then
        MODEL_PATH_VAL="/home/orin/dev/ws/colcon_ws/models/${MODEL_ARG}"
    else
        MODEL_PATH_VAL="$MODEL_ARG"
    fi
    
    echo "-> 指定されたモデルを使用します: ${MODEL_PATH_VAL}"
    LAUNCH_CMD="source /opt/ros/humble/setup.bash && source /home/orin/dev/ws/colcon_ws/install/setup.bash && ros2 launch pius_control pius_launch.py model_path:=${MODEL_PATH_VAL}"
else
    echo "-> デフォルトのモデルを使用します: config.yaml の inference.model_path（models/waypoint.pth）"
    LAUNCH_CMD="source /opt/ros/humble/setup.bash && source /home/orin/dev/ws/colcon_ws/install/setup.bash && ros2 launch pius_control pius_launch.py"
fi

# docker exec でコンテナ内に入り、setup.bashを読み込んでlaunchを実行
docker exec -it roscar2 bash -c "$LAUNCH_CMD"

echo "=============================================="
echo "      PIUS自律走行システムが終了しました。"
echo "=============================================="
