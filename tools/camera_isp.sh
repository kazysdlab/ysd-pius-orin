#!/bin/bash
# Waveshare IMX219-160（160°広角）用の ISP 補正ファイルを導入／削除する。
# 標準の IMX219 用補正はこのレンズに合わず、画像の周辺がピンク色になるため、
# メーカー（Waveshare）配布のレンズシェーディング・色補正を上書きで適用する。
#
# 使い方:  sudo ./tools/camera_isp.sh install   # 導入
#          sudo ./tools/camera_isp.sh remove    # 削除（標準の補正に戻す）
set -euo pipefail

URL="https://files.waveshare.com/upload/e/eb/Camera_overrides.tar.gz"
DEST=/var/nvidia/nvcam/settings/camera_overrides.isp

if [ "$(id -u)" -ne 0 ]; then
    echo "sudo で実行してください: sudo $0 install | remove"
    exit 1
fi

case "${1:-}" in
    install)
        tmp=$(mktemp -d)
        trap 'rm -rf "$tmp"' EXIT
        echo "Waveshare 公式の補正ファイルをダウンロードしています: $URL"
        curl -fsSL -o "$tmp/overrides.tar.gz" "$URL"
        tar xzf "$tmp/overrides.tar.gz" -C "$tmp" camera_overrides.isp
        install -D -m 664 -o root -g root "$tmp/camera_overrides.isp" "$DEST"
        echo "導入しました: $DEST"
        ;;
    remove)
        rm -f "$DEST"
        echo "削除しました（標準の補正に戻ります）: $DEST"
        ;;
    *)
        echo "使い方: sudo $0 install | remove"
        exit 1
        ;;
esac

# 補正はカメラを開くときに読み込まれるため、カメラ制御デーモンを再起動する
systemctl restart nvargus-daemon
echo "nvargus-daemon を再起動しました。動作中のカメラノードは自動で再接続します。"
