#!/bin/bash
# CH340/CH341 USB シリアルドライバ（このディレクトリで make したもの）をインストールする。
# 使い方: make && sudo ./install.sh
set -euo pipefail
cd "$(dirname "$0")"

if [ "$(id -u)" -ne 0 ]; then
    echo "sudo で実行してください: sudo ./install.sh"
    exit 1
fi

KVER=$(uname -r)
if [ ! -f ch341.ko ]; then
    echo "ch341.ko がありません。先に make を実行してください。"
    exit 1
fi
BUILT_FOR=$(modinfo -F vermagic ./ch341.ko | awk '{print $1}')
if [ "$BUILT_FOR" != "$KVER" ]; then
    echo "ch341.ko はカーネル $BUILT_FOR 用です（実行中のカーネルは $KVER）。make clean && make で再ビルドしてください。"
    exit 1
fi

echo "[1/3] brltty（点字ディスプレイ用サービス）が CH340 を横取りしないよう無効化します"
systemctl stop brltty-udev.service brltty.service 2>/dev/null || true
systemctl mask brltty-udev.service brltty.service

echo "[2/3] ドライバを /lib/modules/$KVER/updates/ にインストールします"
install -D -m 644 ch341.ko "/lib/modules/$KVER/updates/ch341.ko"
depmod -a

echo "[3/3] ドライバを読み込みます"
modprobe ch341
lsmod | grep -E "^ch341"

echo "完了しました。センサーを挿し直すと /dev/ttyUSB0 が作成されます（以後は挿すだけで自動で読み込まれます）。"
