#!/bin/bash
# install.sh の変更を元に戻す。使い方: sudo ./uninstall.sh
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "sudo で実行してください: sudo ./uninstall.sh"
    exit 1
fi

KVER=$(uname -r)
modprobe -r ch341 2>/dev/null || true
rm -f "/lib/modules/$KVER/updates/ch341.ko"
depmod -a
systemctl unmask brltty-udev.service brltty.service
echo "ch341 ドライバを削除し、brltty の無効化を解除しました。"
