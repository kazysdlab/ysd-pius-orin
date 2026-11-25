# CH340 / CH341 USB シリアルドライバ（Jetson 用）

WitMotion WTGAHRS3 などに内蔵されている USB シリアル変換チップ **CH340（USB ID `1a86:7523`）** を Jetson で使うためのドライバです。

JetPack 6（L4T R36）のカーネルは `CONFIG_USB_SERIAL_CH341` が無効で、CH340 を挿しても `/dev/ttyUSB0` が作られません。
ここでは Linux カーネル公式の安定版ソース（`drivers/usb/serial/ch341.c`、v5.15.199、GPL-2.0）をそのまま、実行中のカーネル向けにビルドします。

> [!NOTE]
> Ubuntu 22.04 では点字ディスプレイ用の `brltty` が udev ルールで CH340 を自分の機器とみなして横取りします。`install.sh` はこれを無効化（`systemctl mask`）します。パッケージの削除はしません。

## インストール

ホスト（コンテナの外）の端末で実行します。

```bash
cd /home/yl00/pius/drivers/ch341
make
sudo ./install.sh
```

センサーを挿し直すと `/dev/ttyUSB0` が作成されます。以後は再起動後も挿すだけで自動的に読み込まれます。

## 動作確認

`/dev/ttyUSB0` ができていることを確認し、pyserial の入ったコンテナイメージで確認スクリプトを実行します（`--group-add 20` はシリアルポートの所有グループ `dialout` です）。

```bash
ls -l /dev/ttyUSB*
docker run --rm -it --device /dev/ttyUSB0 --group-add 20 -v /home/yl00/pius:/ws namazyake/roscar2:cu126 python3 /ws/tools/witmotion_check.py /dev/ttyUSB0
```

## カーネルを更新したとき

`apt upgrade` などで JetPack のカーネル（`nvidia-l4t-kernel`）が更新されると、ドライバが読み込まれなくなります。再ビルドして入れ直してください。

```bash
make clean && make
sudo ./install.sh
```

カーネルのバージョン（サブレベル）が変わった場合は、対応するソースに差し替えてからビルドしてください（`X` は `uname -r` の3番目の数字）。

```bash
curl -fL -o ch341.c "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v5.15.X"
```

## 元に戻す

```bash
sudo ./uninstall.sh
```

## 補足

*   自前でビルドした（署名のない）モジュールを読み込むと、カーネルに taint の記録が残りますが、動作には影響しません（このカーネルは署名を強制していません）。
*   ビルド時の `the compiler differs from the one used to build the kernel` という警告は、Ubuntu の gcc のパッチ版数の違いによるもので問題ありません。
