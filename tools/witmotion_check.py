"""WitMotion WTGAHRS3 動作確認スクリプト（Windows / Linux 共通）

使い方:
    pip install pyserial
    python witmotion_check.py COM3          # Windows（COM番号はデバイスマネージャーで確認）
    python witmotion_check.py /dev/ttyUSB0  # Linux
    python witmotion_check.py COM3 115200   # ボーレートを変更している場合
Ctrl + C で終了します。
"""
import struct
import sys
import time

import serial


def parse(pkt, s):
    t, d = pkt[1], pkt[2:10]
    if t == 0x51:
        ax, ay, az, temp = struct.unpack("<4h", d)
        s["acc"] = (ax / 32768 * 16, ay / 32768 * 16, az / 32768 * 16)
        s["temp"] = temp / 100
    elif t == 0x52:
        wx, wy, wz, _ = struct.unpack("<4h", d)
        s["gyro"] = (wx / 32768 * 2000, wy / 32768 * 2000, wz / 32768 * 2000)
    elif t == 0x53:
        roll, pitch, yaw, _ = struct.unpack("<4h", d)
        s["angle"] = (roll / 32768 * 180, pitch / 32768 * 180, yaw / 32768 * 180)
    elif t == 0x57:
        s["lonlat"] = tuple(to_degrees(v) for v in struct.unpack("<2i", d))
    elif t == 0x58:
        height, heading, speed = struct.unpack("<hhi", d)
        s["gps"] = (height / 10, heading / 100, speed / 1000)
    elif t == 0x5A:
        sats, pdop, hdop, vdop = struct.unpack("<4h", d)
        s["sats"] = (sats, pdop / 100, hdop / 100, vdop / 100)
    s["types"].add(t)


def to_degrees(raw):
    # dddmm.mmmmm 形式（度 = raw/1e7, 分 = (raw % 1e7)/1e5）を 10進の度に変換
    sign = -1 if raw < 0 else 1
    raw = abs(raw)
    return sign * (raw // 10_000_000 + (raw % 10_000_000) / 100_000 / 60)


def extract(buf, s):
    while len(buf) >= 11:
        if buf[0] != 0x55:
            del buf[0]
            continue
        pkt = bytes(buf[:11])
        if sum(pkt[:10]) & 0xFF != pkt[10]:
            s["bad"] += 1
            del buf[0]
            continue
        del buf[:11]
        s["ok"] += 1
        parse(pkt, s)


def fmt(v, digits=2):
    return " ".join(f"{x:+8.{digits}f}" for x in v) if v else "   (未受信)"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"{port} @ {baud}bps を開きました。Ctrl + C で終了します。")
    s = {"ok": 0, "bad": 0, "types": set()}
    buf = bytearray()
    last = time.time()
    while True:
        buf += ser.read(256)
        extract(buf, s)
        if time.time() - last >= 0.5:
            last = time.time()
            types = " ".join(f"{t:02X}" for t in sorted(s["types"])) or "なし"
            print("-" * 60)
            print(f"受信パケット: 正常 {s['ok']} / チェックサム異常 {s['bad']}   受信した種類: {types}")
            print(f"加速度 [g]    X Y Z : {fmt(s.get('acc'), 3)}   温度 {s.get('temp', float('nan')):.1f} ℃")
            print(f"角速度 [°/s]  X Y Z : {fmt(s.get('gyro'))}")
            print(f"角度 [°] ロール ピッチ ヨー : {fmt(s.get('angle'))}")
            print(f"経度 緯度 [°]       : {fmt(s.get('lonlat'), 6)}")
            print(f"GPS 高度[m] 方位[°] 速度 : {fmt(s.get('gps'))}")
            sats = s.get("sats")
            print(f"衛星数 {sats[0]}  PDOP {sats[1]:.2f} HDOP {sats[2]:.2f} VDOP {sats[3]:.2f}" if sats else "衛星数: (未受信)")
            if s["ok"] == 0:
                print("※ 正常なパケットが届いていません。ポート名・ボーレート・配線を確認してください。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
