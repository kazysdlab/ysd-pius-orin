"""WitMotion WTGAHRS3 の設定ツール（WIT 標準プロトコルのレジスタ読み書き）

    --show           現在の設定と、実際に届いているパケットの頻度を表示する
    --apply          出力を 加速度・角速度・姿勢・GPS位置・GPS速度・衛星情報 に絞り、50 Hz・115200 bps にして保存する
    --factory-reset  工場出荷状態（9600 bps・10 Hz）に戻す

センサーを使うプログラム（ROS 2 の imu_node など）を止めてから実行します。pyserial が必要なため、
Jetson ではコンテナのイメージで実行します:
    docker run --rm -it --device /dev/ttyUSB0 --group-add 20 -v /home/yl00/pius:/ws namazyake/roscar2:cu126 \\
        python3 /ws/tools/imu_config.py --show
"""
import argparse
import struct
import sys
import time

import serial

BAUD_CODES = {4800: 0x01, 9600: 0x02, 19200: 0x03, 38400: 0x04, 57600: 0x05, 115200: 0x06, 230400: 0x07}
RATE_CODES = {0x01: "0.2 Hz", 0x02: "0.5 Hz", 0x03: "1 Hz", 0x04: "2 Hz", 0x05: "5 Hz", 0x06: "10 Hz",
              0x07: "20 Hz", 0x08: "50 Hz", 0x09: "100 Hz", 0x0B: "200 Hz", 0x0C: "単発", 0x0D: "出力なし"}
RSW_BITS = ["時刻(50)", "加速度(51)", "角速度(52)", "姿勢(53)", "地磁気(54)", "ポート(55)", "気圧(56)",
            "GPS位置(57)", "GPS速度(58)", "四元数(59)", "衛星情報(5A)"]

TARGET_RATE = 0x08                                 # 50 Hz
TARGET_BAUD = 115200
TARGET_RSW = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 7) | (1 << 8) | (1 << 10)   # 51 52 53 57 58 5A


def packets(data):
    """Yields (type, payload) for every valid 11-byte packet in data."""
    i = 0
    while i + 11 <= len(data):
        if data[i] == 0x55 and sum(data[i:i + 10]) & 0xFF == data[i + 10]:
            yield data[i + 1], data[i + 2:i + 10]
            i += 11
        else:
            i += 1


def packet_rates(ser, seconds=2.0):
    ser.reset_input_buffer()
    t0 = time.time()
    data = bytearray()
    while time.time() - t0 < seconds:
        data += ser.read(4096)
    counts = {}
    for t, _ in packets(bytes(data)):
        counts[t] = counts.get(t, 0) + 1
    return {t: n / seconds for t, n in sorted(counts.items())}


def detect_baud(port):
    for baud in (9600, 115200, 230400, 57600, 38400, 19200, 4800):
        ser = serial.Serial(port, baud, timeout=0.1)
        rates = packet_rates(ser, 1.2)
        if sum(rates.values()) >= 5:
            return ser, baud
        ser.close()
    sys.exit("センサーからデータが届きません。接続と、他のプログラムがポートを使っていないかを確認してください。")


def send(ser, *cmd):
    ser.write(bytes(cmd))
    ser.flush()
    time.sleep(0.1)


def unlock(ser):
    send(ser, 0xFF, 0xAA, 0x69, 0x88, 0xB5)


def write_reg(ser, addr, value):
    send(ser, 0xFF, 0xAA, addr, value & 0xFF, (value >> 8) & 0xFF)


def save(ser):
    send(ser, 0xFF, 0xAA, 0x00, 0x00, 0x00)


def read_regs(ser, addr, tries=5):
    """Returns the 4 registers starting at addr (the sensor keeps streaming, so pick the 0x5F reply out)."""
    for _ in range(tries):
        ser.reset_input_buffer()
        ser.write(bytes([0xFF, 0xAA, 0x27, addr, 0x00]))
        t0 = time.time()
        data = bytearray()
        while time.time() - t0 < 0.5:
            data += ser.read(4096)
            for t, payload in packets(bytes(data)):
                if t == 0x5F:
                    return struct.unpack("<4h", payload)
    return None


def show(ser, baud):
    print(f"通信速度: {baud} bps")
    regs = read_regs(ser, 0x02)
    if regs:
        rsw, rrate, baud_code, _ = regs
        print(f"出力内容 (RSW 0x02) : 0x{rsw & 0xFFFF:04X} = " + " ".join(n for i, n in enumerate(RSW_BITS) if rsw >> i & 1))
        print(f"出力頻度 (RRATE 0x03): 0x{rrate:02X} = {RATE_CODES.get(rrate, '不明')}")
        print(f"通信速度 (BAUD 0x04) : 0x{baud_code:02X}")
    for addr, name, fmt in ((0x1F, "帯域 (BANDWIDTH 0x1F)", None), (0x24, "姿勢の計算方式 (AXIS6 0x24)", {0: "9軸", 1: "6軸"}),
                            (0x61, "静止判定しきい値 (GYROCALITHR 0x61)", None), (0x63, "自動校正時間 (GYROCALTIME 0x63)", None)):
        r = read_regs(ser, addr)
        if r:
            print(f"{name}: {r[0]}" + (f"（{fmt.get(r[0], '?')}）" if fmt else ""))
    rates = packet_rates(ser)
    print("実際に届いているパケット [回/秒]: " + ", ".join(f"{t:02X}: {r:.0f}" for t, r in rates.items()))


def apply(port, ser, baud):
    print(f"現在 {baud} bps。出力内容と出力頻度（50 Hz）を設定して保存します...")
    unlock(ser)
    write_reg(ser, 0x02, TARGET_RSW)
    write_reg(ser, 0x03, TARGET_RATE)
    save(ser)
    time.sleep(0.5)
    if baud != TARGET_BAUD:
        print(f"通信速度を {TARGET_BAUD} bps に変更します...")
        unlock(ser)
        write_reg(ser, 0x04, BAUD_CODES[TARGET_BAUD])
        ser.close()
        time.sleep(0.5)
        ser = serial.Serial(port, TARGET_BAUD, timeout=0.1)
        if sum(packet_rates(ser, 1.5).values()) < 5:
            ser.close()
            sys.exit("新しい通信速度でデータが届きません。--show で状態を確認するか、--factory-reset で元に戻してください。")
        unlock(ser)
        save(ser)
        time.sleep(0.5)
    print("変更後の状態:")
    show(ser, TARGET_BAUD)
    print("\n※ 保存されたことを確かめるため、USB を抜き差ししてから --show で 115200 bps・50 Hz のままか確認してください。")


def factory_reset(ser):
    unlock(ser)
    write_reg(ser, 0x00, 0x0001)
    time.sleep(1.0)
    print("工場出荷状態に戻しました（9600 bps・10 Hz）。USB を抜き差しして --show で確認してください。")


def main():
    ap = argparse.ArgumentParser(description="WitMotion WTGAHRS3 の設定ツール")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--show", action="store_true")
    g.add_argument("--apply", action="store_true")
    g.add_argument("--factory-reset", action="store_true")
    args = ap.parse_args()

    ser, baud = detect_baud(args.port)
    if args.show:
        show(ser, baud)
    elif args.apply:
        apply(args.port, ser, baud)
    else:
        factory_reset(ser)


if __name__ == "__main__":
    main()
