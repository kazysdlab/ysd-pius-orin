"""Raspberry Pi receiver

Normally runs as the systemd service pius-receiver (tools/pi/pius-receiver.service), started at boot and
independent of any SSH session. For a test it can be run by hand (pigpiod must be running: sudo pigpiod):
    python3 receiver_.py

However it stops (Ctrl + C, kill / SIGTERM, the SSH session that started it closing / SIGHUP, an error), it
outputs the safe state (steering centre, throttle stop, full brake, neutral) and LEAVES those pulses running:
pigpiod keeps generating the last pulses after this program exits, so stopping the PWM (or exiting without
this) would leave the vehicle on its last command or without a valid signal. 2026-11-06: an SSH drop killed
the receiver while driving and pigpiod held the last steering / throttle.
"""

import signal
import struct
import time

import pigpio
import serial

HEADER = b"\xff\xff"
PAYLOAD_SIZE = struct.calcsize("<4h")  # 8 bytes
PACKET_SIZE = len(HEADER) + PAYLOAD_SIZE + 1  # header + payload + checksum = 11 bytes

STEERING_MIN, STEERING_CENTER, STEERING_MAX = 1139, 1500, 1835
THROTTLE_STOP, THROTTLE_MAX = 1000, 2000
# Brake polarity on this vehicle: 1135 = full brake, 1948 = released
BRAKE_FULL, BRAKE_RELEASED = 1135, 1948
STATE_NEUTRAL = 1

RX_TIMEOUT_SEC = 0.5


def log(text):
    """Print, but never fail: after the terminal is gone (SSH dropped) printing raises OSError."""
    try:
        print(text, flush=True)
    except (OSError, ValueError):
        pass


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def extract_packets(buffer: bytearray):
    """Consume complete packets from buffer. Returns (valid_packets, dropped_count)."""
    packets = []
    dropped = 0
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            # Keep a trailing 0xFF: it may be the first half of the next header
            keep = 1 if buffer[-1:] == b"\xff" else 0
            del buffer[: len(buffer) - keep]
            return packets, dropped
        if start > 0:
            del buffer[:start]
        if len(buffer) < PACKET_SIZE:
            return packets, dropped

        payload = bytes(buffer[2 : 2 + PAYLOAD_SIZE])
        checksum = buffer[2 + PAYLOAD_SIZE]
        if (sum(payload) & 0x7F) == checksum:
            packets.append(struct.unpack("<4h", payload))
            del buffer[:PACKET_SIZE]
        else:
            # Misaligned or corrupted: drop one byte and search for the next header
            dropped += 1
            del buffer[:1]


def sanitize(steer_pwm, throttle_pwm, brake_pwm, state_val):
    steer_pwm = clamp(steer_pwm, STEERING_MIN, STEERING_MAX)
    throttle_pwm = clamp(throttle_pwm, THROTTLE_STOP, THROTTLE_MAX)
    brake_pwm = clamp(brake_pwm, BRAKE_FULL, BRAKE_RELEASED)
    if state_val not in (0, 1, 2):
        state_val = STATE_NEUTRAL
    return steer_pwm, throttle_pwm, brake_pwm, state_val


class Receiver:
    def __init__(self):
        self.ser = serial.Serial("/dev/serial0", 115200, timeout=0.05)

        # Initialize to safe state: straight, throttle stopped, full brake, neutral
        self.steering = STEERING_CENTER
        self.throttle = THROTTLE_STOP
        self.brake = BRAKE_FULL
        self.shift_state = "Neutral"

        self.steering_pin = 19
        self.throttle_pin = 12
        self.brake_pin = 13
        self.drive_pin = 16
        self.reverse_pin = 26

        self.pi = pigpio.pi()
        self.pi.set_mode(self.steering_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.throttle_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.brake_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.drive_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.reverse_pin, pigpio.OUTPUT)

        self.pi.set_servo_pulsewidth(self.steering_pin, self.steering)
        self.pi.set_servo_pulsewidth(self.throttle_pin, self.throttle)
        self.pi.set_servo_pulsewidth(self.brake_pin, self.brake)
        self.pi.write(self.drive_pin, 0)
        self.pi.write(self.reverse_pin, 0)

        self.last_rx_time = 0.0
        self.failsafe_active = True
        self.last_shown = None

        time.sleep(2)

    def shift(self, state_val):
        # state_val: 0 = Forward (Drive), 1 = Neutral, 2 = Reverse
        if state_val == 0:
            self.pi.write(self.drive_pin, 1)
            self.pi.write(self.reverse_pin, 0)
            self.shift_state = "Drive"
        elif state_val == 2:
            self.pi.write(self.drive_pin, 0)
            self.pi.write(self.reverse_pin, 1)
            self.shift_state = "Reverse"
        else: # 1 (Neutral) or other unexpected values
            self.pi.write(self.drive_pin, 0)
            self.pi.write(self.reverse_pin, 0)
            self.shift_state = "Neutral"

    def write(self, steer_pwm, throttle_pwm, brake_pwm, state_val):
        self.steering = steer_pwm
        self.throttle = throttle_pwm
        self.brake = brake_pwm

        # Output PWM directly (expects raw microsecond values from Jetson Orin)
        self.pi.set_servo_pulsewidth(self.steering_pin, self.steering)
        self.pi.set_servo_pulsewidth(self.throttle_pin, self.throttle)
        self.pi.set_servo_pulsewidth(self.brake_pin, self.brake)

        self.shift(state_val)

        shown = (self.steering, self.throttle, self.brake, self.shift_state)
        if shown != self.last_shown:          # only changes (30 packets per second otherwise)
            self.last_shown = shown
            log(f"Steering: {self.steering}, Throttle: {self.throttle}, Brake: {self.brake}, Shift: {self.shift_state}")

    def apply_failsafe(self):
        self.write(STEERING_CENTER, THROTTLE_STOP, BRAKE_FULL, STATE_NEUTRAL)

    def run(self):
        """Receive until Ctrl + C (KeyboardInterrupt is not caught by the error handler below)."""
        buffer = bytearray()
        while True:
            try:
                chunk = self.ser.read(64)
                if chunk:
                    buffer.extend(chunk)
                    packets, dropped = extract_packets(buffer)
                    if dropped:
                        log(f"Warning: チェックサム不一致のため {dropped} バイトを破棄しました")
                    if packets:
                        # Only the latest command matters; older ones in the same read are stale
                        self.write(*sanitize(*packets[-1]))
                        self.last_rx_time = time.monotonic()
                        if self.failsafe_active:
                            log("Info: 受信を再開しました")
                            self.failsafe_active = False

                if not self.failsafe_active and time.monotonic() - self.last_rx_time > RX_TIMEOUT_SEC:
                    log(f"Warning: {RX_TIMEOUT_SEC}秒以上受信がないため、停止・フルブレーキにします")
                    self.apply_failsafe()
                    self.failsafe_active = True

            except Exception as e:
                log(f"Error in run loop: {e}")
                time.sleep(0.1)

    def shutdown(self):
        """Safe state, and keep it: pigpiod goes on generating these pulses after this program exits."""
        self.apply_failsafe()
        time.sleep(0.2)
        self.pi.stop()
        self.ser.close()


def _stop(signum, _frame):
    raise KeyboardInterrupt(signal.Signals(signum).name)


if __name__ == "__main__":
    # kill / systemd stop (SIGTERM) and the starting SSH session closing (SIGHUP): same safe stop as Ctrl + C
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGHUP, _stop)
    receiver = Receiver()
    log("受信を開始しました。Ctrl + C で停止します（停止後もステア中央・アクセル停止・フルブレーキを出し続けます）。")
    try:
        receiver.run()
    except KeyboardInterrupt as e:
        log(f"停止します（{str(e) or 'Ctrl + C'}）...")
    finally:
        receiver.shutdown()
        log("停止しました。ステア中央・アクセル停止・フルブレーキの出力を保っています。")
