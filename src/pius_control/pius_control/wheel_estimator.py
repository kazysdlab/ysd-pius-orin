"""Wheel speed estimation from a few-pulses-per-revolution sensor, assisted by the IMU (no ROS dependency).

pulses_per_rev magnets pass the sensor per wheel revolution; every distance below ("one revolution") means
the distance between two pulses, pulse_distance = circumference / pulses_per_rev (magnets equally spaced).

Used by wheel_speed_node, and by tools/speed_replay.py to replay recorded logs (tune it with real logs).

- Standstill: the WitMotion IMU outputs exactly zero angular velocity at rest (its own static detection).
  Standing still = gyro exactly zero AND no forward acceleration, both for still_time. (During a slow start
  the gyro can stay zero for ~2 s, so the gyro alone is not enough.)
- Pulse validation: a pulse is rejected when it is physically impossible:
    too_fast    faster than max_speed since the last valid pulse
    not_moving  the IMU says the car is standing still / has not started moving
    too_short   first pulse after a start, but the IMU distance since the last valid pulse is much less than
                one revolution
    too_sudden  the speed jump it implies is not backed by the IMU acceleration
  A pulse that continues a steady rhythm of pulses is kept even if the IMU disagrees (the IMU is weak at a
  very slow crawl), also when the IMU says standing still: a crawl below ~1.5 km/h is invisible to the IMU
  (zero gyro, acceleration below its noise), but regular pulses a few seconds apart can only come from the
  wheel turning. The speed then follows the pulses (crawl).
  Spurious pulses come from electrical noise when the motor current changes (start, pedal taps, stopping)
  and from the reed switch chattering on slow passes.
- Speed: the direction is fixed when the car starts moving, then the IMU forward acceleration is integrated
  as a non-negative speed. Every valid pulse after continuous motion corrects the speed so that the distance
  between pulses equals one circumference. The speed is 0 while standing still. Without the IMU it falls back to the pulses only.
- Stopping: the accelerometer cannot tell acceleration from tilt. After braking the body pitches and settles
  slowly, so the IMU shows a small false acceleration (~0.1 m/s^2) for seconds, which would make the speed
  linger or even rise after the car has stopped. So the shown speed (speed_imu)
    * below settle_speed, rises only with the part of the acceleration above tilt_acc
    * after more than overdue_ratio revolutions without a pulse (impossible: another pulse would have come)
      cannot rise and fades to 0 (time constant decay_time)
  The pulse validation keeps using the speed without these limits (speed_chk): the limits make the IMU
  distance smaller, which would wrongly reject the first pulse after a stop as too_short.
Everything is causal (uses only past data), so the speed can also be used as a model input.
"""
import math
from collections import deque


class WheelSpeedEstimator:
    def __init__(self, circumference, max_speed=8.0 / 3.6, forward_axis=(-0.338, 0.941),
                 stop_timeout=3.0, min_distance_ratio=0.6, still_time=1.0, still_acc=0.12,
                 start_speed=0.05, jump_margin=0.4, imu_timeout=0.5, decay_time=0.3,
                 overdue_ratio=1.0, crawl_max_interval=8.0, settle_speed=0.3, tilt_acc=0.15, stop_fade_time=2.0,
                 smooth_time=0.4, pulses_per_rev=1):
        self.circumference = float(circumference)      # wheel circumference [m]
        self.pulses_per_rev = max(1, int(pulses_per_rev))
        self.pulse_distance = self.circumference / self.pulses_per_rev   # distance between two pulses [m]
        self.max_speed = float(max_speed)              # [m/s] pulses implying more than this are rejected
        norm = math.hypot(*forward_axis) or 1.0
        self.fwd = (forward_axis[0] / norm, forward_axis[1] / norm)   # car forward direction in IMU x/y
        self.stop_timeout = float(stop_timeout)        # pulse-only fallback: no pulse for this long = stopped
        self.min_distance_ratio = float(min_distance_ratio)
        self.still_time = float(still_time)            # [s] gyro zero and no acceleration this long = still
        self.still_acc = float(still_acc)              # [m/s^2] "no acceleration" threshold
        self.start_speed = float(start_speed)          # [m/s] integrated speed that fixes the direction
        self.jump_margin = float(jump_margin)          # [m/s] allowed speed jump beyond what the IMU explains
        self.imu_timeout = float(imu_timeout)
        self.decay_time = float(decay_time)            # [s] time constant of the speed decay when stopping
        self.overdue_ratio = float(overdue_ratio)      # revolutions without a pulse that are impossible
        self.crawl_max_interval = float(crawl_max_interval)   # [s] slowest regular pulses taken as a crawl
        self.settle_speed = float(settle_speed)        # [m/s] below this, small accelerations are taken as tilt
        self.tilt_acc = float(tilt_acc)                # [m/s^2] false acceleration from the body settling
        self.stop_fade_time = float(stop_fade_time)    # [s] the shown speed fades with this while stopping
        self.smooth_time = float(smooth_time)          # [s] corrections (pulse, stop) reach the shown speed so

        self.speed_imu = 0.0        # non-negative speed from the IMU, as shown [m/s]
        self.speed_chk = 0.0        # the same without the stopping limits: for the distances that validate pulses
        self.offset = 0.0           # shown speed = speed_imu + offset; a correction step fades out (smooth_time)
        self.was_fast = False       # reached settle_speed since the start: slowing below it again = stopping
        self.stopping = False       # slowing down to a stop after driving (until the next valid pulse)
        self.dir = 0                # +1 forward / -1 reversing / 0 not decided (standing or just starting)
        self.v_start = 0.0          # signed speed integrated while the direction is not decided yet
        self.still = False
        self.gyro_zero_since = None
        self.calm_since = None      # since when the forward acceleration has stayed small
        self.last_imu = None
        self.dv = 0.0               # running integral of the acceleration along the direction of travel
        self.dv_hist = deque()      # (t, dv) for the last few seconds
        self.dist = 0.0             # IMU distance since the last valid pulse [m]
        self.move_time = 0.0        # time not standing still since the last valid pulse [s]
        self.stopped = False        # stood still since the last valid pulse (speed error already reset)
        self.onset_v = 0.0          # same-sign gentle acceleration integrated while judged still
        self.raw = []               # recent pulse times that were not too_fast (for the rhythm check)
        self.raw_dist = 0.0         # IMU distance / moving time since the previous such pulse
        self.raw_move_time = 0.0
        self.raw_stopped = False
        self.ref_interval = None    # last valid interval during continuous motion (None after a stop)
        self.crawl_speed = 0.0      # speed from regular pulses while the IMU sees nothing [m/s]
        self.crawl_until = None     # the crawl speed is shown until this time (next pulse expected)

        self.count = 0              # valid pulses
        self.rejected = 0
        self.last_pulse = None
        self.interval = None        # last valid interval for the pulse-only fallback (None after a stop)
        self.raw_interval = None

    # ---- inputs -------------------------------------------------------------------------------------
    def on_imu(self, t, acc, gyro):
        if self.last_imu is None or t <= self.last_imu or t - self.last_imu > self.imu_timeout:
            self.last_imu = t       # first sample, or the IMU was gone: start over
            self.gyro_zero_since = self.calm_since = None
            return
        dt = t - self.last_imu
        self.last_imu = t
        # No offset is subtracted: the sensor outputs zero horizontal acceleration at rest, and the slow
        # speed losses seen while cruising are real (they match the pulse intervals)
        a = acc[0] * self.fwd[0] + acc[1] * self.fwd[1]

        if gyro[0] == 0.0 and gyro[1] == 0.0 and gyro[2] == 0.0:
            self.gyro_zero_since = self.gyro_zero_since if self.gyro_zero_since is not None else t
        else:
            self.gyro_zero_since = None
        if abs(a) < self.still_acc:
            self.calm_since = self.calm_since if self.calm_since is not None else t
        else:
            self.calm_since = None
        self.still = (self.gyro_zero_since is not None and t - self.gyro_zero_since >= self.still_time
                      and self.calm_since is not None and t - self.calm_since >= self.still_time)
        decay = math.exp(-dt / self.decay_time)

        if self.still:
            # A very gentle start stays below still_acc and looks still at first. Keep integrating a run of
            # same-sign acceleration so that it is not lost when the start is finally detected.
            if abs(a) > 0.02 and (self.onset_v == 0.0 or (a > 0) == (self.onset_v > 0)):
                self.onset_v += a * dt
            else:
                self.onset_v = 0.0
            self.dir = 0
            self.v_start = 0.0
            self.stopped = self.raw_stopped = True
            self.dv_hist.clear()
            # Crawling below what the IMU can see: the regular pulses tell the speed
            new = self.crawl_speed if self.crawl_until is not None and t <= self.crawl_until else 0.0
            self._jump(new)
            self.speed_chk = 0.0
            self.was_fast = self.stopping = False
            self._fade(dt)
            return
        if self.dir == 0 and self.v_start == 0.0 and self.onset_v != 0.0:
            self.v_start = self.onset_v
        self.onset_v = 0.0
        self.move_time += dt
        self.raw_move_time += dt
        overdue = (not self.stopped and self.last_pulse is not None and self.pulse_distance > 0
                   and self.dist > self.overdue_ratio * self.pulse_distance)
        if self.dir == 0:
            # Starting: the sign of the first motion decides forward / reverse until the next standstill.
            # (The gyro often stays zero for a while during a real start, so it is not used here.)
            self.v_start += a * dt
            self.speed_imu = self.speed_chk = abs(self.v_start)
            if self.speed_imu > self.start_speed:
                self.dir = 1 if self.v_start > 0 else -1
        else:
            # The car cannot change direction without stopping, so the speed never goes below 0
            self.dv += self.dir * a * dt
            self.dv_hist.append((t, self.dv))
            while self.dv_hist and self.dv_hist[0][0] < t - 10.0:
                self.dv_hist.popleft()
            self.speed_chk = max(0.0, self.speed_chk + self.dir * a * dt)
            if not self.stopped and self.last_pulse is not None and self.dist > 1.1 * self.pulse_distance > 0:
                self.speed_chk = min(self.speed_chk, self.pulse_distance / max(t - self.last_pulse, 1e-3))
            if self.speed_imu >= self.settle_speed:
                self.was_fast = True
            elif self.was_fast:
                self.stopping = True
            if overdue:
                # No pulse for longer than possible: the car has stopped; the shown speed fades out
                self.speed_imu *= decay
                if self.speed_imu < 0.05:
                    self._jump(0.0)
            elif self.stopping:
                # Slowing down to a stop after driving: the body pitches and settles. A small acceleration is
                # the tilt, not speed: only what exceeds tilt_acc can raise the speed (a real restart is stronger,
                # and the next pulse corrects the speed anyway); otherwise the speed fades like a rolling stop
                acc = self.dir * a
                self.speed_imu = max(0.0, self.speed_imu + (min(0.0, acc) + max(0.0, acc - self.tilt_acc)) * dt)
                self.speed_imu *= math.exp(-dt / self.stop_fade_time)
            else:
                self.speed_imu = max(0.0, self.speed_imu + self.dir * a * dt)
            self.speed_imu = min(self.speed_imu, self.speed_chk)
        self._fade(dt)
        # The distances that validate pulses use the speed without the stopping limits (the limits make it
        # smaller, which would reject the first pulse after a stop as too_short)
        self.dist += self.speed_chk * dt
        self.raw_dist += self.speed_chk * dt

    def on_pulse(self, t):
        """Returns (accepted, reason, interval)."""
        C = self.pulse_distance
        interval = None if self.last_pulse is None else t - self.last_pulse
        imu = self.imu_ok(t)
        min_interval = C / self.max_speed if C > 0 else 0.0
        reason = None
        if interval is not None and interval < min_interval:
            reason = "too_fast"
        else:
            regular = self._regular(t, min_interval)
            crawl = self._crawl(t, min_interval)
            self._note_raw(t)
            if imu and (self.still or self.dir == 0) and not crawl:
                reason = "not_moving"
            elif imu and C > 0 and self.ref_interval is None and self.last_pulse is not None \
                    and self.dist < self.min_distance_ratio * C and not regular and not crawl:
                reason = "too_short"
            elif imu and C > 0 and self.ref_interval is not None and not self.stopped and not regular:
                # Average speed over the previous and this interval, centred half an interval back each
                jump = C / interval - C / self.ref_interval
                if jump > self.jump_margin + max(0.0, self._dv_between(t - (self.ref_interval + interval) / 2, t)):
                    reason = "too_sudden"
        if reason:
            self.rejected += 1
            if reason != "too_fast":
                self._reset_raw()
            return False, reason, interval

        if crawl and imu and (self.still or self.dir == 0) and interval is not None:
            # A crawl the IMU cannot see: show the pulse speed until the next pulse is overdue
            self.crawl_speed = C / interval
            self.crawl_until = t + 1.5 * interval
        else:
            self.crawl_until = None
        if imu and C > 0 and self.crawl_until is None:
            if regular and not self.raw_stopped and self.raw_move_time > 0.2:
                # Steady rhythm of pulses: they are one revolution apart even if the IMU disagrees
                err = C / self.raw_interval - self.raw_dist / self.raw_move_time
            elif not self.stopped and self.move_time > 0.2 and 0.4 * C < self.dist < 1.6 * C:
                err = (C - self.dist) / self.move_time
            else:
                err = 0.0
            # err is the average speed error over the interval (the error now may be larger, but correcting by
            # the average is safer when the car was speeding up or slowing down)
            self.speed_chk = max(0.0, self.speed_chk + max(-0.5, min(0.5, err)))
            self._jump(self.speed_chk)
        self.stopping = False
        self.count += 1
        self.last_pulse = t
        self.ref_interval = interval if interval is not None and not self.stopped else None
        self.interval = self.ref_interval if self.ref_interval is not None and self.ref_interval <= self.stop_timeout \
            else None
        self.dist = 0.0
        self.move_time = 0.0
        self.stopped = False
        self._reset_raw()
        return True, None, interval

    def _note_raw(self, t):
        """Remember a pulse that was not too_fast, and the interval to the previous one."""
        self.raw_interval = t - self.raw[-1] if self.raw else None
        self.raw = (self.raw + [t])[-3:]

    def _reset_raw(self):
        self.raw_dist = 0.0
        self.raw_move_time = 0.0
        self.raw_stopped = False

    def _regular(self, t, min_interval):
        """True when this pulse continues a steady rhythm of the previous two (no standstill in between)."""
        if len(self.raw) < 2 or self.raw_stopped:
            return False
        cur, prev = t - self.raw[-1], self.raw[-1] - self.raw[-2]
        return cur >= min_interval and prev >= min_interval and 0.7 <= cur / prev <= 1.5

    def _fade(self, dt):
        """Let a correction step fade out of the shown speed; at a standstill end it at a crawl-free 0."""
        self.offset *= math.exp(-dt / self.smooth_time)
        if self.speed_imu == 0.0 and abs(self.offset) < 0.05:
            self.offset = 0.0

    def _jump(self, new):
        """Set speed_imu to new without a step in the shown speed (the difference fades out)."""
        self.offset += self.speed_imu - new
        self.speed_imu = new

    def _crawl(self, t, min_interval):
        """True when this pulse continues a steady slow rhythm (crawl), even across an IMU standstill."""
        if len(self.raw) < 2:
            return False
        cur, prev = t - self.raw[-1], self.raw[-1] - self.raw[-2]
        return (min_interval <= cur <= self.crawl_max_interval and min_interval <= prev <= self.crawl_max_interval
                and 0.6 <= cur / prev <= 1.7)

    def _dv_between(self, t0, t1):
        """Speed change [m/s] along the direction of travel measured by the IMU between t0 and t1."""
        if not self.dv_hist:
            return 0.0
        v0 = next((dv for t, dv in self.dv_hist if t >= t0), self.dv_hist[-1][1])
        return self.dv - v0

    # ---- outputs ------------------------------------------------------------------------------------
    def imu_ok(self, t):
        return self.last_imu is not None and t - self.last_imu <= self.imu_timeout

    def pulse_speed(self, t):
        """Pulse-only speed [m/s] (the previous method): None until two pulses close enough together."""
        if self.pulse_distance <= 0 or self.last_pulse is None:
            return None
        if t - self.last_pulse > self.stop_timeout:
            return 0.0
        if self.interval is None:
            return None
        return self.pulse_distance / max(self.interval, t - self.last_pulse)

    def speed(self, t):
        """Best speed estimate [m/s] (always >= 0) and its source: 'imu' or 'pulse'."""
        if self.imu_ok(t) and self.pulse_distance > 0:
            return max(0.0, self.speed_imu + self.offset), "imu"
        return self.pulse_speed(t), "pulse"

    def direction(self, t):
        """+1 forward, -1 reversing, 0 standing still / unknown (needs the IMU)."""
        return self.dir if self.imu_ok(t) else 0
