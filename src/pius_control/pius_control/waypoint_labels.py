"""Ground-truth path reconstruction and waypoint labels from recorded wheel pulses and IMU (offline, no ROS).

Used by train_waypoint.py (labels) and tools/trajectory_check.py (calibration drives).

The path of the rear-axle centre is dead-reckoned on the IMU time grid (50 Hz):
- distance: the wheel sensor (rear-left wheel, pulses_per_rev equally spaced magnets) gives exactly
  circumference / pulses_per_rev between valid pulses. Inside each interval the distance is spread following the IMU-assisted speed
  (WheelSpeedEstimator), so stops and speed changes land at the right time. The estimator also rejects
  spurious pulses, the same way as the running node.
- rear-left wheel -> rear-axle centre: v_centre = v_left + yaw_rate * track / 2 (the left wheel runs on the
  inside of a left turn).
- heading: integrated IMU yaw rate (gyro z; the IMU z axis points up).
Labels may use future data (they are targets, not inputs). For each camera frame the next n_points
positions at every `spacing` metres of travelled distance are expressed in that frame's rear-axle frame
(x forward, y left). Points are invalid when the path is not reliably known (before the first / after the
last valid pulse, a reversing section, or an interval whose IMU distance contradicts the wheel).
"""
import json
import math

import numpy as np

try:
    from .wheel_estimator import WheelSpeedEstimator
except ImportError:          # used from tools/ and train_waypoint.py via sys.path
    from wheel_estimator import WheelSpeedEstimator

REAR_TRACK_M = 0.92          # PIUS spec: rear wheel track 920 mm


def reconstruct(imu_t, acc, gyro, pulse_t, circumference, track=REAR_TRACK_M, forward_axis=(-0.338, 0.941),
                yaw_sign=1.0, min_ratio=0.3, max_ratio=3.0, pulses_per_rev=1):
    """Dead-reckon the rear-axle centre. All times in seconds (same clock), gyro in rad/s.

    Returns a dict of arrays on the IMU grid: t, x, y, psi, arc (travelled distance), speed (rear-axle
    centre, m/s, signed), valid (path known at this sample), reverse, and the accepted pulse times.
    """
    imu_t = np.asarray(imu_t, dtype=np.float64)
    acc = np.asarray(acc, dtype=np.float64)
    gyro = np.asarray(gyro, dtype=np.float64)
    pulse_t = np.sort(np.asarray(pulse_t, dtype=np.float64))
    n = len(imu_t)
    ppr = max(1, int(pulses_per_rev or 1))
    C = float(circumference) / ppr          # distance between two pulses

    # 1. Replay the estimator (pulse validation + speed shape), exactly like the node
    est = WheelSpeedEstimator(float(circumference), forward_axis=forward_axis, pulses_per_rev=ppr)
    v_est = np.zeros(n)
    direction = np.zeros(n, dtype=np.int8)
    still = np.zeros(n, dtype=bool)
    accepted = []
    j = 0
    for i in range(n):
        while j < len(pulse_t) and pulse_t[j] <= imu_t[i]:
            if est.on_pulse(pulse_t[j])[0]:
                accepted.append(pulse_t[j])
            j += 1
        est.on_imu(imu_t[i], acc[i], gyro[i])
        v_est[i] = est.speed_imu
        direction[i] = est.dir
        still[i] = est.still
    accepted = np.asarray(accepted)

    dt = np.diff(imu_t, prepend=imu_t[0])
    dt[0] = 0.0
    valid = np.zeros(n, dtype=bool)
    ds_left = np.zeros(n)

    # 2. One circumference per pulse interval, spread along the IMU speed shape (while the IMU sees a
    # standstill the speed is 0, except for a crawl that only the regular pulses reveal)
    weight = np.maximum(v_est, 0.0) * dt
    crawl = still & (v_est > 0)
    for k in range(len(accepted) - 1):
        a, b = np.searchsorted(imu_t, accepted[k], 'right'), np.searchsorted(imu_t, accepted[k + 1], 'right')
        if b <= a:
            continue
        w = weight[a:b]
        total = w.sum()
        moving = ~still[a:b]
        if total > 1e-6:
            ds_left[a:b] = C * w / total
        elif moving.any():
            ds_left[a:b] = C * moving / moving.sum()
        else:
            continue
        ratio = total / C
        # An interval whose IMU distance is wildly different from one revolution (missed pulses, IMU lost)
        valid[a:b] = min_ratio <= ratio <= max_ratio or total < 1e-6

    # A crawl has no known direction (forward or reverse): its distance is kept, but no label crosses it
    valid &= ~crawl

    # 3. Direction (the wheel sensor cannot tell), rear-axle centre, heading
    sign = np.where(direction < 0, -1.0, 1.0)
    reverse = (direction < 0) & (v_est > 0.05)
    yaw_rate = yaw_sign * gyro[:, 2]
    ds = sign * ds_left + yaw_rate * (track / 2.0) * dt * (ds_left > 0)
    psi = np.cumsum(yaw_rate * dt)
    psi_mid = psi - 0.5 * yaw_rate * dt
    x = np.cumsum(ds * np.cos(psi_mid))
    y = np.cumsum(ds * np.sin(psi_mid))
    arc = np.cumsum(np.abs(ds))
    with np.errstate(divide="ignore", invalid="ignore"):
        speed = np.where(dt > 0, ds / np.where(dt > 0, dt, 1.0), 0.0)
    return {"t": imu_t, "x": x, "y": y, "psi": psi, "arc": arc, "speed": speed, "valid": valid,
            "reverse": reverse, "accepted_pulses": accepted, "n_pulses": len(pulse_t)}


def make_labels(traj, frame_t, n_points=5, spacing=1.0, max_time=None):
    """Waypoints for each frame: ([N, n_points, 2] float32 metres, [N, n_points] bool valid)."""
    t, arc = traj["t"], traj["arc"]
    frame_t = np.asarray(frame_t, dtype=np.float64)
    N = len(frame_t)
    wps = np.zeros((N, n_points, 2), dtype=np.float32)
    ok = np.zeros((N, n_points), dtype=bool)
    if len(t) < 2:
        return wps, ok

    bad = ~traj["valid"] | traj["reverse"]
    bad_cum = np.concatenate([[0], np.cumsum(bad)])            # bad samples in [i, j) = bad_cum[j] - bad_cum[i]

    i_f = np.clip(np.searchsorted(t, frame_t), 1, len(t) - 1)
    w = (frame_t - t[i_f - 1]) / np.maximum(t[i_f] - t[i_f - 1], 1e-9)
    w = np.clip(w, 0.0, 1.0)

    def at(arr, i, frac):
        return arr[i - 1] + frac * (arr[i] - arr[i - 1])

    xf, yf, pf, sf = (at(traj[k], i_f, w) for k in ("x", "y", "psi", "arc"))
    inside = (frame_t >= t[0]) & (frame_t <= t[-1])
    cos_f, sin_f = np.cos(pf), np.sin(pf)

    for k in range(n_points):
        target = sf + (k + 1) * spacing
        j = np.searchsorted(arc, target, side="left")
        reach = j < len(arc)
        j = np.clip(j, 1, len(arc) - 1)
        span = np.maximum(arc[j] - arc[j - 1], 1e-9)
        frac = np.clip((target - arc[j - 1]) / span, 0.0, 1.0)
        px, py = at(traj["x"], j, frac), at(traj["y"], j, frac)
        dx, dy = px - xf, py - yf
        wps[:, k, 0] = cos_f * dx + sin_f * dy
        wps[:, k, 1] = -sin_f * dx + cos_f * dy
        good = reach & inside & (bad_cum[j + 1] - bad_cum[np.maximum(i_f - 1, 0)] == 0)
        if max_time is not None:
            good &= (t[j] - frame_t) <= max_time
        ok[:, k] = good
    return wps, ok


# ---- loaders --------------------------------------------------------------------------------------------
def load_speed_log(path):
    """tools/speed_live.py log (JSONL) -> dict(imu_t, acc, gyro, pulse_t, circumference, pulses_per_rev)."""
    imu_t, acc, gyro, pulses = [], [], [], []
    circ = None
    ppr = 1
    axis = None
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:          # a line cut off when the logger was stopped
                continue
            if r["k"] == "imu":
                imu_t.append(r["t"])
                acc.append(r["acc"])
                gyro.append(r["gyro"])
            elif r["k"] in ("pulse", "pulse_rejected"):
                pulses.append(r.get("stamp", r["t"]))
            elif r["k"] == "status" and circ is None and r.get("circumference"):
                circ = r["circumference"]
                ppr = int(r.get("pulses_per_rev") or 1)     # logs before 2026-10 have 1 pulse per revolution
                axis = r.get("imu_forward_axis")
    return {"imu_t": np.array(imu_t), "acc": np.array(acc), "gyro": np.array(gyro),
            "pulse_t": np.array(pulses), "circumference": circ, "pulses_per_rev": ppr,
            "forward_axis": tuple(axis) if axis else None, "yaw_sign": None, "unverified": None}


def load_h5(path):
    """Recorded HDF5 (format v4 or later) -> dict(imu_t, acc, gyro, pulse_t, circumference, pulses_per_rev, frame_t)."""
    import h5py
    with h5py.File(path, "r") as f:
        if "imu_hr" not in f or "wheel" not in f:
            raise ValueError("IMU（imu_hr）または車輪パルス（wheel）の記録がありません（v4 以降の形式が必要です）")
        imu = f["imu_hr"]
        return {"imu_t": imu["stamp"][:], "acc": imu["acc"][:], "gyro": imu["gyro"][:],
                "pulse_t": f["wheel"]["pulse_stamp"][:],
                "circumference": float(f.attrs.get("wheel_circumference_m", 0.0)) or None,
                "pulses_per_rev": int(f.attrs.get("wheel_pulses_per_rev", 1)),
                # IMU mounting at recording time (format v6 or later; None before)
                "forward_axis": tuple(float(x) for x in f.attrs["imu_forward_axis"]) if "imu_forward_axis" in f.attrs else None,
                "yaw_sign": float(f.attrs["vehicle_yaw_sign"]) if "vehicle_yaw_sign" in f.attrs else None,
                "unverified": str(f.attrs.get("unverified_params", "")) if "unverified_params" in f.attrs else None,
                "frame_t": f["image_stamp"][:] if "image_stamp" in f else None}


def imu_mounting(data, config, forward_axis=None, yaw_sign=None):
    """IMU forward axis and yaw sign to use for one recording, and where they came from.

    Order: explicit value (command line) > the file's value if it was already measured when recording (the IMU may
    have been moved since) > config.yaml (wheel.imu_forward_axis, vehicle.yaw_sign: the best value known now,
    e.g. measured after recording) > default. Returns (axis, yaw_sign, {"axis": source, "yaw_sign": source}).
    """
    config = config or {}
    unverified = data.get("unverified")
    measured = unverified is not None
    src = {}
    if forward_axis is not None:
        axis, src["axis"] = tuple(forward_axis), "指定"
    elif data.get("forward_axis") and measured and "wheel.imu_forward_axis" not in unverified:
        axis, src["axis"] = data["forward_axis"], "記録時"
    else:
        cfg_axis = (config.get("wheel") or {}).get("imu_forward_axis")
        axis, src["axis"] = (tuple(cfg_axis), "config") if cfg_axis else ((-0.338, 0.941), "既定")
    if yaw_sign is not None:
        sign, src["yaw_sign"] = float(yaw_sign), "指定"
    elif data.get("yaw_sign") is not None and measured and "vehicle.yaw_sign" not in unverified:
        sign, src["yaw_sign"] = data["yaw_sign"], "記録時"
    else:
        sign, src["yaw_sign"] = float((config.get("vehicle") or {}).get("yaw_sign", 1)), "config"
    return axis, sign, src


def summary(traj):
    """Numbers for a calibration drive: distances, net heading change, and the gap between start and end."""
    v = traj["valid"]
    if not v.any():
        return None
    i0, i1 = np.argmax(v), len(v) - 1 - np.argmax(v[::-1])
    gap = math.hypot(traj["x"][i1] - traj["x"][i0], traj["y"][i1] - traj["y"][i0])
    return {
        "duration": float(traj["t"][i1] - traj["t"][i0]),
        "pulses": int(traj["n_pulses"]), "accepted": int(len(traj["accepted_pulses"])),
        "distance": float(traj["arc"][i1] - traj["arc"][i0]),
        "heading_change_deg": float(math.degrees(traj["psi"][i1] - traj["psi"][i0])),
        "start_end_gap": gap,
    }
