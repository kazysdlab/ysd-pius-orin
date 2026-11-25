"""Waypoint モデル（src/pius_control/pius_control/waypoint_model.py）の学習スクリプト

記録ファイル（HDF5 v5: 320×128 の画像 + IMU の全サンプル + 車輪パルス）から、各フレームの正解 waypoint
（その後に後軸中心が通った位置を 1 m 刻みで 5 点、waypoint_labels.py）を作って学習します。

- 学習用と検証用は **記録ファイル（セッション）単位** で分けます（隣のフレームがほぼ同じ画像なので、
  フレーム単位で分けると検証の成績が実力より良く見えるため）。--val で検証に使うファイルを指定できます。
- 成績は検証データでの平均誤差 ADE（全点の平均）と FDE（最も遠い点）を m 単位で表示します。
- 最も ADE が小さかったエポックのモデルを models/<名前>.pth に保存します（推論ノードがそのまま読める形式）。

使い方（コンテナ内で実行。走行システムを止めてから）:
    cd /home/orin/dev/ws/colcon_ws
    python3 train_waypoint.py                                    # data/ のすべての v5 ファイル
    python3 train_waypoint.py --val pius_drive_20261010_*.h5 --epochs 40 --name waypoint_v1
"""
import argparse
import fnmatch
import glob
import json
import os
import random
import sys
import time

import h5py
import numpy as np
import torch
import torchvision.transforms.functional as TF
from torch.utils.data import DataLoader, Dataset

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'pius_control', 'pius_control'))
from calibration import load_config, unverified, warn_lines  # noqa: E402
from waypoint_labels import REAR_TRACK_M, imu_mounting, load_h5, make_labels, reconstruct  # noqa: E402
from waypoint_model import DEFAULT_CONFIG, WaypointModel, save_checkpoint, to_tensor  # noqa: E402


# ---------------------------------------------------------------------------------------------------------
# Data
# ---------------------------------------------------------------------------------------------------------
def build_index(paths, cfg, args):
    """For each file: label every frame. Returns a list of (path, frame_idx, waypoints, valid, controls)."""
    samples, report = [], []
    H, W = cfg["image_size"]
    for path in paths:
        name = os.path.basename(path)
        try:
            with h5py.File(path, "r") as f:
                shape = f["image"].shape
                version = int(f.attrs.get("format_version", 0))
                # NaN when no controller command was received (format v6: pedal driving); masked in the loss
                steering = f["steering"][:] if "steering" in f else np.full(shape[0], np.nan)
                throttle = f["throttle"][:] if "throttle" in f else np.full(shape[0], np.nan)
            if tuple(shape[1:3]) != (H, W):
                report.append(f"  {name}: 画像サイズ {shape[2]}×{shape[1]} がモデル（{W}×{H}）と違うため除外")
                continue
            data = load_h5(path)
        except Exception as e:
            report.append(f"  {name}: 読み込めないため除外（{e}）")
            continue
        C = args.circumference or data["circumference"]
        if not C:
            report.append(f"  {name}: タイヤ外周が記録されていないため除外（--circumference で指定できます）")
            continue
        with h5py.File(path, "r") as f:
            provisional = str(f.attrs.get("unverified_params", ""))
        if "wheel.circumference_m" in provisional and not args.circumference:
            report.append(f"  {name}: ⚠ 記録時のタイヤ外周 {C} m は仮の値です（実測値を --circumference で指定すると正解が正確になります）")
        ppr = args.pulses_per_rev or data["pulses_per_rev"]
        axis, sign, src = imu_mounting(data, args.cfg_all, args.forward_axis, args.yaw_sign)
        traj = reconstruct(data["imu_t"], data["acc"], data["gyro"], data["pulse_t"], C, track=args.track,
                           forward_axis=axis, yaw_sign=sign, pulses_per_rev=ppr)
        wps, ok = make_labels(traj, data["frame_t"], cfg["n_points"], cfg["spacing"], max_time=args.max_time)
        keep = ok.sum(axis=1) >= args.min_points
        for i in np.nonzero(keep)[0]:
            samples.append((path, int(i), wps[i], ok[i], np.array([steering[i], throttle[i]], dtype=np.float32)))
        report.append(f"  {name}: v{version}, {shape[0]} フレーム中 {int(keep.sum())} フレームに正解あり"
                      f"（外周 {C} m、パルス採用 {len(traj['accepted_pulses'])}/{traj['n_pulses']}、"
                      f"IMU 前方向 ({axis[0]:+.3f}, {axis[1]:+.3f}) [{src['axis']}]・ヨーレートの符号 {sign:+.0f} [{src['yaw_sign']}]）")
    return samples, report


class WaypointDataset(Dataset):
    def __init__(self, samples, augment=False, hflip=0.0, level="basic"):
        self.samples = samples
        self.augment = augment
        self.hflip = hflip
        self.level = level          # "basic" (photometric) or "strong" (+ colour range, blur, noise, roll, pitch)
        self.files = {}

    def __len__(self):
        return len(self.samples)

    def _image(self, path, idx):
        f = self.files.get(path)
        if f is None:
            f = self.files[path] = h5py.File(path, "r")
        return f["image"][idx]

    def __getitem__(self, i):
        path, idx, wps, ok, ctrl = self.samples[i]
        img = to_tensor(self._image(path, idx))[0]
        wps, ctrl = wps.copy(), ctrl.copy()
        if self.augment:
            img, wps, ctrl = self._augment(img, wps, ctrl)
        # [steering, throttle, has steering, has throttle]: frames without a controller command have NaN controls
        known = np.isfinite(ctrl)
        ctrl = np.concatenate([np.where(known, ctrl, 0.0), known.astype(np.float32)]).astype(np.float32)
        return img, torch.from_numpy(wps), torch.from_numpy(ok.astype(np.float32)), torch.from_numpy(ctrl)

    def _augment(self, img, wps, ctrl):
        # Mirror: the path mirrors too (only valid if the camera is on the centre line, so off by default)
        if random.random() < self.hflip:
            img = img.flip(2)
            wps[:, 1] = -wps[:, 1]
            ctrl[0] = -ctrl[0]
        strong = self.level == "strong"
        # Small camera roll / pitch (body roll, braking dive, mounting tolerance). The path in the rear-axle frame
        # hardly changes for these small angles, so the labels are kept. Rotation is about the optical centre
        # (row 38 of the crop for the 1280x720 camera cropped to rows 208..720)
        if strong and random.random() < 0.5:
            _, h, w = img.shape
            fill = [float(c) for c in img.mean(dim=(1, 2))]
            img = TF.affine(img, angle=random.uniform(-3.0, 3.0), translate=[0, random.randint(-2, 2)], scale=1.0,
                            shear=[0.0], center=[w / 2, 38], fill=fill,
                            interpolation=TF.InterpolationMode.BILINEAR)
        # Brightness / contrast / colour balance (sunlight, shade, time of day; the purple tint varies with light)
        if random.random() < 0.8:
            img = img * random.uniform(0.6, 1.4)
            mean = img.mean()
            img = (img - mean) * random.uniform(0.7, 1.3) + mean
            img = img * torch.empty(3, 1, 1).uniform_(*((0.8, 1.2) if strong else (0.9, 1.1)))
        if strong and random.random() < 0.5:
            img = img.clamp(0, 1) ** random.uniform(0.8, 1.25)          # gamma
        # Blur (vibration, focus) and sensor noise (dark scenes)
        if strong and random.random() < 0.2:
            k = random.choice([3, 5])
            img = TF.gaussian_blur(img, kernel_size=[k, k], sigma=random.uniform(0.3, 1.2))
        if strong and random.random() < 0.3:
            img = img + torch.randn_like(img) * random.uniform(0.005, 0.03)
        # Shadow: darken a random slanted band
        if random.random() < 0.3:
            _, h, w = img.shape
            xs = torch.arange(w).view(1, w).float()
            ys = torch.arange(h).view(h, 1).float()
            x0, slope, width = random.uniform(0, w), random.uniform(-1.5, 1.5), random.uniform(0.1, 0.4) * w
            band = ((xs - x0 - slope * ys).abs() < width / 2).float()
            img = img * (1 - band * random.uniform(0.3, 0.6))
        # Random erasing (occlusion)
        if random.random() < 0.3:
            _, h, w = img.shape
            eh, ew = int(h * random.uniform(0.1, 0.3)), int(w * random.uniform(0.1, 0.3))
            y0, x0 = random.randint(0, h - eh), random.randint(0, w - ew)
            img[:, y0:y0 + eh, x0:x0 + ew] = torch.rand(3, 1, 1)
        return img.clamp_(0, 1), wps, ctrl


# ---------------------------------------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------------------------------------
def masked_l1(pred, target, mask):
    err = (pred - target).abs().sum(-1) * mask
    return err.sum() / mask.sum().clamp(min=1)


def evaluate(model, loader, device):
    model.eval()
    sums = {"ade": 0.0, "fde": 0.0, "lat3": 0.0}
    counts = {"ade": 0, "fde": 0, "lat3": 0}
    with torch.no_grad():
        for img, wps, ok, _ in loader:
            img, wps, ok = img.to(device), wps.to(device), ok.to(device)
            pred, _ = model(img)
            dist = (pred - wps).norm(dim=-1)
            sums["ade"] += float((dist * ok).sum())
            counts["ade"] += int(ok.sum())
            last = ok[:, -1] > 0
            sums["fde"] += float(dist[:, -1][last].sum())
            counts["fde"] += int(last.sum())
            k = min(2, wps.shape[1] - 1)          # the 3 m point: lateral error matters most for steering
            m = ok[:, k] > 0
            sums["lat3"] += float((pred[:, k, 1] - wps[:, k, 1]).abs()[m].sum())
            counts["lat3"] += int(m.sum())
    return {k: sums[k] / max(counts[k], 1) for k in sums}


def split_files(paths, args):
    if args.val:
        val = [p for p in paths if any(fnmatch.fnmatch(os.path.basename(p), pat) for pat in args.val)]
    else:
        # Default: the most recent sessions (by file name = recording time) for validation
        n_val = max(1, round(len(paths) * args.val_ratio)) if len(paths) > 1 else 0
        val = sorted(paths)[-n_val:] if n_val else []
    train = [p for p in paths if p not in val]
    return train, val


def main():
    ap = argparse.ArgumentParser(description="waypoint モデルの学習")
    ap.add_argument("--data_dir", default="/home/orin/dev/ws/colcon_ws/data")
    ap.add_argument("--model_dir", default="/home/orin/dev/ws/colcon_ws/models")
    ap.add_argument("--name", default="waypoint", help="保存するモデルの名前（models/<名前>.pth）")
    ap.add_argument("--files", nargs="*", default=None, help="使う記録ファイル（既定: data_dir の *.h5 すべて）")
    ap.add_argument("--val", nargs="*", default=None, help="検証に使うファイル名のパターン（既定: 新しい順に val_ratio 分）")
    ap.add_argument("--val_ratio", type=float, default=0.2)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch_size", type=int, default=32)
    ap.add_argument("--lr", type=float, default=3e-4, help="ヘッドの学習率（バックボーンはその 1/3）")
    ap.add_argument("--weight_decay", type=float, default=1e-4)
    ap.add_argument("--control_weight", type=float, default=0.5, help="補助の制御分岐の損失の重み（0 で制御分岐なし）")
    ap.add_argument("--n_points", type=int, default=DEFAULT_CONFIG["n_points"])
    ap.add_argument("--spacing", type=float, default=DEFAULT_CONFIG["spacing"])
    ap.add_argument("--min_points", type=int, default=3, help="正解の点がこの数以上あるフレームだけ使う")
    ap.add_argument("--max_time", type=float, default=30.0, help="この秒数以内に届かない点は正解に使わない")
    ap.add_argument("--circumference", type=float, default=None, help="タイヤ外周 [m]（既定: 記録ファイルの値）")
    ap.add_argument("--pulses_per_rev", type=int, default=None,
                    help="1 回転あたりのパルス数（車輪の磁石の数。既定: 記録ファイルの値。記録時の設定が誤っていたときだけ指定）")
    ap.add_argument("--track", type=float, default=None, help="後輪の輪距 [m]（既定: config.yaml の vehicle.rear_track_m）")
    ap.add_argument("--yaw_sign", type=float, default=None,
                    help="IMU のヨーレートの符号（既定: 記録時に実測済みならその値、仮の値だったら config.yaml の vehicle.yaw_sign）")
    ap.add_argument("--forward_axis", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="IMU の前方向（既定: 記録時に実測済みならその値、仮の値だったら config.yaml の wheel.imu_forward_axis）")
    ap.add_argument("--hflip", type=float, default=0.0, help="左右反転の確率（カメラが車体中心にある場合のみ）")
    ap.add_argument("--aug", choices=["basic", "strong"], default="basic",
                    help="オーグメンテーション: basic = 明るさ・コントラスト・色味 ±10%・影・隠す、"
                         "strong = さらに色味 ±20%・ガンマ・ぼかし・ノイズ・小さな傾き（±3°）と上下のずれ（±2 画素）")
    ap.add_argument("--no_pretrained", action="store_true", help="ImageNet の事前学習重みを使わない（動作確認用）")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    cfg_all = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml"))
    veh = cfg_all.get("vehicle", {})
    args.track = args.track or float(veh.get("rear_track_m", REAR_TRACK_M))
    args.cfg_all = cfg_all
    label_items = [i for i in unverified(cfg_all) if i[0] in ("vehicle.yaw_sign", "vehicle.rear_track_m", "wheel.imu_forward_axis")
                   or (i[0] == "wheel.circumference_m" and not args.circumference)]
    if label_items:
        print("\n".join(warn_lines(label_items, "正解データの作成に使う値のうち、仮の値のものがあります")))
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    cfg = dict(DEFAULT_CONFIG, n_points=args.n_points, spacing=args.spacing, control_branch=args.control_weight > 0)

    paths = sorted(args.files or glob.glob(os.path.join(args.data_dir, "*.h5")))
    train_paths, val_paths = split_files(paths, args)
    print(f"使用デバイス: {device}\n学習用ファイル {len(train_paths)} 本、検証用ファイル {len(val_paths)} 本")
    train_samples, rep_t = build_index(train_paths, cfg, args)
    val_samples, rep_v = build_index(val_paths, cfg, args)
    print("[学習用]\n" + "\n".join(rep_t) + "\n[検証用]\n" + "\n".join(rep_v or ["  （なし）"]))
    if not train_samples:
        sys.exit("学習に使えるフレームがありません（v5 形式で、IMU と車速センサーが記録されたファイルが必要です）")
    if not val_samples:
        print("⚠ 検証用のデータがありません。成績は学習データでの値になり、実力より良く見えます。")

    train_loader = DataLoader(WaypointDataset(train_samples, augment=True, hflip=args.hflip, level=args.aug),
                              batch_size=args.batch_size,
                              shuffle=True, drop_last=len(train_samples) > args.batch_size, num_workers=args.workers)
    val_loader = DataLoader(WaypointDataset(val_samples or train_samples), batch_size=args.batch_size,
                            num_workers=args.workers)

    os.makedirs(args.model_dir, exist_ok=True)
    os.environ.setdefault("TORCH_HOME", os.path.join(args.model_dir, "torch_cache"))
    model = WaypointModel(cfg, pretrained=not args.no_pretrained).to(device)
    backbone = list(model.backbone.parameters())
    head = [p for n, p in model.named_parameters() if not n.startswith("backbone.")]
    opt = torch.optim.AdamW([{"params": backbone, "lr": args.lr / 3}, {"params": head, "lr": args.lr}],
                            weight_decay=args.weight_decay)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    scaler = torch.amp.GradScaler(enabled=device.type == "cuda")

    out_path = os.path.join(args.model_dir, f"{args.name}.pth")
    best, history = None, []
    for epoch in range(args.epochs):
        model.train()
        t0, total, n = time.time(), 0.0, 0
        for img, wps, ok, ctrl in train_loader:
            img, wps, ok, ctrl = img.to(device), wps.to(device), ok.to(device), ctrl.to(device)
            with torch.autocast(device.type, enabled=device.type == "cuda"):
                pred, pctrl = model(img)
            loss = masked_l1(pred.float(), wps, ok)
            if pctrl is not None:
                known = ctrl[:, 2:]
                if known.sum() > 0:
                    err = (pctrl.float() - ctrl[:, :2]).abs() * known
                    loss = loss + args.control_weight * err.sum() / known.sum()
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            total += float(loss) * img.shape[0]
            n += img.shape[0]
        sched.step()
        m = evaluate(model, val_loader, device)
        history.append({"epoch": epoch + 1, "train_loss": total / max(n, 1), **m})
        mark = ""
        if best is None or m["ade"] < best["ade"]:
            best = dict(m, epoch=epoch + 1)
            save_checkpoint(out_path, model, extra={
                "metrics": best, "augmentation": args.aug, "train_files": [os.path.basename(p) for p in train_paths],
                "val_files": [os.path.basename(p) for p in val_paths],
                "label": {"circumference": args.circumference, "pulses_per_rev": args.pulses_per_rev, "track": args.track,
                          "yaw_sign": args.yaw_sign, "forward_axis": args.forward_axis,
                          "max_time": args.max_time, "unverified": [i[0] for i in label_items]},
                "created": time.strftime("%Y-%m-%d %H:%M:%S")})
            mark = "  ← 保存"
        print(f"エポック {epoch + 1:3d}/{args.epochs}  学習損失 {total / max(n, 1):.4f}  検証 ADE {m['ade']:.3f} m"
              f"  FDE {m['fde']:.3f} m  3 m 先の横ずれ {m['lat3']:.3f} m  ({time.time() - t0:.0f} 秒){mark}", flush=True)

    with open(os.path.splitext(out_path)[0] + "_history.json", "w") as f:
        json.dump(history, f, indent=1)
    print(f"\n最良: エポック {best['epoch']}  ADE {best['ade']:.3f} m  FDE {best['fde']:.3f} m"
          f"  3 m 先の横ずれ {best['lat3']:.3f} m\nモデル: {out_path}")


if __name__ == "__main__":
    main()
