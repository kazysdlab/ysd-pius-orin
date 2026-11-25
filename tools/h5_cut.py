"""記録ファイル（HDF5）から時間の範囲を切り出して、別のファイルに保存する（元のファイルは変えない）

使い道: 1 回の記録を学習用と検証用に分ける、走行と関係ない部分（車庫入れなど）を除く。

- 画像ごとのデータ（image・steering など）は、画像の時刻が [--from, --to) のものだけを残します。
- 高頻度データ（imu_hr・gps_hr・wheel）は、範囲の前後に余分を残します（正解の進路は、その画像の後に走った軌跡
  から作るため。後ろは --margin 秒、前は 10 秒）。属性はそのまま写し、切り出した範囲を属性 cut_* に残します。
- 時刻は日本時間の「HH:MM:SS」（記録の日付を使う）か、UNIX 秒で指定します。

使い方（コンテナ内、h5py が必要）:
    python3 tools/h5_cut.py data/pius_drive_20261106_060920.h5 data/course1_train.h5 --to 15:24:00
    python3 tools/h5_cut.py data/pius_drive_20261106_060920.h5 data/course1_val.h5 --from 15:24:00 --to 15:27:25
    python3 tools/h5_cut.py data/<ファイル> --list          # 記録の切れ目（止まっていた間）の一覧
"""
import argparse
import datetime
import os
import sys

import h5py
import numpy as np

JST = datetime.timezone(datetime.timedelta(hours=9))
STREAM_STAMPS = {"imu_hr": "stamp", "gps_hr": "stamp", "wheel": "pulse_stamp"}


def to_stamp(text, day):
    """'HH:MM:SS' (JST, on the recording day) or UNIX seconds -> UNIX seconds."""
    if text is None:
        return None
    try:
        return float(text)
    except ValueError:
        t = datetime.datetime.strptime(text, "%H:%M:%S").time()
        return datetime.datetime.combine(day, t, tzinfo=JST).timestamp()


def jst(stamp):
    return datetime.datetime.fromtimestamp(stamp, JST).strftime("%H:%M:%S")


def list_gaps(f, min_gap):
    ft = f["image_stamp"][:]
    print(f"記録 {jst(ft[0])}〜{jst(ft[-1])}（日本時間）、画像 {len(ft)} 枚")
    gaps = np.where(np.diff(ft) > min_gap)[0]
    print(f"{min_gap:.0f} 秒を超える切れ目（止まっていた・後退していた・記録を止めていた間）: {len(gaps)} 個")
    for i in gaps:
        print(f"  {jst(ft[i])} 〜 {jst(ft[i + 1])}（{ft[i + 1] - ft[i]:5.1f} 秒）  その前までの画像 {i + 1} 枚")


def copy_rows(src, dst, name, idx, block=500):
    """Copy rows idx (sorted) of dataset src[name] into a new dataset dst[name], block by block."""
    ds = src[name]
    shape = (len(idx),) + ds.shape[1:]
    out = dst.create_dataset(name, shape=shape, maxshape=(None,) + ds.shape[1:], dtype=ds.dtype,
                             chunks=ds.chunks if ds.chunks else True)
    for k in range(0, len(idx), block):
        part = idx[k:k + block]
        lo, hi = part[0], part[-1] + 1
        out[k:k + len(part)] = ds[lo:hi][part - lo]


def main():
    ap = argparse.ArgumentParser(description="記録ファイルから時間の範囲を切り出す")
    ap.add_argument("src")
    ap.add_argument("dst", nargs="?")
    ap.add_argument("--from", dest="t_from", default=None, help="開始（日本時間 HH:MM:SS か UNIX 秒。既定: 最初）")
    ap.add_argument("--to", dest="t_to", default=None, help="終了（この時刻の画像は含まない。既定: 最後）")
    ap.add_argument("--margin", type=float, default=35.0,
                    help="範囲の後ろに残す高頻度データの秒数（既定 35。学習の max_time 30 秒より長く）")
    ap.add_argument("--list", action="store_true", help="切れ目の一覧を表示するだけ")
    ap.add_argument("--min-gap", type=float, default=3.0)
    args = ap.parse_args()

    with h5py.File(args.src, "r") as f:
        if args.list:
            list_gaps(f, args.min_gap)
            return
        if not args.dst:
            sys.exit("切り出し先のファイル名を指定してください")
        if os.path.exists(args.dst):
            sys.exit(f"{args.dst} はすでにあります（上書きしません）")
        ft = f["image_stamp"][:]
        n = len(ft)
        day = datetime.datetime.fromtimestamp(ft[0], JST).date()
        t0 = to_stamp(args.t_from, day) if args.t_from else ft[0]
        t1 = to_stamp(args.t_to, day) if args.t_to else ft[-1] + 1e-3
        idx = np.nonzero((ft >= t0) & (ft < t1))[0]
        if len(idx) == 0:
            sys.exit("範囲に画像がありません")
        with h5py.File(args.dst, "w") as g:
            for k, v in f.attrs.items():
                g.attrs[k] = v
            g.attrs["cut_source"] = os.path.basename(args.src)
            g.attrs["cut_from"] = float(t0)
            g.attrs["cut_to"] = float(t1)
            for name, obj in f.items():
                if isinstance(obj, h5py.Dataset) and obj.shape[:1] == (n,):
                    copy_rows(f, g, name, idx)
                elif isinstance(obj, h5py.Group) and name in STREAM_STAMPS:
                    stamps = obj[STREAM_STAMPS[name]][:]
                    keep = np.nonzero((stamps >= t0 - 10.0) & (stamps < t1 + args.margin))[0]
                    grp = g.create_group(name)
                    for sub in obj:
                        if len(keep):
                            copy_rows(obj, grp, sub, keep)
                        else:
                            grp.create_dataset(sub, shape=(0,) + obj[sub].shape[1:], maxshape=(None,) + obj[sub].shape[1:],
                                               dtype=obj[sub].dtype)
                elif isinstance(obj, h5py.Group):
                    f.copy(obj, g, name=name)
                else:
                    f.copy(obj, g, name=name)
        print(f"{args.dst}: 画像 {len(idx)} 枚（{jst(ft[idx[0]])}〜{jst(ft[idx[-1]])}、元の {n} 枚のうち）、"
              f"高頻度データは {jst(t0 - 10.0)}〜{jst(t1 + args.margin)} を残しました")


if __name__ == "__main__":
    main()
