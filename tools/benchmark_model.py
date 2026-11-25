"""waypoint モデルの推論時間とメモリを Jetson 上で実測する（コンテナ内で実行）

PyTorch（FP32 / FP16）と、TensorRT（FP16、trtexec があれば）で、1 枚ずつ推論したときの時間を測ります。
制御に効くのは平均よりばらつきなので、平均・中央値・99 パーセンタイル・最大を表示します。
学習前でも、ランダムな重みのモデルで計測できます（計算時間は重みの値によらない）。

使い方（走行システムを止めてから。動かしたままでも測れますが、値は実運用に近い代わりに揺れます）:
    cd /home/orin/dev/ws/colcon_ws
    python3 tools/benchmark_model.py                     # ランダムな重みのモデル
    python3 tools/benchmark_model.py --model models/waypoint.pth --iters 1000
    python3 tools/benchmark_model.py --no-trt             # TensorRT を使わない
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

import numpy as np
import torch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from waypoint_model import WaypointModel, load_checkpoint  # noqa: E402


def stats(ms):
    ms = np.asarray(ms)
    return f"平均 {ms.mean():6.2f} ms  中央値 {np.median(ms):6.2f}  99% {np.percentile(ms, 99):6.2f}  最大 {ms.max():6.2f}"


def bench_torch(model, x, iters, warmup=30):
    times = []
    with torch.no_grad():
        for i in range(iters + warmup):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            model(x)
            torch.cuda.synchronize()
            if i >= warmup:
                times.append((time.perf_counter() - t0) * 1000)
    return times


def main():
    ap = argparse.ArgumentParser(description="waypoint モデルの推論時間の実測")
    ap.add_argument("--model", default=None, help="学習済みモデル（省略時はランダムな重み）")
    ap.add_argument("--iters", type=int, default=500)
    ap.add_argument("--no-trt", action="store_true")
    args = ap.parse_args()
    if not torch.cuda.is_available():
        sys.exit("CUDA が使えません（コンテナ内で実行してください）")

    if args.model:
        model, _ = load_checkpoint(args.model)
    else:
        model = WaypointModel(pretrained=False)
    model = model.cuda().eval()
    h, w = model.config["image_size"]
    x = torch.rand(1, 3, h, w, device="cuda")
    print(f"入力 {w}×{h}、パラメータ {sum(p.numel() for p in model.parameters()) / 1e6:.1f} M")

    torch.cuda.reset_peak_memory_stats()
    print(f"PyTorch FP32 : {stats(bench_torch(model, x, args.iters))}")
    model_h, x_h = model.half(), x.half()
    print(f"PyTorch FP16 : {stats(bench_torch(model_h, x_h, args.iters))}")
    print(f"  GPU メモリ（PyTorch のピーク） {torch.cuda.max_memory_allocated() / 2**20:.0f} MB")

    trtexec = shutil.which("trtexec") or "/usr/src/tensorrt/bin/trtexec"
    if args.no_trt or not os.path.exists(trtexec):
        print("TensorRT     : 省略" + ("" if args.no_trt else "（trtexec が見つかりません）"))
        return
    with tempfile.TemporaryDirectory() as d:
        onnx = os.path.join(d, "waypoint.onnx")
        class WaypointsOnly(torch.nn.Module):     # the controls are not used at inference
            def __init__(self, m):
                super().__init__()
                self.m = m

            def forward(self, image):
                return self.m(image)[0]
        try:
            torch.onnx.export(WaypointsOnly(model.float()), x.float(), onnx, input_names=["image"],
                              output_names=["waypoints"], opset_version=17, dynamo=False)
        except Exception as e:
            print(f"TensorRT     : 省略（ONNX への変換に失敗: {e}）")
            if "onnx is not installed" in str(e):
                print("  コンテナに onnx がありません（コンテナからはインターネットにつながりません）。ホストで\n"
                      "    pip3 download onnx ml_dtypes --only-binary=:all: --platform manylinux_2_28_aarch64 --python-version 3.10 "
                      "--implementation cp --no-deps -d /tmp/wheels\n"
                      "  を実行し、コンテナで pip install --no-index --no-deps --target /tmp/pylib /tmp/wheels/*.whl のあと\n"
                      "  PYTHONPATH=/tmp/pylib python3 tools/benchmark_model.py として実行してください。")
            return
        cmd = [trtexec, f"--onnx={onnx}", "--fp16", f"--iterations={args.iters}", "--warmUp=500",
               "--useSpinWait", "--noDataTransfers", "--memPoolSize=workspace:512"]
        print("TensorRT FP16: エンジンを作成して計測しています（1〜3 分かかります）...", flush=True)
        out = subprocess.run(cmd, capture_output=True, text=True)
        lines = [ln for ln in out.stdout.splitlines() if "GPU Compute Time" in ln or "Latency:" in ln]
        if out.returncode != 0 or not lines:
            print("  失敗しました:\n" + "\n".join(out.stdout.splitlines()[-15:] + out.stderr.splitlines()[-5:]))
        for ln in lines:
            print("  " + ln.split("] ", 1)[-1])


if __name__ == "__main__":
    main()
