"""仮の値（未確定）のパラメータの一覧と、実際の値の求め方を表示する

config.yaml の各セクションの `unverified` に並んでいる項目が対象です。実測した値を入れたら、その名前を
unverified から消してください。「AUTO に必須」の項目が残っている間は、制御ノードは AUTO 用の操作量を出しません
（controller.allow_unverified: true で一時的に解除できますが、車輪を浮かせた試験専用です）。

使い方（ホストでもコンテナでも可）:
    python3 tools/check_calibration.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config, unverified  # noqa: E402


def main():
    cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    items = unverified(cfg)
    if not items:
        print("仮の値のパラメータはありません。すべて実測済みです。")
        return 0
    critical = [i for i in items if i[3]]
    print(f"仮の値のパラメータ: {len(items)} 件（うち AUTO に必須: {len(critical)} 件）\n")
    for name, value, how, crit in sorted(items, key=lambda i: (not i[3], i[0])):
        print(f"{'★' if crit else '・'} {name} = {value}")
        print(f"    求め方: {how}")
    print("\n★ = AUTO に必須（仮の値の間は AUTO に切り替わりません）")
    if cfg.get("controller", {}).get("allow_unverified"):
        print("⚠ controller.allow_unverified が true です。仮の値のままでも AUTO 用の操作量が出ます（車輪を浮かせた試験専用）。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
