# legacy（使わなくなったもの）

waypoint モデルに移行する前に使っていたスクリプトなどです。今のシステムでは使いません。参照用に残しています。

| ファイル | 内容 |
| :--- | :--- |
| `train.py` | 以前の操作量モデル（画像 → ステア・スロットル）の学習スクリプト |
| `train_backup.py` | `train.py` の古い版 |
| `convert_donkey_to_h5.py` | DonkeyCar の記録を HDF5 に変換するスクリプト（以前のモデル用） |
| `run_test.sh` | 以前の簡易統合テスト（コンテナ内でシステムを起動して記録を確認） |
| `src/cap_test/` | カメラの動作確認用の試験パッケージ |

このフォルダーには `COLCON_IGNORE` があるため、`colcon build` の対象になりません。
