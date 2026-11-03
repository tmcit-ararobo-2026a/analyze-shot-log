#!/usr/bin/env bash
# ログ -> CSV -> モデル -> ROS 2 パッケージの係数ファイル、までを一括で実行する。
#   使い方: bash tools/update_model.sh [convert に渡す引数]
#   例    : bash tools/update_model.sh --target 4:5.0,0.0 --target 5:4.0,0.5,0.58
#           bash tools/update_model.sh -i /path/to/trials.json --target 4:5.0,0.0
# 入力の既定は /shot-trial-log/trials.json。中間出力は tools/out/ (git 管理外)。
set -euo pipefail
cd "$(dirname "$0")"
PKG=../launch_model_node

python3 launch-model.py convert --include-all "$@"
python3 launch-model.py fit --use-all
python3 launch-model.py export-yaml --file "$PKG/config/launch_model_params.yaml"
python3 launch-model.py export-cpp  --file "$PKG/include/launch_model_node/launch_formula.hpp"

echo
echo "更新しました: $PKG/config/launch_model_params.yaml"
echo "              $PKG/include/launch_model_node/launch_formula.hpp"
echo "次の手順: git diff で係数の変化を確認 -> colcon build -> コミット"
