#!/usr/bin/env bash
# ros1_bridge のビルドには ROS1(pkg-config経由) と ROS2(find_package経由) を
# 同時に見える状態にする必要があるが、CMAKE_PREFIX_PATH に両方を混ぜると
# 同名メッセージパッケージ(sensor_msgs/std_msgs等)が衝突する。
# そのため ROS2側の環境(ros2_env)をベースに activate しつつ、
# ROS1側(ros1_env + 自作lang_sam_msgs)が提供する
# PATH / PKG_CONFIG_PATH / PYTHONPATH / ROS_PACKAGE_PATH だけを追加で載せる。
# 注意: -u(nounset)は使わない。RoboStackのconda activate.dスクリプトが
# 未初期化変数($CONDA_BUILD等)を参照しており、-uだとそこで落ちる。
set -eo pipefail

eval "$(micromamba shell hook --shell bash)"

# ROS1側の増分を隔離したサブシェルで確定させる(CMAKE_PREFIX_PATH等は捨てる)
bash -c '
  eval "$(micromamba shell hook --shell bash)"
  micromamba activate ros1_env
  source /ws/ros1_ws/devel/setup.bash
  echo "export ROS1_PATH_ADD=\"$PATH\""
  echo "export ROS1_PKG_CONFIG_ADD=\"$PKG_CONFIG_PATH\""
  echo "export ROS1_PYTHONPATH_ADD=\"$PYTHONPATH\""
  echo "export ROS1_ROS_PACKAGE_PATH=\"$ROS_PACKAGE_PATH\""
' > /tmp/ros1_env_vars.sh
source /tmp/ros1_env_vars.sh

micromamba activate ros2_env
export PATH="$ROS1_PATH_ADD:$PATH"
export PKG_CONFIG_PATH="$ROS1_PKG_CONFIG_ADD:${PKG_CONFIG_PATH:-}"
export PYTHONPATH="$ROS1_PYTHONPATH_ADD:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$ROS1_ROS_PACKAGE_PATH"

cd /ws/ros2_ws
# BUILD_TESTING=OFF: ros1_bridge同梱のgtest_vendorが新しいコンパイラでビルド
# エラーになる(uintptr_t関連, 上流の既知の非互換)ため、テストは無効化する
colcon build --symlink-install --packages-select lang_sam_msgs ros1_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DBUILD_TESTING=OFF
