#!/usr/bin/env bash
# ros1_env(Noetic) と ros2_env(Humble) の両方をこのシェルに載せたうえで、
# ローカルビルドしたworkspace(lang_sam_msgs等)をoverlayしてから
# 渡されたコマンド(デフォルトはdynamic_bridge)を実行する。
#
# 実行時はcmake configureが走らないためCMAKE_PREFIX_PATHの衝突は問題にならないが、
# micromamba activateを連続で呼ぶとPATH等が上書きされ、ros1_env側のroscore/rostopic等が
# 消えてしまう(dynamic_bridge本体はRPATHでライブラリを解決するため動作はするが、
# コンテナ内でroscore/rostopicを使ったデバッグができなくなる)。
# そのためbuild_ros1_bridge.shと同じ要領でROS1側の増分を明示的にマージする。
#
# 注意: PYTHONPATHはマージしない。ROS1側のstd_msgs等のPythonバインディングが
# ROS2側より前に見えてしまい、`ros2 topic echo`等が
# "The message type 'std_msgs/msg/String' is invalid" のように壊れることを実機で確認済み。
# dynamic_bridge本体はコンパイル済みバイナリでPythonに依存しないため実害はない。
# ROS1側のPython CLI(rostopic等)をこのコンテナ内でどうしても使いたい場合は、
# `docker exec -it <container> micromamba run -n ros1_env bash` のように隔離したシェルで使うこと。
#
# 注意: -u(nounset)は使わない。RoboStackのconda activate.dスクリプトが
# 未初期化変数を参照しており、-uだとそこで落ちる。
set -eo pipefail

eval "$(micromamba shell hook --shell bash)"

eval "$(bash -c '
  eval "$(micromamba shell hook --shell bash)"
  micromamba activate ros1_env
  source /ws/ros1_ws/devel/setup.bash
  echo "export ROS1_PATH_ADD=\"$PATH\""
  echo "export ROS1_PKG_CONFIG_ADD=\"$PKG_CONFIG_PATH\""
  echo "export ROS1_ROS_PACKAGE_PATH=\"$ROS_PACKAGE_PATH\""
  echo "export ROS1_LD_LIBRARY_PATH_ADD=\"$LD_LIBRARY_PATH\""
')"

micromamba activate ros2_env
source /ws/ros2_ws/install/setup.bash

export PATH="$ROS1_PATH_ADD:$PATH"
export PKG_CONFIG_PATH="$ROS1_PKG_CONFIG_ADD:${PKG_CONFIG_PATH:-}"
export ROS_PACKAGE_PATH="$ROS1_ROS_PACKAGE_PATH"
export LD_LIBRARY_PATH="$ROS1_LD_LIBRARY_PATH_ADD:${LD_LIBRARY_PATH:-}"

exec "$@"
