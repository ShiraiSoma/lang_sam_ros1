#!/usr/bin/env bash
# docker build 用のビルドコンテキストに、ROS1版/ROS2版それぞれの lang_sam_msgs を
# 配置する。Dockerのビルドコンテキストはこのディレクトリ配下に限定されるため、
# 別ワークスペースにあるROS2版 lang_sam_msgs を事前にコピーしておく必要がある。
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS1版 lang_sam_msgs (このリポジトリ内)
ROS1_MSGS="${LANG_SAM_ROS1_MSGS:-$HERE/../lang_sam_msgs}"

# ROS2版 lang_sam_msgs (lang_sam_ros2 リポジトリ)。既定では ~/lang_sam_ws を想定
ROS2_MSGS="${LANG_SAM_ROS2_MSGS:-$HOME/lang_sam_ws/src/lang_sam_ros2/lang_sam_msgs}"

if [ ! -d "$ROS1_MSGS" ]; then
  echo "ROS1版 lang_sam_msgs が見つかりません: $ROS1_MSGS" >&2
  exit 1
fi
if [ ! -d "$ROS2_MSGS" ]; then
  echo "ROS2版 lang_sam_msgs が見つかりません: $ROS2_MSGS" >&2
  echo "環境変数 LANG_SAM_ROS2_MSGS でパスを指定してください" >&2
  exit 1
fi

rm -rf "$HERE/context"
mkdir -p "$HERE/context/ros1_ws/src" "$HERE/context/ros2_ws/src"

cp -r "$ROS1_MSGS" "$HERE/context/ros1_ws/src/lang_sam_msgs"
cp -r "$ROS2_MSGS" "$HERE/context/ros2_ws/src/lang_sam_msgs"

echo "コンテキストを準備しました: $HERE/context"
