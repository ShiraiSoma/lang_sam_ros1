# ros1_bridge (ROS1 Noetic <-> ROS2 Humble) を1コンテナ内で動かすためのイメージ。
#
# Ubuntu 22.04(Jammy)には ROS1 Noeticの公式aptパッケージが存在しないため、
# RoboStack(conda-forge / robostack-staging)のROS配布物を使う。
#
# 重要: NoeticとHumbleは"同一"conda環境にインストールしない。
# 両者はsensor_msgs/std_msgs等、同じ相対パス(share/<pkg>/...)にファイルをインストールするため、
# 1つの環境に混在させると片方が他方を上書きし、catkin/cmakeが誤った方を解決してしまう
# (実際にこれで一度ビルドに失敗している)。
# ros1_bridge本家もこれを前提に、ROS1側はpkg-config、ROS2側はfind_package(CMAKE_PREFIX_PATH)
# という別々の経路で解決する設計になっている。そのためここでは
#   ros1_env: ROS1(Noetic)専用
#   ros2_env: ROS2(Humble)専用
# の2つのconda環境に分離し、ros1_bridgeビルド時のみ ros2_env をベースに
# ros1_env由来のPATH/PKG_CONFIG_PATH/PYTHONPATHを追加で載せる(build_ros1_bridge.sh参照)。
#
# 事前に prepare_context.sh を実行し、ROS1側/ROS2側それぞれの lang_sam_msgs を
# context/ros1_ws, context/ros2_ws 以下に配置してからビルドすること。

FROM mambaorg/micromamba:1.5.8-jammy

# Fast-DDS(rmw_fastrtps_cpp)のデフォルトSHM(共有メモリ)トランスポートは、
# --network host + --ipc host で本物のROS1マスター経由のブリッジ実データ配信を
# 検証した際に、discovery(topic list/type/QoSの解決)は成功するのにpublish
# したデータが一切subscriber側に届かないという問題を実際に引き起こした
# (docker上の別ビルドのFast-DDS同士でSHMセグメントの受け渡しに失敗する既知の類の問題)。
# UDPv4のみに強制することで解消することを確認済み。detector側(ホスト)は変更不要。
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

RUN micromamba create -y -n ros1_env \
      -c robostack-staging -c conda-forge \
      ros-noetic-ros-base \
      "empy<4" \
      compilers cmake pkg-config make ninja git catkin_tools \
    && micromamba create -y -n ros2_env \
      -c robostack-staging -c conda-forge \
      ros-humble-ros-base \
      ros-humble-rmw-fastrtps-cpp \
      compilers cmake pkg-config make ninja git colcon-common-extensions \
    && micromamba clean --all --yes

USER root
RUN mkdir -p /ws && chown $MAMBA_USER:$MAMBA_USER /ws
USER $MAMBA_USER
WORKDIR /ws

# --- ROS1側: lang_sam_msgs をビルド(ros1_envのみ) ---
# RoboStackのcatkinが同梱するtoplevel.cmakeがCMake 3.5未満を要求しており、
# 最近のCMakeではそのままでは弾かれるため、CMAKE_POLICY_VERSION_MINIMUMで許容する
COPY --chown=$MAMBA_USER:$MAMBA_USER context/ros1_ws /ws/ros1_ws
RUN micromamba run -n ros1_env bash -lc "\
      cd /ws/ros1_ws && \
      catkin_make --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5"

# --- ROS2側: lang_sam_msgs + ros1_bridge をビルド ---
COPY --chown=$MAMBA_USER:$MAMBA_USER context/ros2_ws /ws/ros2_ws
RUN micromamba run -n ros2_env git clone --depth 1 https://github.com/ros2/ros1_bridge.git /ws/ros2_ws/src/ros1_bridge

COPY --chown=$MAMBA_USER:$MAMBA_USER build_ros1_bridge.sh /ws/build_ros1_bridge.sh
RUN bash /ws/build_ros1_bridge.sh

COPY --chown=$MAMBA_USER:$MAMBA_USER entrypoint.sh /ws/entrypoint.sh
USER root
RUN chmod +x /ws/entrypoint.sh
USER $MAMBA_USER

# ベースイメージの _entrypoint.sh(micromambaの初期化)に、
# 自前のentrypoint.sh(両環境のactivate + workspace overlay)を連結する
ENTRYPOINT ["/usr/local/bin/_entrypoint.sh", "/ws/entrypoint.sh"]
CMD ["ros2", "run", "ros1_bridge", "dynamic_bridge", "--bridge-all-topics"]
