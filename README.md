### lang_sam_ros1

[lang_sam_ros2](https://github.com/open-rdc/lang_sam_ros2) の2台構成(検出側/追跡側)を、追跡側についてはROS1(Noetic)で動かせるように移植したもの。GPU負荷の大きい検出(LangSAM: GroundingDINO+SAM2)はROS2側のまま維持し、`ros1_bridge`でROS1(追跡側)と接続する。

---
#### 構成

```
[検出PC: ROS2 Humble]                        [ロボットPC: ROS1 Noetic]
lang_sam_detector (lang_sam_ros2、無変更)          lang_sam_tracker
  sub: /lang_sam/detect_request  <--bridge--       pub: /lang_sam/detect_request
  pub: /lang_sam/detections      --bridge-->       sub: /lang_sam/detections
                                                   lang_sam_person_following
                                                     sub: /lang_sam/tracks (ROS1内で完結)
                                                     pub: /cmd_vel
```

- `lang_sam_msgs`: ROS2側([lang_sam_ros2/lang_sam_msgs](../../lang_sam_ws/src/lang_sam_ros2/lang_sam_msgs))とフィールド定義が完全一致するROS1版を本リポジトリに同梱。`ros1_bridge`の動的ブリッジは、パッケージ名・メッセージ名・フィールドが一致するカスタムメッセージを自動でマッピングできる。
- `lang_sam_tracker`, `lang_sam_person_following`: ROS2版をrospy/roscppに移植したもの。
- `lang_sam_bringup`: 追跡側(ROS1)を起動するlaunchファイル・パラメータ一式(ROS2版`lang_sam_executor`に相当)。
- `lang_sam_detector`(検出ノード本体)はこのリポジトリには含まない。従来通り`lang_sam_ros2`側のものをそのまま使う。

---
#### このリポジトリのインストール(ロボット側PC, ROS1 Noetic)

```bash
mkdir -p ~/lang_sam_ros1_ws/src
cd ~/lang_sam_ros1_ws/src
# (このディレクトリを配置)
cd ~/lang_sam_ros1_ws
catkin_make        # または catkin build
source devel/setup.bash
```

Cutie(VOS)まわりの依存はROS2版と同じ(`pip install cutie torch torchvision supervision` 等)。

---
#### 起動(2台構成)

検出PC(GPU側、ROS2 Humble。従来通り):
```bash
export ROS_DOMAIN_ID=<検出PCとブリッジホストで共通のID>
ros2 launch lang_sam_executor lang_sam_detector.launch.py
```

ロボット側PC(ROS1 Noetic):
```bash
export ROS_MASTER_URI=http://<roscoreを動かすホスト>:11311
export ROS_IP=<このPCのIP>
roslaunch lang_sam_bringup lang_sam_tracker.launch
```

---
#### ros1_bridge のセットアップ(検出PC上でDocker)

検出PC(このリポジトリを動かしているホスト)はUbuntu 22.04でROS1 Noeticの公式aptパッケージが無いため、
`ros1_bridge`はネイティブインストールではなく **Docker** で動かす。詳細手順は [bridge/README.md](bridge/README.md) を参照。

```bash
cd bridge
./prepare_context.sh      # ROS1/ROS2両方の lang_sam_msgs をビルドコンテキストへ配置
docker build -t lang_sam_ros1_bridge .

export ROS_MASTER_URI=http://<ロボット側PCのIP>:11311
export ROS_IP=<この検出PCのIP>
export ROS_DOMAIN_ID=<lang_sam_detectorと共通のID>
docker compose up
```

このDockerビルド・起動・実データ疎通(カスタムメッセージ`lang_sam_msgs`を含む)は、
`ros:noetic-ros-core`コンテナで疑似ROS1マスターを立てた上でこのホストの実際のROS2 Humbleと
組み合わせて実機検証済み(手順は[bridge/README.md](bridge/README.md)参照)。

起動後、以下で双方向にトピックが見えることを確認する。
```bash
rostopic list          # ROS1側:  /lang_sam/detect_request, /lang_sam/detections が見える
ros2 topic list        # ROS2側: 同上
```

なお、ブリッジは検出PC以外(ロボットPCや第3のホスト)で動かしても構わない。その場合はROS1とROS2の両方が
使える環境であれば方式は問わない([bridge/README.md](bridge/README.md)のRoboStack構成はその一例)。

---
#### 単一マシンでの動作確認について

検出ノードがROS2専用のため、真の意味での「1台構成」は成立しない(最低限、同一マシン内でROS1・ROS2・ros1_bridgeの3者を起動する形になる)。動作確認だけならROS2版の[lang_sam_executor.launch.py](../../lang_sam_ws/src/lang_sam_ros2/lang_sam_executor/launch/lang_sam_executor.launch.py)(全ノードROS2)を使う方が簡単。

---
#### 未検証事項について

`bridge/`配下のDockerイメージ(ビルド・`ros1_bridge`によるカスタムメッセージのブリッジ・実データ疎通)は
このマシン上で実機検証済み(検証手順は[bridge/README.md](bridge/README.md)参照)。

一方、このマシンにはROS1(Noetic)本体が入っていないため、`lang_sam_msgs`/`lang_sam_tracker`/
`lang_sam_person_following`/`lang_sam_bringup`(ロボット側PCで動かすROS1パッケージ本体)は
`catkin_make`によるビルド確認ができていない(Python構文チェックのみ実施)。ロボット側PCの
実機ROS1 Noetic環境でビルド・動作確認してほしい。

---
#### License
This project is licensed under the Apache 2.0 License
