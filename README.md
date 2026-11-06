### lang_sam_ros1

[lang_sam_ros2](https://github.com/open-rdc/lang_sam_ros2) の2台構成(検出側/追跡側)を、追跡側についてはROS1(Noetic)で動かせるように移植したもの。GPU負荷の大きい検出(LangSAM: GroundingDINO+SAM2)はROS2側のまま維持し、`ros1_bridge`でROS1(追跡側)と接続する。

---
#### 構成

検出(LangSAM)・追跡(Cutie)の両方を検出PC(GPU, ROS2)側で動かし、ロボットPC(ROS1)はフレーム送信と結果の描画・追従制御だけを行う構成が標準(ロボットPCにGPU不要)。

```
[検出PC: ROS2 Humble]                                   [ロボットPC: ROS1 Noetic]
lang_sam_server.launch.py (lang_sam_ros2)                    lang_sam_client (フレーム送信・結果描画)
  lang_sam_tracker (Cutie)                                     pub: /lang_sam/frame (JPEG)
    sub: /lang_sam/frame        <--bridge--                    sub: /lang_sam/tracks
    pub: /lang_sam/tracks       --bridge-->                    sub: /lang_sam/track_mask (16bit PNG)
    pub: /lang_sam/track_mask   --bridge-->                    sub: /lang_sam/detections_lite
      ▲ detect_request / detections (ROS2内で完結)             sub: /lang_sam/detection_mask (16bit PNG)
      ▼
  lang_sam_detector (LangSAM)
    pub: /lang_sam/detections_lite --bridge-->
    pub: /lang_sam/detection_mask  --bridge-->
```

ブリッジは[bridge/bridge_topics.yaml](bridge/bridge_topics.yaml)に列挙した上記5トピックだけを`parameter_bridge`で転送する(理由は[bridge/README.md](bridge/README.md)参照)。

結果には送信したフレームと同じ`header.stamp`が付いて返ってくるので、`lang_sam_client`は手元に保持している元フレームに結果を重ねて表示する(結果画像そのものは転送しない)。

- `lang_sam_msgs`: ROS2側(lang_sam_ros2/lang_sam_msgs)とフィールド定義が完全一致するROS1版を本リポジトリに同梱。`ros1_bridge`の動的ブリッジは、パッケージ名・メッセージ名・フィールドが一致するカスタムメッセージを自動でマッピングできる。
- `lang_sam_client`: ROS2版`lang_sam_client`をrospyに移植したもの。推論を行わないためtorch不要(opencv, pillow, supervisionのみ)。
- `lang_sam_person_following`: ROS2版をroscppに移植したもの。`/cmd_vel`を使わないため、現在はどのlaunchからも起動しない(パッケージのみ残している)。
- `lang_sam_tracker`: Cutie追跡をロボットPC(ROS1)側で動かす旧構成用(後述)。ロボットPCにGPUがある場合のみ使う。
- `lang_sam_bringup`: ロボットPC(ROS1)側を起動するlaunchファイル・パラメータ一式(ROS2版`lang_sam_executor`に相当)。
- 検出PC側のノード(`lang_sam_detector`, `lang_sam_tracker`, `lang_sam_server_node.py`)はこのリポジトリには含まない。`lang_sam_ros2`側のものをそのまま使う。

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

標準構成(`lang_sam_client`)の追加依存は`pip install supervision`のみ(torch/GPU不要。OpenCV・PillowはNoetic環境のものを使う)。
旧構成でロボットPC側に`lang_sam_tracker`を置く場合は、Cutie(VOS)まわりの依存もROS2版と同様に必要(`pip install cutie torch torchvision supervision` 等)。

---
#### 起動(2台構成)

検出PC(GPU側、ROS2 Humble。lang_sam_ros2):
```bash
export ROS_DOMAIN_ID=<検出PCとブリッジホストで共通のID>
ros2 launch lang_sam_executor lang_sam_server.launch.py   # detector + tracker(Cutie)を1プロセスで起動
# 地面追従+障害物回避で走らせる場合は代わりに(ground_navの/cmd_velがブリッジ経由でロボットへ届く):
# ros2 launch lang_sam_executor lang_sam_ground_nav.launch.py
```

ロボット側PC(ROS1 Noetic):
```bash
export ROS_MASTER_URI=http://<roscoreを動かすホスト>:11311
export ROS_IP=<このPCのIP>
roslaunch lang_sam_bringup lang_sam_client.launch          # client(送信・描画)
```

左に検出結果、右に追跡結果を並べたウィンドウがロボットPC上に表示される。
回線やブリッジが詰まる場合は[lang_sam_client.yaml](lang_sam_bringup/config/lang_sam_client.yaml)の`stream_max_fps`/`jpeg_quality`を下げる。

##### (旧構成)ロボットPC側でCutieを動かす場合

ロボットPCにGPUがあり、検出だけを検出PCに任せたい場合は従来通り以下を使う。
検出PC側は`lang_sam_detector`だけを起動すること(`lang_sam_server.launch.py`と同時に使うと
`/lang_sam/tracks`の発行元が2つになる)。

```bash
roslaunch lang_sam_bringup lang_sam_tracker.launch         # tracker(Cutie)
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
rostopic list          # ROS1側:  /lang_sam/frame, /lang_sam/tracks, /lang_sam/track_mask, /lang_sam/detections_lite, /lang_sam/detection_mask が見える
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

一方、このマシンにはROS1(Noetic)本体が入っていないため、`lang_sam_msgs`/`lang_sam_client`/`lang_sam_tracker`/
`lang_sam_person_following`/`lang_sam_bringup`(ロボット側PCで動かすROS1パッケージ本体)は
`catkin_make`によるビルド確認ができていない(Python構文チェックのみ実施)。ロボット側PCの
実機ROS1 Noetic環境でビルド・動作確認してほしい。

---
#### License
This project is licensed under the Apache 2.0 License
