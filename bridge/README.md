# lang_sam ros1_bridge (Docker)

検出PC(このホスト、Ubuntu 22.04 / ROS2 Humble)の上で `ros1_bridge` をDockerコンテナとして動かし、
ROS1(ロボット側PC, Noetic)の `lang_sam_tracker` とROS2(このホスト)の `lang_sam_detector` を接続する。

**このDockerイメージは実際にビルド・実行し、`ros:noetic-ros-core` コンテナ + このホスト本来の
ROS2 Humbleネイティブ環境を使って `/lang_sam/detect_request`(カスタム型を含む)の実データ疎通まで
確認済み。**

## なぜDockerか

- ROS1 Noeticの公式aptパッケージはUbuntu 20.04(Focal)向けのみで、22.04(Jammy)には存在しない。
- `ros-humble-ros1-bridge` のようなビルド済みaptパッケージもROS2公式リポジトリには存在しない(確認済み)。
- そのため、[RoboStack](https://robostack.github.io/)(conda-forge / robostack-staging)が配布する
  ROS1 NoeticとROS2 Humbleを使い、`ros1_bridge` をソースからビルドする。ホストOS本体には手を入れず、
  コンテナの中だけで完結させる。

**重要**: NoeticとHumbleは同一conda環境には入れない。両者は `sensor_msgs`/`std_msgs` 等を
同じ相対パス(`share/<pkg>/...`)にインストールするため、1つの環境に混在させると片方が他方を
上書きし、catkinのメッセージ生成が誤った方(ROS2側)を解決してビルドが壊れる(実際にこれで
一度失敗した)。そのため `ros1_env`(Noetic専用)と `ros2_env`(Humble専用)の2つのconda環境に
分離し、`ros1_bridge` のビルド時・実行時にだけ `ros2_env` をベースに `ros1_env` 由来の
`PATH`/`PKG_CONFIG_PATH`/`LD_LIBRARY_PATH` を追加で載せる([build_ros1_bridge.sh](build_ros1_bridge.sh),
[entrypoint.sh](entrypoint.sh))。`PYTHONPATH` はあえてマージしない(ROS1側の `std_msgs` 等の
Pythonバインディングが先に見えてしまい `ros2 topic echo` 等が壊れることを確認したため)。

## 前提

- `docker` コマンドが実行できること(`docker` グループへの追加は `sudo usermod -aG docker $USER`
  の後、再ログインするか `newgrp docker` / `sg docker -c '...'` で反映させる)
- `~/lang_sam_ws/src/lang_sam_ros2/lang_sam_msgs`(ROS2版メッセージ定義)が既に存在すること

## ビルド

```bash
cd ~/lang_sam_ros1_ws/src/lang_sam_ros1/bridge

# ROS1版/ROS2版 lang_sam_msgs をビルドコンテキストにコピー
./prepare_context.sh
# ROS2版の場所が既定(~/lang_sam_ws/src/lang_sam_ros2/lang_sam_msgs)と違う場合:
#   LANG_SAM_ROS2_MSGS=/path/to/lang_sam_msgs ./prepare_context.sh

docker build -t lang_sam_ros1_bridge .
# または: docker compose build
```

conda環境の構築(RoboStackから ros-noetic-ros-base / ros-humble-ros-base 等を取得)と
`ros1_bridge` のソースビルドを合わせて、初回は5分前後かかる(検証時の実測)。

ビルド後、`lang_sam_msgs` が正しくブリッジ対象として認識されているかは以下で確認できる:

```bash
docker run --rm lang_sam_ros1_bridge ros2 run ros1_bridge dynamic_bridge --print-pairs | grep lang_sam
```

次のように4つとも表示されれば成功:

```
- 'lang_sam_msgs/msg/Detection' (ROS 2) <=> 'lang_sam_msgs/Detection' (ROS 1)
- 'lang_sam_msgs/msg/DetectionArray' (ROS 2) <=> 'lang_sam_msgs/DetectionArray' (ROS 1)
- 'lang_sam_msgs/msg/Track' (ROS 2) <=> 'lang_sam_msgs/Track' (ROS 1)
- 'lang_sam_msgs/msg/TrackArray' (ROS 2) <=> 'lang_sam_msgs/TrackArray' (ROS 1)
```

## 実行

環境変数を設定して起動する。`ROS_MASTER_URI` はロボット側PCの `roscore` を指す。

```bash
export ROS_MASTER_URI=http://<ロボット側PCのIP>:11311
export ROS_IP=<この検出PCのIP>
export ROS_DOMAIN_ID=<lang_sam_detectorと共通のID>

docker compose up
# または:
docker run --rm --network host --ipc host \
  -e ROS_MASTER_URI=$ROS_MASTER_URI \
  -e ROS_IP=$ROS_IP \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  lang_sam_ros1_bridge
```

- `--network host`: ROS1のTCPROS接続とROS2のDDS探索の両方を、コンテナがホストと同じネットワーク上で
  行う必要があるため必須。
- `--ipc host`: 必須ではないが、Fast-DDSの共有メモリ transportを使う場合に備えて付けている
  (下記のFASTDDS_BUILTIN_TRANSPORTSにより通常は使われない)。
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`: Dockerfile内で既定値として設定済み。検証時、Fast-DDSの
  既定の共有メモリ(SHM)transportだと、DDSのdiscovery(トピック名・型・QoSの解決)は成功するのに
  実データが一切subscriber側に届かないという現象が発生した(コンテナ内外の別ビルドのFast-DDS間で
  SHMセグメントの受け渡しに失敗する既知の類の問題)。UDPv4のみに強制することで解消することを
  実機で確認済み。**detector側(ホストのlang_sam_detector)は変更不要**、このコンテナ側だけの設定で足りる。

## 動作確認

```bash
# ロボット側PC(ROS1)で
rostopic list | grep lang_sam
rostopic hz /lang_sam/tracks

# 検出PC(ROS2、ホスト側で)で
ros2 topic list | grep lang_sam
ros2 topic hz /lang_sam/frame
```

双方に `/lang_sam/frame`, `/lang_sam/tracks`, `/lang_sam/track_mask`, `/lang_sam/detections` が現れ、
`lang_sam_client` の起動後に `rostopic hz`/`ros2 topic hz` でデータが流れていることを確認する。
(標準構成では `/lang_sam/detect_request` は検出PCのROS2内で完結するためブリッジされない。
旧構成(ロボットPCで`lang_sam_tracker`)の場合は `/lang_sam/detect_request`, `/lang_sam/detections` を確認する)

## 動作検証の方法(このホストだけで疑似2台構成を再現する場合)

実機のロボットPCがまだ無くても、`ros:noetic-ros-core` コンテナで疑似的なROS1マスターを
このホスト上に立てて疎通確認ができる(実際にこの方法で検証済み):

```bash
# 疑似ROS1マスター
docker run -d --rm --network host --name roscore_test ros:noetic-ros-core roscore

# ブリッジ
docker run -d --rm --network host --ipc host --name bridge_test \
  -e ROS_MASTER_URI=http://localhost:11311 -e ROS_DOMAIN_ID=<ID> \
  lang_sam_ros1_bridge

# ROS1側からテストpublish
docker run -d --rm --network host -e ROS_MASTER_URI=http://localhost:11311 ros:noetic-ros-core \
  rostopic pub /lang_sam/detect_request std_msgs/String 'hello' -r 2

# このホストのネイティブROS2 Humbleで受信できるか確認
ros2 topic echo /lang_sam/detect_request
```

## 既知の注意点

- `ros1_bridge` の同梱テスト(`BUILD_TESTING`)は、上流のgtest_vendorが新しいコンパイラで
  ビルドエラーになるため無効化している(`-DBUILD_TESTING=OFF`)。ブリッジ本体の動作には影響しない。
- 本家 `ros2/ros1_bridge` リポジトリに `humble` 等ディストロ別ブランチは存在せず、`master` を
  そのままビルドする(distro非依存のコードのため)。

### 実データが流れない場合(discoveryは成功するのにtopic echo/hzで何も来ない)

ブリッジのログに以下のようなエラーが出ていないか確認する。

```
[ERROR] couldn't resolve publisher host [<ROS1側のホスト名>]
Failed to connect to /xxx (<ホスト名>:<port>)
```

これはROS1ノードが`ROS_IP`/`ROS_HOSTNAME`未設定のままOSのホスト名で自身を広告し、
ブリッジ側からそのホスト名を名前解決できないために実データ転送(TCPROS)だけが失敗している状態
(roscoreへのトピック一覧問い合わせ等、マスター経由のXML-RPC通信は名前解決不要なので成功して見える)。

**根本対策**: ROS1ホスト側の全ターミナル(roscore・カメラドライバ・tracker等、起動するもの全て)で
`ROS_IP`(または`ROS_HOSTNAME`)を明示的に設定してから起動する。`~/.bashrc`に書いておくと
ターミナルごとの設定漏れを防げる。

```bash
echo 'export ROS_IP=<ROS1ホストのIP>' >> ~/.bashrc
```

**保険**: `docker-compose.yml`の`extra_hosts`でブリッジコンテナにホスト名解決を教えておく。

```bash
export ROS1_HOSTNAME=<ROS1ホストの`hostname`出力>
export ROS1_HOST_IP=<ROS1ホストのIP>
docker compose up
```
