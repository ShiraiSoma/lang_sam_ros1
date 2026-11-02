#!/usr/bin/env python3
"""LangSAMクライアントノード(ロボット側PC, ROS1)

カメラ画像をJPEG圧縮して検出PC(ROS2)のlang_sam_tracker(Cutie)へros1_bridge経由で送り、
返ってきた追跡結果(TrackArray + インデックスマスク)と検出結果(DetectionArray)を
手元に保持している元フレームに重ねて描画・表示する。
(ROS2版 lang_sam_ros2/lang_sam_client をrospyに移植したもの)

推論は一切行わないためtorch/GPUは不要。結果のheader.stampは送信したフレームの
stampがそのまま返ってくるので、それをキーに元フレームと対応付ける
(結果画像そのものは受け取らないので、下りの通信量はマスク分だけで済む)。
"""
import threading
import time
from collections import OrderedDict, deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ROSImage

from lang_sam_client.vis_utils import draw_image
from lang_sam_msgs.msg import DetectionArray, TrackArray

# 結果(tracks/mask, detections_lite/mask)の片方だけ届いた状態で保持しておく最大件数
MAX_PENDING_RESULTS = 30

# 追跡FPS・RTTを集計する時間窓(秒)。結果は回線の都合でまとめて届くことがあり、
# 到着間隔の逆数(1/dt)だと値が跳ね上がるため、窓内の件数で数える
STATS_WINDOW_SEC = 1.0

# rospyは既定の受信バッファが小さく、画像のような大きいメッセージだと遅延が溜まるため広げる
IMAGE_BUFF_SIZE = 2 ** 24


def _stamp_key(stamp):
    return (int(stamp.secs), int(stamp.nsecs))


class LangSamClientNode:
    def __init__(self):
        rospy.init_node('lang_sam_client')
        rospy.loginfo('Initializing LangSAM Client Node...')

        self._setup_parameters()

        self.bridge = CvBridge()

        # rospyはサブスクライバごとに別スレッドでコールバックを呼ぶため、共有状態はロックで保護する
        self.lock = threading.Lock()

        # 送信済みフレーム(JPEGバイト列, 送信時刻)をstampで引けるよう保持する
        # (送信時刻はRTT計測用。ロボットPCの時計だけで測るので両PCの時計ずれの影響を受けない)
        self.sent_frames = OrderedDict()
        self.last_sent_time = None

        # 追跡結果・検出結果はそれぞれ2トピックに分かれて届くため、stampが揃うまで待つ
        self.pending_tracks = OrderedDict()
        self.pending_masks = OrderedDict()
        self.pending_dets = OrderedDict()
        self.pending_det_masks = OrderedDict()

        # 表示用の状態(cv2.imshowはメインスレッドで行うため、描画済み画像を受け渡す)
        self.latest_det_vis = None
        self.latest_view = None
        self.fps_alpha = 0.1
        self.track_events = deque()   # (受信時刻, RTT[s]) 直近STATS_WINDOW_SEC分
        self.det_last_time = None
        self.det_fps = 0.0

        # 処理時間・件数の計測(カクつき・遅延の切り分け用)。stats_log_sec周期でまとめてログ出力する
        self._reset_stats()
        if self.stats_log_sec > 0:
            self.stats_timer = rospy.Timer(rospy.Duration(self.stats_log_sec), self._log_stats)

        self.frame_pub = rospy.Publisher(self.frame_topic, CompressedImage, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, ROSImage, self.image_callback, queue_size=1, buff_size=IMAGE_BUFF_SIZE)
        self.tracks_sub = rospy.Subscriber(self.tracks_topic, TrackArray, self._on_tracks, queue_size=1)
        self.track_mask_sub = rospy.Subscriber(
            self.track_mask_topic, CompressedImage, self._on_track_mask, queue_size=1, buff_size=IMAGE_BUFF_SIZE)
        # 検出結果はマスク抜きのDetectionArrayとPNGマスクに分かれて届く
        # (非圧縮マスク付きの/lang_sam/detectionsは購読しないので、ros1_bridgeも転送しない)
        self.detections_sub = rospy.Subscriber(
            self.detections_lite_topic, DetectionArray, self._on_detections, queue_size=1)
        self.detection_mask_sub = rospy.Subscriber(
            self.detection_mask_topic, CompressedImage, self._on_detection_mask, queue_size=1,
            buff_size=IMAGE_BUFF_SIZE)

        rospy.loginfo(f'Image topic: {self.image_topic}')
        rospy.loginfo(f'Frame topic (to tracker): {self.frame_topic}')
        rospy.loginfo(f'Tracks topic: {self.tracks_topic}')
        rospy.loginfo(f'Track mask topic: {self.track_mask_topic}')
        rospy.loginfo(f'Detections lite topic: {self.detections_lite_topic}')
        rospy.loginfo(f'Detection mask topic: {self.detection_mask_topic}')
        rospy.loginfo('LangSAM Client initialized.')

    def _setup_parameters(self):
        self.image_topic = rospy.get_param('~image_topic', '/image_raw')
        self.frame_topic = rospy.get_param('~frame_topic', '/lang_sam/frame')
        self.tracks_topic = rospy.get_param('~tracks_topic', '/lang_sam/tracks')
        self.track_mask_topic = rospy.get_param('~track_mask_topic', '/lang_sam/track_mask')
        self.detections_lite_topic = rospy.get_param('~detections_lite_topic', '/lang_sam/detections_lite')
        self.detection_mask_topic = rospy.get_param('~detection_mask_topic', '/lang_sam/detection_mask')
        self.jpeg_quality = int(rospy.get_param('~jpeg_quality', 85))              # 送信フレームのJPEG品質
        self.stream_max_fps = float(rospy.get_param('~stream_max_fps', 0.0))       # 送信フレームレートの上限。0以下で無制限(カメラのレート)
        self.frame_buffer_size = int(rospy.get_param('~frame_buffer_size', 300))   # 結果と対応付けるため保持する送信済みフレーム数
        self.visualize = bool(rospy.get_param('~visualize', True))
        self.stats_log_sec = float(rospy.get_param('~stats_log_sec', 5.0))       # 計測ログを出す周期(秒)。0以下で無効

    # --- 処理時間・件数の計測 ---
    def _reset_stats(self):
        """ロック取得済み、または初期化時に呼ぶこと"""
        self.stats = {
            'sent': 0, 'recv_tracks': 0, 'recv_masks': 0,    # 送信フレーム数 / 受信した追跡結果の各トピック数
            'paired': 0,                                     # tracksとmaskの対応が取れた件数
            'encode': [], 'render': [], 'rtt': [],           # JPEG化 / 描画 / 往復時間(秒)
        }
        self.stats_since = time.time()

    def _log_stats(self, _event=None):
        with self.lock:
            s = self.stats
            elapsed = max(time.time() - self.stats_since, 1e-6)
            self._reset_stats()

        def fmt(vals):
            if not vals:
                return '-'
            return f'avg {1000 * sum(vals) / len(vals):.0f} / max {1000 * max(vals):.0f} ms'

        rospy.loginfo(
            f'[stats {elapsed:.0f}s] 送信 {s["sent"] / elapsed:.1f} fps (JPEG {fmt(s["encode"])}) | '
            f'受信 tracks {s["recv_tracks"] / elapsed:.1f} / mask {s["recv_masks"] / elapsed:.1f} fps, '
            f'対応付け {s["paired"] / elapsed:.1f} fps | 描画 {fmt(s["render"])} | RTT {fmt(s["rtt"])}')

    # --- EMA による FPS 更新の共通処理 ---
    def _update_ema_fps(self, now, last_name, fps_name):
        last = getattr(self, last_name)
        fps = getattr(self, fps_name)
        setattr(self, last_name, now)
        if last is None:
            return
        dt = now - last
        if dt > 1e-6:
            inst = 1.0 / dt
            setattr(self, fps_name, (1.0 - self.fps_alpha) * fps + self.fps_alpha * inst if fps > 0 else inst)

    def _lookup_frame(self, stamp):
        with self.lock:
            entry = self.sent_frames.get(_stamp_key(stamp))
        if entry is None:
            return None
        return cv2.imdecode(np.frombuffer(entry[0], dtype=np.uint8), cv2.IMREAD_COLOR)

    # --- 追跡FPS・RTT(直近STATS_WINDOW_SEC秒の窓で集計) ---
    def _record_track_event(self, stamp):
        now = time.time()
        with self.lock:
            entry = self.sent_frames.get(_stamp_key(stamp))
            rtt = now - entry[1] if entry is not None else None
            self.track_events.append((now, rtt))
            self._trim_track_events(now)
            self.stats['paired'] += 1
            if rtt is not None:
                self.stats['rtt'].append(rtt)

    def _trim_track_events(self, now):
        while self.track_events and now - self.track_events[0][0] > STATS_WINDOW_SEC:
            self.track_events.popleft()

    def _track_stats(self):
        """(追跡FPS, 平均RTT[ms] or None) を返す。ロック取得済みで呼ぶこと"""
        self._trim_track_events(time.time())
        fps = len(self.track_events) / STATS_WINDOW_SEC
        rtts = [r for _, r in self.track_events if r is not None]
        rtt_ms = 1000.0 * sum(rtts) / len(rtts) if rtts else None
        return fps, rtt_ms

    # --- フレーム送信 ---
    def image_callback(self, msg: ROSImage):
        now = time.time()
        if self.stream_max_fps > 0 and self.last_sent_time is not None \
                and now - self.last_sent_time < 1.0 / self.stream_max_fps:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t_enc = time.perf_counter()
        ok, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        t_enc = time.perf_counter() - t_enc
        if not ok:
            rospy.logwarn('JPEGエンコードに失敗したためフレームを送信しません')
            return

        frame_msg = CompressedImage()
        frame_msg.header.frame_id = msg.header.frame_id
        # stampは結果との対応付けに使うため、ドライバが入れていない場合は現在時刻で埋める
        frame_msg.header.stamp = rospy.Time.now() if msg.header.stamp.is_zero() else msg.header.stamp
        frame_msg.format = 'jpeg'
        frame_msg.data = jpeg.tobytes()

        with self.lock:
            self.sent_frames[_stamp_key(frame_msg.header.stamp)] = (frame_msg.data, time.time())
            while len(self.sent_frames) > self.frame_buffer_size:
                self.sent_frames.popitem(last=False)
            self.stats['sent'] += 1
            self.stats['encode'].append(t_enc)

        self.frame_pub.publish(frame_msg)
        self.last_sent_time = now

    # --- 追跡結果の受信 ---
    def _on_tracks(self, msg: TrackArray):
        with self.lock:
            self.stats['recv_tracks'] += 1
        pair = self._store_pending(self.pending_tracks, self.pending_masks, msg)
        if pair is not None:
            self._render_tracking(*pair)

    def _on_track_mask(self, msg: CompressedImage):
        with self.lock:
            self.stats['recv_masks'] += 1
        pair = self._store_pending(self.pending_masks, self.pending_tracks, msg)
        if pair is not None:
            self._render_tracking(pair[1], pair[0])

    def _store_pending(self, store, other, msg):
        """msgをstoreに入れ、同じstampの片割れがotherにあれば (msg, 片割れ) を返す"""
        key = _stamp_key(msg.header.stamp)
        with self.lock:
            store[key] = msg
            while len(store) > MAX_PENDING_RESULTS:
                store.popitem(last=False)
            if key not in other:
                return None
            store.pop(key)
            partner = other.pop(key)
            # 対応が取れたものより古い片割れは今後揃うことがないので捨てる
            for s in (store, other):
                for k in [k for k in s if k < key]:
                    del s[k]
        return msg, partner

    def _render_tracking(self, tracks_msg: TrackArray, mask_msg: CompressedImage):
        self._record_track_event(tracks_msg.header.stamp)
        if not self.visualize:
            return
        t_render = time.perf_counter()

        frame = self._lookup_frame(tracks_msg.header.stamp)
        if frame is None:
            rospy.logdebug('対応する送信済みフレームがない追跡結果を破棄します')
            return
        idx_mask = cv2.imdecode(np.frombuffer(bytes(mask_msg.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if idx_mask is None or idx_mask.shape[:2] != frame.shape[:2]:
            rospy.logwarn('トラックマスクのデコードに失敗、またはフレームとサイズが一致しません')
            return

        tracks = list(tracks_msg.tracks)
        h, w = frame.shape[:2]
        masks = np.zeros((len(tracks), h, w), dtype=bool)
        for i, t in enumerate(tracks):
            masks[i] = idx_mask == t.id
        boxes = np.asarray([[t.x_min, t.y_min, t.x_max, t.y_max] for t in tracks], dtype=np.int32) \
            if tracks else np.zeros((0, 4), dtype=np.int32)
        scores = np.asarray([t.score for t in tracks], dtype=np.float32)
        labels = [t.label for t in tracks]

        track_vis = self._draw(frame, masks, boxes, scores, labels)
        if track_vis is None:
            return
        view = self._compose(track_vis)
        with self.lock:
            self.latest_view = view
            self.stats['render'].append(time.perf_counter() - t_render)

    # --- 検出結果の受信 ---
    def _on_detections(self, msg: DetectionArray):
        pair = self._store_pending(self.pending_dets, self.pending_det_masks, msg)
        if pair is not None:
            self._render_detection(*pair)

    def _on_detection_mask(self, msg: CompressedImage):
        pair = self._store_pending(self.pending_det_masks, self.pending_dets, msg)
        if pair is not None:
            self._render_detection(pair[1], pair[0])

    def _render_detection(self, msg: DetectionArray, mask_msg: CompressedImage):
        with self.lock:
            self._update_ema_fps(time.time(), 'det_last_time', 'det_fps')
        if not self.visualize:
            return

        frame = self._lookup_frame(msg.header.stamp)
        if frame is None:
            rospy.logdebug('対応する送信済みフレームがない検出結果を破棄します')
            return
        idx_mask = cv2.imdecode(np.frombuffer(bytes(mask_msg.data), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if idx_mask is None or idx_mask.shape[:2] != frame.shape[:2]:
            rospy.logwarn('検出マスクのデコードに失敗、またはフレームとサイズが一致しません')
            return
        idx_mask = idx_mask.astype(np.int32)

        n = len(msg.detections)
        h, w = idx_mask.shape
        masks = np.zeros((n, h, w), dtype=bool)
        for i in range(n):
            masks[i] = idx_mask == (i + 1)
        boxes = np.asarray([[d.x_min, d.y_min, d.x_max, d.y_max] for d in msg.detections], dtype=np.int32) \
            if n > 0 else np.zeros((0, 4), dtype=np.int32)
        scores = np.asarray([d.score for d in msg.detections], dtype=np.float32)
        labels = [d.label for d in msg.detections]

        det_vis = self._draw(frame, masks, boxes, scores, labels)
        if det_vis is not None:
            with self.lock:
                self.latest_det_vis = det_vis

    # --- 描画 ---
    def _draw(self, frame_bgr, masks, boxes, scores, labels):
        try:
            pil = PILImage.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
            vis = draw_image(image_rgb=pil, masks=masks, xyxy=boxes, probs=scores, labels=labels)
            return cv2.cvtColor(np.array(vis), cv2.COLOR_RGB2BGR)
        except Exception as e:
            rospy.logwarn(f'draw_image失敗: {e}')
            return None

    @staticmethod
    def _put_fps(image, fps, rtt_ms=None):
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.6, image.shape[1] / 1000.0)
        thickness = 2
        lines = [f'FPS: {fps:.1f}']
        if rtt_ms is not None:
            lines.append(f'RTT: {rtt_ms:.0f} ms')   # フレーム送信から追跡結果受信までの往復時間
        y = 10
        for text in lines:
            (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
            x = max(10, image.shape[1] - 10 - tw)  # 右端に寄せ、最小余白を確保
            y += th
            cv2.putText(image, text, (x, y), font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)
            y += 10

    def _compose(self, track_vis):
        # 検出可視化と追跡可視化を横並びにする(FPS・RTTを描画)
        with self.lock:
            latest_det_vis = self.latest_det_vis
            det_fps = self.det_fps
            track_fps, rtt_ms = self._track_stats()
        h_trk, w_trk = track_vis.shape[:2]
        if latest_det_vis is None:
            det_vis = np.zeros((h_trk, w_trk, 3), dtype=np.uint8)
            cv2.putText(det_vis, 'No detection yet', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
        else:
            det_vis = latest_det_vis.copy()
            # サイズ合わせ(高さを基準に揃える)
            h_det, w_det = det_vis.shape[:2]
            if h_det != h_trk:
                det_vis = cv2.resize(det_vis, (int(w_det * h_trk / h_det), h_trk))

        self._put_fps(det_vis, det_fps)
        self._put_fps(track_vis, track_fps, rtt_ms)

        combined = np.hstack([det_vis, track_vis])

        # ラベル描画: 左=Detection, 右=Tracking
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(0.6, combined.shape[1] / 1000.0)
        cv2.putText(combined, 'Detection', (10, 30), font, font_scale, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(combined, 'Tracking', (det_vis.shape[1] + 10, 30), font, font_scale, (0, 255, 0), 2, cv2.LINE_AA)
        return combined

    def spin(self):
        # OpenCVのGUIはスレッド安全でないため、表示はメインスレッドのループでまとめて行う
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.visualize:
                with self.lock:
                    view, self.latest_view = self.latest_view, None
                if view is not None:
                    cv2.imshow('LangSAM', view)
                cv2.waitKey(1)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


def main():
    node = LangSamClientNode()
    try:
        node.spin()
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
