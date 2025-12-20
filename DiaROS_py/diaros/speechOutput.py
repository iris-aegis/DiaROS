# ============================================================
# ログレベル設定
# ============================================================
SHOW_BASIC_LOGS = True   # 基本ログ表示（音声再生、エラーなど）
SHOW_DEBUG_LOGS = False  # デバッグログ表示（詳細な処理内容、中間データなど）

import sys
import os
import time
from datetime import datetime
import pygame
import queue
import threading

# Docker環境用の低遅延設定
pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=256)
pygame.mixer.init()

from pydub import AudioSegment


class SpeechOutput:
    """音声出力モジュール

    相槌音声とシステム応答音声を再生するモジュール。
    ROS2トピック経由で再生リクエストを受け取り、優先度に基づいて再生を制御する。
    """

    def __init__(self):
        """初期化"""
        # 再生状態管理
        self.is_playing = False
        self.current_audio_type = None  # "backchannel" | "response"
        self.current_stage = None  # "first" | "second"
        self.current_request_id = None
        self.playback_end_time = None
        self.current_session_id = None

        # リクエストキュー
        self.request_queue = queue.Queue()

        # 再生スレッド
        self.playback_thread = None
        self.running = False

        if SHOW_BASIC_LOGS:
            sys.stdout.write("[SO] speechOutput モジュール初期化完了\n")
            sys.stdout.flush()

    def start(self):
        """音声再生スレッドを開始"""
        self.running = True
        self.playback_thread = threading.Thread(target=self._playback_loop, daemon=True)
        self.playback_thread.start()

        if SHOW_BASIC_LOGS:
            sys.stdout.write("[SO] 音声再生スレッド開始\n")
            sys.stdout.flush()

    def stop(self):
        """音声再生スレッドを停止"""
        self.running = False
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=1.0)

        # 再生中の音声を停止
        if pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()

        if SHOW_BASIC_LOGS:
            sys.stdout.write("[SO] 音声再生スレッド停止\n")
            sys.stdout.flush()

    def play_request_callback(self, audio_type, stage, wav_path, duration_sec,
                             request_id, timestamp_ns, session_id):
        """音声再生リクエストを受け取るコールバック

        Args:
            audio_type (str): 音声タイプ（"backchannel" | "response"）
            stage (str): ステージ（"first" | "second"）
            wav_path (str): 音声ファイルパス
            duration_sec (float): 音声長（秒）
            request_id (int): リクエストID
            timestamp_ns (int): タイムスタンプ（ナノ秒）
            session_id (str): セッションID
        """
        # リクエストをキューに追加
        request_data = {
            'audio_type': audio_type,
            'stage': stage,
            'wav_path': wav_path,
            'duration_sec': duration_sec,
            'request_id': request_id,
            'timestamp_ns': timestamp_ns,
            'session_id': session_id
        }

        if SHOW_DEBUG_LOGS:
            now_dt = datetime.now()
            timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}] [SO-DEBUG] 再生リクエスト受信: type={audio_type}, stage={stage}, request_id={request_id}\n")
            sys.stdout.flush()

        # キューに追加
        self.request_queue.put(request_data)

    def _playback_loop(self):
        """音声再生ループ（バックグラウンドスレッドで実行）"""
        while self.running:
            # 再生終了チェック
            if self.is_playing and self.playback_end_time is not None:
                if time.time() >= self.playback_end_time:
                    # 再生完了
                    if pygame.mixer.music.get_busy():
                        # まだ再生中の場合は待機
                        time.sleep(0.01)
                        continue

                    # 再生完了をログ出力
                    if SHOW_BASIC_LOGS:
                        now_dt = datetime.now()
                        timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                        sys.stdout.write(f"[{timestamp}] [SO] 音声再生完了: type={self.current_audio_type}, stage={self.current_stage}, request_id={self.current_request_id}\n")
                        sys.stdout.flush()

                    # 状態をリセット
                    self.is_playing = False
                    self.current_audio_type = None
                    self.current_stage = None
                    self.current_request_id = None
                    self.playback_end_time = None
                else:
                    # まだ再生中の場合は、キューから取り出さずに待機
                    time.sleep(0.01)
                    continue

            # 再生中でない場合のみキューから次のリクエストを取得
            try:
                request = self.request_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # 再生判定
            if self._should_play(request):
                self._play_audio(request)
            else:
                if SHOW_DEBUG_LOGS:
                    now_dt = datetime.now()
                    timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[{timestamp}] [SO-DEBUG] 再生リクエストをスキップ: type={request['audio_type']}, stage={request['stage']}, request_id={request['request_id']}\n")
                    sys.stdout.flush()

    def _should_play(self, request):
        """再生すべきかどうかを判定

        Args:
            request (dict): 再生リクエスト

        Returns:
            bool: 再生すべきならTrue
        """
        # 再生中でない場合は再生
        if not self.is_playing:
            return True

        # 現在応答再生中で、新しいリクエストが相槌の場合は棄却
        if self.current_audio_type == "response" and request['audio_type'] == "backchannel":
            if SHOW_DEBUG_LOGS:
                now_dt = datetime.now()
                timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                sys.stdout.write(f"[{timestamp}] [SO-DEBUG] 応答再生中のため相槌を棄却\n")
                sys.stdout.flush()
            return False

        # それ以外の場合は再生待機（キューで順次処理）
        # - 相槌再生中に応答が来た場合 → 応答をキューイングして順次再生
        # - first_stage再生中にsecond_stageが来た場合 → キューイングして順次再生
        return True

    def _play_audio(self, request):
        """音声ファイルを再生

        Args:
            request (dict): 再生リクエスト
        """
        wav_path = request['wav_path']

        # ファイル存在確認
        if not os.path.exists(wav_path):
            if SHOW_BASIC_LOGS:
                sys.stdout.write(f"[SO-ERROR] 音声ファイルが見つかりません: {wav_path}\n")
                sys.stdout.flush()
            return

        try:
            # 音声ファイルを読み込み
            audio = AudioSegment.from_wav(wav_path)
            actual_duration_sec = len(audio) / 1000.0

            # 再生開始
            pygame.mixer.music.load(wav_path)
            pygame.mixer.music.play()

            # 状態を更新
            self.is_playing = True
            self.current_audio_type = request['audio_type']
            self.current_stage = request['stage']
            self.current_request_id = request['request_id']
            self.current_session_id = request['session_id']
            self.playback_end_time = time.time() + actual_duration_sec

            # ログ出力
            if SHOW_BASIC_LOGS:
                now_dt = datetime.now()
                timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                audio_type_ja = "相槌" if request['audio_type'] == "backchannel" else "応答"
                stage_ja = "第1段階" if request['stage'] == "first" else "第2段階"
                sys.stdout.write(f"[{timestamp}] [SO] 🔊 音声再生開始: {audio_type_ja}（{stage_ja}）, 長さ={actual_duration_sec:.2f}秒, request_id={request['request_id']}\n")
                sys.stdout.flush()

            if SHOW_DEBUG_LOGS:
                sys.stdout.write(f"[SO-DEBUG] 音声ファイル: {wav_path}\n")
                sys.stdout.flush()

        except Exception as e:
            if SHOW_BASIC_LOGS:
                sys.stdout.write(f"[SO-ERROR] 音声再生エラー: {e}\n")
                sys.stdout.flush()


def speechOutput(callback):
    """音声出力のメインループ

    Args:
        callback (function): 外部からのコールバック関数
    """
    so = SpeechOutput()
    so.start()

    try:
        # callbackを登録
        if callback:
            callback(so.play_request_callback)

        # メインループ
        while so.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        if SHOW_BASIC_LOGS:
            sys.stdout.write("\n[SO] KeyboardInterrupt を検出しました。終了します。\n")
            sys.stdout.flush()
    finally:
        so.stop()
