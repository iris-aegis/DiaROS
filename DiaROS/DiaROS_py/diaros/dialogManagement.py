import sys
import socket
import time
from datetime import datetime, timedelta
import pygame
# Docker環境用の低遅延設定
pygame.mixer.pre_init(frequency=22050, size=-16, channels=2, buffer=256)
pygame.mixer.init()
import random
import numpy as np
import webrtcvad
import pyaudio
import queue
import threading
import librosa
import glob
import difflib
import requests
import json
import wave

### power制御用 ###
import statistics
###---###

### 音声ファイル長計測 ###
from pydub import AudioSegment
###---###

### 音声ファイルソート ###
import os
import glob
###---###

### タイミング統合システム ###
try:
    from timing_integration import get_timing_logger, log_timing
    TIMING_AVAILABLE = True
except ImportError:
    TIMING_AVAILABLE = False
###---###

class DialogManagement:
    # グローバル変数を定義
    audio_queue = queue.Queue()  # マイクからの音声データを保存するキュー
    # 設定 - Docker環境用低遅延化
    mic_sample_rate = 48000
    sample_rate     = 16000
    frame_duration  = 20  # ms (30→20に短縮)
    CHUNK           = int(mic_sample_rate * frame_duration / 1000)

    ### 音声ファイル長計測関数 ###
    def get_audio_length(self, filename):
        audio = AudioSegment.from_wav(filename)
        return len(audio) / 1000.0  # 長さを秒単位で返す

    def synthesize_first_stage_backchannel(self, text):
        """First stage相槌を音声合成（VOICEVOX APIを使用）"""
        try:
            synthesis_start_time = time.time()
            speaker = 58
            host = "localhost"
            port = 50021
            params = (
                ('text', text),
                ('speaker', speaker),
            )

            # 音声クエリ生成
            response1 = requests.post(
                f'http://{host}:{port}/audio_query',
                params=params,
                timeout=5
            )
            if response1.status_code != 200:
                sys.stdout.write(f"[ERROR] VOICEVOX audio_query失敗: {response1.status_code}\n")
                sys.stdout.flush()
                return None

            response1_data = response1.json()

            # ★無音を0秒に設定（前後のポーズを除去）
            response1_data["prePhonemeLength"] = 0.0
            response1_data["postPhonemeLength"] = 0.0

            # デバッグ：設定確認
            sys.stdout.write(f"[TTS-DEBUG] VOICEVOX パラメータ設定: prePhonemeLength={response1_data.get('prePhonemeLength')}, postPhonemeLength={response1_data.get('postPhonemeLength')}\n")
            sys.stdout.flush()

            modified_json_str = json.dumps(response1_data)

            # 音声合成
            headers = {'Content-Type': 'application/json'}
            response2 = requests.post(
                f'http://{host}:{port}/synthesis',
                headers=headers,
                params=params,
                data=modified_json_str.encode('utf-8'),
                timeout=5
            )
            if response2.status_code != 200:
                sys.stdout.write(f"[ERROR] VOICEVOX synthesis失敗: {response2.status_code}\n")
                sys.stdout.flush()
                return None

            # ファイル保存
            current_time = datetime.now().strftime("%Y%m%d%H%M%S%f")
            output_file = f'./tmp/first_stage_{current_time}.wav'

            with wave.open(output_file, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(24000)
                wf.writeframes(response2.content)

            synthesis_duration_ms = (time.time() - synthesis_start_time) * 1000
            sys.stdout.write(f"[TTS] First stage相槌音声合成完了 (処理時間: {synthesis_duration_ms:.1f}ms, ファイル: {output_file})\n")
            sys.stdout.flush()

            return output_file

        except Exception as e:
            sys.stdout.write(f"[ERROR] First stage相槌音声合成エラー: {e}\n")
            sys.stdout.flush()
            return None

    def play_sound(self, filename, block=True):
        """pygame.mixerを使用して音声ファイルを再生"""
        try:
            if not os.path.exists(filename):
                print(f"音声ファイルが見つかりません: {filename}")
                return False

            # 既存の音楽が再生中の場合は停止してメモリを解放
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()

            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()

            if block:
                while pygame.mixer.music.get_busy():
                    pygame.time.wait(10)  # Docker環境では短い間隔でチェック
                # 再生完了後にリソースを解放
                pygame.mixer.music.unload()

            return True
        except Exception as e:
            print(f"音声再生エラー: {e}")
            return False

    def play_error_audio(self, error_type):
        """エラーメッセージ音声を再生

        Args:
            error_type (str): エラータイプ
                - 'first_stage': 1段階目の応答生成に失敗
                - 'second_stage': 2段階目の応答生成に失敗
                - 'timeout': 2段階目の応答生成が間に合わなかった
        """
        error_files = {
            'first_stage': './tmp/error_first_stage.wav',
            'second_stage': './tmp/error_second_stage.wav',
            'timeout': './tmp/error_timeout.wav'
        }

        error_messages = {
            'first_stage': '1段階目の応答生成に失敗しました',
            'second_stage': '2段階目の応答生成に失敗しました',
            'timeout': '2段階目の応答生成が間に合いませんでした'
        }

        error_file = error_files.get(error_type)
        error_msg = error_messages.get(error_type)

        if error_file and os.path.exists(error_file):
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"\n[ERROR] {error_msg}\n")
            sys.stdout.write(f"[{timestamp}] エラー音声を再生: {error_file}\n")
            sys.stdout.flush()

            # エラー音声をブロッキング再生
            self.play_sound(error_file, block=True)

            # 再生完了後
            now_end = datetime.now()
            timestamp_end = now_end.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp_end}] エラー音声再生完了\n")
            sys.stdout.flush()

            return True
        else:
            sys.stdout.write(f"\n[ERROR] エラー音声ファイルが見つかりません: {error_file}\n")
            sys.stdout.flush()
            return False

    def __init__(self):
        self.word = ""
        self.asr = { "you": "", "is_final": False }
        self.asr_history = []  # 追加: 音声認識履歴（{"text": str, "timestamp_ns": int, "is_final": bool}の辞書形式）
        self.user_speak_is_final = False
        self.recognition_result_is_confirmed = False
        self.sa = { "prevgrad" : 0.0,
                    "frequency": 0.0,
                    "grad"     : 0.0,
                    "power"    : 0.0,
                    "zerocross": 0   }
        self.ss = { "is_speaking" : False}# test
        self.power_list = []# powerの過去200msの平均を取るためのリスト
        self.power_ave = 0.0# powerの過去200msの平均
        self.power_calib_list = []
        self.power_calib_ave = 0.0
        self.prev_power_get_time = datetime.now()
        self.speaking_time = datetime.now()
        self.response_pause_length = 1#応答の間隔をあけるための時間
        self.back_channel_pause_length = 2#相槌の間隔をあけるための時間
        self.prev_response_time = datetime.now()
        self.additional_asr_start_time = False
        self.prev_back_channel_time = datetime.now()
        self.response_cnt = 0# 固定応答再生用
        self.back_channel_cnt = 0# 相槌内容確認用
        self.response_numbers = list(range(1, 19))
        self.final_prev = ""
        random.shuffle(self.response_numbers)

        self.prev_response_filename = ""

        self.prev_send_unity_time = datetime.now()# Unityにリップ・シンク停止信号を以前いつ出したか

        self.system_response_length = 3# システムの応答の音声ファイルの長さ

        self.response_update = False  # ← これを必ず__init__で初期化

        self.prev_bc_time = None  # 前回BackChannel受信時刻

        self.audio_player_path = "/home/DiaROS/DiaROS_deep_model/DiaROS_py/diaros/hai.wav"
        self.last_back_channel_play_time = 0

        sys.stdout.write('DialogManagement start up.\n')
        sys.stdout.write('=====================================================\n')

        # static_response_archive内のwavファイル一覧を取得し、ソートして保存
        self.static_response_files = sorted(
            glob.glob("static_response_archive/static_response_*.wav")
        )
        self.static_response_index = 0

        # ros2_dm.pyから受け取ったデータと受信時刻
        self.latest_tt_data = None
        self.latest_tt_time = None
        self.latest_bc_data = None
        self.latest_bc_time = None
        self.latest_synth_filename = None # 追加: 音声合成ファイル名を保存する変数
        self.latest_dialogue_result = ""  # ★追加: 最新の対話生成結果を保存
        
        # ★対話生成時刻情報を保存する変数
        self.latest_request_id = 0
        self.latest_worker_name = ""
        self.latest_start_timestamp_ns = 0
        self.latest_completion_timestamp_ns = 0
        self.latest_inference_duration_ms = 0.0

        self.prev_asr_you = ""  # 直前のASR結果をインスタンス変数に
        self.last_response_update_asr = ""  # 前回response_updateがTrueになった時のASR結果
        
        # タイミング統合システム初期化
        if TIMING_AVAILABLE:
            self.timing_logger = get_timing_logger()
            self.current_session_id = None
        else:
            self.timing_logger = None
            
        # 各処理段階のタイミング情報
        self.asr_start_ns = 0
        self.asr_completion_ns = 0
        self.tts_start_ns = 0
        self.tts_completion_ns = 0

        # 二段階応答生成用の変数
        self.first_stage_backchannel = ""  # NLG PCから受け取ったfirst_stage相槌
        self.first_stage_backchannel_available = False  # first_stage相槌が利用可能か
        self.waiting_for_second_stage = False  # second_stage応答待ちフラグ
        self.second_stage_request_pending = False  # second_stageリクエスト保留フラグ
        self.second_stage_ready_to_play = False  # second_stage再生準備完了フラグ（合成完了）
        self.turn_taking_decision_timestamp_ns = 0  # TurnTaking判定時刻（ナノ秒） - 分散実行時のNLG連携用
        self.second_stage_wait_start_time = None  # second_stage待機開始時刻
        self.second_stage_timeout_seconds = 5.0  # second_stageタイムアウト秒数
        self.second_stage_timeout_played = False  # タイムアウトエラー既出フラグ
        # ★TurnTaking判定時のASR履歴保存（Second stage用）
        self.asr_history_at_tt_decision = []  # TurnTaking判定時点でのASR履歴を保存（全件）
        self.asr_history_at_tt_decision_2_5s = []  # TurnTaking判定時点での2.5秒間隔ASR結果を保存
        # ★TurnTaking判定時に再生予定の First stage相槌を保存（Second stage用）
        self.first_stage_backchannel_at_tt_decision = ""  # TurnTaking判定時に再生する相槌内容
    
    def calculate_dialogue_timing(self, current_time_ns):
        """対話生成開始・完了からの経過時間を計算"""
        if self.latest_start_timestamp_ns == 0 or self.latest_completion_timestamp_ns == 0:
            return None, None
        
        # ナノ秒からミリ秒に変換
        start_elapsed_ms = (current_time_ns - self.latest_start_timestamp_ns) / 1_000_000
        completion_elapsed_ms = (current_time_ns - self.latest_completion_timestamp_ns) / 1_000_000
        
        return start_elapsed_ms, completion_elapsed_ms

    def run(self):
        prev = ""
        carry = ""
        silent_start_time = datetime.now()
        silent_start_check = False
        silent = False
        allow_silence_seconds = 1
        silent_triggered_response = False
        end_announce_flag = False
        back_channel_reservation = False
        user_spoken = False
        user_speak_start_time = False
        user_pause_end_time = datetime.now()

        turn_taking_delay_start_time = False
        turn_taking_threshold = 0.75 
        turn_taking_response_delay_length = 0.9
        last_handled_tt_time = None
        last_response_end_time = None  # 応答音声再生終了時刻
        is_playing_response = False    # 応答音声再生中フラグ
        last_back_channel_time = 0     # 最後に相槌を打った時刻
        is_playing_backchannel = False # 相槌音声再生中フラグ
        last_backchannel_end_time = None # 相槌音声再生終了時刻
        pending_tt_data = None         # 相槌再生中に来た応答判定を一時保存
        pending_tt_time = None
        next_back_channel_allowed_time = 0  # 次に相槌を打てる時刻
        next_back_channel_after_response = 0  # 応答後に相槌を打てる時刻

        BACK_CHANNEL_HIGH_THRESHOLD = 0.75
        BACK_CHANNEL_LOW_THRESHOLD = 0.60
        back_channel_threshold = BACK_CHANNEL_HIGH_THRESHOLD
        last_handled_bc_time = None
        back_channel_cooldown_length = 0.3  # 相槌クールダウン時間（秒）
        back_channel_cooldown_until = None  # 相槌クールダウン終了時刻

        thread_start_time = datetime.now()

        voice_available = False
        standard_power = 0.0
        power_calibration = True

        DEBUG = True
        
        # メモリクリーンアップカウンター
        cleanup_counter = 0
        CLEANUP_INTERVAL = 10000  # 10000ループごとにクリーンアップ


        BAR_MEM = 20  # バーの長さ
        YELLOW = "\033[33m"
        RESET = "\033[0m"

        while True:
            # 定期的なメモリクリーンアップ
            cleanup_counter += 1
            if cleanup_counter >= CLEANUP_INTERVAL:
                # pygame.mixerのクリーンアップ
                if not pygame.mixer.music.get_busy():
                    pygame.mixer.quit()
                    pygame.mixer.init()
                cleanup_counter = 0
            
            # ここでNLG用にASR結果をwordにセット
            if self.asr["you"]:
                # 前回response_updateがTrueになった時のASR結果と比較
                diff = list(difflib.ndiff(self.last_response_update_asr, self.asr["you"]))
                changed_chars = sum(1 for d in diff if d.startswith('+ ') or d.startswith('- '))
                # 前回response_updateがTrueになった時のASR結果と1文字以上変わった場合のみ判定
                if changed_chars >= 1 and self.asr["you"] != self.last_response_update_asr:
                    self.word = self.asr["you"]
                    self.response_update = True
                    self.last_response_update_asr = self.asr["you"]  # 更新時のASR結果を保存
                    # asr_historyとresponse_updateの値を出力
                    # print(f"[DEBUG] asr_history: {self.asr_history}")
                    # print(f"[DEBUG] response_update: {self.response_update}")
                    # sys.stdout.write(f"ASR結果: {self.asr['you']}\n")  # コメントアウト：頻繁出力を避ける
                    sys.stdout.flush()
                    # asr_historyとresponse_updateの値を出力
                    # print(f"[DEBUG] asr_history: {self.asr_history}")
                    # print(f"[DEBUG] response_update: {self.response_update}")
                self.prev_asr_you = self.asr["you"]  # 直前のASR結果は常に更新

            # TTデータの判定・再生
            if self.latest_tt_data is not None and self.latest_tt_time != last_handled_tt_time:
                # TT判定処理開始タイミング出力
                judgment_start_time = datetime.now()
                judgment_timestamp = judgment_start_time.strftime('%H:%M:%S.%f')[:-3]
                # print(f"[{judgment_timestamp}][DM_run] TT判定処理開始")
                # sys.stdout.flush()
                
                tt_data = self.latest_tt_data
                tt_time = self.latest_tt_time
                probability = float(tt_data.get('confidence', 0.0))
                now = time.time()
                # 相槌音声再生中ならpendingに保存してスキップ
                if is_playing_backchannel and last_backchannel_end_time is not None and now < last_backchannel_end_time:
                    pending_tt_data = tt_data
                    pending_tt_time = tt_time
                    last_handled_tt_time = tt_time
                    continue
                # 応答音声再生中はTTデータを無視
                if is_playing_response and last_response_end_time is not None and now < last_response_end_time:
                    last_handled_tt_time = tt_time
                    continue
                if probability >= turn_taking_threshold:
                    # ★TurnTaking判定時刻を記録（分散実行時のNLG連携用）
                    now_dt = datetime.now()
                    self.turn_taking_decision_timestamp_ns = int(now_dt.timestamp() * 1_000_000_000)
                    timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                    # ★視覚的なマーカーを追加してTurnTaking判定時刻を明確に表示
                    sys.stdout.write(f"\n{'='*70}\n")
                    sys.stdout.write(f"🔊 【TurnTaking 話者交代判定】@ {timestamp}\n")
                    sys.stdout.write(f"{'='*70}\n")
                    sys.stdout.flush()

                    # ★TurnTaking判定時点のASR履歴を保存（Second stage用）
                    self.asr_history_at_tt_decision = [entry["text"] for entry in self.asr_history]
                    sys.stdout.write(f"[DEBUG-TT] ASR履歴を保存: {len(self.asr_history_at_tt_decision)}件\n")
                    sys.stdout.flush()

                    # ★TurnTaking判定時の2.5秒間隔ASR結果を計算して保存
                    self.asr_history_at_tt_decision_2_5s = []
                    if len(self.asr_history) > 0:
                        # 最新のエントリから開始
                        latest_entry = self.asr_history[-1]
                        self.asr_history_at_tt_decision_2_5s.append(latest_entry["text"])
                        current_timestamp_ns = latest_entry["timestamp_ns"]

                        # 2.5秒間隔で過去に遡る
                        interval_ns = 2_500_000_000  # 2.5秒 = 2,500,000,000ナノ秒
                        while True:
                            target_timestamp_ns = current_timestamp_ns - interval_ns
                            # target_timestamp_nsに最も近い過去のエントリを探す
                            closest_entry = None
                            closest_diff = float('inf')

                            for entry in self.asr_history:
                                if entry["timestamp_ns"] <= target_timestamp_ns:
                                    diff = target_timestamp_ns - entry["timestamp_ns"]
                                    if diff < closest_diff:
                                        closest_diff = diff
                                        closest_entry = entry

                            # 見つからない場合は最も古いエントリを採用
                            if closest_entry is None:
                                if len(self.asr_history) > 1:
                                    oldest_entry = self.asr_history[0]
                                    self.asr_history_at_tt_decision_2_5s.append(oldest_entry["text"])
                                break
                            else:
                                self.asr_history_at_tt_decision_2_5s.append(closest_entry["text"])
                                current_timestamp_ns = closest_entry["timestamp_ns"]

                        # 古いもの→新しいものの順に並べ替え
                        self.asr_history_at_tt_decision_2_5s.reverse()
                        sys.stdout.write(f"[DEBUG-TT] 2.5秒間隔ASR結果を保存: {len(self.asr_history_at_tt_decision_2_5s)}件\n")
                        sys.stdout.flush()

                    # ★修正：Second stageリクエストフラグを設定（First stage再生前に設定）
                    # これにより、First stage の再生と並行して Second stage の生成が開始される
                    self.second_stage_request_pending = True
                    self.waiting_for_second_stage = True
                    timestamp_tt = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[TT] Second stage リクエスト処理開始（First stage再生と並行） @ {timestamp_tt}\n")
                    sys.stdout.flush()

                    # First stage相槌を再生（準備がある場合）
                    if self.first_stage_backchannel_available and self.first_stage_backchannel:
                        # ★修正：TurnTaking判定時に再生予定の相槌を保存（Second stage用）
                        self.first_stage_backchannel_at_tt_decision = self.first_stage_backchannel
                        sys.stdout.write(f"[TT] First stage相槌再生: '{self.first_stage_backchannel}' (TT判定時相槌として保存)\n")
                        sys.stdout.flush()

                        # ★事前合成済みのfirst_stageファイルがあれば使用、なければ合成
                        if hasattr(self, 'first_stage_backchannel_wav') and os.path.exists(self.first_stage_backchannel_wav):
                            first_stage_wav_path = self.first_stage_backchannel_wav
                            now = datetime.now()
                            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
                            sys.stdout.write(f"[TT] 事前合成済みのfirst_stage音声を使用 @ {timestamp}\n")
                            sys.stdout.flush()
                        else:
                            # 合成済みファイルがない場合は新規合成
                            first_stage_wav_path = self.synthesize_first_stage_backchannel(self.first_stage_backchannel)

                        if first_stage_wav_path and os.path.exists(first_stage_wav_path):
                            # 音声ファイル長を取得
                            try:
                                first_stage_audio = AudioSegment.from_wav(first_stage_wav_path)
                                first_stage_duration_sec = len(first_stage_audio) / 1000.0
                            except Exception as e:
                                sys.stdout.write(f"[ERROR] First stage音声ファイル長取得エラー: {e}\n")
                                sys.stdout.flush()
                                first_stage_duration_sec = 0.5  # デフォルト値

                            # ★時刻を記録
                            now = datetime.now()
                            timestamp = now.strftime('%H:%M:%S.%f')[:-3]

                            # ブロッキング再生（相槌が終わるまで待つ）
                            sys.stdout.write(f"[TT] First stage相槌再生開始: {first_stage_wav_path} @ {timestamp}\n")
                            sys.stdout.flush()
                            self.play_sound(first_stage_wav_path, block=True)

                            now_end = datetime.now()
                            timestamp_end = now_end.strftime('%H:%M:%S.%f')[:-3]
                            sys.stdout.write(f"[TT] First stage相槌再生完了 @ {timestamp_end} (長さ: {first_stage_duration_sec:.2f}秒)\n")
                            sys.stdout.flush()
                        else:
                            sys.stdout.write(f"[ERROR] First stage相槌音声ファイルエラー、スキップします\n")
                            sys.stdout.flush()

                        # ★Second stage本応答の再生チェック（First stage再生完了直後）
                        # 再生準備ができていれば、すぐに再生
                        if hasattr(self, 'second_stage_ready_to_play') and self.second_stage_ready_to_play and hasattr(self, 'latest_synth_filename') and self.latest_synth_filename and os.path.exists(self.latest_synth_filename):
                            second_stage_wav_path = self.latest_synth_filename
                            now = datetime.now()
                            timestamp = now.strftime('%H:%M:%S.%f')[:-3]

                            try:
                                # 音声ファイル長を取得
                                second_stage_audio = AudioSegment.from_wav(second_stage_wav_path)
                                second_stage_duration_sec = len(second_stage_audio) / 1000.0

                                # ブロッキング再生（本応答が終わるまで待つ）
                                sys.stdout.write(f"[TT] Second stage本応答再生開始: {second_stage_wav_path} @ {timestamp}\n")
                                sys.stdout.flush()
                                self.play_sound(second_stage_wav_path, block=True)

                                now_end = datetime.now()
                                timestamp_end = now_end.strftime('%H:%M:%S.%f')[:-3]
                                sys.stdout.write(f"[TT] Second stage本応答再生完了 @ {timestamp_end} (長さ: {second_stage_duration_sec:.2f}秒)\n")
                                sys.stdout.flush()

                            except Exception as e:
                                sys.stdout.write(f"[ERROR] Second stage本応答の再生エラー: {e}\n")
                                sys.stdout.flush()
                            finally:
                                # 再生準備フラグをクリア
                                self.second_stage_ready_to_play = False
                                self.waiting_for_second_stage = False
                                self.latest_synth_filename = ""

                        # First stage相槌をリセット
                        self.first_stage_backchannel_available = False

                        # ★Second stage待機開始時刻を記録（タイムアウト検出用）
                        self.second_stage_wait_start_time = datetime.now()
                        self.second_stage_timeout_played = False

                    self.asr_history = []  # ★TT応答再生直後のみ履歴を初期化

            # ★Second stage生成タイムアウト検出
            if self.waiting_for_second_stage and self.second_stage_wait_start_time is not None and not self.second_stage_timeout_played:
                elapsed_time = (datetime.now() - self.second_stage_wait_start_time).total_seconds()
                if elapsed_time >= self.second_stage_timeout_seconds:
                    # タイムアウト発生 → エラー音声を再生
                    sys.stdout.write(f"\n[WARNING] Second stage生成タイムアウト検出 (待機時間: {elapsed_time:.1f}秒)\n")
                    sys.stdout.flush()
                    self.play_error_audio('timeout')
                    self.second_stage_timeout_played = True
                    self.waiting_for_second_stage = False
                    self.second_stage_wait_start_time = None

                    # Second stage待機をリセット
                    self.latest_synth_filename = ""

                    # 次のリクエストを受け入れられるように初期化
                    self.asr_history = []

                    # Second stage本応答が準備できたら再生
                    if hasattr(self, 'latest_synth_filename') and self.latest_synth_filename and os.path.exists(self.latest_synth_filename) and not self.waiting_for_second_stage:
                        wav_path = self.latest_synth_filename
                        try:
                            audio = AudioSegment.from_wav(wav_path)
                            duration_sec = len(audio) / 1000.0
                        except Exception as e:
                            print(f"[ERROR] 合成音声ファイル読み込みエラー: {e}")
                            duration_sec = 2.0
                        # ★応答音声再生時の詳細タイミング分析
                        now_dt = datetime.now()
                        timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                        current_time_ns = int(now_dt.timestamp() * 1_000_000_000)
                        
                        # タイミングログに音声再生開始を記録
                        if TIMING_AVAILABLE and self.timing_logger and self.current_session_id:
                            self.timing_logger.log_event(
                                session_id=self.current_session_id,
                                event_type="audio_playback_start",
                                timestamp_ns=current_time_ns,
                                data={
                                    "filename": wav_path,
                                    "duration_sec": duration_sec,
                                    "request_id": self.latest_request_id
                                }
                            )
                        
                        sys.stdout.write(f"\n{'='*50}\n")
                        sys.stdout.write(f"[{timestamp}] 🔊 応答音声再生開始\n")
                        sys.stdout.write(f"{'='*50}\n")
                        
                        # 音声合成処理の詳細情報
                        if self.latest_start_timestamp_ns > 0 and self.latest_completion_timestamp_ns > 0:
                            # ナノ秒から時刻への変換関数
                            def ns_to_readable_time(ns_timestamp):
                                if ns_timestamp <= 0:
                                    return "未設定"
                                dt = datetime.fromtimestamp(ns_timestamp / 1_000_000_000)
                                return dt.strftime('%H:%M:%S.%f')[:-3]  # ミリ秒まで表示
                            
                            # 各処理の完了時刻を人間が読みやすい形式で表示
                            sys.stdout.write(f"📊 各処理完了時刻:\n")
                            if self.asr_completion_ns > 0:
                                sys.stdout.write(f"  • ASR処理完了:     {ns_to_readable_time(self.asr_completion_ns)}\n")
                            sys.stdout.write(f"  • NLG処理開始:     {ns_to_readable_time(self.latest_start_timestamp_ns)}\n")
                            sys.stdout.write(f"  • NLG処理完了:     {ns_to_readable_time(self.latest_completion_timestamp_ns)}\n")
                            # TTS完了時刻のデバッグ出力
                            tts_completion_val = getattr(self, 'tts_completion_ns', 0)
                            if tts_completion_val > 0:
                                sys.stdout.write(f"  • TTS処理完了:     {ns_to_readable_time(tts_completion_val)}\n")
                            else:
                                # デバッグ：なぜTTS完了時刻が設定されていないかを確認
                                sys.stdout.write(f"  • TTS処理完了:     未設定 (値: {tts_completion_val})\n")
                            sys.stdout.write(f"  • 音声再生開始:     {ns_to_readable_time(current_time_ns)}\n")
                            
                            # 各処理にかかった時間
                            asr_processing_time = (self.asr_completion_ns - self.asr_start_ns) / 1_000_000 if self.asr_start_ns > 0 and self.asr_completion_ns > 0 else 0
                            nlg_processing_time = (self.latest_completion_timestamp_ns - self.latest_start_timestamp_ns) / 1_000_000
                            tts_processing_time = (self.tts_completion_ns - self.tts_start_ns) / 1_000_000 if self.tts_start_ns > 0 and self.tts_completion_ns > 0 else 0
                            # TTS処理時間が取得できない場合、合成→再生時間をTTS処理時間として使用
                            if tts_processing_time == 0:
                                tts_processing_time = (current_time_ns - self.latest_completion_timestamp_ns) / 1_000_000
                            total_response_time = nlg_processing_time + tts_processing_time
                            synthesis_to_playback = (current_time_ns - self.latest_completion_timestamp_ns) / 1_000_000
                            
                            sys.stdout.write(f"\n⏱️  各処理にかかった時間:\n")
                            if asr_processing_time > 0:
                                sys.stdout.write(f"  • ASR処理時間:     {asr_processing_time:.1f}ms\n")
                            sys.stdout.write(f"  • NLG処理時間:     {nlg_processing_time:.1f}ms\n")
                            sys.stdout.write(f"  • TTS処理時間:     {tts_processing_time:.1f}ms\n")
                            sys.stdout.write(f"  • 総応答時間:      {total_response_time:.1f}ms (NLG+TTS)\n")
                            
                            # 処理詳細情報
                            sys.stdout.write(f"\n📋 処理詳細:\n")
                            sys.stdout.write(f"  • Request ID:      {self.latest_request_id}\n")
                            sys.stdout.write(f"  • Worker:          {self.latest_worker_name}\n")
                            sys.stdout.write(f"  • 音声長:          {duration_sec:.1f}秒\n")
                            
                            # パフォーマンス評価
                            if total_response_time <= 1000:
                                perf_status = "🟢 優秀"
                            elif total_response_time <= 1500:
                                perf_status = "🟡 良好"
                            else:
                                perf_status = "🔴 要改善"
                            sys.stdout.write(f"  • 応答性能:        {perf_status} ({total_response_time:.1f}ms)\n")
                            
                            # タイミングログファイルにも詳細情報を出力
                            log_file_path = f"/tmp/diaros_timing/timing_{self.current_session_id if TIMING_AVAILABLE and self.timing_logger and self.current_session_id else int(time.time())}.log"
                            try:
                                os.makedirs("/tmp/diaros_timing", exist_ok=True)
                                with open(log_file_path, "a", encoding="utf-8") as f:
                                    f.write(f"\n{'='*60}\n")
                                    f.write(f"[{timestamp}] 🔊 応答音声再生開始\n")
                                    f.write(f"{'='*60}\n")
                                    f.write(f"📊 各処理完了時刻:\n")
                                    if self.asr_completion_ns > 0:
                                        f.write(f"  • ASR処理完了:     {ns_to_readable_time(self.asr_completion_ns)}\n")
                                    f.write(f"  • NLG処理開始:     {ns_to_readable_time(self.latest_start_timestamp_ns)}\n")
                                    f.write(f"  • NLG処理完了:     {ns_to_readable_time(self.latest_completion_timestamp_ns)}\n")
                                    if tts_completion_val > 0:
                                        f.write(f"  • TTS処理完了:     {ns_to_readable_time(tts_completion_val)}\n")
                                    f.write(f"  • 音声再生開始:     {ns_to_readable_time(current_time_ns)}\n")
                                    f.write(f"\n⏱️  各処理にかかった時間:\n")
                                    if asr_processing_time > 0:
                                        f.write(f"  • ASR処理時間:     {asr_processing_time:.1f}ms\n")
                                    f.write(f"  • NLG処理時間:     {nlg_processing_time:.1f}ms\n")
                                    f.write(f"  • TTS処理時間:     {tts_processing_time:.1f}ms\n")
                                    f.write(f"  • 総応答時間:      {total_response_time:.1f}ms (NLG+TTS)\n")
                                    f.write(f"\n📋 処理詳細:\n")
                                    f.write(f"  • Request ID:      {self.latest_request_id}\n")
                                    f.write(f"  • Worker:          {self.latest_worker_name}\n")
                                    f.write(f"  • 音声長:          {duration_sec:.1f}秒\n")
                                    f.write(f"  • 応答性能:        {perf_status} ({total_response_time:.1f}ms)\n")
                                    f.write(f"{'='*60}\n\n")
                            except Exception as e:
                                sys.stdout.write(f"[ERROR] ログファイル書き込みエラー: {e}\n")
                        else:
                            sys.stdout.write(f"⚠️  タイミング情報が不完全です\n")
                            sys.stdout.write(f"  • 音声長:         {duration_sec:.1f}秒\n")
                        
                        sys.stdout.write(f"{'='*50}\n")
                        sys.stdout.flush()
                        # ...existing code...
                        self.play_sound(wav_path, False)  # ノンブロッキング再生
                        last_response_end_time = time.time() + duration_sec
                        is_playing_response = True
                        next_back_channel_after_response = last_response_end_time + back_channel_cooldown_length
                        self.latest_synth_filename = ""
                    else:
                        # 合成音声ファイルがない場合のデバッグ情報
                        sys.stdout.write(f"[WARNING] 合成音声ファイルが利用できません。再生をスキップします\n")
                        sys.stdout.write(f"  latest_synth_filename: '{getattr(self, 'latest_synth_filename', 'None')}'\n")
                        if hasattr(self, 'latest_synth_filename') and self.latest_synth_filename:
                            sys.stdout.write(f"  ファイル存在確認: {os.path.exists(self.latest_synth_filename)}\n")
                        sys.stdout.write(f"[INFO] 第2段階の音声合成待機中か、合成に失敗しています\n")
                        sys.stdout.flush()
                last_handled_tt_time = tt_time
            # 応答音声再生終了後にフラグをリセット
            if is_playing_response and last_response_end_time is not None and time.time() >= last_response_end_time:
                is_playing_response = False
                last_response_end_time = None

            # 相槌音声再生終了後にpendingしていた応答判定があれば処理
            if is_playing_backchannel and last_backchannel_end_time is not None and time.time() >= last_backchannel_end_time:
                is_playing_backchannel = False
                last_backchannel_end_time = None
                if pending_tt_data is not None:
                    probability = float(pending_tt_data.get('confidence', 0.0))
                    now = time.time()
                    if not (is_playing_response and last_response_end_time is not None and now < last_response_end_time):
                        if probability >= turn_taking_threshold:
                            if hasattr(self, 'latest_synth_filename') and self.latest_synth_filename and os.path.exists(self.latest_synth_filename):
                                wav_path = self.latest_synth_filename
                                try:
                                    audio = AudioSegment.from_wav(wav_path)
                                    duration_sec = len(audio) / 1000.0
                                except Exception as e:
                                    print(f"[ERROR] 合成音声ファイル読み込みエラー（pending）: {e}")
                                    duration_sec = 2.0
                                # ★応答音声再生時の詳細タイミング分析（pending処理）
                                now_dt = datetime.now()
                                timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
                                current_time_ns = int(now_dt.timestamp() * 1_000_000_000)
                                
                                # タイミングログに音声再生開始を記録
                                if TIMING_AVAILABLE and self.timing_logger and self.current_session_id:
                                    self.timing_logger.log_event(
                                        session_id=self.current_session_id,
                                        event_type="audio_playback_start",
                                        timestamp_ns=current_time_ns,
                                        data={
                                            "filename": wav_path,
                                            "duration_sec": duration_sec,
                                            "request_id": self.latest_request_id,
                                            "pending": True
                                        }
                                    )
                                
                                sys.stdout.write(f"\n{'='*50}\n")
                                sys.stdout.write(f"[{timestamp}] 🔊 応答音声再生開始（相槌後処理）\n")
                                sys.stdout.write(f"{'='*50}\n")
                                
                                # 音声合成処理の詳細情報
                                if self.latest_start_timestamp_ns > 0 and self.latest_completion_timestamp_ns > 0:
                                    # ナノ秒から時刻への変換関数
                                    def ns_to_readable_time(ns_timestamp):
                                        if ns_timestamp <= 0:
                                            return "未設定"
                                        dt = datetime.fromtimestamp(ns_timestamp / 1_000_000_000)
                                        return dt.strftime('%H:%M:%S.%f')[:-3]  # ミリ秒まで表示
                                    
                                    # 各処理の完了時刻を人間が読みやすい形式で表示
                                    sys.stdout.write(f"📊 各処理完了時刻:\n")
                                    if self.asr_completion_ns > 0:
                                        sys.stdout.write(f"  • ASR処理完了:     {ns_to_readable_time(self.asr_completion_ns)}\n")
                                    sys.stdout.write(f"  • NLG処理開始:     {ns_to_readable_time(self.latest_start_timestamp_ns)}\n")
                                    sys.stdout.write(f"  • NLG処理完了:     {ns_to_readable_time(self.latest_completion_timestamp_ns)}\n")
                                    if self.tts_completion_ns > 0:
                                        sys.stdout.write(f"  • TTS処理完了:     {ns_to_readable_time(self.tts_completion_ns)}\n")
                                    sys.stdout.write(f"  • 音声再生開始:     {ns_to_readable_time(current_time_ns)}\n")
                                    
                                    # 各処理にかかった時間
                                    asr_processing_time = (self.asr_completion_ns - self.asr_start_ns) / 1_000_000 if self.asr_start_ns > 0 and self.asr_completion_ns > 0 else 0
                                    nlg_processing_time = (self.latest_completion_timestamp_ns - self.latest_start_timestamp_ns) / 1_000_000
                                    tts_processing_time = (self.tts_completion_ns - self.tts_start_ns) / 1_000_000 if self.tts_start_ns > 0 and self.tts_completion_ns > 0 else 0
                                    # TTS処理時間が取得できない場合、合成→再生時間をTTS処理時間として使用
                                    if tts_processing_time == 0:
                                        tts_processing_time = (current_time_ns - self.latest_completion_timestamp_ns) / 1_000_000
                                    total_response_time = nlg_processing_time + tts_processing_time  # 音声再生時間を除外
                                    synthesis_to_playback = (current_time_ns - self.latest_completion_timestamp_ns) / 1_000_000
                                    
                                    sys.stdout.write(f"\n⏱️  各処理にかかった時間:\n")
                                    if asr_processing_time > 0:
                                        sys.stdout.write(f"  • ASR処理時間:     {asr_processing_time:.1f}ms\n")
                                    sys.stdout.write(f"  • NLG処理時間:     {nlg_processing_time:.1f}ms\n")
                                    sys.stdout.write(f"  • TTS処理時間:     {tts_processing_time:.1f}ms\n")
                                    sys.stdout.write(f"  • 総応答時間:      {total_response_time:.1f}ms (NLG+TTS)\n")
                                    
                                    # 処理詳細情報
                                    sys.stdout.write(f"\n📋 処理詳細:\n")
                                    sys.stdout.write(f"  • Request ID:      {self.latest_request_id}\n")
                                    sys.stdout.write(f"  • Worker:          {self.latest_worker_name}\n")
                                    sys.stdout.write(f"  • 音声長:          {duration_sec:.1f}秒\n")
                                    
                                    # パフォーマンス評価
                                    if total_response_time <= 1000:
                                        perf_status = "🟢 優秀"
                                    elif total_response_time <= 1500:
                                        perf_status = "🟡 良好"
                                    else:
                                        perf_status = "🔴 要改善"
                                    sys.stdout.write(f"  • 応答性能:        {perf_status} ({total_response_time:.1f}ms)\n")
                                    
                                    # タイミングログファイルにも詳細情報を出力
                                    if TIMING_AVAILABLE and self.timing_logger:
                                        log_file_path = f"/tmp/diaros_timing/timing_{self.current_session_id}.log"
                                        try:
                                            os.makedirs("/tmp/diaros_timing", exist_ok=True)
                                            with open(log_file_path, "a", encoding="utf-8") as f:
                                                f.write(f"\n{'='*60}\n")
                                                f.write(f"[{timestamp}] 🔊 応答音声再生開始（相槌後処理）\n")
                                                f.write(f"{'='*60}\n")
                                                f.write(f"📊 各処理完了時刻:\n")
                                                if asr_completion_ms > 0:
                                                    f.write(f"  • ASR処理完了:     {asr_completion_ms:.1f}ms\n")
                                                f.write(f"  • NLG処理開始:     {nlg_start_ms:.1f}ms\n")
                                                f.write(f"  • NLG処理完了:     {nlg_completion_ms:.1f}ms\n")
                                                if tts_completion_ms > 0:
                                                    f.write(f"  • TTS処理完了:     {tts_completion_ms:.1f}ms\n")
                                                f.write(f"  • 音声再生開始:     {playback_start_ms:.1f}ms\n")
                                                f.write(f"\n⏱️  各処理にかかった時間:\n")
                                                if asr_processing_time > 0:
                                                    f.write(f"  • ASR処理時間:     {asr_processing_time:.1f}ms\n")
                                                f.write(f"  • NLG処理時間:     {nlg_processing_time:.1f}ms\n")
                                                if tts_processing_time > 0:
                                                    f.write(f"  • TTS処理時間:     {tts_processing_time:.1f}ms\n")
                                                f.write(f"  • 合成→再生時間:   {synthesis_to_playback:.1f}ms\n")
                                                f.write(f"  • 総応答時間:      {total_response_time:.1f}ms\n")
                                                f.write(f"\n📋 処理詳細:\n")
                                                f.write(f"  • Request ID:      {self.latest_request_id}\n")
                                                f.write(f"  • Worker:          {self.latest_worker_name}\n")
                                                f.write(f"  • 音声長:          {duration_sec:.1f}秒\n")
                                                f.write(f"  • 応答性能:        {perf_status} ({total_response_time:.1f}ms)\n")
                                                f.write(f"{'='*60}\n\n")
                                        except Exception as e:
                                            sys.stdout.write(f"[ERROR] ログファイル書き込みエラー: {e}\n")
                                else:
                                    sys.stdout.write(f"⚠️  タイミング情報が不完全です\n")
                                    sys.stdout.write(f"  • 音声長:         {duration_sec:.1f}秒\n")
                                
                                sys.stdout.write(f"{'='*50}\n")
                                sys.stdout.flush()
                                # ...existing code...
                                self.play_sound(wav_path, False)  # ノンブロッキング再生
                                self.asr_history = []  # ★TT応答再生時のみ履歴を初期化
                                self.latest_synth_filename = ""
                                last_response_end_time = time.time() + duration_sec
                                is_playing_response = True
                                next_back_channel_after_response = last_response_end_time + back_channel_cooldown_length
                            elif self.static_response_files:
                                wav_path = self.static_response_files[self.static_response_index]
                                try:
                                    audio = AudioSegment.from_wav(wav_path)
                                    duration_sec = len(audio) / 1000.0
                                except Exception:
                                    duration_sec = 2.0
                                sys.stdout.write(f"[TT] 再生音声長 duration_sec={duration_sec}\n")
                                sys.stdout.flush()
                                
                                # ★応答音声再生時刻と対話生成結果を出力（静的応答）
                                now = datetime.now()
                                timestamp = now.strftime('%H:%M:%S.%f')[:-3]
                                current_time_ns = int(now.timestamp() * 1_000_000_000)
                                
                                sys.stdout.write(f"[{timestamp}][音声再生開始] {wav_path}\n")
                                if hasattr(self, 'latest_dialogue_result') and self.latest_dialogue_result:
                                    sys.stdout.write(f"[{timestamp}][対話内容] {self.latest_dialogue_result}\n")
                                    # ★対話生成時刻との差分を計算・出力（対話生成結果がある場合のみ）
                                    start_elapsed_ms, completion_elapsed_ms = self.calculate_dialogue_timing(current_time_ns)
                                    if start_elapsed_ms is not None and completion_elapsed_ms is not None:
                                        sys.stdout.write(f"[{timestamp}][タイミング分析] 対話生成開始から{start_elapsed_ms:.1f}ms, 完了から{completion_elapsed_ms:.1f}ms経過\n")
                                        sys.stdout.write(f"[{timestamp}][対話生成情報] ID:{self.latest_request_id}, Worker:{self.latest_worker_name}, 推論時間:{self.latest_inference_duration_ms:.1f}ms\n")
                                else:
                                    sys.stdout.write(f"[{timestamp}][対話内容] （静的応答）\n")
                                sys.stdout.flush()
                                
                                self.play_sound(wav_path, False)  # ノンブロッキング再生
                                self.asr_history = []  # ★TT応答再生直後のみ履歴を初期化
                                self.static_response_index += 1
                                if self.static_response_index >= len(self.static_response_files):
                                    self.static_response_index = 0
                                last_response_end_time = time.time() + duration_sec
                                is_playing_response = True
                                next_back_channel_after_response = last_response_end_time + back_channel_cooldown_length
                            else:
                                sys.stdout.write("[ERROR] static_response_archiveに音声ファイルがありません\n")
                    pending_tt_data = None
                    pending_tt_time = None

            # BCデータの判定・再生
            if self.latest_bc_data is not None and self.latest_bc_time != last_handled_bc_time:
                bc_data = self.latest_bc_data
                bc_time = self.latest_bc_time
                now = time.time()
                probability = float(bc_data.get('confidence', 0.0))
                # 応答音声再生直後のクールダウン or 直近の相槌から相槌音声長+cooldown秒未満は相槌を打たない
                if (now < next_back_channel_after_response) or \
                   (now < next_back_channel_allowed_time) or is_playing_backchannel:
                    last_handled_bc_time = bc_time
                    continue
                if probability >= back_channel_threshold:
                    try:
                        wav_path = f"static_back_channel_{random.randint(1, 2)}.wav"
                        audio = AudioSegment.from_wav(wav_path)
                        duration_sec = len(audio) / 1000.0
                        self.play_sound(wav_path, False)  # ノンブロッキング相槌再生
                        last_back_channel_time = time.time()
                        is_playing_backchannel = True
                        last_backchannel_end_time = last_back_channel_time + duration_sec
                        # 相槌音声の長さ+クールダウンだけ次の相槌を禁止
                        next_back_channel_allowed_time = last_back_channel_time + duration_sec + back_channel_cooldown_length
                    except Exception as e:
                        sys.stdout.write(f"\n[ERROR] 相槌音声再生失敗: {e}\n")
                        sys.stdout.flush()
                last_handled_bc_time = bc_time

            #現在の時刻をmsまで表示
            # if DEBUG:sys.stdout.write("ループタイミング："+datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f')[:-3])
            # self.sa["power"]を表示
                                    
            ### パワーによる無声区間検出 ###
            # 声を張って話すとパワーが0.69ぐらい
            # ぼそぼそ話すとパワーが0.36ぐらい
            # 動画のパワーが0.046ぐらい
            # キャリブレーション用の音声の返しが0.032
            # 会場の環境音は0.06
            
            if power_calibration:
                # if DEBUG:sys.stdout.write("\n"+f"power: {self.sa['power']}")
                # if DEBUG:sys.stdout.write("\n"+f"standard_power: {standard_power}")
                if DEBUG:sys.stdout.flush()
                
                self.power_calib_list.append(self.sa["power"])
                # メモリリーク防止: キャリブレーションリストを制限
                if len(self.power_calib_list) > 200:
                    self.power_calib_list.pop(0)
                time_difference = datetime.now() - thread_start_time
                if time_difference >= timedelta(seconds=2.0):
                    self.power_calib_ave = statistics.mean(self.power_calib_list)
                    standard_power = self.power_calib_ave * 8
                    power_calibration = False
                    if DEBUG:sys.stdout.write("\n"+f"power: {self.sa['power']}\n")
                    if DEBUG:sys.stdout.write("\n"+f"standard_power: {standard_power}\n")
                    if DEBUG:sys.stdout.flush()
            else:
                standard_power = 0.20

            # system_response_length秒以上時間が経過していたら
            if self.sa["power"] < standard_power:
                # if DEBUG:sys.stdout.write('\r'+f"無声")
                # if DEBUG:sys.stdout.flush()
                # voice_available = False
                user_speak_start_time = False
                user_pause_end_time = datetime.now()
            else:
                # if DEBUG:sys.stdout.write('\r'+f"有声")
                # if DEBUG:sys.stdout.flush()
                # voice_available = True
                time_difference = datetime.now() - user_pause_end_time
                if time_difference >= timedelta(seconds=0.2):# ユーザ発話が0.5秒以上のとき
                    # Unityに応答停止信号を送信# デバッグ中
                    # if DEBUG:sys.stdout.write('\r'+f"Unityに応答停止信号を送信")
                    # if DEBUG:sys.stdout.flush()
                    # dummy_signal = "STOP"
                    # client.sendto(dummy_signal.encode('utf-8'),(HOST,PORT))
                    pass
                time_difference = datetime.now() - self.prev_response_time                            

            #1msごとの過去200msのパワーの平均を出す
            time_difference = datetime.now() - self.prev_power_get_time
            if time_difference >= timedelta(seconds=0.001):
                self.prev_power_get_time = datetime.now()
                # 変数power_aveに過去20回のself.sa["power"]の平均値を保存していく
                
                # self.power_listの最初の要素を削除する
                self.power_list.append(self.sa["power"])
                if len(self.power_list) > 200:  # 要素数が200を超えていたら
                    self.power_list.pop(0)  # 最初の要素を削除
                self.power_ave = statistics.mean(self.power_list)  # 全要素の平均値を計算
            if self.power_ave > standard_power:
                # user_spoken = True # ユーザが一度話したことを記録
                # sys.stdout.write('\n'+f"user_spoken:{user_spoken}")
                voice_available = True
                # sys.stdout.write('\n'+f"voice_available:{voice_available}")
                silent_start_time = datetime.now() # 有声である限り無声区間の開始時刻を更新し続ける
                # if DEBUG:sys.stdout.write('\n'+f"Unityに応答停止信号を送信")
                # # if DEBUG:sys.stdout.flush()

                # time_difference = datetime.now() - self.prev_send_unity_time
                # if time_difference >= timedelta(seconds=0.16):
                #     self.prev_send_unity_time = datetime.now()
                #     dummy_signal = "STOP"
                #     client.sendto(dummy_signal.encode('utf-8'),(HOST,PORT))
            else:
                voice_available = False
            time_difference = datetime.now() - silent_start_time

            if self.additional_asr_start_time == False and voice_available == False and user_spoken == True and time_difference >= timedelta(seconds=1.5):# ユーザが過去に一度話していて、現在は黙っていて、1.5s無声のとき
                time_difference = datetime.now() - self.prev_response_time
                if time_difference >= timedelta(seconds=self.system_response_length + 1.0): # システムが話し終わるまで応答しない
                    # if DEBUG:sys.stdout.write('\n'+f"1.5秒の無音で応答した時刻{datetime.now()}\n")
                    # if DEBUG:sys.stdout.flush()
                    
                    # ./tmp/ ディレクトリ内の .wav ファイルを名前順にソート
                    filenames = sorted(glob.glob("./tmp/*.wav"))

                    # 名前順で最新のファイル名を取得
                    latest_filename = filenames[-1] if filenames else ""
                    # ★デバッグモード：DEBUG_DM_AUDIOがtrueの場合のみ表示
                    debug_dm_audio = os.environ.get('DEBUG_DM_AUDIO', '').lower() == 'true'
                    if debug_dm_audio:
                        sys.stdout.write('\n最新の音声ファイル名' + latest_filename +  '\n')
                        sys.stdout.write('\n前回の音声ファイル名' + self.prev_response_filename +  '\n')
                        sys.stdout.flush()


                    # 最新のファイル名が self.prev_response_filename と異なる場合に限り、そのファイル名を出力
                    if latest_filename != self.prev_response_filename:
                        self.prev_response_filename = latest_filename
                        sys.stdout.write('\n1.5秒無音' + latest_filename + '\n')
                        # filenameのファイルが存在すればファイルを開く
                        try:
                            with open(latest_filename, 'r'):
                                # client.sendto(latest_filename.encode('utf-8'),(HOST,PORT))
                                self.system_response_length = self.get_audio_length(latest_filename)
                                self.additional_asr_start_time = False
                                self.response_cnt = self.response_cnt + 1
                                prev = self.asr["you"] # システムが応答・相槌を返答する
                                carry = ""
                                self.prev_response_time = datetime.now()
                                silent_start_time = datetime.now()
                                user_spoken = False
                                user_speak_start_time = False
                        except FileNotFoundError:
                            pass
                    else:
                        self.additional_asr_start_time = datetime.now()
                        sys.stdout.write('\nadditional start' + '\n')
                        # if os.path.exists("additional_asr_response.wav"):
                        #     self.play_sound("additional_asr_response.wav", False)
                        # print(f"The length of the audio file is {self.system_response_length} seconds.")
                    
            time_difference = datetime.now() - self.prev_response_time
            if self.additional_asr_start_time == False and time_difference >= timedelta(seconds=self.system_response_length + 1.0) and prev != self.asr["you"] and self.asr["is_final"]: # 音声認識結果で発話の同定を行った上でAPIが発話終了判定を出したとき
                # if DEBUG:sys.stdout.write("\n"+f"APIの発話終了判定で応答を返す\n")
                # if DEBUG:sys.stdout.flush()
                prev = self.asr["you"] # システムが応答・相槌を返答する
                carry = ""
                self.prev_response_time = datetime.now()
                # ./tmp/ ディレクトリ内の .wav ファイルを名前順にソート
                filenames = sorted(glob.glob("./tmp/*.wav"))

                # 名前順で最新のファイル名を取得
                latest_filename = filenames[-1] if filenames else ""
                # ★デバッグモード：DEBUG_DM_AUDIOがtrueの場合のみ表示
                debug_dm_audio = os.environ.get('DEBUG_DM_AUDIO', '').lower() == 'true'
                if debug_dm_audio:
                    sys.stdout.write('\n最新の音声ファイル名' + latest_filename +  '\n')
                    sys.stdout.write('\n前回の音声ファイル名' + self.prev_response_filename +  '\n')
                    sys.stdout.flush()

                # 最新のファイル名が self.prev_response_filename と異なる場合に限り、そのファイル名を出力
                if latest_filename != self.prev_response_filename:
                    self.prev_response_filename = latest_filename

                    # ★デバッグモード：Unityに応答の信号を送信（デバッグ表示）
                    if debug_dm_audio:
                        sys.stdout.write('\napiで応答' + latest_filename + '\n')
                    # dummy_signalのファイルが存在するか確認
                    try:
                        with open(latest_filename, 'r'):
                            # client.sendto(latest_filename.encode('utf-8'),(HOST,PORT))
                            self.system_response_length = self.get_audio_length(latest_filename)
                            self.additional_asr_start_time = False
                            self.response_cnt = self.response_cnt + 1
                            # print(f"The length of the audio file is {self.system_response_length} seconds.")
                            silent_start_time = datetime.now()
                            user_spoken = False
                            user_speak_start_time = False

                    except FileNotFoundError:
                        pass
                        # if os.path.exists("additional_asr_response.wav"):
                        #     self.play_sound("additional_asr_response.wav", False)
                else:
                    self.additional_asr_start_time = datetime.now()
                    sys.stdout.write('\nadditional start' + '\n')

    # 応答・相槌が切り替わらなくとも対話管理をさせる
    def pubDM(self):
        """NLGへfirst_stage（相槌生成）リクエストを送信"""
        if self.response_update is True:
            self.response_update = False
            # asr_historyとresponse_updateの値を出力
            # print(f"[DEBUG] asr_history: {self.asr_history}")
            # print(f"[DEBUG] response_update: {self.response_update}")

            # 任意の秒数間隔でタイムスタンプベース選択
            words = []
            if len(self.asr_history) > 0:
                # 最新のエントリから開始
                latest_entry = self.asr_history[-1]
                words.append(latest_entry["text"])
                current_timestamp_ns = latest_entry["timestamp_ns"]

                # 音声認識結果のリスト作成時に遡る間隔（ナノ秒単位）
                # 2.5秒 = 2,500,000,000ナノ秒
                interval_ns = 2_500_000_000

                # 2.5秒間隔で過去に遡る
                while True:
                    target_timestamp_ns = current_timestamp_ns - interval_ns

                    # target_timestamp_nsに最も近い過去のエントリを探す
                    closest_entry = None
                    closest_diff = float('inf')

                    for entry in self.asr_history:
                        if entry["timestamp_ns"] <= target_timestamp_ns:
                            diff = target_timestamp_ns - entry["timestamp_ns"]
                            if diff < closest_diff:
                                closest_diff = diff
                                closest_entry = entry

                    # 見つからない場合は最も古いエントリを採用
                    if closest_entry is None:
                        if len(self.asr_history) > 1:  # 最新以外にエントリがある場合
                            oldest_entry = self.asr_history[0]
                            words.append(oldest_entry["text"])
                        break
                    else:
                        words.append(closest_entry["text"])
                        current_timestamp_ns = closest_entry["timestamp_ns"]

                # 古いもの→新しいものの順に並べ替え
                words.reverse()

            # ★旧方式（コメントアウト）: 25個おきに遡る間隔送信
            # words = []
            # n = len(self.asr_history)
            # if n > 0:
            #     idx = n - 1
            #     while idx >= 0:
            #         words.append(self.asr_history[idx]["text"])  # textフィールドを取得
            #         idx -= 25
            #     words.reverse()  # 古いもの→新しいもの

            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            # sys.stdout.write(f"[{timestamp}][pubDM] 送信する音声認識履歴リスト（25個おき、全{len(words)}件）: {words}\n")
            # sys.stdout.flush()
            return { "words": words, "update": True, "stage": "first"}
        else:
            return { "words": [], "update": False, "stage": "first"}

    def pubDM_second_stage(self):
        """NLGへsecond_stage（本応答生成）リクエストを送信"""
        # ★修正: TurnTaking判定時に保存したASR履歴を使用
        words = []
        turn_taking_decision_timestamp_ns = self.turn_taking_decision_timestamp_ns

        # ★TurnTaking判定時に保存したASR履歴を使用（self.asr_history_at_tt_decision）
        if len(self.asr_history_at_tt_decision) > 0:
            words = self.asr_history_at_tt_decision.copy()
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[DM-second] TurnTaking判定時保存のASR履歴を使用: {len(words)}件 @ {timestamp}\n")
            sys.stdout.flush()
        else:
            # ASR履歴がない場合は空リストで送信（NLGが前回のfirst_stage結果を再利用）
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[DM-second] ASR履歴なし、空リストを送信 @ {timestamp}\n")
            sys.stdout.flush()

        return {
            "words": words,
            "update": True,
            "stage": "second",
            "turn_taking_decision_timestamp_ns": turn_taking_decision_timestamp_ns,  # ★NLG用に時刻情報も送信
            "first_stage_backchannel_at_tt": self.first_stage_backchannel_at_tt_decision,  # ★TT判定時の相槌内容を送信
            "asr_history_2_5s": self.asr_history_at_tt_decision_2_5s  # ★2.5秒間隔ASR結果をNLGに送信
        }

    def updateNLG(self, nlg_data):
        """NLG PCからの応答を受信"""
        stage = nlg_data.get('stage', 'single')
        reply = nlg_data.get('reply', '')
        request_id = getattr(nlg_data, 'request_id', 0) if hasattr(nlg_data, 'request_id') else nlg_data.get('request_id', 0)
        nlg_start_timestamp_ns = nlg_data.get('start_timestamp_ns', 0)
        nlg_completion_timestamp_ns = nlg_data.get('completion_timestamp_ns', 0)
        inference_duration_ms = nlg_data.get('inference_duration_ms', 0.0)

        # ナノ秒から時刻への変換関数
        def ns_to_readable_time(ns_timestamp):
            if ns_timestamp <= 0:
                return "未設定"
            dt = datetime.fromtimestamp(ns_timestamp / 1_000_000_000)
            return dt.strftime('%H:%M:%S.%f')[:-3]  # ミリ秒まで表示

        # 現在時刻を取得
        now_dt = datetime.now()
        timestamp = now_dt.strftime('%H:%M:%S.%f')[:-3]
        current_time_ns = int(now_dt.timestamp() * 1_000_000_000)

        if stage == 'first':
            # First stage相槌を保存
            self.first_stage_backchannel = reply

            # ★詳細な処理情報を出力（コメントアウト：TurnTaking時のみ表示）
            # sys.stdout.write(f"\n{'='*60}\n")
            # sys.stdout.write(f"[{timestamp}] 🔊 First stage相槌生成完了\n")
            # sys.stdout.write(f"{'='*60}\n")
            # sys.stdout.write(f"📋 内容: '{reply}'\n\n")

            # # NLG処理の詳細情報
            # if nlg_start_timestamp_ns > 0 and nlg_completion_timestamp_ns > 0:
            #     sys.stdout.write(f"📊 NLG処理タイミング:\n")
            #     sys.stdout.write(f"  • 開始時刻:       {ns_to_readable_time(nlg_start_timestamp_ns)}\n")
            #     sys.stdout.write(f"  • 完了時刻:       {ns_to_readable_time(nlg_completion_timestamp_ns)}\n")

            #     nlg_processing_time = (nlg_completion_timestamp_ns - nlg_start_timestamp_ns) / 1_000_000
            #     sys.stdout.write(f"\n⏱️  処理時間:\n")
            #     sys.stdout.write(f"  • NLG推論時間:    {inference_duration_ms:.1f}ms\n")
            #     sys.stdout.write(f"  • NLG総処理時間:  {nlg_processing_time:.1f}ms\n")

            # sys.stdout.write(f"\n📋 処理詳細:\n")
            # sys.stdout.write(f"  • Request ID:     {request_id}\n")
            # sys.stdout.write(f"  • ステージ:       First Stage (相槌)\n")
            # sys.stdout.write(f"{'='*60}\n")
            # sys.stdout.flush()

            # ★即座に音声合成を実行（バックグラウンド非同期処理）
            try:
                first_stage_wav_path = self.synthesize_first_stage_backchannel(reply)
                if first_stage_wav_path and os.path.exists(first_stage_wav_path):
                    # 合成済みファイルを保存
                    self.first_stage_backchannel_wav = first_stage_wav_path
                    self.first_stage_backchannel_available = True
                    now = datetime.now()
                    timestamp_synth = now.strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[DM-first_synth] First stage相槌の音声合成完了: {first_stage_wav_path} @ {timestamp_synth}\n")
                    sys.stdout.flush()

                    # ★注：Second stageリクエストはTurnTaking判定時に設定される（優先度制御のため）
                else:
                    # First stage合成失敗 → エラー音声を再生
                    sys.stdout.write(f"[ERROR] First stage相槌の音声合成に失敗しました: {reply}\n")
                    sys.stdout.flush()
                    self.play_error_audio('first_stage')
                    self.first_stage_backchannel_available = False
            except Exception as e:
                sys.stdout.write(f"[ERROR] First stage相槌の音声合成エラー: {e}\n")
                sys.stdout.flush()
                # エラー音声を再生
                self.play_error_audio('first_stage')
                self.first_stage_backchannel_available = False

        elif stage == 'second':
            # Second stage本応答を保存
            sys.stdout.write(f"[{timestamp}][DM] ⭐ Second stage応答受信: '{reply}'\n")
            sys.stdout.flush()

            # ★即座に音声合成を実行（VOICEVOX API を使用）
            second_stage_synthesis_success = False
            try:
                sys.stdout.write(f"[{timestamp}][DM-second] 🎤 Second stage応答の音声合成開始: '{reply}'\n")
                sys.stdout.flush()

                second_stage_wav_path = self.synthesize_first_stage_backchannel(reply)

                sys.stdout.write(f"[{timestamp}][DM-second] 音声合成結果: path={second_stage_wav_path}, exists={os.path.exists(second_stage_wav_path) if second_stage_wav_path else False}\n")
                sys.stdout.flush()

                if second_stage_wav_path and os.path.exists(second_stage_wav_path):
                    self.latest_synth_filename = second_stage_wav_path
                    second_stage_synthesis_success = True
                    now = datetime.now()
                    timestamp_synth = now.strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[{timestamp_synth}][DM-second_synth] ✅ Second stage応答の音声合成完了: {second_stage_wav_path}\n")
                    sys.stdout.flush()
                else:
                    # Second stage合成失敗 → エラー音声を再生
                    self.latest_synth_filename = ""
                    sys.stdout.write(f"[{timestamp}][ERROR] ❌ Second stage応答の音声合成に失敗しました: {reply}\n")
                    sys.stdout.flush()
                    self.play_error_audio('second_stage')
            except Exception as e:
                sys.stdout.write(f"[{timestamp}][ERROR] ❌ Second stage応答の音声合成エラー: {e}\n")
                sys.stdout.flush()
                import traceback
                traceback.print_exc()
                # エラー音声を再生
                self.play_error_audio('second_stage')
                # エラー時は明示的にクリア
                self.latest_synth_filename = ""

            # 合成成功時のみwaitingフラグをクリア（失敗時はキープして再生試行をスキップ）
            if second_stage_synthesis_success:
                # ⭐ ファイルパスをフラグとして保存のみ（再生はpubDMで実行）
                self.second_stage_ready_to_play = True  # 再生準備完了フラグ
                sys.stdout.write(f"[{timestamp}][DM] ✨ Second stage本応答の合成完了、再生待機中\n")
                sys.stdout.flush()
            else:
                sys.stdout.write(f"[{timestamp}][WARNING] ⚠️  Second stage本応答の再生をスキップします（合成失敗）\n")
                sys.stdout.flush()
                # ★失敗時もタイムアウトフラグをリセット
                self.second_stage_wait_start_time = None
                self.second_stage_timeout_played = False
                self.waiting_for_second_stage = False  # リクエストを続行できるよう初期化

    def updateASR(self, asr):
        # ここでASR結果の履歴を管理
        self.asr["you"] = asr["you"]
        self.asr["is_final"] = asr["is_final"]
        
        # タイムスタンプ情報を含むasr_historyに追加
        asr_entry = {
            "text": asr["you"],
            "timestamp_ns": asr.get("timestamp_ns", int(time.time_ns())),  # タイムスタンプがない場合は現在時刻
            "is_final": asr["is_final"]
        }
        self.asr_history.append(asr_entry)
        
        # ASRタイミング情報を記録
        current_time_ns = int(time.time_ns())
        if "start_timestamp_ns" in asr:
            self.asr_start_ns = asr["start_timestamp_ns"]
        if "completion_timestamp_ns" in asr:
            self.asr_completion_ns = asr["completion_timestamp_ns"]
        else:
            # タイムスタンプが提供されていない場合、現在時刻を使用
            self.asr_completion_ns = current_time_ns
            
        # タイミングログに記録
        if TIMING_AVAILABLE and self.timing_logger and asr["is_final"]:
            if self.current_session_id is None:
                self.current_session_id = self.timing_logger.start_session()
            
            # ASR完了をログに記録
            self.timing_logger.log_event(
                session_id=self.current_session_id,
                event_type="asr_complete",
                timestamp_ns=self.asr_completion_ns,
                data={
                    "text": asr["you"],
                    "processing_time_ms": (self.asr_completion_ns - self.asr_start_ns) / 1_000_000 if self.asr_start_ns > 0 else 0
                }
            )
        
        # asr_historyとresponse_updateの値を出力
        # print(f"[DEBUG] asr_history: {self.asr_history}")
        # print(f"[DEBUG] response_update: {self.response_update}")
        # メモリリーク防止: 履歴を最大500個に制限
        if len(self.asr_history) > 500:
            self.asr_history = self.asr_history[-250:]  # 最新250個を保持

    def updateSA(self, sa):
        self.sa["prevgrad"] = sa["prevgrad"]
        self.sa["frequency"] = sa["frequency"]
        self.sa["grad"] = sa["grad"]
        self.sa["power"] = sa["power"]
        self.sa["zerocross"] = sa["zerocross"]

    def updateSS(self, ss):
        self.ss["is_speaking"] = ss["is_speaking"]  # test
        self.ss["timestamp"] = ss["timestamp"]
        
        # 追加: 音声合成ファイル名を受信したらTT閾値超え時に再生用に保存
        if "filename" in ss and ss["filename"]:
            self.latest_synth_filename = ss["filename"]
            # TTS完了時刻を記録
            self.tts_completion_ns = int(time.time_ns())
        # ★追加: 対話生成結果を受信して保存
        if "dialogue_text" in ss and ss["dialogue_text"]:
            self.latest_dialogue_result = ss["dialogue_text"]
        
        # TTSタイミング情報を受信・保存
        if "tts_start_timestamp_ns" in ss:
            self.tts_start_ns = ss["tts_start_timestamp_ns"]
            # print(f"[DEBUG-updateSS] TTS開始時刻設定: {self.tts_start_ns}")
        if "tts_completion_timestamp_ns" in ss:
            received_tts_completion = ss["tts_completion_timestamp_ns"]
            # print(f"[DEBUG-updateSS] TTS完了時刻受信: {received_tts_completion}")
            if received_tts_completion > 0:
                self.tts_completion_ns = received_tts_completion
                # print(f"[DEBUG-updateSS] TTS完了時刻設定: {self.tts_completion_ns}")
            # else:
                # print(f"[DEBUG-updateSS] TTS完了時刻が0のため、現在設定値を維持: {self.tts_completion_ns}")
        
        # ★対話生成時刻情報を受信・保存（デバッグ出力追加）
        if "request_id" in ss:
            self.latest_request_id = ss["request_id"]
        if "worker_name" in ss:
            self.latest_worker_name = ss["worker_name"]
        if "start_timestamp_ns" in ss:
            self.latest_start_timestamp_ns = ss["start_timestamp_ns"]
        if "completion_timestamp_ns" in ss:
            self.latest_completion_timestamp_ns = ss["completion_timestamp_ns"]
            
            # NLG + TTS完了をタイミングログに記録
            if TIMING_AVAILABLE and self.timing_logger and self.current_session_id:
                self.timing_logger.log_event(
                    session_id=self.current_session_id,
                    event_type="nlg_complete",
                    timestamp_ns=self.latest_completion_timestamp_ns,
                    data={
                        "request_id": self.latest_request_id,
                        "worker_name": self.latest_worker_name,
                        "dialogue_text": self.latest_dialogue_result,
                        "processing_time_ms": (self.latest_completion_timestamp_ns - self.latest_start_timestamp_ns) / 1_000_000
                    }
                )
                
                # TTS完了をタイミングログに記録
                if self.tts_completion_ns > 0:
                    self.timing_logger.log_event(
                        session_id=self.current_session_id,
                        event_type="tts_complete", 
                        timestamp_ns=self.tts_completion_ns,
                        data={
                            "filename": self.latest_synth_filename,
                            "processing_time_ms": (self.tts_completion_ns - self.tts_start_ns) / 1_000_000 if self.tts_start_ns > 0 else 0
                        }
                    )
                
        if "inference_duration_ms" in ss:
            self.latest_inference_duration_ms = ss["inference_duration_ms"]
            
        # print(f"[ROS2] {ss['timestamp']}")
        if self.ss["is_speaking"] is True:
            self.speaking_time = datetime.now()

    def updateTT(self, data):
        # ros2_dm.pyからデータを受け取った時刻を記録
        self.latest_tt_data = data
        self.latest_tt_time = datetime.now()
        
        # updateTT処理タイミング出力
        timestamp_str = self.latest_tt_time.strftime('%H:%M:%S.%f')[:-3]
        # print(f"[{timestamp_str}][DM_updateTT] TT処理完了 (result={data['result']}, conf={data['confidence']:.3f})")
        # sys.stdout.flush()

    def updateBC(self, data):
        # ros2_dm.pyからデータを受け取った時刻を記録
        self.latest_bc_data = data
        self.latest_bc_time = datetime.now()
        # 受信時刻と推論値を全桁出力
        now = self.latest_bc_time