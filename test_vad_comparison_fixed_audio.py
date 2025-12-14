#!/usr/bin/env python3
"""
SileroVAD vs VADIterator vs WebRTC VAD 比較テスト - 固定音声ファイル版 + TurnTaking
音声ファイルを10ms毎に読み込んで3手法を比較し、VADIteratorとWebRTC VADで100ms無声検出時にTurnTaking推論を実行
"""

import torch
import numpy as np
import time
from datetime import datetime
import soundfile as sf
import sys
from collections import deque
import argparse
import os
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import torch.nn as nn
from scipy.io import wavfile
import webrtcvad

# SileroVADのロード
silero_vad_model = None
get_speech_timestamps = None
VADIterator = None

class TurnTakingModel:
    """TurnTakingモデル"""
    def __init__(self, model_id="SiRoZaRuPa/japanese-wav2vec2-base-turntaking-CSJ"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[INFO] TurnTakingモデル初期化中... (device: {self.device})")
        self.model = Wav2Vec2ForSequenceClassification.from_pretrained(
            model_id, token=True
        ).to(self.device)
        self.model.eval()
        self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(
            model_id, token=True
        )
        print(f"[INFO] TurnTakingモデル初期化完了")

    def predict(self, audio, threshold=0.75):
        inputs = self.feature_extractor(
            audio, sampling_rate=16000, return_tensors="pt"
        )
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            output = self.model(**inputs).logits
        sigmoid = nn.Sigmoid()
        probability = float(sigmoid(output)[0])
        pred = 0 if probability < threshold else 1
        return pred, probability

try:
    torch.set_num_threads(1)
    silero_vad_model, utils = torch.hub.load(
        repo_or_dir='snakers4/silero-vad',
        model='silero_vad',
        force_reload=False,
        onnx=False
    )
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    print("[INFO] SileroVAD loaded successfully")
except Exception as e:
    print(f"[ERROR] SileroVAD load failed: {e}")
    sys.exit(1)

class AudioBuffer32ms:
    """10ms入力→32ms処理用バッファ（SileroVAD高速判定）"""
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル（32ms）
        self.chunk_96ms_samples = self.chunk_32ms_samples * 3  # 1536サンプル（96ms）
        # 連続音声バッファ（最大200ms分保持）
        self.buffer = deque(maxlen=int(sample_rate * 0.2))
        
    def add_samples(self, samples):
        """10ms音声データを追加"""
        self.buffer.extend(samples)
        
    def get_latest_96ms_as_3chunks(self):
        """最新96ms分を3つの32msチャンクとして取得"""
        if len(self.buffer) < self.chunk_96ms_samples:
            # 不足分をゼロパディング
            padded = np.zeros(self.chunk_96ms_samples, dtype=np.float32)
            if len(self.buffer) > 0:
                available = list(self.buffer)[-len(self.buffer):]
                padded[-len(available):] = available
            audio_96ms = padded
        else:
            audio_96ms = np.array(list(self.buffer)[-self.chunk_96ms_samples:], dtype=np.float32)
        
        # 3つの32msチャンクに分割
        chunks = []
        for i in range(3):
            start_idx = i * self.chunk_32ms_samples
            end_idx = start_idx + self.chunk_32ms_samples
            chunks.append(audio_96ms[start_idx:end_idx])
        return chunks
        
    def has_96ms_data(self):
        """96ms分のデータが蓄積されているか"""
        return len(self.buffer) >= self.chunk_96ms_samples

class VADIteratorBuffer:
    """VADIterator用バッファ（32ms単位処理）"""
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル（32ms）
        self.buffer = np.array([], dtype=np.float32)
        
    def add_samples(self, samples):
        """10ms音声データを追加"""
        self.buffer = np.concatenate([self.buffer, samples])
        
    def get_32ms_chunk_if_ready(self):
        """32ms分のチャンクが準備できていれば取得し、バッファから削除"""
        if len(self.buffer) >= self.chunk_32ms_samples:
            chunk = self.buffer[:self.chunk_32ms_samples].copy()
            self.buffer = self.buffer[self.chunk_32ms_samples:]  # 使用済みデータを削除
            return chunk
        return None

def judge_3chunks_silence(chunks_32ms):
    """3つの32msチャンクが無声かどうかを判定"""
    try:
        speech_probs = []
        
        # 各32msチャンクに対して個別に推論
        for i, chunk in enumerate(chunks_32ms):
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                speech_prob = silero_vad_model(chunk_tensor, 16000).item()
                speech_probs.append(speech_prob)
        
        # 3チャンクの平均確率で判定
        avg_prob = np.mean(speech_probs)
        is_speech = avg_prob >= 0.5
        
        return is_speech, avg_prob, speech_probs
        
    except Exception as e:
        print(f"[ERROR] SileroVAD 3チャンク判定エラー: {e}")
        return None, 0.0, [0.0, 0.0, 0.0]

def test_vad_comparison(audio_file, save_results=None):
    """両手法の比較テスト + TurnTaking推論"""
    print("SileroVAD vs VADIterator 比較テスト（固定音声ファイル版）+ TurnTaking")
    print("=" * 70)

    # TurnTakingモデル初期化
    tt_model = TurnTakingModel()
    TurnJudgeThreshold = 0.650

    # 音声ファイル読み込み
    try:
        audio_data, sample_rate = sf.read(audio_file)
        print(f"[INFO] 音声ファイル読み込み: {audio_file}")
        print(f"[INFO] サンプリングレート: {sample_rate}Hz, 長さ: {len(audio_data)}サンプル ({len(audio_data)/sample_rate:.2f}秒)")
        
        # 16kHzにリサンプル（必要な場合）
        if sample_rate != 16000:
            print(f"[WARNING] サンプリングレートが16kHzではありません。16kHzと仮定して処理を続行します。")
            sample_rate = 16000
        
        # モノラル変換
        if audio_data.ndim > 1:
            audio_data = np.mean(audio_data, axis=1)
        
        # float32に変換
        audio_data = audio_data.astype(np.float32)
        
    except Exception as e:
        print(f"[ERROR] 音声ファイル読み込みエラー: {e}")
        return
    
    # 10msチャンクサイズ
    chunk_10ms_samples = int(sample_rate * 0.01)  # 160サンプル
    
    # バッファ初期化
    # SileroVAD高速判定用
    silero_buffer = AudioBuffer32ms(sample_rate)
    silero_speech_detected = False
    silero_silent_start_time = None
    silero_silence_100ms_detected = False
    silero_total_judgments = 0
    silero_speech_judgments = 0
    
    # VADIterator用
    vad_iterator = VADIterator(
        silero_vad_model,
        threshold=0.5,
        sampling_rate=sample_rate,
        min_silence_duration_ms=96,
        speech_pad_ms=0
    )
    vad_iterator_buffer = VADIteratorBuffer(sample_rate)
    vad_iterator_speech_detected = False
    vad_iterator_silent_start_time = None
    vad_iterator_silence_100ms_detected = False
    vad_iterator_total_events = 0

    # 複数回のTurnTaking実行用変数
    vad_iterator_detection_time = None
    vad_iterator_tt_schedule = []  # 実行予定時刻のリスト（100ms, 200ms, 300ms, 400ms, 500ms後）

    # TurnTaking用の音声バッファと結果リスト
    sound = np.empty(0, dtype='float32')
    turntaking_results = []

    # WebRTC VAD用
    webrtc_vad = webrtcvad.Vad()
    webrtc_vad.set_mode(3)  # 最も厳格なモード
    webrtc_frame_duration_ms = 30  # 30msフレーム
    webrtc_sound_available = False
    webrtc_sound_count = 0
    webrtc_silent_count = 0
    webrtc_frame_buffer = np.array([], dtype=np.int16)
    webrtc_turntaking_results = []

    print("\n[INFO] 処理開始")
    print(f"[INFO] TurnTaking閾値: {TurnJudgeThreshold}")
    print(f"[INFO] WebRTC VAD モード: 3 (最も厳格), フレーム長: {webrtc_frame_duration_ms}ms")
    print("時刻フォーマット: [手法][ファイル内時刻ms] メッセージ")
    print("-" * 70)
    
    # 10ms毎に音声ファイルを処理
    total_chunks = (len(audio_data) + chunk_10ms_samples - 1) // chunk_10ms_samples
    
    for chunk_idx in range(total_chunks):
        # 現在のファイル内時刻
        file_time_ms = chunk_idx * 10

        # 10msチャンクを取得
        start_idx = chunk_idx * chunk_10ms_samples
        end_idx = min(start_idx + chunk_10ms_samples, len(audio_data))
        audiodata = audio_data[start_idx:end_idx]

        # 不足分をゼロパディング
        if len(audiodata) < chunk_10ms_samples:
            audiodata = np.pad(audiodata, (0, chunk_10ms_samples - len(audiodata)))

        # 音声データバッファに追加（TurnTaking用）
        sound = np.concatenate([sound, audiodata])

        # ★ 複数回のTurnTaking実行チェック（100ms毎に500msまで）
        if vad_iterator_tt_schedule and vad_iterator_detection_time is not None:
            # 実行可能なスケジュールをチェック
            executed_schedules = []
            for delay_ms in vad_iterator_tt_schedule:
                if file_time_ms >= vad_iterator_detection_time + delay_ms:
                    executed_schedules.append(delay_ms)
                    print(f"[VAD_ITER][{file_time_ms:04d}ms] 検出から{delay_ms}ms経過後のTurnTaking実行!")

                    try:
                        process_start_time = time.perf_counter()

                        # 100ms削った音声を使用
                        samples_100ms = int(sample_rate * 0.1)
                        if sound.shape[0] > samples_100ms:
                            sound_for_tt = sound[:-samples_100ms]
                        else:
                            sound_for_tt = sound

                        if len(sound_for_tt) > 0:
                            # TurnTaking実行用の音声をファイルに保存（上書き）
                            audio_save_path = f"tt_audio_detection_{delay_ms}ms.wav"
                            wavfile.write(audio_save_path, sample_rate, sound_for_tt)
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] TurnTaking入力音声を保存: {audio_save_path}")

                            sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                            # 5秒分に制限
                            max_samples = int(5 * sample_rate)
                            if len(sound_comp) > max_samples:
                                sound_comp = sound_comp[:max_samples]

                            pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                            processing_time = (time.perf_counter() - process_start_time) * 1000

                            # 結果を保存
                            tt_result = {
                                'detection_time_ms': file_time_ms,
                                'timing': f'detection+{delay_ms}ms',
                                'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                'prediction': pred,
                                'probability': probability,
                                'processing_time_ms': processing_time,
                                'audio_length_sec': len(sound_for_tt) / sample_rate,
                                'audio_length_samples': len(sound_for_tt)
                            }
                            turntaking_results.append(tt_result)

                            print(f"[VAD_ITER][{file_time_ms:04d}ms] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")

                    except Exception as e:
                        print(f"[ERROR] {delay_ms}ms後のTurnTaking実行エラー: {e}")

            # 実行済みのスケジュールをリストから削除
            for delay_ms in executed_schedules:
                vad_iterator_tt_schedule.remove(delay_ms)

        # ★ SileroVAD高速判定システム（10ms毎に直近96ms判定）
        silero_buffer.add_samples(audiodata)
        
        # 10ms毎に判定実行（96ms分のデータがあれば）
        if silero_buffer.has_96ms_data():
            # 最新96ms分を3つの32msチャンクとして切り出し
            chunks_32ms = silero_buffer.get_latest_96ms_as_3chunks()
            is_speech, speech_prob, individual_probs = judge_3chunks_silence(chunks_32ms)
            
            if is_speech is not None:
                silero_total_judgments += 1
                
                if is_speech:
                    silero_speech_judgments += 1
                
                # 状態変化検出
                if is_speech and not silero_speech_detected:
                    # 音声開始
                    silero_speech_detected = True
                    silero_silent_start_time = None
                    silero_silence_100ms_detected = False
                    print(f"[SILERO][{file_time_ms:04d}ms] 音声開始検出 (avg_prob={speech_prob:.3f})")
                    
                elif not is_speech and silero_speech_detected:
                    # 音声終了→無声開始
                    silero_speech_detected = False
                    silero_silent_start_time = file_time_ms
                    print(f"[SILERO][{file_time_ms:04d}ms] 音声終了、無声区間開始 (avg_prob={speech_prob:.3f})")
                
                # 100ms無声継続チェック
                if not is_speech and silero_silent_start_time is not None and not silero_silence_100ms_detected:
                    silent_duration_ms = file_time_ms - silero_silent_start_time
                    if silent_duration_ms >= 100:
                        print(f"[SILERO][{silero_silent_start_time:04d}ms] 100ms無声区間検出完了 ({silent_duration_ms}ms継続)")
                        silero_silence_100ms_detected = True
        
        # ★ VADIterator（32ms処理） - SileroVAD出力完了後に実行
        vad_iterator_buffer.add_samples(audiodata)
        
        chunk_32ms = vad_iterator_buffer.get_32ms_chunk_if_ready()
        if chunk_32ms is not None:
            try:
                vad_result = vad_iterator(chunk_32ms)
                vad_iterator_total_events += 1
                
                # VADIterator結果の解析
                result_type = None
                speech_start = False
                speech_end = False
                
                if vad_result is not None and isinstance(vad_result, dict):
                    if 'start' in vad_result:
                        result_type = 'start'
                        speech_start = True
                    elif 'end' in vad_result:
                        result_type = 'end'
                        speech_end = True
                
                # 状態管理
                if speech_start and not vad_iterator_speech_detected:
                    vad_iterator_speech_detected = True
                    vad_iterator_silent_start_time = None
                    vad_iterator_silence_100ms_detected = False
                    print(f"[VAD_ITER][{file_time_ms:04d}ms] 音声開始検出 (type={result_type})")
                    
                elif speech_end and vad_iterator_speech_detected:
                    # VADIteratorがendイベントを返した = 既に100ms無声が検出済み
                    vad_iterator_speech_detected = False
                    vad_iterator_silent_start_time = file_time_ms
                    print(f"[VAD_ITER][{file_time_ms:04d}ms] 100ms無声検出完了 → TurnTaking実行(1回目)!")

                    # 1回目のTurnTakingモデル実行（100ms削った音声で実行）
                    try:
                        process_start_time = time.perf_counter()

                        # 100ms（1600サンプル）削った音声を使用
                        samples_100ms = int(sample_rate * 0.1)
                        if sound.shape[0] > samples_100ms:
                            sound_for_tt = sound[:-samples_100ms]  # 最後の100msを削除
                        else:
                            sound_for_tt = sound  # 音声が短い場合はそのまま使用

                        if len(sound_for_tt) == 0:
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] [WARNING] 音声データが不足、TurnTakingスキップ")
                        else:
                            # TurnTaking実行用の音声をファイルに保存（上書き）
                            audio_save_path = "tt_audio_detection.wav"
                            wavfile.write(audio_save_path, sample_rate, sound_for_tt)
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] TurnTaking入力音声を保存: {audio_save_path}")

                            sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                            # 5秒分に制限
                            max_samples = int(5 * sample_rate)
                            if len(sound_comp) > max_samples:
                                sound_comp = sound_comp[:max_samples]

                            pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                            processing_time = (time.perf_counter() - process_start_time) * 1000

                            # 結果を保存（timing="detection"を追加）
                            tt_result = {
                                'detection_time_ms': file_time_ms,
                                'timing': 'detection',
                                'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                'prediction': pred,
                                'probability': probability,
                                'processing_time_ms': processing_time,
                                'audio_length_sec': len(sound_for_tt) / sample_rate,
                                'audio_length_samples': len(sound_for_tt)
                            }
                            turntaking_results.append(tt_result)

                            print(f"[VAD_ITER][{file_time_ms:04d}ms] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] GPU使用: {tt_model.device}")

                            # 複数回のTurnTaking実行を予約（100ms, 200ms, 300ms, 400ms, 500ms後）
                            vad_iterator_detection_time = file_time_ms
                            vad_iterator_tt_schedule = [100, 200, 300, 400, 500]
                            print(f"[VAD_ITER][{file_time_ms:04d}ms] 追加TurnTaking実行を予約: +100ms, +200ms, +300ms, +400ms, +500ms")

                    except Exception as e:
                        print(f"[ERROR] TurnTaking実行エラー: {e}")
                
                # 詳細ログ（結果がある場合のみ）
                if vad_result is not None:
                    print(f"[VAD_ITER][{file_time_ms:04d}ms] イベント#{vad_iterator_total_events} - {vad_result}")
                    
            except Exception as e:
                print(f"[VAD_ITER][{file_time_ms:04d}ms] 処理エラー: {e}")

        # ★ WebRTC VAD（30msフレーム単位処理）
        # 10msチャンクをint16に変換してバッファに追加
        audiodata_int16 = np.array(audiodata * 32767, dtype=np.int16)
        webrtc_frame_buffer = np.concatenate([webrtc_frame_buffer, audiodata_int16])

        # 30ms分（480サンプル）が溜まったら処理
        webrtc_frame_samples = int(sample_rate * webrtc_frame_duration_ms / 1000)
        if len(webrtc_frame_buffer) >= webrtc_frame_samples:
            # 30msフレームを取り出し
            webrtc_frame = webrtc_frame_buffer[:webrtc_frame_samples]
            webrtc_frame_buffer = webrtc_frame_buffer[webrtc_frame_samples:]  # 使用済みデータを削除

            # WebRTC VAD判定
            webrtc_frame_bytes = webrtc_frame.tobytes()
            is_speech = webrtc_vad.is_speech(webrtc_frame_bytes, sample_rate)

            if is_speech:
                # 音声セグメント長の計測
                webrtc_silent_count = 0
                webrtc_sound_count += 1

                if webrtc_sound_count >= (200 / webrtc_frame_duration_ms):  # 200msより短い音声区間ならノイズとして無視
                    webrtc_sound_available = True
                    if webrtc_sound_count == int(200 / webrtc_frame_duration_ms):
                        print(f"[WEBRTC][{file_time_ms:04d}ms] 200ms音声区間検出 → sound_available=True")

            elif sound.shape[0] >= 5 * sample_rate:
                # 無音長の計測
                webrtc_sound_count = 0
                webrtc_silent_count += 1

                if webrtc_silent_count >= (100 / webrtc_frame_duration_ms):  # 100ms以上の無声区間検出
                    if webrtc_sound_available:
                        print(f"[WEBRTC][{file_time_ms:04d}ms] 100ms無声区間検出完了 → TurnTaking実行!")

                        # TurnTakingモデル実行（100ms削った音声で実行）
                        try:
                            process_start_time = time.perf_counter()

                            # 100ms（1600サンプル）削った音声を使用
                            samples_100ms = int(sample_rate * 0.1)
                            if sound.shape[0] > samples_100ms:
                                sound_for_tt = sound[:-samples_100ms]
                            else:
                                sound_for_tt = sound

                            if len(sound_for_tt) > 0:
                                # TurnTaking実行用の音声をファイルに保存（上書き）
                                audio_save_path = "tt_audio_webrtc_detection.wav"
                                wavfile.write(audio_save_path, sample_rate, sound_for_tt)
                                print(f"[WEBRTC][{file_time_ms:04d}ms] TurnTaking入力音声を保存: {audio_save_path}")

                                sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                                # 5秒分に制限
                                max_samples = int(5 * sample_rate)
                                if len(sound_comp) > max_samples:
                                    sound_comp = sound_comp[:max_samples]

                                pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                                processing_time = (time.perf_counter() - process_start_time) * 1000

                                # 結果を保存
                                tt_result = {
                                    'detection_time_ms': file_time_ms,
                                    'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                    'prediction': pred,
                                    'probability': probability,
                                    'processing_time_ms': processing_time,
                                    'audio_length_sec': len(sound_for_tt) / sample_rate,
                                    'audio_length_samples': len(sound_for_tt)
                                }
                                webrtc_turntaking_results.append(tt_result)

                                print(f"[WEBRTC][{file_time_ms:04d}ms] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                                print(f"[WEBRTC][{file_time_ms:04d}ms] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")

                        except Exception as e:
                            print(f"[ERROR] WebRTC TurnTaking実行エラー: {e}")

                        webrtc_sound_available = False

        # 音声バッファのサイズ管理（5.1秒に制限）
        if sound.shape[0] >= int(5.1 * sample_rate):
            sound = sound[-int(5.1 * sample_rate):]

    print("-" * 70)
    print("処理完了")
    print(f"総ファイル時間: {total_chunks * 10}ms ({(total_chunks * 10)/1000:.2f}秒)")
    print(f"SileroVAD判定回数: {silero_total_judgments}")
    print(f"VADIterator実行回数: {vad_iterator_total_events}")
    print(f"SileroVAD音声率: {(silero_speech_judgments/silero_total_judgments*100) if silero_total_judgments > 0 else 0:.1f}%")
    print(f"VADIterator TurnTaking実行回数: {len(turntaking_results)}")
    print(f"WebRTC VAD TurnTaking実行回数: {len(webrtc_turntaking_results)}")

    # VADIterator TurnTaking結果の詳細表示（時系列順にソート）
    if turntaking_results:
        # 時系列順にソート
        turntaking_results_sorted = sorted(turntaking_results, key=lambda x: x['detection_time_ms'])

        print("\n=== VADIterator TurnTaking結果詳細（時系列順） ===")
        print("検出時刻[ms] | タイミング        | 予測 | 確率  | 処理時間[ms] | 音声長[s] | デバイス")
        print("-" * 95)
        for result in turntaking_results_sorted:
            detection_time = result['detection_time_ms']
            timing = result.get('timing', 'detection')
            device_info = str(tt_model.device)
            print(f"{detection_time:12d} | {timing:17s} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f} | {device_info}")

    # WebRTC VAD TurnTaking結果の詳細表示
    if webrtc_turntaking_results:
        print("\n=== WebRTC VAD TurnTaking結果詳細 ===")
        print("検出時刻[ms] | 予測 | 確率  | 処理時間[ms] | 音声長[s] | デバイス")
        print("-" * 75)
        for result in webrtc_turntaking_results:
            detection_time = result['detection_time_ms']
            device_info = str(tt_model.device)
            print(f"{detection_time:12d} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f} | {device_info}")

    # 結果保存（任意、時系列順）
    if save_results and turntaking_results:
        import json
        turntaking_results_sorted = sorted(turntaking_results, key=lambda x: x['detection_time_ms'])
        with open(save_results, 'w', encoding='utf-8') as f:
            json.dump(turntaking_results_sorted, f, ensure_ascii=False, indent=2)
        print(f"\n[INFO] 結果を保存しました: {save_results}")

    return turntaking_results

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='VADIterator + TurnTaking統合テスト')
    parser.add_argument('audio_file', nargs='?', default='/workspace/DiaROS/script3.wav', help='テスト用音声ファイルパス')
    parser.add_argument('--save-results', help='結果を保存するJSONファイルパス（任意）')

    args = parser.parse_args()

    # 音声ファイルの存在確認
    if not os.path.exists(args.audio_file):
        print(f"[ERROR] 音声ファイルが見つかりません: {args.audio_file}")
        sys.exit(1)

    # テスト実行
    test_vad_comparison(args.audio_file, args.save_results)

if __name__ == "__main__":
    main()