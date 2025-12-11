#!/usr/bin/env python3
"""
TurnTaking音声ファイルテスト - 完全版
turnTaking.pyの全体ロジックを移植して音声ファイルからTurnTaking出力を取得
"""

import torch
import numpy as np
import time
from datetime import datetime
import soundfile as sf
import sys
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import torch.nn as nn
import argparse
import os

class TurnTakingModel:
    """TurnTakingモデル（turnTaking.pyから移植）"""
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

class TurnTakingAudioFileTest:
    def __init__(self, audio_file_path):
        self.audio_file = audio_file_path
        
        # TurnTakingモデル初期化
        self.tt_model = TurnTakingModel()
        self.TurnJudgeThreshold = 0.650
        
        # SileroVAD初期化
        self.silero_vad_model = None
        self.VADIterator = None
        self.get_speech_timestamps = None
        self._load_silero_vad()
        
        # 音声設定
        self.sample_rate = 16000
        self.frame_duration = 10  # ms
        self.chunk_10ms_samples = int(self.sample_rate * 0.01)  # 160サンプル
        
    def _load_silero_vad(self):
        """SileroVADをロード"""
        try:
            torch.set_num_threads(1)
            self.silero_vad_model, utils = torch.hub.load(
                repo_or_dir='snakers4/silero-vad',
                model='silero_vad',
                force_reload=False,
                onnx=False
            )
            self.get_speech_timestamps, save_audio, read_audio, self.VADIterator, collect_chunks = utils
            print("[INFO] SileroVAD loaded successfully")
        except Exception as e:
            print(f"[ERROR] SileroVAD load failed: {e}")
            sys.exit(1)
    
    def load_audio_file(self):
        """音声ファイルを読み込み"""
        try:
            audio_data, sample_rate = sf.read(self.audio_file)
            print(f"[INFO] 音声ファイル読み込み: {self.audio_file}")
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
            
            return audio_data, sample_rate
            
        except Exception as e:
            print(f"[ERROR] 音声ファイル読み込みエラー: {e}")
            return None, None
    
    def test_vad_iterator_turntaking(self):
        """VADIteratorベースのTurnTaking処理をテスト"""
        print("\n" + "=" * 70)
        print("VADIterator TurnTaking 音声ファイルテスト")
        print("=" * 70)
        
        # 音声ファイル読み込み
        audio_data, sample_rate = self.load_audio_file()
        if audio_data is None:
            return
        
        # VADIterator初期化（turnTaking.pyの設定に準拠）
        vad_iterator = self.VADIterator(
            self.silero_vad_model,
            threshold=0.5,
            sampling_rate=sample_rate,
            min_silence_duration_ms=100,    # 100ms無声で音声終了判定
            speech_pad_ms=30                # 30ms音声継続で音声開始判定
        )
        
        # VADIterator用変数
        vad_iterator_buffer = np.array([], dtype=np.float32)
        vad_iterator_speech_detected = False
        vad_iterator_silent_start_time = None
        vad_iterator_100ms_detected = False
        vad_iterator_mode = "WAITING_SPEECH_30MS"  # webRTCVAD準拠：初期状態は30ms音声復帰待機
        
        # 音声データバッファ
        sound = np.empty(0, dtype='float32')
        
        # TurnTaking結果を保存
        turntaking_results = []
        
        print(f"[INFO] VADIterator設定: min_silence=100ms, speech_pad=30ms")
        print(f"[INFO] 処理開始: 10ms毎に音声ファイルを処理")
        print("時刻フォーマット: [ファイル内時刻ms][VAD_ITERATOR] メッセージ")
        print("-" * 70)
        
        # 10ms毎に音声ファイルを処理
        total_chunks = (len(audio_data) + self.chunk_10ms_samples - 1) // self.chunk_10ms_samples
        
        for chunk_idx in range(total_chunks):
            # 現在のファイル内時刻
            file_time_ms = chunk_idx * 10
            
            # 10msチャンクを取得
            start_idx = chunk_idx * self.chunk_10ms_samples
            end_idx = min(start_idx + self.chunk_10ms_samples, len(audio_data))
            audiodata = audio_data[start_idx:end_idx]
            
            # 不足分をゼロパディング
            if len(audiodata) < self.chunk_10ms_samples:
                audiodata = np.pad(audiodata, (0, self.chunk_10ms_samples - len(audiodata)))
            
            # 音声データバッファに追加
            sound = np.concatenate([sound, audiodata])
            
            # VADIterator処理（turnTaking.pyから移植）
            vad_iterator_buffer = np.concatenate([vad_iterator_buffer, audiodata])
            
            # 32ms分のデータが溜まったら処理
            chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル
            while len(vad_iterator_buffer) >= chunk_32ms_samples:
                chunk_32ms = vad_iterator_buffer[:chunk_32ms_samples]
                vad_iterator_buffer = vad_iterator_buffer[chunk_32ms_samples:]  # 使用済みデータを削除
                
                current_time = datetime.now()
                timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]
                
                try:
                    # VADIterator実行
                    vad_result = vad_iterator(chunk_32ms)
                    
                    # VADIterator結果の解析
                    speech_start = False
                    speech_end = False
                    
                    if vad_result is not None and isinstance(vad_result, dict):
                        if 'start' in vad_result:
                            speech_start = True
                        elif 'end' in vad_result:
                            speech_end = True
                    
                    # webRTCVAD準拠の状態管理
                    if vad_iterator_mode == "WAITING_SPEECH_30MS":
                        # 30ms音声復帰待機モード
                        if speech_start:
                            vad_iterator_speech_detected = True
                            vad_iterator_mode = "DETECTING_SILENCE"
                            vad_iterator_silent_start_time = None
                            vad_iterator_100ms_detected = False
                            print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 30ms音声復帰検出、無声検出モードに移行")
                    
                    elif vad_iterator_mode == "DETECTING_SILENCE":
                        # 無声検出モード
                        if speech_start and not vad_iterator_speech_detected:
                            # 音声再開：無声区間をリセット
                            vad_iterator_speech_detected = True
                            vad_iterator_silent_start_time = None
                            vad_iterator_100ms_detected = False
                            print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 音声再開、無声区間リセット")
                        elif speech_end and vad_iterator_speech_detected:
                            # 音声終了→無声開始
                            vad_iterator_speech_detected = False
                            vad_iterator_silent_start_time = current_time
                            vad_iterator_100ms_detected = False  # リセット
                            print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 音声終了、無声区間開始 (開始時刻: {current_time.strftime('%H:%M:%S.%f')[:-3]})")
                        
                        # 100ms無声継続チェック
                        if not vad_iterator_speech_detected and vad_iterator_silent_start_time and not vad_iterator_100ms_detected:
                            silent_duration_ms = (current_time - vad_iterator_silent_start_time).total_seconds() * 1000
                            # より詳細なデバッグ出力
                            if silent_duration_ms >= 10:  # 10ms以上の無声を詳細表示
                                print(f"[{file_time_ms:04d}ms][SILENCE_DEBUG] 無声継続中: {silent_duration_ms:.1f}ms (目標: 100ms)")
                            if silent_duration_ms >= 100:
                                print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 100ms無声区間検出完了、TurnTaking実行!")
                                vad_iterator_100ms_detected = True
                                
                                # TurnTakingモデル実行（100ms削った音声で実行）
                                try:
                                    process_start_time = time.perf_counter()
                                    
                                    # 100ms（1600サンプル）削った音声を使用
                                    samples_100ms = int(sample_rate * 0.1)
                                    if sound.shape[0] > samples_100ms:
                                        sound_for_tt = sound[:-samples_100ms]  # 最後の100msを削除
                                    else:
                                        sound_for_tt = sound  # 音声が短い場合はそのまま使用
                                    
                                    sound_comp = sound_for_tt / np.abs(sound_for_tt).max()
                                    
                                    # 5秒分に制限
                                    max_samples = int(5 * sample_rate)
                                    if len(sound_comp) > max_samples:
                                        sound_comp = sound_comp[:max_samples]
                                    
                                    pred, probability = self.tt_model.predict(sound_comp, threshold=self.TurnJudgeThreshold)
                                    processing_time = (time.perf_counter() - process_start_time) * 1000
                                    
                                    # 結果を保存
                                    tt_result = {
                                        'file_time_ms': file_time_ms,
                                        'timestamp': timestamp_str,
                                        'prediction': pred,
                                        'probability': probability,
                                        'processing_time_ms': processing_time,
                                        'audio_length_sec': len(sound_for_tt) / sample_rate,
                                        'audio_length_samples': len(sound_for_tt)
                                    }
                                    turntaking_results.append(tt_result)
                                    
                                    print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                                    print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                                    
                                    # 次の状態に移行：30ms音声復帰待機
                                    vad_iterator_mode = "WAITING_SPEECH_30MS"
                                    print(f"[{file_time_ms:04d}ms][VAD_ITERATOR] 30ms音声復帰待機モードに移行")
                                    
                                except Exception as e:
                                    print(f"[ERROR] TurnTaking実行エラー: {e}")
                    
                    # 詳細ログ（結果がある場合のみ）
                    if vad_result is not None:
                        print(f"[{file_time_ms:04d}ms][VAD_ITERATOR_DETAIL] 結果: {vad_result}, モード: {vad_iterator_mode}")
                        
                except Exception as e:
                    print(f"[ERROR] VADIterator処理エラー: {e}")
            
            # 音声バッファのサイズ管理（5.1秒に制限）
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]
        
        # ファイル終端で残存VADバッファを強制処理
        print(f"[EOF][DEBUG] 残存VADIteratorバッファサイズ: {len(vad_iterator_buffer)}サンプル")
        while len(vad_iterator_buffer) > 0:
            # 残存データを32msずつ処理（不足分はゼロパディング）
            chunk_32ms_samples = int(sample_rate * 0.032)
            if len(vad_iterator_buffer) < chunk_32ms_samples:
                # 不足分をゼロパディング
                chunk_32ms = np.pad(vad_iterator_buffer, (0, chunk_32ms_samples - len(vad_iterator_buffer)))
                vad_iterator_buffer = np.array([], dtype=np.float32)
                print(f"[EOF][DEBUG] 最後のチャンク処理（ゼロパディング適用）: {len(chunk_32ms)}サンプル")
            else:
                chunk_32ms = vad_iterator_buffer[:chunk_32ms_samples]
                vad_iterator_buffer = vad_iterator_buffer[chunk_32ms_samples:]
                print(f"[EOF][DEBUG] 残存チャンク処理: {len(chunk_32ms)}サンプル")
            
            try:
                # VADIterator実行
                vad_result = vad_iterator(chunk_32ms)
                current_time = datetime.now()
                
                # 無声継続チェック
                if vad_iterator_mode == "DETECTING_SILENCE" and not vad_iterator_speech_detected and vad_iterator_silent_start_time and not vad_iterator_100ms_detected:
                    silent_duration_ms = (current_time - vad_iterator_silent_start_time).total_seconds() * 1000
                    print(f"[EOF][SILENCE_DEBUG] 最終無声継続: {silent_duration_ms:.1f}ms")
                    
                    if silent_duration_ms >= 100:
                        print(f"[EOF][VAD_ITERATOR] 最終処理で100ms無声区間検出、TurnTaking実行!")
                        vad_iterator_100ms_detected = True
                        
                        # TurnTaking実行
                        try:
                            process_start_time = time.perf_counter()
                            
                            # 100ms削った音声で実行
                            samples_100ms = int(sample_rate * 0.1)
                            if sound.shape[0] > samples_100ms:
                                sound_for_tt = sound[:-samples_100ms]
                            else:
                                sound_for_tt = sound
                            
                            sound_comp = sound_for_tt / np.abs(sound_for_tt).max()
                            
                            # 5秒分に制限
                            max_samples = int(5 * sample_rate)
                            if len(sound_comp) > max_samples:
                                sound_comp = sound_comp[:max_samples]
                            
                            pred, probability = self.tt_model.predict(sound_comp, threshold=self.TurnJudgeThreshold)
                            processing_time = (time.perf_counter() - process_start_time) * 1000
                            
                            # 結果を保存
                            tt_result = {
                                'file_time_ms': total_chunks * 10,
                                'timestamp': current_time.strftime('%H:%M:%S.%f')[:-3],
                                'prediction': pred,
                                'probability': probability,
                                'processing_time_ms': processing_time,
                                'audio_length_sec': len(sound_for_tt) / sample_rate,
                                'audio_length_samples': len(sound_for_tt),
                                'trigger': 'EOF_FINAL'
                            }
                            turntaking_results.append(tt_result)
                            
                            print(f"[EOF][VAD_ITERATOR] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                            print(f"[EOF][VAD_ITERATOR] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                            
                        except Exception as e:
                            print(f"[ERROR] 最終TurnTaking実行エラー: {e}")
                        
                        break  # TurnTaking実行後はループを終了
                
                if vad_result is not None:
                    print(f"[EOF][VAD_DETAIL] 最終VAD結果: {vad_result}")
                    
            except Exception as e:
                print(f"[ERROR] 最終VADIterator処理エラー: {e}")
                break
        
        # 従来の最終チェック（上記で処理されていない場合のフォールバック）
        if vad_iterator_mode == "DETECTING_SILENCE" and not vad_iterator_speech_detected and vad_iterator_silent_start_time and not vad_iterator_100ms_detected:
            final_silent_duration_ms = (datetime.now() - vad_iterator_silent_start_time).total_seconds() * 1000
            print(f"[EOF][FINAL_CHECK] フォールバック最終無声区間の長さ: {final_silent_duration_ms:.1f}ms")
            
            if final_silent_duration_ms >= 100:
                print(f"[EOF][VAD_ITERATOR] ファイル終端で100ms無声区間検出、TurnTaking実行!")
                try:
                    process_start_time = time.perf_counter()
                    
                    # 100ms削った音声で実行
                    samples_100ms = int(sample_rate * 0.1)
                    if sound.shape[0] > samples_100ms:
                        sound_for_tt = sound[:-samples_100ms]
                    else:
                        sound_for_tt = sound
                    
                    sound_comp = sound_for_tt / np.abs(sound_for_tt).max()
                    
                    # 5秒分に制限
                    max_samples = int(5 * sample_rate)
                    if len(sound_comp) > max_samples:
                        sound_comp = sound_comp[:max_samples]
                    
                    pred, probability = self.tt_model.predict(sound_comp, threshold=self.TurnJudgeThreshold)
                    processing_time = (time.perf_counter() - process_start_time) * 1000
                    
                    # 結果を保存
                    tt_result = {
                        'file_time_ms': total_chunks * 10,  # ファイル終端時刻
                        'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                        'prediction': pred,
                        'probability': probability,
                        'processing_time_ms': processing_time,
                        'audio_length_sec': len(sound_for_tt) / sample_rate,
                        'audio_length_samples': len(sound_for_tt),
                        'trigger': 'EOF'
                    }
                    turntaking_results.append(tt_result)
                    
                    print(f"[EOF][VAD_ITERATOR] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                    print(f"[EOF][VAD_ITERATOR] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                    
                except Exception as e:
                    print(f"[ERROR] ファイル終端TurnTaking実行エラー: {e}")
        
        print("-" * 70)
        print("処理完了")
        print(f"総ファイル時間: {total_chunks * 10}ms ({(total_chunks * 10)/1000:.2f}秒)")
        print(f"TurnTaking実行回数: {len(turntaking_results)}")
        
        # TurnTaking結果の詳細表示
        if turntaking_results:
            print("\n=== TurnTaking結果詳細 ===")
            print("時刻[ms] | 予測 | 確率  | 処理時間[ms] | 音声長[s]")
            print("-" * 55)
            for result in turntaking_results:
                print(f"{result['file_time_ms']:8d} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f}")
        
        return turntaking_results

    def test_vad_iterator_turntaking_realtime_sim(self):
        """実時間シミュレーション版VADIteratorベースのTurnTaking処理をテスト"""
        print("\\n" + "=" * 70)
        print("VADIterator TurnTaking 実時間シミュレーションテスト")
        print("=" * 70)
        
        # 音声ファイル読み込み
        audio_data, sample_rate = self.load_audio_file()
        if audio_data is None:
            return
        
        # VADIterator初期化（turnTaking.pyの設定に準拠）
        vad_iterator = self.VADIterator(
            self.silero_vad_model,
            threshold=0.5,
            sampling_rate=sample_rate,
            min_silence_duration_ms=100,
            speech_pad_ms=30
        )
        
        # VADIterator用変数
        vad_iterator_buffer = np.array([], dtype=np.float32)
        vad_iterator_speech_detected = False
        vad_iterator_silent_start_time = None
        vad_iterator_100ms_detected = False
        vad_iterator_mode = "WAITING_SPEECH_30MS"
        
        # 音声データバッファ
        sound = np.empty(0, dtype='float32')
        
        # TurnTaking結果を保存
        turntaking_results = []
        
        print(f"[INFO] 実時間シミュレーション設定: 32ms毎にVAD処理、実時間待機あり")
        print(f"[INFO] VADIterator設定: min_silence=100ms, speech_pad=30ms")
        print("時刻フォーマット: [実経過時間ms][VAD_ITERATOR] メッセージ")
        print("-" * 70)
        
        # 開始時刻を記録
        simulation_start_time = time.perf_counter()
        
        # 10ms毎に音声ファイルを処理
        total_chunks = (len(audio_data) + self.chunk_10ms_samples - 1) // self.chunk_10ms_samples
        
        for chunk_idx in range(total_chunks):
            # 10msチャンクを取得
            start_idx = chunk_idx * self.chunk_10ms_samples
            end_idx = min(start_idx + self.chunk_10ms_samples, len(audio_data))
            audiodata = audio_data[start_idx:end_idx]
            
            # 不足分をゼロパディング
            if len(audiodata) < self.chunk_10ms_samples:
                audiodata = np.pad(audiodata, (0, self.chunk_10ms_samples - len(audiodata)))
            
            # 音声データバッファに追加
            sound = np.concatenate([sound, audiodata])
            
            # VADIterator処理（turnTaking.pyから移植）
            vad_iterator_buffer = np.concatenate([vad_iterator_buffer, audiodata])
            
            # 32ms分のデータが溜まったら処理
            chunk_32ms_samples = int(sample_rate * 0.032)
            while len(vad_iterator_buffer) >= chunk_32ms_samples:
                chunk_32ms = vad_iterator_buffer[:chunk_32ms_samples]
                vad_iterator_buffer = vad_iterator_buffer[chunk_32ms_samples:]
                
                # 実時間シミュレーション：32ms待機
                time.sleep(0.032)
                
                # 実際の経過時間を計算
                elapsed_time_ms = (time.perf_counter() - simulation_start_time) * 1000
                current_datetime = datetime.now()
                
                try:
                    # VADIterator実行
                    vad_result = vad_iterator(chunk_32ms)
                    
                    # VADIterator結果の解析
                    speech_start = False
                    speech_end = False
                    
                    if vad_result is not None and isinstance(vad_result, dict):
                        if 'start' in vad_result:
                            speech_start = True
                        elif 'end' in vad_result:
                            speech_end = True
                    
                    # webRTCVAD準拠の状態管理
                    if vad_iterator_mode == "WAITING_SPEECH_30MS":
                        if speech_start:
                            vad_iterator_speech_detected = True
                            vad_iterator_mode = "DETECTING_SILENCE"
                            vad_iterator_silent_start_time = None
                            vad_iterator_100ms_detected = False
                            print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 30ms音声復帰検出、無声検出モードに移行")
                    
                    elif vad_iterator_mode == "DETECTING_SILENCE":
                        if speech_start and not vad_iterator_speech_detected:
                            vad_iterator_speech_detected = True
                            vad_iterator_silent_start_time = None
                            vad_iterator_100ms_detected = False
                            print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 音声再開、無声区間リセット")
                        elif speech_end and vad_iterator_speech_detected:
                            vad_iterator_speech_detected = False
                            vad_iterator_silent_start_time = current_datetime
                            vad_iterator_100ms_detected = False
                            print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 音声終了、無声区間開始")
                        
                        # 100ms無声継続チェック（実時間ベース）
                        if not vad_iterator_speech_detected and vad_iterator_silent_start_time and not vad_iterator_100ms_detected:
                            silent_duration_ms = (current_datetime - vad_iterator_silent_start_time).total_seconds() * 1000
                            
                            if silent_duration_ms >= 10:
                                print(f"[{elapsed_time_ms:06.0f}ms][SILENCE_DEBUG] 無声継続: {silent_duration_ms:.1f}ms")
                            
                            if silent_duration_ms >= 100:
                                print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 100ms無声区間検出完了、TurnTaking実行!")
                                vad_iterator_100ms_detected = True
                                
                                # TurnTakingモデル実行
                                try:
                                    process_start_time = time.perf_counter()
                                    
                                    # 100ms削った音声で実行
                                    samples_100ms = int(sample_rate * 0.1)
                                    if sound.shape[0] > samples_100ms:
                                        sound_for_tt = sound[:-samples_100ms]
                                    else:
                                        sound_for_tt = sound
                                    
                                    sound_comp = sound_for_tt / np.abs(sound_for_tt).max()
                                    
                                    # 5秒分に制限
                                    max_samples = int(5 * sample_rate)
                                    if len(sound_comp) > max_samples:
                                        sound_comp = sound_comp[:max_samples]
                                    
                                    pred, probability = self.tt_model.predict(sound_comp, threshold=self.TurnJudgeThreshold)
                                    processing_time = (time.perf_counter() - process_start_time) * 1000
                                    
                                    # 結果を保存
                                    tt_result = {
                                        'elapsed_time_ms': elapsed_time_ms,
                                        'timestamp': current_datetime.strftime('%H:%M:%S.%f')[:-3],
                                        'prediction': pred,
                                        'probability': probability,
                                        'processing_time_ms': processing_time,
                                        'audio_length_sec': len(sound_for_tt) / sample_rate,
                                        'audio_length_samples': len(sound_for_tt),
                                        'trigger': 'REALTIME_SIM'
                                    }
                                    turntaking_results.append(tt_result)
                                    
                                    print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                                    print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                                    
                                    # 次の状態に移行
                                    vad_iterator_mode = "WAITING_SPEECH_30MS"
                                    print(f"[{elapsed_time_ms:06.0f}ms][VAD_ITERATOR] 30ms音声復帰待機モードに移行")
                                    
                                except Exception as e:
                                    print(f"[ERROR] TurnTaking実行エラー: {e}")
                    
                    # 詳細ログ（結果がある場合のみ）
                    if vad_result is not None:
                        print(f"[{elapsed_time_ms:06.0f}ms][VAD_DETAIL] 結果: {vad_result}, モード: {vad_iterator_mode}")
                        
                except Exception as e:
                    print(f"[ERROR] VADIterator処理エラー: {e}")
            
            # 音声バッファのサイズ管理
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]
        
        print("-" * 70)
        print("実時間シミュレーションテスト完了")
        print(f"総実行時間: {(time.perf_counter() - simulation_start_time) * 1000:.0f}ms")
        print(f"TurnTaking実行回数: {len(turntaking_results)}")
        
        return turntaking_results

    def test_vad_iterator_simple(self):
        """VADIteratorの100ms判定に完全依存したシンプルなTurnTaking処理"""
        print("\\n" + "=" * 70)
        print("VADIterator シンプル版 TurnTakingテスト")
        print("=" * 70)
        
        # 音声ファイル読み込み
        audio_data, sample_rate = self.load_audio_file()
        if audio_data is None:
            return
        
        # VADIterator初期化（100ms無声判定をIteratorに完全委任）
        vad_iterator = self.VADIterator(
            self.silero_vad_model,
            threshold=0.5,
            sampling_rate=sample_rate,
            min_silence_duration_ms=100,    # 100ms無声でend判定
            speech_pad_ms=30                # 30ms音声でstart判定
        )
        
        # シンプルな状態管理
        vad_iterator_buffer = np.array([], dtype=np.float32)
        sound = np.empty(0, dtype='float32')
        turntaking_results = []
        
        # 状態追跡（シンプル化）
        last_speech_detected = False
        
        print(f"[INFO] VADIterator設定: min_silence=100ms（Iterator判定）, speech_pad=30ms")
        print(f"[INFO] 処理方針: VADIterator end判定で即座にTurnTaking実行")
        print("時刻フォーマット: [ファイル内時刻ms][VAD_SIMPLE] メッセージ")
        print("-" * 70)
        
        # 10ms毎に音声ファイルを処理
        total_chunks = (len(audio_data) + self.chunk_10ms_samples - 1) // self.chunk_10ms_samples
        
        for chunk_idx in range(total_chunks):
            # 現在のファイル内時刻
            file_time_ms = chunk_idx * 10
            
            # 10msチャンクを取得
            start_idx = chunk_idx * self.chunk_10ms_samples
            end_idx = min(start_idx + self.chunk_10ms_samples, len(audio_data))
            audiodata = audio_data[start_idx:end_idx]
            
            # 不足分をゼロパディング
            if len(audiodata) < self.chunk_10ms_samples:
                audiodata = np.pad(audiodata, (0, self.chunk_10ms_samples - len(audiodata)))
            
            # 音声データバッファに追加
            sound = np.concatenate([sound, audiodata])
            
            # VADIterator処理
            vad_iterator_buffer = np.concatenate([vad_iterator_buffer, audiodata])
            
            # 32ms分のデータが溜まったら処理
            chunk_32ms_samples = int(sample_rate * 0.032)
            while len(vad_iterator_buffer) >= chunk_32ms_samples:
                chunk_32ms = vad_iterator_buffer[:chunk_32ms_samples]
                vad_iterator_buffer = vad_iterator_buffer[chunk_32ms_samples:]
                
                try:
                    # VADIterator実行
                    vad_result = vad_iterator(chunk_32ms)
                    
                    if vad_result is not None:
                        print(f"[{file_time_ms:04d}ms][VAD_SIMPLE] VAD結果: {vad_result}")
                        
                        # start判定: 音声開始
                        if 'start' in vad_result:
                            last_speech_detected = True
                            print(f"[{file_time_ms:04d}ms][VAD_SIMPLE] 音声開始検出")
                        
                        # end判定: VADIteratorが100ms無声を判定 → 即座にTurnTaking実行
                        elif 'end' in vad_result and last_speech_detected:
                            last_speech_detected = False
                            print(f"[{file_time_ms:04d}ms][VAD_SIMPLE] 100ms無声検出（VADIterator判定）→ TurnTaking実行!")
                            
                            # TurnTakingモデル実行
                            try:
                                process_start_time = time.perf_counter()
                                
                                # 100ms削った音声で実行
                                samples_100ms = int(sample_rate * 0.1)
                                if sound.shape[0] > samples_100ms:
                                    sound_for_tt = sound[:-samples_100ms]
                                else:
                                    sound_for_tt = sound
                                
                                if len(sound_for_tt) == 0:
                                    print(f"[{file_time_ms:04d}ms][WARNING] 音声データが不足、TurnTakingスキップ")
                                    continue
                                
                                # 正規化
                                sound_comp = sound_for_tt / np.abs(sound_for_tt).max()
                                
                                # 5秒分に制限
                                max_samples = int(5 * sample_rate)
                                if len(sound_comp) > max_samples:
                                    sound_comp = sound_comp[:max_samples]
                                
                                # TurnTaking推論実行
                                pred, probability = self.tt_model.predict(sound_comp, threshold=self.TurnJudgeThreshold)
                                processing_time = (time.perf_counter() - process_start_time) * 1000
                                
                                # 結果を保存
                                tt_result = {
                                    'file_time_ms': file_time_ms,
                                    'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                    'prediction': pred,
                                    'probability': probability,
                                    'processing_time_ms': processing_time,
                                    'audio_length_sec': len(sound_for_tt) / sample_rate,
                                    'audio_length_samples': len(sound_for_tt),
                                    'trigger': 'VAD_ITERATOR_END'
                                }
                                turntaking_results.append(tt_result)
                                
                                print(f"[{file_time_ms:04d}ms][VAD_SIMPLE] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                                print(f"[{file_time_ms:04d}ms][VAD_SIMPLE] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")
                                
                            except Exception as e:
                                print(f"[ERROR] TurnTaking実行エラー: {e}")
                        
                except Exception as e:
                    print(f"[ERROR] VADIterator処理エラー: {e}")
            
            # 音声バッファのサイズ管理
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]
        
        print("-" * 70)
        print("VADIterator シンプル版テスト完了")
        print(f"総ファイル時間: {total_chunks * 10}ms ({(total_chunks * 10)/1000:.2f}秒)")
        print(f"TurnTaking実行回数: {len(turntaking_results)}")
        
        # TurnTaking結果の詳細表示
        if turntaking_results:
            print("\\n=== TurnTaking結果詳細（VADIterator判定） ===")
            print("時刻[ms] | 予測 | 確率  | 処理時間[ms] | 音声長[s]")
            print("-" * 55)
            for result in turntaking_results:
                print(f"{result['file_time_ms']:8d} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f}")
        
        return turntaking_results

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='TurnTaking音声ファイルテスト')
    parser.add_argument('audio_file', help='テスト用音声ファイルパス')
    parser.add_argument('--save-results', help='結果を保存するJSONファイルパス（任意）')
    parser.add_argument('--all-tests', action='store_true', help='全テスト実行（デフォルトはシンプル版のみ）')
    
    args = parser.parse_args()
    
    # 音声ファイルの存在確認
    if not os.path.exists(args.audio_file):
        print(f"[ERROR] 音声ファイルが見つかりません: {args.audio_file}")
        sys.exit(1)
    
    print("TurnTaking音声ファイルテスト - 完全版")
    print("=" * 70)
    
    # テスト実行
    test = TurnTakingAudioFileTest(args.audio_file)
    
    if args.all_tests:
        # 全テスト実行
        print("=== VADIteratorシンプル版テスト（推奨） ===")
        results_simple = test.test_vad_iterator_simple()
        
        print("\\n=== 標準テスト（参考・ファイルベース処理） ===")
        results_standard = test.test_vad_iterator_turntaking()
        
        print("\\n=== 実時間シミュレーションテスト（参考） ===")
        results_realtime = test.test_vad_iterator_turntaking_realtime_sim()
        
        # 結果統合
        results = results_simple + results_standard + results_realtime
    else:
        # VADIteratorシンプル版テストのみ実行（デフォルト）
        results = test.test_vad_iterator_simple()
    
    # 結果保存（任意）
    if args.save_results and results:
        import json
        with open(args.save_results, 'w', encoding='utf-8') as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
        print(f"\n[INFO] 結果を保存しました: {args.save_results}")

if __name__ == "__main__":
    main()