#!/usr/bin/env python3
"""
turnTaking.pyのVAD設定に完全に合わせた検証スクリプト
- フレーム長: 10ms（turnTaking.pyと同じ）
- VADモード: 3（アグレッシブ、ノイズ耐性高）
- 200ms判定: sound_count >= 20フレーム
- 100ms判定: silent_count >= 10フレーム
- バッファ条件: 5秒チェックなし（turnTaking.pyと同じ）

複数の音声ファイル（script1-10）をストリーミング処理して、
turnTakingモデル実行タイミングを検証
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
import glob

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

def test_vad_streaming(audio_files, save_results=None):
    """turnTaking.py設定でのVAD検証 - ストリーミング処理"""
    print("=" * 80)
    print("turnTaking.py設定でのVAD検証 - ストリーミング処理")
    print("=" * 80)
    print(f"[INFO] 処理音声ファイル数: {len(audio_files)}")
    print(f"[INFO] フレーム長: 10ms（turnTaking.pyと同じ）")
    print(f"[INFO] VADモード: 3（アグレッシブ）")
    print(f"[INFO] 200ms判定: sound_count >= 20フレーム")
    print(f"[INFO] 100ms判定: silent_count >= 10フレーム")
    print(f"[INFO] バッファ条件: なし（5秒チェックなし - turnTaking.pyと同じ）")
    print("=" * 80)

    # TurnTakingモデル初期化
    tt_model = TurnTakingModel()
    TurnJudgeThreshold = 0.650

    # turnTaking.py設定
    sample_rate = 16000
    frame_duration = 10  # ms - turnTaking.pyと同じ
    CHUNK = int(sample_rate * frame_duration / 1000)  # 160サンプル

    # WebRTC VAD初期化
    webrtc_vad = webrtcvad.Vad()
    webrtc_vad.set_mode(3)  # mode=3（アグレッシブ）- turnTaking.pyと同じ

    # 状態管理変数
    sound = np.empty(0, dtype='float32')
    sound_available = False
    sound_count = 0  # 連続音声フレームカウント
    silent_count = 0
    silent_start_time = None
    speech_detected_for_200ms = False  # 200ms以上の音声検出フラグ

    turntaking_results = []
    total_processed_samples = 0
    total_files_processed = 0

    print("\n[INFO] ストリーミング処理開始")
    print("-" * 80)

    # 各音声ファイルを処理
    for file_idx, audio_file in enumerate(audio_files, 1):
        try:
            # 音声ファイル読み込み
            audio_data, sr = sf.read(audio_file)
            print(f"\n[FILE {file_idx}/{len(audio_files)}] {os.path.basename(audio_file)}")
            print(f"  サンプリングレート: {sr}Hz, 長さ: {len(audio_data)}サンプル ({len(audio_data)/sr:.2f}秒)")

            # 16kHzにリサンプル
            if sr != 16000:
                print(f"  [WARNING] サンプリングレート {sr}Hz を 16kHz と仮定して処理")
                sr = 16000

            # モノラル変換
            if audio_data.ndim > 1:
                audio_data = np.mean(audio_data, axis=1)

            # float32に変換
            audio_data = audio_data.astype(np.float32)

            # 10msチャンク単位で処理
            total_chunks = (len(audio_data) + CHUNK - 1) // CHUNK
            file_time_ms = 0

            for chunk_idx in range(total_chunks):
                # ファイル内時刻
                file_time_ms = chunk_idx * frame_duration

                # 10msチャンクを取得
                start_idx = chunk_idx * CHUNK
                end_idx = min(start_idx + CHUNK, len(audio_data))
                audiodata = audio_data[start_idx:end_idx]

                # 不足分をゼロパディング
                if len(audiodata) < CHUNK:
                    audiodata = np.pad(audiodata, (0, CHUNK - len(audiodata)))

                # 音声バッファに追加
                sound = np.concatenate([sound, audiodata])

                # webRTC VAD判定（turnTaking.pyと同じ）
                audio_int16 = (audiodata * 32767).astype(np.int16)
                audio_bytes = audio_int16.tobytes()

                current_time = datetime.now()
                timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]

                try:
                    is_speech = webrtc_vad.is_speech(audio_bytes, sample_rate)

                    if not is_speech:
                        # 無音フレーム検出
                        if silent_start_time is None:
                            # 無音開始
                            if sound_count > 0:
                                print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] 無音開始検出 (直前音声フレーム数: {sound_count}, 音声時間: {sound_count * frame_duration:.0f}ms, 200msフラグ: {speech_detected_for_200ms})")
                            silent_start_time = current_time
                            silent_count = 1
                            sound_count = 0
                        else:
                            # 無音継続
                            silent_count += 1

                            # 100ms無音検出（フレーム数ベース）
                            if silent_count >= (100 / frame_duration):  # 100ms / 10ms = 10フレーム
                                print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] 100ms無音検出 (speech_flag={speech_detected_for_200ms}, sound_available={sound_available})")

                            if silent_count >= (100 / frame_duration) and not sound_available and speech_detected_for_200ms:
                                print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] ★★★ TurnTaking実行! ★★★")

                                sound_available = True

                                # TurnTakingモデル実行
                                try:
                                    process_start_time = time.perf_counter()

                                    # 100ms削った音声を使用
                                    samples_100ms = int(sample_rate * 0.1)
                                    if sound.shape[0] > samples_100ms:
                                        sound_for_tt = sound[:-samples_100ms]
                                    else:
                                        sound_for_tt = sound

                                    if len(sound_for_tt) == 0:
                                        print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] [WARNING] 音声データ不足、スキップ")
                                    else:
                                        sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                                        # 5秒に制限
                                        max_samples = int(5 * sample_rate)
                                        if len(sound_comp) > max_samples:
                                            sound_comp = sound_comp[:max_samples]

                                        pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                                        processing_time = (time.perf_counter() - process_start_time) * 1000

                                        # 結果を保存
                                        tt_result = {
                                            'file_idx': file_idx,
                                            'filename': os.path.basename(audio_file),
                                            'file_time_ms': file_time_ms,
                                            'timestamp': timestamp_str,
                                            'prediction': pred,
                                            'probability': probability,
                                            'processing_time_ms': processing_time,
                                            'audio_length_sec': len(sound_for_tt) / sample_rate,
                                            'audio_length_samples': len(sound_for_tt)
                                        }
                                        turntaking_results.append(tt_result)

                                        print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                                        print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")

                                        # フラグリセット（次のサイクルに備える）
                                        speech_detected_for_200ms = False
                                        sound_available = False
                                        sound_count = 0

                                except Exception as e:
                                    print(f"[ERROR] TurnTaking実行エラー: {e}")
                                    speech_detected_for_200ms = False
                                    sound_available = False
                                    sound_count = 0

                    else:
                        # 音声フレーム検出
                        if silent_start_time is not None:
                            # 無音から音声に復帰
                            print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] 音声復帰検出、無音状態リセット (前フラグ: {speech_detected_for_200ms})")
                            silent_start_time = None
                            silent_count = 0
                            sound_count = 0
                            sound_available = False

                        # 連続音声フレームをカウント
                        sound_count += 1

                        # 200ms以上の音声判定
                        if sound_count >= (200 / frame_duration):  # 200 / 10 = 20フレーム
                            if not speech_detected_for_200ms:
                                print(f"[{timestamp_str}][{file_idx}][{file_time_ms:04d}ms] 200ms以上の音声を検出 → TurnTaking実行準備完了")
                            speech_detected_for_200ms = True

                except Exception as e:
                    print(f"[ERROR] VAD処理エラー: {e}")

                # 音声バッファのサイズ管理（5.1秒に制限）
                if sound.shape[0] >= int(5.1 * sample_rate):
                    sound = sound[-int(5.1 * sample_rate):]

            total_processed_samples += len(audio_data)
            total_files_processed += 1

        except Exception as e:
            print(f"[ERROR] ファイル処理エラー ({audio_file}): {e}")
            continue

    print("-" * 80)
    print("\n[INFO] 処理完了")
    print(f"処理ファイル数: {total_files_processed}/{len(audio_files)}")
    print(f"総サンプル数: {total_processed_samples} ({total_processed_samples/sample_rate:.2f}秒)")
    print(f"TurnTaking実行回数: {len(turntaking_results)}")

    # 結果表示
    if turntaking_results:
        print("\n=== TurnTaking実行結果（時系列順） ===")
        print("ファイル | ファイル内時刻[ms] | 予測 | 確率  | 処理時間[ms] | 音声長[s]")
        print("-" * 75)
        for result in turntaking_results:
            print(f"script{result['file_idx']:2d}.wav | {result['file_time_ms']:15d} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f}")

    # 結果保存（任意）
    if save_results and turntaking_results:
        import json
        with open(save_results, 'w', encoding='utf-8') as f:
            json.dump(turntaking_results, f, ensure_ascii=False, indent=2)
        print(f"\n[INFO] 結果を保存しました: {save_results}")

    return turntaking_results

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='turnTaking.py設定でのVAD検証（ストリーミング処理）')
    parser.add_argument('--audio-dir', default='/workspace/DiaROS', help='音声ファイル検索ディレクトリ')
    parser.add_argument('--save-results', help='結果を保存するJSONファイルパス（任意）')

    args = parser.parse_args()

    # script1-10.wavを探す
    audio_files = sorted(glob.glob(os.path.join(args.audio_dir, 'script[0-9].wav'))) + \
                  sorted(glob.glob(os.path.join(args.audio_dir, 'script[0-9][0-9].wav')))

    if not audio_files:
        print(f"[ERROR] 音声ファイル（script*.wav）が見つかりません: {args.audio_dir}")
        sys.exit(1)

    print(f"[INFO] 検出されたファイル: {[os.path.basename(f) for f in audio_files]}")

    # テスト実行
    test_vad_streaming(audio_files, args.save_results)

if __name__ == "__main__":
    main()
