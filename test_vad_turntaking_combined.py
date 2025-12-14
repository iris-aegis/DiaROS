#!/usr/bin/env python3
"""
VADIterator + TurnTaking統合テスト
test_vad_comparison_fixed_audio.pyのVADIterator処理に、
test_turntaking_audio_file.pyと同様のTurnTaking推論を統合
"""

import torch
import numpy as np
import time
from datetime import datetime
import soundfile as sf
import sys
import argparse
import os
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import torch.nn as nn

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

def test_vad_turntaking_combined(audio_file, save_results=None):
    """VADIterator + TurnTaking統合テスト"""
    print("VADIterator + TurnTaking統合テスト")
    print("=" * 70)

    # SileroVADのロード
    print("[INFO] SileroVADロード中...")
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
        return None

    # 10msチャンクサイズ
    chunk_10ms_samples = int(sample_rate * 0.01)  # 160サンプル

    # VADIterator初期化
    vad_iterator = VADIterator(
        silero_vad_model,
        threshold=0.5,
        sampling_rate=sample_rate,
        min_silence_duration_ms=100,  # 100ms無声で音声終了判定
        speech_pad_ms=30               # 30ms音声継続で音声開始判定
    )
    vad_iterator_buffer = VADIteratorBuffer(sample_rate)
    vad_iterator_speech_detected = False
    vad_iterator_silent_start_time = None
    vad_iterator_silence_100ms_detected = False
    vad_iterator_total_events = 0

    # 音声データバッファ（TurnTaking用）
    sound = np.empty(0, dtype='float32')

    # TurnTaking結果を保存
    turntaking_results = []

    print("\n[INFO] 処理開始")
    print(f"[INFO] VADIterator設定: min_silence=100ms, speech_pad=30ms")
    print(f"[INFO] TurnTaking閾値: {TurnJudgeThreshold}")
    print("時刻フォーマット: [VAD_TT][ファイル内時刻ms] メッセージ")
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

        # VADIterator処理（32ms処理）
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
                if speech_end and vad_iterator_speech_detected:
                    # 音声終了検出
                    vad_iterator_speech_detected = False
                    vad_iterator_silent_start_time = file_time_ms
                    vad_iterator_silence_100ms_detected = False  # 無声区間検出フラグをリセット
                    print(f"[VAD_TT][{file_time_ms:04d}ms] 音声終了、無声区間開始 (type={result_type})")

                elif speech_start:
                    # 音声開始検出（無声区間中の音声再開も含む）
                    if vad_iterator_silent_start_time is not None:
                        # 無声区間中に音声再開 → 無声区間リセット
                        print(f"[VAD_TT][{file_time_ms:04d}ms] 音声再開、無声区間リセット")
                    else:
                        # 新規音声開始
                        print(f"[VAD_TT][{file_time_ms:04d}ms] 音声開始検出 (type={result_type})")

                    vad_iterator_speech_detected = True
                    vad_iterator_silent_start_time = None
                    vad_iterator_silence_100ms_detected = False

                # 詳細ログ（結果がある場合のみ）
                if vad_result is not None:
                    print(f"[VAD_TT][{file_time_ms:04d}ms] イベント#{vad_iterator_total_events} - {vad_result}")

            except Exception as e:
                print(f"[VAD_TT][{file_time_ms:04d}ms] 処理エラー: {e}")

        # 100ms無声継続チェック（10ms毎に常に実行）
        if not vad_iterator_speech_detected and vad_iterator_silent_start_time is not None and not vad_iterator_silence_100ms_detected:
            silent_duration_ms = file_time_ms - vad_iterator_silent_start_time
            if silent_duration_ms >= 100:
                print(f"[VAD_TT][{vad_iterator_silent_start_time:04d}ms] 100ms無声区間検出完了 → TurnTaking実行!")
                vad_iterator_silence_100ms_detected = True

                # TurnTakingモデル実行（100ms削った音声で実行）
                try:
                    process_start_time = time.perf_counter()

                    # 100ms（1600サンプル）削った音声を使用
                    samples_100ms = int(sample_rate * 0.1)
                    if sound.shape[0] > samples_100ms:
                        sound_for_tt = sound[:-samples_100ms]  # 最後の100msを削除
                    else:
                        sound_for_tt = sound  # 音声が短い場合はそのまま使用

                    if len(sound_for_tt) == 0:
                        print(f"[VAD_TT][{file_time_ms:04d}ms] [WARNING] 音声データが不足、TurnTakingスキップ")
                    else:
                        sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                        # 5秒分に制限
                        max_samples = int(5 * sample_rate)
                        if len(sound_comp) > max_samples:
                            sound_comp = sound_comp[:max_samples]

                        pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                        processing_time = (time.perf_counter() - process_start_time) * 1000

                        # 結果を保存
                        tt_result = {
                            'file_time_ms': file_time_ms,
                            'silent_start_time_ms': vad_iterator_silent_start_time,
                            'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                            'prediction': pred,
                            'probability': probability,
                            'processing_time_ms': processing_time,
                            'audio_length_sec': len(sound_for_tt) / sample_rate,
                            'audio_length_samples': len(sound_for_tt)
                        }
                        turntaking_results.append(tt_result)

                        print(f"[VAD_TT][{file_time_ms:04d}ms] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                        print(f"[VAD_TT][{file_time_ms:04d}ms] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")

                except Exception as e:
                    print(f"[ERROR] TurnTaking実行エラー: {e}")

        # 音声バッファのサイズ管理（5.1秒に制限）
        if sound.shape[0] >= int(5.1 * sample_rate):
            sound = sound[-int(5.1 * sample_rate):]

    # ファイル終端処理（残存VADバッファ処理）
    print(f"[EOF][DEBUG] 残存VADIteratorバッファサイズ: {len(vad_iterator_buffer.buffer)}サンプル")

    # 残存バッファを処理
    while len(vad_iterator_buffer.buffer) > 0:
        chunk_32ms_samples = int(sample_rate * 0.032)
        if len(vad_iterator_buffer.buffer) < chunk_32ms_samples:
            # 不足分をゼロパディング
            chunk_32ms = np.pad(vad_iterator_buffer.buffer, (0, chunk_32ms_samples - len(vad_iterator_buffer.buffer)))
            vad_iterator_buffer.buffer = np.array([], dtype=np.float32)
        else:
            chunk_32ms = vad_iterator_buffer.buffer[:chunk_32ms_samples]
            vad_iterator_buffer.buffer = vad_iterator_buffer.buffer[chunk_32ms_samples:]

        try:
            vad_result = vad_iterator(chunk_32ms)

            # 無声継続チェック
            if not vad_iterator_speech_detected and vad_iterator_silent_start_time is not None and not vad_iterator_silence_100ms_detected:
                silent_duration_ms = total_chunks * 10 - vad_iterator_silent_start_time

                if silent_duration_ms >= 100:
                    print(f"[EOF][VAD_TT] 最終処理で100ms無声区間検出 → TurnTaking実行!")
                    vad_iterator_silence_100ms_detected = True

                    # TurnTaking実行
                    try:
                        process_start_time = time.perf_counter()

                        # 100ms削った音声で実行
                        samples_100ms = int(sample_rate * 0.1)
                        if sound.shape[0] > samples_100ms:
                            sound_for_tt = sound[:-samples_100ms]
                        else:
                            sound_for_tt = sound

                        if len(sound_for_tt) > 0:
                            sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                            # 5秒分に制限
                            max_samples = int(5 * sample_rate)
                            if len(sound_comp) > max_samples:
                                sound_comp = sound_comp[:max_samples]

                            pred, probability = tt_model.predict(sound_comp, threshold=TurnJudgeThreshold)
                            processing_time = (time.perf_counter() - process_start_time) * 1000

                            # 結果を保存
                            tt_result = {
                                'file_time_ms': total_chunks * 10,
                                'silent_start_time_ms': vad_iterator_silent_start_time,
                                'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                                'prediction': pred,
                                'probability': probability,
                                'processing_time_ms': processing_time,
                                'audio_length_sec': len(sound_for_tt) / sample_rate,
                                'audio_length_samples': len(sound_for_tt),
                                'trigger': 'EOF'
                            }
                            turntaking_results.append(tt_result)

                            print(f"[EOF][VAD_TT] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms")
                            print(f"[EOF][VAD_TT] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)")

                    except Exception as e:
                        print(f"[ERROR] 最終TurnTaking実行エラー: {e}")

                    break  # TurnTaking実行後はループを終了

            if vad_result is not None:
                print(f"[EOF][VAD_DETAIL] 最終VAD結果: {vad_result}")

        except Exception as e:
            print(f"[ERROR] 最終VADIterator処理エラー: {e}")
            break

    print("-" * 70)
    print("処理完了")
    print(f"総ファイル時間: {total_chunks * 10}ms ({(total_chunks * 10)/1000:.2f}秒)")
    print(f"VADIterator実行回数: {vad_iterator_total_events}")
    print(f"TurnTaking実行回数: {len(turntaking_results)}")

    # TurnTaking結果の詳細表示
    if turntaking_results:
        print("\n=== TurnTaking結果詳細 ===")
        print("時刻[ms] | 無声開始[ms] | 予測 | 確率  | 処理時間[ms] | 音声長[s]")
        print("-" * 70)
        for result in turntaking_results:
            silent_start = result.get('silent_start_time_ms', 0)
            print(f"{result['file_time_ms']:8d} | {silent_start:12d} | {result['prediction']:4d} | {result['probability']:5.3f} | {result['processing_time_ms']:11.1f} | {result['audio_length_sec']:9.2f}")

    # 結果保存（任意）
    if save_results and turntaking_results:
        import json
        with open(save_results, 'w', encoding='utf-8') as f:
            json.dump(turntaking_results, f, ensure_ascii=False, indent=2)
        print(f"\n[INFO] 結果を保存しました: {save_results}")

    return turntaking_results

def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(description='VADIterator + TurnTaking統合テスト')
    parser.add_argument('audio_file', help='テスト用音声ファイルパス')
    parser.add_argument('--save-results', help='結果を保存するJSONファイルパス（任意）')

    args = parser.parse_args()

    # 音声ファイルの存在確認
    if not os.path.exists(args.audio_file):
        print(f"[ERROR] 音声ファイルが見つかりません: {args.audio_file}")
        sys.exit(1)

    # テスト実行
    test_vad_turntaking_combined(args.audio_file, args.save_results)

if __name__ == "__main__":
    main()
