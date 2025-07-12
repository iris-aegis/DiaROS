### turnTaking.py ###
""""
新仕様 - 常時推論版
- 100msごとに常にモデル推論を実行
- 無声区間の待機なし
- 5秒間のスライディングウィンドウで常時ターンテイキング判定
- リアルタイム性の向上
"""
import numpy as np
import webrtcvad
import time
import queue
import sys
from scipy.io.wavfile import write
import torch
import torch.nn as nn
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import transformers
from datetime import datetime
transformers.logging.set_verbosity_error()

# グローバルキュー（ros2_turn_taking.py から共有）
stream_queue = queue.Queue()
turn_taking_result_queue = queue.Queue()

THRESHOLD = 0.75

def push_audio_data(data):
    stream_queue.put(data)
    sys.stdout.flush()

def get_audio_data():
    if not stream_queue.empty():
        data = stream_queue.get()
        sys.stdout.flush()
        return data
    sys.stdout.flush()
    return None

class TurnTakingModel:
    def __init__(self, model_id="SiRoZaRuPa/japanese-wav2vec2-base-turntaking-CSJ"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[TurnTaking] 使用デバイス: {self.device}")
        self.model = Wav2Vec2ForSequenceClassification.from_pretrained(
            model_id, token=True
        ).to(self.device)
        self.model.eval()
        self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(
            model_id, token=True
        )
        print("[TurnTaking] モデル読み込み完了")

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

def TurnTaking():
    model = TurnTakingModel()
    
    sample_rate = 16000
    frame_duration = 10  # ms
    CHUNK = int(sample_rate * frame_duration / 1000)  # 160サンプル
    
    # 常時推論設定
    inference_interval_ms = 100  # 100msごとに推論
    inference_interval_samples = int(sample_rate * inference_interval_ms / 1000)  # 1600サンプル
    
    sound_buffer = np.empty(0, dtype='float32')
    last_inference_time = time.time()
    samples_since_last_inference = 0
    inference_count = 0

    sys.stdout.write("[INFO] TurnTaking started - 常時推論モード\n")
    sys.stdout.write(f"[INFO] 推論間隔: {inference_interval_ms}ms\n")
    sys.stdout.flush()

    while True:
        try:
            audiodata = get_audio_data()
            if audiodata is None:
                time.sleep(0.001)  # 1ms待機
                continue

            # 音声データをバッファに追加
            sound_buffer = np.concatenate([sound_buffer, audiodata])
            samples_since_last_inference += len(audiodata)
            
            # 5.1秒を超えた分は削除（スライディングウィンドウ）
            max_samples = int(5.1 * sample_rate)
            if len(sound_buffer) > max_samples:
                sound_buffer = sound_buffer[-max_samples:]

            # 推論条件: 100ms間隔 AND 5秒分のデータが蓄積
            current_time = time.time()
            should_inference = (
                len(sound_buffer) >= 5 * sample_rate and  # 5秒分のデータ
                samples_since_last_inference >= inference_interval_samples  # 100ms間隔
            )

            if should_inference:
                # 推論実行
                process_start_time = time.perf_counter()
                
                # 最新5秒分のデータで推論
                inference_data = sound_buffer[-int(5 * sample_rate):]
                
                # 正規化
                if np.abs(inference_data).max() > 0:
                    sound_comp = inference_data / np.abs(inference_data).max()
                else:
                    sound_comp = inference_data
                
                # 推論実行
                pred, probability = model.predict(sound_comp, threshold=THRESHOLD)
                turn_taking_result_queue.put((pred, probability))
                
                processing_time = (time.perf_counter() - process_start_time) * 1000
                inference_count += 1
                
                # 遅延測定ログ出力
                timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                sys.stdout.write(f"[🔄 TT_INFERENCE] {timestamp_str} | 推論#{inference_count} | 処理時間: {processing_time:.1f}ms | 確信度: {probability:.3f} | 判定: {'ターン交代' if pred else '継続'}\n")
                sys.stdout.flush()
                
                # 推論カウンターリセット
                samples_since_last_inference = 0
                last_inference_time = current_time
                
                # デバッグ用: 音声ファイル保存（最初の10回のみ）
                if inference_count <= 10:
                    try:
                        write(f'model_input_sound_{inference_count}.wav', sample_rate, 
                              (sound_comp * 32767).astype(np.int16))
                    except Exception as e:
                        pass  # ファイル保存エラーは無視

        except KeyboardInterrupt:
            sys.stdout.write(f"\n[INFO] TurnTaking terminated - 総推論回数: {inference_count}\n")
            sys.stdout.flush()
            break
        except Exception as e:
            sys.stdout.write(f"[ERROR] TurnTaking error: {e}\n")
            sys.stdout.flush()
            time.sleep(0.1)