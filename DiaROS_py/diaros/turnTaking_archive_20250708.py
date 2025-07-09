### turnTaking.py ###
""""
仕様
200ms以上の無音でバッファ削除 フラグを建てる
200ms以上のバッファ && 200ms以上の無音 -> 音声をモデルに入力
フラグが立っている状態で音声が入力される -> フラグを消してバッファに音声を貯める
200ms未満のバッファ && 200ms以上の無音 -> バッファ削除 && フラグを建てる
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
transformers.logging.set_verbosity_error()

# グローバルキュー（ros2_turn_taking.py から共有）
stream_queue = queue.Queue()
turn_taking_result_queue = queue.Queue()

THRESHOLD = 0.75

def push_audio_data(data):
    stream_queue.put(data)
    # 追加: キューサイズを表示
    # sys.stdout.write(f"[turnTaking.py] push_audio_data: stream_queue size={stream_queue.qsize()}\n")
    sys.stdout.flush()

def get_audio_data():
    if not stream_queue.empty():
        data = stream_queue.get()
        # sys.stdout.write(f"[turnTaking.py] get_audio_data: stream_queue size={stream_queue.qsize()}\n")
        sys.stdout.flush()
        return data
    # sys.stdout.write(f"[turnTaking.py] get_audio_data: stream_queue EMPTY\n")
    sys.stdout.flush()
    return None

    
# draw bar ##############################################################
def draw_bar(volume, max_volume=1.0, bar_length=50):
    """
    音量に応じたバーをCUIに表示します。
    volume: 現在の音量
    max_volume: 最大音量値
    bar_length: バーの最大の長さ
    """
    normalized_volume = int((volume / max_volume) * bar_length)
    bar = "■" * normalized_volume + " " * (bar_length - normalized_volume)
    # sys.stdout.write("\r" + bar)
    # sys.stdout.flush()

class TurnTakingModel:
    def __init__(self, model_id="SiRoZaRuPa/japanese-wav2vec2-base-turntaking-CSJ"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = Wav2Vec2ForSequenceClassification.from_pretrained(
            model_id, token=True
        ).to(self.device)
        self.model.eval()
        self.feature_extractor = Wav2Vec2FeatureExtractor.from_pretrained(
            model_id, token=True
        )

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
    vad = webrtcvad.Vad()
    vad.set_mode(3)

    sample_rate = 16000
    frame_duration = 10  # ms
    CHUNK = int(sample_rate * frame_duration / 1000)  # 160サンプル
    TurnJudgeThreshold = 0.650

    sound = np.empty(0, dtype='float32')
    sound_available = False
    sound_count = 0
    silent_count = 0

    sys.stdout.write("[INFO] TurnTaking started\n")
    sys.stdout.flush()

    BAR_MEM = 20  # バーの長さ
    while True:
        try:
            audiodata = get_audio_data()
            if audiodata is None:
                time.sleep(0.01)
                continue

            sys.stdout.flush()
            sound = np.concatenate([sound, audiodata])

            # volume = np.sqrt(np.mean(audiodata ** 2))
            # draw_bar(volume)
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]

            # VAD判定は常に最新CHUNK分で行う
            audio_checkvad = (sound[-CHUNK:] * 32767).astype(np.int16)
            if vad.is_speech(audio_checkvad.tobytes(), sample_rate):
                silent_count = 0
                sound_count += 1
                if sound_count >= (200 / frame_duration):
                    sound_available = True
            elif sound.shape[0] >= 5 * sample_rate:
                sound_count = 0
                silent_count += 1
                if silent_count >= (100 / frame_duration):
                    if sound_available:
                        process_start_time = time.perf_counter()
                        sound_comp = sound / np.abs(sound).max()
                        write('model_input_sound.wav', sample_rate, (sound_comp[:int(5 * sample_rate)] * 32767).astype(np.int16))
                        sound_comp16 = np.array(sound_comp * 32767, dtype='int16')
                        sound_comp16[:int(5 * sample_rate)].tofile("model_input_sound.raw")

                        # 推論処理（関数化）
                        pred, probability = model.predict(sound_comp[:int(5 * sample_rate)], threshold=THRESHOLD)
                        turn_taking_result_queue.put((pred, probability))

                        processing_time = (time.perf_counter() - process_start_time) * 1000
                        # 20メモリバー＋閾値位置にバー
                        # bar_len = int(round(probability * BAR_MEM))
                        # threshold_pos = int(round(THRESHOLD * BAR_MEM))
                        # bar = ""
                        # for i in range(BAR_MEM):
                        #     if i == threshold_pos:
                        #         bar += "|"
                        #     elif i < bar_len:
                        #         bar += "■"
                        #     else:
                        #         bar += " "
                        # now = time.time()
                        # now_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now*1000)%1000):03d}"
                        # sys.stdout.write(f"  |{bar}| {probability:.3f} [処理時間: {processing_time:.1f}ms] [{now_str}]\n")
                        # sys.stdout.flush()

                        sound_available = False
        except KeyboardInterrupt:
            sys.stdout.write("\n[INFO] TurnTaking terminated by KeyboardInterrupt\n")
            sys.stdout.flush()
            break