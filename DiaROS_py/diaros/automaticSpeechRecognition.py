# import time
# import sys
# import threading
# import queue
# import numpy as np

# import torch
# from transformers import AutoModelForCTC, Wav2Vec2Processor, Wav2Vec2CTCTokenizer
# from transformers.utils import logging
# import difflib
# import warnings

# # Audio recording parameters
# SAMPLE_RATE = 16000
# CHUNK_SIZE = 240
# MODEL_ID = 'SiRoZaRuPa/wav2vec2-base-kanji-unigram-RS-s-1120'
# AUDIO_DURATION = 5  # seconds
# INPUT_LEN = int(SAMPLE_RATE * AUDIO_DURATION)
# USE_GPU = True

# logging.set_verbosity_error()
# warnings.filterwarnings('ignore')

# def create_diff_list(old, new):
#     diff = list(difflib.ndiff(old, new))
#     lines = []
#     current_text = ""
#     is_change = False
#     for i in diff:
#         if i[0] == ' ':
#             if is_change:
#                 lines.append((1, current_text))
#                 current_text = ""
#             is_change = False
#             current_text += i[2:]
#         elif i[0] == '-':
#             continue
#         elif i[0] == '+':
#             if not is_change:
#                 if current_text:
#                     lines.append((0, current_text))
#                     current_text = ""
#                 is_change = True
#             current_text += i[2:]
#     if current_text:
#         lines.append((is_change, current_text))
#     return lines

# def apply_color_to_diff(lines, end_string=']'):
#     result = ""
#     for is_change, text in lines:
#         if is_change:
#             if lines[-1] == (1, text):
#                 if end_string in text and text.endswith(end_string):
#                     if text[-3] == '雑':
#                         result += f'\033[91m{text[:-4]}\033[0m' + f'\033[42m{text[-4:]}\033[0m'
#                     elif text[-3] == '無':
#                         result += f'\033[91m{text[:-4]}\033[0m' + f'\033[44m{text[-4:]}\033[0m'
#                 else:
#                     result += f'\033[91m{text}\033[0m'
#             else:
#                 result += f'\033[93m{text}\033[0m'
#         else:
#             result += text
#     return result

# class AutomaticSpeechRecognition:
#     def __init__(self):
#         self.last_audio = None
#         self.word = ""
#         self.is_final = False
#         self.recv_count = 0
#         self.audio_buffer = np.array([], dtype=np.float32)
#         self.audio_queue = queue.Queue()
#         self.running = True
#         self.last_sent = ""
#         self.model = None
#         self.processor = None
#         self.tokenizer = None
#         self.model_thread = threading.Thread(target=self.recognition_thread)
#         self.model_thread.daemon = True
#         self.model_thread.start()
#         sys.stdout.write('ASR node start up.\n')
#         sys.stdout.write('=====================================================\n')

#     def update_audio(self, audio_np):
#         self.audio_queue.put(audio_np)
#         self.recv_count += 1

#     def pubASR(self):
#         return {"you": self.word, "is_final": self.is_final}

#     def run(self):
#         while self.running:
#             time.sleep(0.1)

#     def recognition_thread(self):
#         sys.stdout.write('Loading ASR model...\n')
#         self.tokenizer = Wav2Vec2CTCTokenizer.from_pretrained(MODEL_ID)
#         self.processor = Wav2Vec2Processor.from_pretrained(MODEL_ID, tokenizer=self.tokenizer)
#         self.model = AutoModelForCTC.from_pretrained(MODEL_ID)
#         self.model.eval()
#         if USE_GPU and torch.cuda.is_available():
#             device = torch.device("cuda")
#             self.model.to(device)
#         else:
#             device = torch.device("cpu")
#         sys.stdout.write('ASR model loaded.\n')
#         sys.stdout.flush()

#         mic_input = np.array([], dtype=np.float32)
#         last_sent = ""
#         start_time = time.time()
#         last_time = time.time()
#         last_infer_len = 0  # 前回推論時のmic_inputの長さ
#         try:
#             while self.running:
#                 new_data_added = False
#                 while not self.audio_queue.empty():
#                     data = self.audio_queue.get()
#                     mic_input = np.append(mic_input, data)
#                     new_data_added = True
#                 # 5秒を超えたら古いデータから捨てる
#                 if len(mic_input) > INPUT_LEN:
#                     mic_input = mic_input[-INPUT_LEN:]
#                 # 新たな音声が100ms分溜まっていたら推論
#                 # 修正: last_infer_lenの更新タイミングを推論後にし、推論条件を「新しいデータが100ms分以上溜まっている場合」に限定
#                 if len(mic_input) >= int(SAMPLE_RATE * 0.1) and (len(mic_input) - last_infer_len >= int(SAMPLE_RATE * 0.1)):
#                     print(f"Received audio data length: {len(mic_input)}")
#                     sys.stdout.flush()
#                     array = mic_input.astype(np.float32)
#                     inputs = self.processor(array, sampling_rate=SAMPLE_RATE, return_tensors="pt", padding=True)
#                     if USE_GPU and torch.cuda.is_available():
#                         inputs = {k: v.to(device) for k, v in inputs.items()}
#                         self.model = self.model.to(device)
#                     with torch.no_grad():
#                         logits = self.model(**inputs).logits
#                     predicted_ids = torch.argmax(logits, dim=-1)
#                     sentence = self.processor.batch_decode(predicted_ids)[0]
#                     now = time.time()
#                     elapsed_time = now - start_time
#                     process_time = int(1000 * (now - last_time))
#                     last_time = now
#                     diff = create_diff_list(last_sent, sentence)
#                     colored = apply_color_to_diff(diff)
#                     output = f'{elapsed_time:7.3f} ({process_time:5d} ms): {colored}'
#                     if last_sent != sentence:
#                         print(output)
#                     else:
#                         sys.stdout.write("\r" + output + " " * 20 + "\r")
#                         sys.stdout.flush()
#                     self.word = sentence
#                     self.is_final = True if sentence.strip() != "" else False
#                     last_sent = sentence
#                     last_infer_len = len(mic_input)  # 推論後に更新
#                 time.sleep(0.01)  # ループが高速すぎる場合のCPU負荷軽減
#         except Exception as e:
#             print(f"Error in recognition_thread: {e}")

# NOTE 音声入力長固定
import time
import sys
import threading
import queue
import numpy as np

import torch
from transformers import AutoModelForCTC, Wav2Vec2Processor, Wav2Vec2CTCTokenizer
from transformers.utils import logging
import difflib
import warnings

# Audio recording parameters
SAMPLE_RATE = 16000
CHUNK_SIZE = 240
# MODEL_ID = 'SiRoZaRuPa/wav2vec2-base-kanji-unigram-RS-s-1120'
MODEL_ID = 'SiRoZaRuPa/japanese-HuBERT-base-VADLess-ASR-RSm'
AUDIO_DURATION = 5  # seconds
INPUT_LEN = int(SAMPLE_RATE * AUDIO_DURATION)
USE_GPU = True
SHOW_BASIC_LOGS = False
SHOW_DEBUG_LOGS = False

logging.set_verbosity_error()
warnings.filterwarnings('ignore')

def create_diff_list(old, new):
    diff = list(difflib.ndiff(old, new))
    lines = []
    current_text = ""
    is_change = False
    for i in diff:
        if i[0] == ' ':
            if is_change:
                lines.append((1, current_text))
                current_text = ""
            is_change = False
            current_text += i[2:]
        elif i[0] == '-':
            continue
        elif i[0] == '+':
            if not is_change:
                if current_text:
                    lines.append((0, current_text))
                    current_text = ""
                is_change = True
            current_text += i[2:]
    if current_text:
        lines.append((is_change, current_text))
    return lines

def apply_color_to_diff(lines, end_string=']'):
    result = ""
    for is_change, text in lines:
        if is_change:
            if lines[-1] == (1, text):
                if end_string in text and text.endswith(end_string):
                    if text[-3] == '雑':
                        result += f'\033[91m{text[:-4]}\033[0m' + f'\033[42m{text[-4:]}\033[0m'
                    elif text[-3] == '無':
                        result += f'\033[91m{text[:-4]}\033[0m' + f'\033[44m{text[-4:]}\033[0m'
                else:
                    result += f'\033[91m{text}\033[0m'
            else:
                result += f'\033[93m{text}\033[0m'
        else:
            result += text
    return result

class AutomaticSpeechRecognition:
    def __init__(self):
        self.last_audio = None
        self.word = ""
        self.is_final = False
        self.recv_count = 0
        self.audio_buffer = np.array([], dtype=np.float32)
        self.audio_queue = queue.Queue()
        self.running = True
        self.last_sent = ""
        self.model = None
        self.processor = None
        self.tokenizer = None
        self.new_result = False  # 追加: 新しい認識結果フラグ
        self.model_thread = threading.Thread(target=self.recognition_thread)
        self.model_thread.daemon = True
        self.model_thread.start()
        if SHOW_BASIC_LOGS:
            sys.stdout.write('ASR node start up.\n')
            sys.stdout.write('=====================================================\n')

    def update_audio(self, audio_np):
        import time
        import hashlib
        from datetime import datetime
        
        # ASRでの受信時刻を記録
        asr_receive_timestamp = time.time()
        
        # 音声データから一意IDを生成（speechInput.pyと同じアルゴリズム）
        if len(audio_np) >= 10:
            sample_data = audio_np[:10].astype(np.float32).tobytes()
            # タイムスタンプは使わず、音声データのみでIDを生成（元データとの一致を保証）
            audio_id = hashlib.md5(sample_data).hexdigest()[:8]
        else:
            audio_id = hashlib.md5(audio_np.astype(np.float32).tobytes()).hexdigest()[:8]
        
        # ASR音声受信をログ出力（デバッグ）
        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        # データ対応確認用: 先頭5サンプルの具体的な値を表示
        sample_values = audio_np[:5].tolist() if len(audio_np) >= 5 else audio_np.tolist()
        sample_str = '[' + ','.join([f'{v:.6f}' for v in sample_values]) + ']'

        # sys.stdout.write(f"[🧠 ASR_RECEIVE] {timestamp_str} | AudioID:{audio_id} | 受信数:{self.recv_count} | サンプル:{len(audio_np)}\n")
        # sys.stdout.flush()
        
        # メタデータ付きでキューに追加
        audio_metadata = {
            'audio_data': audio_np,
            'audio_id': audio_id,
            'asr_receive_timestamp': asr_receive_timestamp,
            'sample_count': len(audio_np)
        }
        
        self.audio_queue.put(audio_metadata)
        self.recv_count += 1

    def pubASR(self):
        if self.new_result:
            self.new_result = False
            # ★デバッグ: pubASR() で返される結果をログ出力
            if SHOW_DEBUG_LOGS:
                if not hasattr(self, 'pubASR_call_count'):
                    self.pubASR_call_count = 0
                self.pubASR_call_count += 1
                if self.pubASR_call_count % 10 == 0 or len(self.word) > 0:  # 認識結果がある場合は毎回表示
                    from datetime import datetime
                    timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    sys.stdout.write(f"[{timestamp_str}][DEBUG-pubASR] Returning result: '{self.word}' (len={len(self.word)}), is_final={self.is_final}\n")
                    sys.stdout.flush()
            return {"you": self.word, "is_final": self.is_final}
        else:
            return None

    def run(self):
        while self.running:
            time.sleep(0.1)

    def recognition_thread(self):
        if SHOW_BASIC_LOGS:
            sys.stdout.write('Loading ASR model...\n')
        self.tokenizer = Wav2Vec2CTCTokenizer.from_pretrained(MODEL_ID)
        self.processor = Wav2Vec2Processor.from_pretrained(MODEL_ID, tokenizer=self.tokenizer)
        self.model = AutoModelForCTC.from_pretrained(MODEL_ID)
        self.model.eval()
        if USE_GPU and torch.cuda.is_available():
            device = torch.device("cuda")
            self.model.to(device)
        else:
            device = torch.device("cpu")
        if SHOW_BASIC_LOGS:
            sys.stdout.write('ASR model loaded.\n')
            sys.stdout.flush()

        mic_input = np.array([], dtype=np.float32)  # モデル入力用バッファ（5秒分の音声を保持）
        last_sent = ""
        start_time = time.time()
        last_inference_time = time.time()  # 前回推論実行時刻
        new_audio_samples = 0  # 前回推論以降の新しい音声サンプル数
        
        try:
            # sys.stdout.write('[DEBUG] ASR recognition_thread started\n')
            # sys.stdout.flush()
            while self.running:
                # キューから音声データを一気に全て読み込み
                audio_metadata_list = []  # 音声メタデータの記録
                new_data_found = False
                
                # キューが空になるまで全てのデータを取得
                batch_count = 0
                while not self.audio_queue.empty():
                    try:
                        queue_item = self.audio_queue.get_nowait()
                        process_timestamp = time.time()
                        
                        if isinstance(queue_item, dict) and 'audio_data' in queue_item:
                            # 新しいメタデータ付きフォーマット
                            data = queue_item['audio_data']
                            audio_id = queue_item['audio_id']
                            asr_receive_timestamp = queue_item['asr_receive_timestamp']
                            sample_count = queue_item['sample_count']
                            
                            audio_metadata_list.append({
                                'audio_id': audio_id,
                                'asr_receive_timestamp': asr_receive_timestamp,
                                'process_timestamp': process_timestamp,
                                'sample_count': sample_count
                            })
                            
                        elif isinstance(queue_item, tuple):
                            # 旧形式（タプル）
                            data, timestamp = queue_item
                            data_id = f"{data[0]:.6f},{data[1]:.6f},{data[2]:.6f}" if len(data) >= 3 else "short_data"
                            audio_metadata_list.append({
                                'audio_id': data_id,
                                'asr_receive_timestamp': timestamp,
                                'process_timestamp': process_timestamp,
                                'sample_count': len(data)
                            })
                        else:
                            # 古い形式への対応
                            data = queue_item
                            data_id = f"{data[0]:.6f},{data[1]:.6f},{data[2]:.6f}" if len(data) >= 3 else "short_data"
                            audio_metadata_list.append({
                                'audio_id': data_id,
                                'asr_receive_timestamp': process_timestamp,
                                'process_timestamp': process_timestamp,
                                'sample_count': len(data)
                            })
                        
                        # 最初のデータのみログ出力（コメントアウト）
                        # if batch_count == 0:
                        #     from datetime import datetime
                        #     timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        #     if audio_metadata_list:
                        #         metadata = audio_metadata_list[-1]
                        #         latency_ms = (process_timestamp - metadata['asr_receive_timestamp']) * 1000
                        #         
                        #         # データ対応確認用: 先頭5サンプルの具体的な値を表示
                        #         sample_values = data[:5].tolist() if len(data) >= 5 else data.tolist()
                        #         sample_str = '[' + ','.join([f'{v:.6f}' for v in sample_values]) + ']'
                        #         
                        #         # 音声データの統計値も追加
                        #         rms = np.sqrt(np.mean(data**2))
                        #         max_val = np.max(np.abs(data))
                        #         
                        #         sys.stdout.write(f"[⚙️ ASR_PROCESS] {timestamp_str} | AudioID:{metadata['audio_id']} | ProcessLatency:{latency_ms:.2f}ms | Batch:{batch_count+1} | Data:{sample_str} | RMS:{rms:.6f} | Max:{max_val:.6f}\n")
                        #         sys.stdout.flush()
                        
                        # 音声データをスタックに追加
                        mic_input = np.append(mic_input, data)
                        new_audio_samples += len(data)
                        new_data_found = True
                        batch_count += 1
                        
                    except queue.Empty:
                        break
                
                # デバッグ用: バッチ処理情報を表示（コメントアウト）
                # if batch_count > 0:
                #     sys.stdout.write(f"[🔄 SDS_BATCH] Processed {batch_count} chunks, new_audio: {new_audio_samples}samples\n")
                #     sys.stdout.flush()
                
                # 5秒を超えた分は古いデータから削除（スライディングウィンドウ）
                if len(mic_input) > INPUT_LEN:
                    removed_samples = len(mic_input) - INPUT_LEN
                    mic_input = mic_input[-INPUT_LEN:]
                    # スライディングウィンドウでは新しい音声カウントはリセットしない
                    # new_audio_samplesは推論実行時のみリセットする
                
                # 推論実行条件をチェック
                current_time = time.time()
                should_run_inference = (
                    len(mic_input) >= int(SAMPLE_RATE * 0.1) and  # 最低100ms分のデータ
                    new_audio_samples >= int(SAMPLE_RATE * 0.1)   # 前回推論から100ms以上の新しい音声
                )
                
                # デバッグ用: 推論条件を定期的に表示
                if hasattr(self, 'debug_count'):
                    self.debug_count += 1
                else:
                    self.debug_count = 0

                if self.debug_count % 50 == 0:  # 50回に1回表示
                    if SHOW_DEBUG_LOGS:
                        from datetime import datetime
                        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        min_data = int(SAMPLE_RATE * 0.1)
                        min_new = int(SAMPLE_RATE * 0.1)
                        sys.stdout.write(f"[{timestamp_str}][ASR_INFER_CONDITION] mic:{len(mic_input)}samples (min:{min_data}) | new:{new_audio_samples}samples (min:{min_new}) | should_infer:{should_run_inference} | queue:{self.audio_queue.qsize()}\n")
                        sys.stdout.flush()
                
                # 推論条件に近づいた時も表示
                # if new_audio_samples >= 1400:  # 1600に近づいた時
                #     sys.stdout.write(f"[DEBUG] Almost ready for inference: new_audio: {new_audio_samples}/1600 samples\n")
                #     sys.stdout.flush()
                
                if should_run_inference:
                    try:
                        # ASR推論開始時刻
                        inference_start_time = time.time()
                        from datetime import datetime
                        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        if SHOW_DEBUG_LOGS:
                            sys.stdout.write(f"[{timestamp_str}][ASR_INFERENCE_START] mic:{len(mic_input)}samples | new_audio:{new_audio_samples}samples\n")
                            sys.stdout.flush()

                        # 最新5秒分のデータで推論（すでにmic_inputに蓄積済み）
                        inference_data = mic_input[-int(5 * SAMPLE_RATE):] if len(mic_input) >= int(5 * SAMPLE_RATE) else mic_input

                        array = inference_data.astype(np.float32)
                        inputs = self.processor(array, sampling_rate=SAMPLE_RATE, return_tensors="pt", padding=True)
                        if USE_GPU and torch.cuda.is_available():
                            inputs = {k: v.to(device) for k, v in inputs.items()}
                            self.model = self.model.to(device)
                        with torch.no_grad():
                            logits = self.model(**inputs).logits
                        predicted_ids = torch.argmax(logits, dim=-1)
                        sentence = self.processor.batch_decode(predicted_ids)[0]

                        # ASR推論完了時刻と処理時間計算
                        inference_end_time = time.time()
                        inference_duration_ms = (inference_end_time - inference_start_time) * 1000

                        # 音声認識結果を標準出力（毎回出力）
                        from datetime import datetime
                        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                        if audio_metadata_list:
                            # 最も古い音声データからの遅延計算
                            oldest_metadata = min(audio_metadata_list, key=lambda x: x['asr_receive_timestamp'])
                            total_latency_ms = (inference_end_time - oldest_metadata['asr_receive_timestamp']) * 1000

                            # 関連する音声IDを収集
                            audio_ids = [metadata['audio_id'] for metadata in audio_metadata_list]
                            audio_ids_str = ','.join(audio_ids[:3]) if len(audio_ids) <= 3 else f"{','.join(audio_ids[:2])}...(+{len(audio_ids)-2})"

                            # sys.stdout.write(f"[💬 ASR_INFERENCE] {timestamp_str} | 推論時間:{inference_duration_ms:.1f}ms | 総遅延:{total_latency_ms:.1f}ms | IDs:[{audio_ids_str}] | 新音声:{new_audio_samples}samples | 認識:'{sentence}'\n")
                            # sys.stdout.flush()
                        else:
                            # フォールバック
                            # sys.stdout.write(f"[💬 ASR_INFERENCE] {timestamp_str} | 推論時間:{inference_duration_ms:.1f}ms | 新音声:{new_audio_samples}samples | 認識:'{sentence}'\n")
                            # sys.stdout.flush()
                            pass

                        # 推論実行後の状態更新
                        last_inference_time = inference_end_time
                        new_audio_samples = 0  # 新しい音声カウントをリセット

                        now = time.time()
                        elapsed_time = now - start_time
                        diff = create_diff_list(last_sent, sentence)
                        colored = apply_color_to_diff(diff)

                        self.word = sentence
                        self.is_final = True
                        self.new_result = True  # 追加: 新しい認識結果が得られた
                        last_sent = sentence

                        if SHOW_BASIC_LOGS:
                            sys.stdout.write(f"[{timestamp_str}][ASR_INFERENCE_COMPLETE] 認識結果: '{sentence}' (遅延:{inference_duration_ms:.1f}ms)\n")
                            sys.stdout.flush()

                    except Exception as e:
                        if SHOW_BASIC_LOGS:
                            from datetime import datetime
                            timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            sys.stdout.write(f"[{timestamp_str}][ERROR-ASR_INFERENCE] {type(e).__name__}: {str(e)}\n")
                            import traceback
                            sys.stdout.write(f"[{timestamp_str}][ERROR-TRACEBACK]\n{traceback.format_exc()}\n")
                            sys.stdout.flush()
                
                # キューが空の場合は少し待機
                if not new_data_found:
                    time.sleep(0.01)  # 10ms待機でCPU負荷軽減
        except Exception as e:
            if SHOW_BASIC_LOGS:
                sys.stdout.write(f"[ERROR] ASR recognition_thread crashed: {e}\n")
                sys.stdout.write(f"[ERROR] Queue size at crash: {self.audio_queue.qsize()}\n")
                import traceback
                sys.stdout.write(f"[ERROR] Traceback: {traceback.format_exc()}\n")
                sys.stdout.flush()
            raise


