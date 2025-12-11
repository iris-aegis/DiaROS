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
import datetime
import queue
import sys
from scipy.io.wavfile import write
import torch
import torch.nn as nn
from transformers import Wav2Vec2ForSequenceClassification, Wav2Vec2FeatureExtractor
import transformers
transformers.logging.set_verbosity_error()

# SileroVADのインポート
silero_vad_model = None
get_speech_timestamps = None
collect_chunks = None
VADIterator = None

try:
    import torch.hub
    import warnings
    warnings.filterwarnings("ignore", message="You are using `torch.load` with `weights_only=False`")
    
    # SileroVADモデルをロード（正しい方法）
    torch.set_num_threads(1)
    silero_vad_model, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad',
                                             model='silero_vad',
                                             force_reload=False,
                                             onnx=False)
    get_speech_timestamps, save_audio, read_audio, VADIterator, collect_chunks = utils
    print("[INFO] SileroVAD loaded successfully (including VADIterator)")
except Exception as e:
    print(f"[ERROR] SileroVAD load failed: {e}")
    silero_vad_model = None
    get_speech_timestamps = None
    collect_chunks = None
    VADIterator = None

# グローバルキュー（ros2_turn_taking.py から共有）
stream_queue = queue.Queue()
turn_taking_result_queue = queue.Queue()
asr_result_queue = queue.Queue()  # ASR結果受信用キュー
silero_vad_result_queue = queue.Queue()  # SileroVAD結果送信用キュー（RQT監視用）
vad_iterator_result_queue = queue.Queue()  # VADIterator結果送信用キュー（RQT監視用）

THRESHOLD = 0.75

def push_audio_data(data):
    stream_queue.put(data)
    # 追加: キューサイズを表示
    # sys.stdout.write(f"[turnTaking.py] push_audio_data: stream_queue size={stream_queue.qsize()}\n")
    sys.stdout.flush()

def push_asr_result(asr_text, is_final):
    """ASR結果をキューに送信"""
    asr_result_queue.put({"text": asr_text, "is_final": is_final})
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

def judge_silero_3chunks(silero_model, chunks_32ms, sample_rate=16000):
    """3つの32msチャンクでSileroVAD判定"""
    try:
        speech_probs = []
        for chunk in chunks_32ms:
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                speech_prob = silero_model(chunk_tensor, sample_rate).item()
                speech_probs.append(speech_prob)
        
        avg_prob = np.mean(speech_probs)
        is_speech = avg_prob >= 0.5
        return is_speech, avg_prob, speech_probs
    except Exception as e:
        sys.stdout.write(f"[ERROR] SileroVAD 3チャンク判定エラー: {e}\n")
        sys.stdout.flush()
        return None, 0.0, [0.0, 0.0, 0.0]

def judge_silero_6chunks(silero_model, chunks_32ms, sample_rate=16000):
    """6つの32msチャンクでSileroVAD判定（192ms用）"""
    try:
        speech_probs = []
        for chunk in chunks_32ms:
            with torch.no_grad():
                chunk_tensor = torch.FloatTensor(chunk).unsqueeze(0)
                speech_prob = silero_model(chunk_tensor, sample_rate).item()
                speech_probs.append(speech_prob)
        
        avg_prob = np.mean(speech_probs)
        is_speech = avg_prob >= 0.5
        return is_speech, avg_prob, speech_probs
    except Exception as e:
        sys.stdout.write(f"[ERROR] SileroVAD 6チャンク判定エラー: {e}\n")
        sys.stdout.flush()
        return None, 0.0, [0.0] * 6

def TurnTaking():
    model = TurnTakingModel()
    vad = webrtcvad.Vad()
    vad.set_mode(3)  # mode=3（アグレッシブ、ノイズ耐性高）- mic_input_original.pyと同じ

    sample_rate = 16000
    frame_duration = 10  # ms
    CHUNK = int(sample_rate * frame_duration / 1000)  # 160サンプル
    TurnJudgeThreshold = 0.650

    # webRTC VAD設定情報をログ出力（コメントアウト）
    # sys.stdout.write(f"[INFO] webRTC VAD初期化: frame_duration={frame_duration}ms, mode=3（アグレッシブ）\n")
    # sys.stdout.write(f"[INFO] 最小有声継続時間: 200ms（{int(200/frame_duration)}フレーム）\n")
    # sys.stdout.write(f"[INFO] 最小無音検出時間: 100ms（{int(100/frame_duration)}フレーム）\n")
    # sys.stdout.flush()

    sound = np.empty(0, dtype='float32')
    sound_available = False
    sound_count = 0  # webRTC VAD: 連続音声フレームカウント（200ms判定用）
    silent_count = 0
    silent_start_time = None  # 無声区間開始時刻を記録
    speech_detected_for_200ms = False  # 200ms以上の音声が検出されたかのフラグ（ノイズ除外用）
    
    # ★powerベース変数をコメントアウト
    # power_threshold = 0.1  # パワー閾値
    # below_threshold_logged = False  # パワー閾値を下回った時の初回ログ用フラグ
    # power_silent_start_time = None  # パワーベース無声区間開始時刻
    # power_silent_count = 0  # パワーベース無声フレーム数
    # power_100ms_detected = False  # パワーベース100ms検出フラグ
    # power_measurement_active = True  # パワー計測アクティブフラグ

    # ★SileroVAD関連変数（コメントアウト）
    # silero_vad_enabled = silero_vad_model is not None
    # silero_silent_start_time = None  # SileroVAD無声区間開始時刻
    # silero_100ms_detected = False  # SileroVAD 100ms検出フラグ
    # silero_speech_detected = False  # 音声検出状態
    # silero_buffer = np.array([], dtype=np.float32)  # SileroVAD用音声バッファ
    # 
    # # SileroVADの初期化（CPU推論）
    # if silero_vad_enabled:
    #     silero_vad_model.eval()
    #     silero_threshold = 0.5  # 音声検出閾値
    #     sys.stdout.write(f"[INFO] SileroVAD initialized for CPU inference (threshold: {silero_threshold}, min_silence: 100ms)\n")
    # else:
    #     sys.stdout.write("[WARNING] SileroVAD not available, falling back to WebRTC VAD\n")
    
    # ★powerベース変数をコメントアウト
    # power_threshold = 0.1  # パワー閾値
    # below_threshold_logged = False  # パワー閾値を下回った時の初回ログ用フラグ
    # power_silent_start_time = None  # パワーベース無声区間開始時刻
    # power_silent_count = 0  # パワーベース無声フレーム数
    # power_100ms_detected = False  # パワーベース100ms検出フラグ
    # power_measurement_active = True  # パワー計測アクティブフラグ

    # ★SileroVAD TurnTaking前処理システム
    silero_tt_enabled = silero_vad_model is not None
    
    # SileroVAD状態管理
    silero_tt_mode = "DETECTING_SILENCE"  # DETECTING_SILENCE / WAITING_SPEECH_200MS / DETECTING_SPEECH_192MS
    silero_tt_silent_start_time = None
    silero_tt_100ms_detected = False
    silero_tt_input_counter = 0  # 200ms音声復帰待機カウンタ
    
    # 音声バッファ管理（collections.dequeを使用）
    from collections import deque
    silero_tt_buffer = deque(maxlen=int(sample_rate * 0.2))  # 200ms分保持
    
    if silero_tt_enabled:
        silero_vad_model.eval()
        sys.stdout.write("[INFO] SileroVAD TurnTaking前処理システム初期化完了\n")
        sys.stdout.flush()
    else:
        sys.stdout.write("[WARNING] SileroVAD未使用、PowerベースVADに切り替え\n")
        sys.stdout.flush()
        # PowerベースVADに切り替え
        power_threshold = 0.1
        below_threshold_logged = False
        power_silent_start_time = None
        power_silent_count = 0
        power_100ms_detected = False
        power_measurement_active = True

    # ★VADIterator TurnTaking前処理システム（無効化：webRTCVADに戻す）
    vad_iterator_enabled = False  # VADIteratorを無効化

    if False and vad_iterator_enabled:
        # VADIterator初期化
        vad_iterator = VADIterator(
            silero_vad_model,
            threshold=0.5,
            sampling_rate=sample_rate,
            min_silence_duration_ms=100,    # 100ms無声で音声終了判定
            speech_pad_ms=32                # 32ms音声パディング
        )

        # VADIterator用変数（簡素化）
        vad_iterator_buffer = np.array([], dtype=np.float32)
        vad_iterator_speech_detected = False

        sys.stdout.write("[INFO] VADIterator TurnTaking前処理システム初期化完了\n")
        sys.stdout.write(f"[INFO] 設定: min_silence=100ms, speech_pad=32ms\n")
        sys.stdout.flush()
    else:
        vad_iterator = None
        sys.stdout.write("[WARNING] VADIterator未使用、PowerベースVADに切り替え\n")
        sys.stdout.flush()

    sys.stdout.write("[INFO] TurnTaking started\n")
    sys.stdout.flush()

    while True:
        try:
            audiodata = get_audio_data()
            if audiodata is None:
                time.sleep(0.01)
                continue

            sys.stdout.flush()
            sound = np.concatenate([sound, audiodata])

            # ★SileroVAD TurnTaking前処理システム（コメントアウト：VADIteratorに切り替え）
            if False:  # silero_tt_enabled:
                # 10ms音声データをバッファに追加
                silero_tt_buffer.extend(audiodata)
                silero_tt_input_counter += 1
                
                current_time = datetime.datetime.now()
                timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]
                
                # 状態に応じた処理分岐
                if silero_tt_mode == "DETECTING_SILENCE":
                    # 96ms分のデータが蓄積されたら3チャンク無声判定
                    if len(silero_tt_buffer) >= int(sample_rate * 0.096):
                        # 最新96ms分を取得して3つの32msチャンクに分割
                        latest_96ms = np.array(list(silero_tt_buffer)[-int(sample_rate * 0.096):])
                        chunk_32ms_samples = int(sample_rate * 0.032)
                        chunks_3 = [
                            latest_96ms[i*chunk_32ms_samples:(i+1)*chunk_32ms_samples] 
                            for i in range(3)
                        ]
                        
                        # SileroVAD 3チャンク判定
                        is_speech, avg_prob, probs = judge_silero_3chunks(silero_vad_model, chunks_3)
                        
                        if is_speech is not None:
                            if not is_speech:
                                # 無声区間検出
                                if silero_tt_silent_start_time is None:
                                    silero_tt_silent_start_time = current_time
                                    sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 無声区間開始 (avg_prob={avg_prob:.3f})\n")
                                    sys.stdout.flush()
                                else:
                                    # 100ms継続チェック
                                    silent_duration_ms = (current_time - silero_tt_silent_start_time).total_seconds() * 1000
                                    if silent_duration_ms >= 100 and not silero_tt_100ms_detected:
                                        sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 100ms無声区間検出完了、TurnTaking実行!\n")
                                        sys.stdout.flush()
                                        silero_tt_100ms_detected = True
                                        
                                        # TurnTakingモデル実行
                                        try:
                                            process_start_time = time.perf_counter()
                                            sound_comp = sound / np.abs(sound).max()
                                            pred, probability = model.predict(sound_comp[:int(5 * sample_rate)], threshold=TurnJudgeThreshold)
                                            processing_time = (time.perf_counter() - process_start_time) * 1000
                                            
                                            # 結果をキューに送信
                                            turn_taking_result_queue.put((pred, probability))
                                            sys.stdout.write(f"[{timestamp_str}][SILERO_TT] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms\n")
                                            sys.stdout.flush()
                                            
                                            # 次の状態に移行：200ms音声復帰待機
                                            silero_tt_mode = "WAITING_SPEECH_200MS"
                                            silero_tt_input_counter = 0
                                            sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 200ms音声復帰待機モードに移行\n")
                                            sys.stdout.flush()
                                            
                                        except Exception as e:
                                            sys.stdout.write(f"[ERROR] TurnTaking実行エラー: {e}\n")
                                            sys.stdout.flush()
                            else:
                                # 音声検出：無声区間リセット
                                if silero_tt_silent_start_time is not None:
                                    sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 音声復帰、無声区間リセット (avg_prob={avg_prob:.3f})\n")
                                    sys.stdout.flush()
                                silero_tt_silent_start_time = None
                                silero_tt_100ms_detected = False
                
                elif silero_tt_mode == "WAITING_SPEECH_200MS":
                    # 200ms（20回の10ms入力）待機
                    if silero_tt_input_counter >= 20:
                        sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 200ms経過、音声判定モードに移行\n")
                        sys.stdout.flush()
                        silero_tt_mode = "DETECTING_SPEECH_192MS"
                        silero_tt_input_counter = 0
                
                elif silero_tt_mode == "DETECTING_SPEECH_192MS":
                    # 192ms分のデータが蓄積されたら6チャンク音声判定
                    if len(silero_tt_buffer) >= int(sample_rate * 0.192):
                        # 最新192ms分を取得して6つの32msチャンクに分割
                        latest_192ms = np.array(list(silero_tt_buffer)[-int(sample_rate * 0.192):])
                        chunk_32ms_samples = int(sample_rate * 0.032)
                        chunks_6 = [
                            latest_192ms[i*chunk_32ms_samples:(i+1)*chunk_32ms_samples] 
                            for i in range(6)
                        ]
                        
                        # SileroVAD 6チャンク判定
                        is_speech, avg_prob, probs = judge_silero_6chunks(silero_vad_model, chunks_6)
                        
                        if is_speech is not None and is_speech:
                            # 音声復帰検出：無声判定モードに戻る
                            sys.stdout.write(f"[{timestamp_str}][SILERO_TT] 音声復帰検出、無声判定モードに戻る (avg_prob={avg_prob:.3f})\n")
                            sys.stdout.flush()
                            silero_tt_mode = "DETECTING_SILENCE"
                            silero_tt_silent_start_time = None
                            silero_tt_100ms_detected = False
            
            # ★VADIterator TurnTaking前処理システム（簡素化版）
            elif vad_iterator_enabled and vad_iterator is not None:
                # 10ms音声データをバッファに蓄積（32ms単位で処理）
                vad_iterator_buffer = np.concatenate([vad_iterator_buffer, audiodata])

                # 32ms分のデータが溜まったら処理
                chunk_32ms_samples = int(sample_rate * 0.032)  # 512サンプル
                while len(vad_iterator_buffer) >= chunk_32ms_samples:
                    chunk_32ms = vad_iterator_buffer[:chunk_32ms_samples]
                    vad_iterator_buffer = vad_iterator_buffer[chunk_32ms_samples:]  # 使用済みデータを削除

                    current_time = datetime.datetime.now()
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

                        # 状態管理
                        if speech_start and not vad_iterator_speech_detected:
                            vad_iterator_speech_detected = True
                            sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR] 音声開始検出\n")
                            sys.stdout.flush()

                        elif speech_end and vad_iterator_speech_detected:
                            # VADIteratorがendイベントを返した = 既に100ms無声が検出済み
                            vad_iterator_speech_detected = False
                            sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR] 100ms無声検出完了 → TurnTaking実行!\n")
                            sys.stdout.flush()

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
                                    sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR] [WARNING] 音声データが不足、TurnTakingスキップ\n")
                                    sys.stdout.flush()
                                else:
                                    sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                                    # 5秒分に制限
                                    max_samples = int(5 * sample_rate)
                                    if len(sound_comp) > max_samples:
                                        sound_comp = sound_comp[:max_samples]

                                    pred, probability = model.predict(sound_comp, threshold=TurnJudgeThreshold)
                                    processing_time = (time.perf_counter() - process_start_time) * 1000

                                    # 結果をキューに送信
                                    turn_taking_result_queue.put((pred, probability))
                                    sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms\n")
                                    sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)\n")
                                    sys.stdout.flush()

                            except Exception as e:
                                sys.stdout.write(f"[ERROR] VADIterator TurnTaking実行エラー: {e}\n")
                                sys.stdout.flush()

                        # 詳細ログ（結果がある場合のみ）
                        if vad_result is not None:
                            sys.stdout.write(f"[{timestamp_str}][VAD_ITERATOR_DETAIL] 結果: {vad_result}\n")
                            sys.stdout.flush()

                    except Exception as e:
                        sys.stdout.write(f"[ERROR] VADIterator処理エラー: {e}\n")
                        sys.stdout.flush()
            
            # ★webRTCVAD TurnTaking前処理システム
            else:
                current_time = datetime.datetime.now()
                timestamp_str = current_time.strftime('%H:%M:%S.%f')[:-3]

                # webRTCVADで10msフレームを処理（160サンプル = 10ms @ 16kHz）
                # audiodataが10msフレーム（160サンプル）であることを前提
                if len(audiodata) == CHUNK:
                    # float32 → int16に変換
                    audio_int16 = (audiodata * 32767).astype(np.int16)
                    audio_bytes = audio_int16.tobytes()

                    try:
                        # webRTCVADで音声判定
                        is_speech = vad.is_speech(audio_bytes, sample_rate)

                        if not is_speech:
                            # 無音フレーム検出
                            if silent_start_time is None:
                                # 無音開始
                                # if sound_count > 0:
                                #     sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 無音開始検出 (直前音声フレーム数: {sound_count}, 音声時間: {sound_count * frame_duration:.0f}ms, 200msフラグ: {speech_detected_for_200ms})\n")
                                #     sys.stdout.flush()
                                silent_start_time = current_time
                                silent_count = 1
                                sound_count = 0  # 無音開始時に音声カウントをリセット
                            else:
                                # 無音継続
                                silent_count += 1

                                # 100ms無音検出でTurnTaking実行（フレーム数ベース）
                                # 条件: 100ms以上無音 && 200ms以上の音声が事前に検出されている && 未実行
                                # if silent_count >= (100 / frame_duration):  # 100ms / 10ms = 10フレーム
                                #     sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 100ms無音検出 (speech_flag={speech_detected_for_200ms}, sound_available={sound_available})\n")
                                #     sys.stdout.flush()

                                if silent_count >= (100 / frame_duration) and not sound_available and speech_detected_for_200ms:
                                    # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 100ms無音検出完了 → TurnTaking実行!\n")
                                    # sys.stdout.flush()
                                    sound_available = True  # フラグを立てて再実行を防ぐ

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
                                            sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] [WARNING] 音声データが不足、TurnTakingスキップ\n")
                                            sys.stdout.flush()
                                        else:
                                            sound_comp = sound_for_tt / np.abs(sound_for_tt).max()

                                            # 5秒分に制限
                                            max_samples = int(5 * sample_rate)
                                            if len(sound_comp) > max_samples:
                                                sound_comp = sound_comp[:max_samples]

                                            pred, probability = model.predict(sound_comp, threshold=TurnJudgeThreshold)
                                            processing_time = (time.perf_counter() - process_start_time) * 1000

                                            # 結果をキューに送信
                                            turn_taking_result_queue.put((pred, probability))
                                            # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] TurnTaking完了: pred={pred}, prob={probability:.3f}, 処理時間={processing_time:.1f}ms\n")
                                            # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 音声長: {len(sound_for_tt)/sample_rate:.2f}s (100ms削除済み)\n")
                                            # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 次のサイクルに備えてフラグをリセット\n")
                                            # sys.stdout.flush()

                                            # TurnTaking実行後にフラグをリセット（次のサイクルに備える）
                                            speech_detected_for_200ms = False
                                            sound_available = False  # ★重要: sound_availableもリセット
                                            sound_count = 0

                                    except Exception as e:
                                        sys.stdout.write(f"[ERROR] webRTC_VAD TurnTaking実行エラー: {e}\n")
                                        sys.stdout.flush()
                                        # エラー時もフラグをリセット
                                        speech_detected_for_200ms = False
                                        sound_available = False
                                        sound_count = 0

                        else:
                            # 音声フレーム検出
                            # 200ms以上の連続音声判定（mic_input_original.py と同じロジック）
                            if silent_start_time is not None:
                                # 無音から音声に復帰
                                # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 音声復帰検出、無音状態リセット (前フラグ: {speech_detected_for_200ms})\n")
                                # sys.stdout.flush()
                                silent_start_time = None
                                silent_count = 0
                                sound_count = 0  # 音声カウントを0にリセット（200ms再判定用）
                                sound_available = False  # ★重要: 音声復帰時にsound_availableもリセット
                                # ★重要: speech_detected_for_200ms はリセット**しない**

                            # 連続音声フレームをカウント
                            sound_count += 1

                            # 200ms以上の音声が連続で検出されたか判定
                            if sound_count >= (200 / frame_duration):  # 200 / 10 = 20フレーム
                                if not speech_detected_for_200ms:
                                    # sys.stdout.write(f"[{timestamp_str}][webRTC_VAD] 200ms以上の音声を検出 → TurnTaking実行準備完了\n")
                                    # sys.stdout.flush()
                                    pass
                                speech_detected_for_200ms = True

                    except Exception as e:
                        sys.stdout.write(f"[ERROR] webRTC_VAD処理エラー: {e}\n")
                        sys.stdout.flush()
            
            # 音声バッファのサイズ管理（5.1秒に制限）
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]
                
        except KeyboardInterrupt:
            sys.stdout.write("\n[INFO] TurnTaking stopped by user\n")
            sys.stdout.flush()
            break
        except Exception as e:
            sys.stdout.write(f"[ERROR] TurnTaking main loop error: {e}\n")
            sys.stdout.flush()
            time.sleep(0.01)
            continue
