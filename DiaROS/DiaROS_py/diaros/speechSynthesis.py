# TurtTaking,back_channelのプログラムをDiaROS内に入れる開発 Unityにjsonファイルで共有する 一旦履歴諦め
from datetime import datetime, timedelta
import pygame
pygame.mixer.init()
import shutil
import os
import json
import requests
import wave
import sys
import numpy as np
import time
import subprocess
from diaros.timing_integration import get_timing_logger, log_audio_playback_start, end_timing_session

### VAD ###
import queue
import webrtcvad
import librosa
import pyaudio
import threading
###---###

### UDP通信設定 ###
import socket
HOST = '127.0.0.1'
PORT = 50021
client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
###---###

DEBUG = False

class SpeechSynthesis():
    # ### VAD ###
    # audio_queue = queue.Queue()  # マイクからの音声データを保存するキュー
    # mic_sample_rate = 48000
    # sample_rate     = 16000
    # frame_duration  = 30  # ms
    # CHUNK           = int(mic_sample_rate * frame_duration / 1000)

    # vad = webrtcvad.Vad()
    # vad.set_mode(3)
    # sound_available = False
    # sound_count = 0
    # silent_count = 0
    # sound = np.empty(0) # モデルに入力する音声まとめ(長い)
    # ###---###
    
    # # audio start ###############################################################
    # def audiostart(self):
    #     audio = pyaudio.PyAudio() 

    #     # Sennheiser USB headset のデバイスIDを探す -----------------------------
    #     device_id = None
    #     for i in range(audio.get_device_count()):
    #         info = audio.get_device_info_by_index(i)
    #         # if 'Sennheiser USB headset' in info['name']:
    #         # if 'USB Microphone' in info['name']:
    #         if 'default' == info['name']:
    #             device_id = info['index']
    #             break
        
    #     # Sennheiser USB headset が見つからなかったら終了 ----------------------
    #     if device_id is None:
    #         print("Sennheiser USB headset が見つかりませんでした。")
    #         exit()

    #     # print(f"[Sennheiser USB headset] index:{device_id}")# ゼンハイザーを指定した時
    #     print(f"[Default] index:{device_id}")

    #     # 見つかったデバイスIDを使用して、ストリームを開く ---------------------
    #     stream = audio.open(format = pyaudio.paInt16,
    #                         rate = self.mic_sample_rate,
    #                         channels = 1, 
    #                         input = True, 
    #                         frames_per_buffer = self.CHUNK,
    #                         input_device_index=device_id)

    #     return audio, stream

    # # audio stop #############################################################
    # def audiostop(self, audio, stream):
    #     stream.stop_stream()
    #     stream.close()
    #     audio.terminate()

    # # マイク入力を別スレッドで実行 ##########################################
    # def mic_input_thread(self, sample_rate, CHUNK):
    #     (audio, stream) = self.audiostart()  # スレッド内でaudiostart()を実行

    #     while True:
    #         data = stream.read(CHUNK)
    #         audiodata = np.frombuffer(data, dtype='int16')
    #         self.audio_queue.put(audiodata)
        
    #     # スレッド終了時にaudiostop()を実行
    #     self.audiostop(audio, stream)

    def __init__(self):
        self.tl = "ja"
        self.TMP_DIR = './tmp/'

        # remove TMP directory & remake ----
        if os.path.exists(self.TMP_DIR):
            du = shutil.rmtree(self.TMP_DIR)
            time.sleep(0.3)

        os.mkdir(self.TMP_DIR)

        self.speak_end = False
        self.response_pause_length = 1
        self.prev_response_time = datetime.now()
        self.source_words = []
        self.last_tts_file = ""  # 直近の合成ファイル名を記録
        # power_calibration.wavは無効化
        
        # 時間計測用
        self.timing_logger = get_timing_logger()
        self.current_session_id = None

        # Check if VOICEVOX is running, start if not
        self._ensure_voicevox_running()
        # Check if VOICEVOX is running on GPU
        self._check_voicevox_gpu()

    def _ensure_voicevox_running(self):
        """Check if VOICEVOX is running and display status message"""
        try:
            import requests
            response = requests.get('http://localhost:50021/speakers', timeout=2)
            if response.status_code == 200:
                print("✓ VOICEVOX is running and ready")
                return
        except Exception:
            pass
        print("=" * 60)
        print("⚠️  VOICEVOX ENGINE IS NOT RUNNING")
        print("=" * 60)
        print("Please start VOICEVOX engine manually with one of these commands:")
        print("1. /opt/voicevox_engine/linux-nvidia/run --host 127.0.0.1 --port 50021")
        print("2. voicevox --host 127.0.0.1 --port 50021")
        print("=" * 60)
        print("Speech synthesis will fail until VOICEVOX is started.")
        print("=" * 60)

    def _check_voicevox_gpu(self):
        """VOICEVOXがGPU上で動作しているか確認し、警告を出す"""
        try:
            import subprocess
            result = subprocess.run(['nvidia-smi', '--query-compute-apps=pid,process_name,gpu_uuid', '--format=csv,noheader'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            found = False
            for line in result.stdout.splitlines():
                if 'voicevox' in line.lower() or 'voicevox_engine' in line.lower():
                    found = True
                    break
            if found:
                print("✓ VOICEVOX ENGINE is running on GPU (nvidia-smi detected)")
            else:
                print("=" * 60)
                print("⚠️  VOICEVOX ENGINE IS NOT RUNNING ON GPU (nvidia-smi did not detect it)")
                print("=" * 60)
                print("Please ensure you are using the GPU version of VOICEVOX engine.")
                print("If you are using the CPU version, synthesis will be slow.")
                print("=" * 60)
        except Exception as e:
            print("Could not check GPU status (nvidia-smi not available or error).")
            # print(e)

    def play_sound(self, filename, block=True):
        """pygame.mixerを使用して音声ファイルを再生"""
        try:
            # 時間計測: 音声再生開始
            if self.current_session_id is None:
                sessions = self.timing_logger.session_data
                if sessions:
                    self.current_session_id = list(sessions.keys())[-1]  # 最新セッション
            
            if self.current_session_id:
                log_audio_playback_start(self.current_session_id, filename)
            
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            
            if block:
                while pygame.mixer.music.get_busy():
                    pygame.time.wait(100)
                
                # 時間計測: セッション終了（音声再生完了）
                if self.current_session_id:
                    total_ms = end_timing_session(self.current_session_id, "音声再生完了")
                    print(f"🎉 対話セッション完了: 総計時間 {total_ms:.1f}ms")
                    
        except Exception as e:
            print(f"音声再生エラー: {e}")
    
    def trim_wav(self, input_file, output_file, trim_duration=0.1):# VOICEVOXのノイズ除去用
        # 入力ファイルを開く
        with wave.open(input_file, 'rb') as input_wav:
            # 入力ファイルのパラメータを取得
            params = input_wav.getparams()

            # 出力ファイルを作成
            with wave.open(output_file, 'wb') as output_wav:
                # 出力ファイルに入力ファイルのパラメータを設定
                output_wav.setparams(params)

                # 0.1 秒分のサンプル数を計算
                trim_frames = int(trim_duration * params.framerate)

                # 先頭の 0.1 秒をスキップして残りを書き込む
                input_wav.readframes(trim_frames)
                output_wav.writeframes(input_wav.readframes(params.nframes - trim_frames))

    def run(self, text):
        """NLGから受信したテキストを即座に音声合成し、ファイル名を返す（同期処理）"""
        tts_file = None
        try:
            speaker = 60
            start_time = datetime.now()
            host = "localhost"
            port = 50021
            params = (
                ('text', text),
                ('speaker', speaker),
            )
            response1 = requests.post(
                f'http://{host}:{port}/audio_query',
                params=params
            )
            response1_data = response1.json()
            response1_data["prePhonemeLength"] = 0.275
            response1_data["postPhonemeLength"] = 0.0
            modified_json_str = json.dumps(response1_data)
            all_segments = []
            for accent_phrase in response1_data['accent_phrases']:
                for mora in accent_phrase['moras']:
                    vowel_type = mora['vowel']
                    consonant_length = mora['consonant_length']
                    vowel_length = mora['vowel_length']
                    if consonant_length is None:
                        duration = vowel_length
                    else:
                        duration = vowel_length + consonant_length
                    segment = {
                        "vowel_type": vowel_type,
                        "length": duration
                    }
                    all_segments.append(segment)
                pause = 0.0
                if isinstance(accent_phrase['pause_mora'], dict):
                    pause_mora = accent_phrase['pause_mora']
                    pause_consonant_length = pause_mora['consonant_length']
                    pause_vowel_length = pause_mora['vowel_length']
                    if pause_consonant_length is not None:
                        pause += pause_consonant_length
                    if pause_vowel_length is not None:
                        pause += pause_vowel_length
                    pause_segment = {
                        "vowel_type": "silent_vowel",
                        "length": pause
                    }
                    all_segments.append(pause_segment)
            wrapped_data = {"data": all_segments}
            json_str = json.dumps(wrapped_data)
            current_time = datetime.now().strftime("%Y%m%d%H%M%S%f")
            json_file = './tmp/' + str(current_time) + '.json'
            with open(json_file, 'w', encoding='utf-8') as f:
                json.dump(wrapped_data, f, ensure_ascii=False, indent=4)
            headers = {'Content-Type': 'application/json',}
            response2 = requests.post(
                f'http://{host}:{port}/synthesis',
                headers=headers,
                params=params,
                data=modified_json_str.encode('utf-8')
            )
            input_file = './{}/input_{}.wav'.format(self.TMP_DIR, current_time)
            with wave.open(input_file, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(24000)
                wf.writeframes(response2.content)
            tts_file = './tmp/' + str(current_time) + '.wav'
            self.trim_wav(input_file, tts_file)
            self.speak_end = True
            self.last_tts_file = tts_file  # 最新の合成ファイル名を常に保持
        except Exception as e:
            print('VOICEVOXerror: VOICEVOX sound is not generated. Do you launch VOICEVOX?')
            print(e.args)
            self.last_tts_file = ""
            self.speak_end = True
        return self.last_tts_file

    def updateNLG(self, nlg):
        """NLGからテキストと音声認識結果を受信"""
        if nlg.get("reply"):
            self.txt = nlg["reply"]
            # ★時刻情報を保存
            self.request_id = nlg.get("request_id", 0)
            self.worker_name = nlg.get("worker_name", "")
            self.start_timestamp_ns = nlg.get("start_timestamp_ns", 0)
            self.completion_timestamp_ns = nlg.get("completion_timestamp_ns", 0)
            self.inference_duration_ms = nlg.get("inference_duration_ms", 0.0)
            
            # デバッグ出力：時刻情報受信確認
            now = datetime.now()
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            print(f"[{timestamp}][DEBUG-speechSynthesis] 時刻情報受信:")
            print(f"[{timestamp}][DEBUG-speechSynthesis] start_ns: {self.start_timestamp_ns}")
            print(f"[{timestamp}][DEBUG-speechSynthesis] completion_ns: {self.completion_timestamp_ns}")
            print(f"[{timestamp}][DEBUG-speechSynthesis] request_id: {self.request_id}")
            print(f"[{timestamp}][DEBUG-speechSynthesis] worker_name: {self.worker_name}")
            sys.stdout.flush()
            
            # ★受信データを標準出力で確認（コメントアウト：応答時のみ表示）
            # from datetime import datetime
            # now = datetime.now()
            # timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            # print(f"[{timestamp}][NLG受信確認] request_id:{self.request_id}, worker:{self.worker_name}")
            # print(f"[{timestamp}][NLG受信確認] start_ns:{self.start_timestamp_ns}, completion_ns:{self.completion_timestamp_ns}")
            # print(f"[{timestamp}][NLG受信確認] inference_ms:{self.inference_duration_ms}")
            # sys.stdout.flush()
            
            # ★音声認識結果リストを受信・保存（コメントアウト：頻繁出力を避ける）
            if nlg.get("source_words"):
                self.source_words = nlg["source_words"]
                # print(f"[SS] 音声認識結果リスト受信: {len(self.source_words)}個")
                # print(f"[SS] 対話生成根拠となった音声認識履歴（全{len(self.source_words)}個）:")
                # for i, word in enumerate(self.source_words):
                #     print(f"    [{i+1:3d}] {word}")
                # sys.stdout.flush()
            else:
                self.source_words = []

if __name__ == "__main__":
    tts = SpeechSynthesis()

    while True:
        text = input('input: ')
        tts.run(text)