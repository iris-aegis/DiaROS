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

        # power_calibration.wavは無効化
        
        # Check if VOICEVOX is running, start if not
        self._ensure_voicevox_running()
    
    def _ensure_voicevox_running(self):
        """Check if VOICEVOX is running and display status message"""
        try:
            # Check if VOICEVOX is running by making a health check request
            response = requests.get('http://localhost:50021/speakers', timeout=2)
            if response.status_code == 200:
                print("✓ VOICEVOX is running and ready")
                return
        except requests.exceptions.RequestException:
            pass
        
        # VOICEVOX is not running - display clear message
        print("=" * 60)
        print("⚠️  VOICEVOX ENGINE IS NOT RUNNING")
        print("=" * 60)
        print("Please start VOICEVOX engine manually with one of these commands:")
        print("1. /opt/voicevox_engine/linux-nvidia/run --host 127.0.0.1 --port 50021")
        print("2. voicevox --host 127.0.0.1 --port 50021")
        print("=" * 60)
        print("Speech synthesis will fail until VOICEVOX is started.")
        print("=" * 60)
    
    def play_sound(self, filename, block=True):
        """pygame.mixerを使用して音声ファイルを再生"""
        try:
            pygame.mixer.music.load(filename)
            pygame.mixer.music.play()
            if block:
                while pygame.mixer.music.get_busy():
                    pygame.time.wait(100)
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
        response_cnt = 0
        print(f'TTS:{text}')
        sys.stdout.flush()
        tts_file = None
        try:
            # VOICEVOXパターン
            speaker = 60
            # 開始時間を記録
            start_time = datetime.now()

            host = "localhost"
            port = 50021
            params = (
                ('text', text),
                ('speaker', speaker),
            )
            # 音声合成のリクエストを送るJSONのプリセットを要求
            response1 = requests.post(
                f'http://{host}:{port}/audio_query',
                params=params
            )
            # response1からJSONオブジェクトを取得
            response1_data = response1.json()
            
            # 発話前後の無音時間でリップシンクを調整
            response1_data["prePhonemeLength"] = 0.275
            response1_data["postPhonemeLength"] = 0.0

            # 変更したJSONオブジェクトを文字列に変換
            modified_json_str = json.dumps(response1_data)
            
            ### Unity送信用 ###
            # 時系列音素抽出用変数
            all_segments = []

            # アクセントフレーズ内の全てのモーラに対して処理
            for accent_phrase in response1_data['accent_phrases']:
                for mora in accent_phrase['moras']:
                    vowel_type = mora['vowel']
        #                     # moraごとのconsonant_length, vowel_length, vowelを表示
                    consonant_length = mora['consonant_length']
                    vowel_length = mora['vowel_length']
        #                     vowel = mora['vowel']
        #                     print(f"Consonant Length: {consonant_length}, Vowel Length: {vowel_length}, Vowel: {vowel}")
        #                     # 各モーラのvowelを連結
        #                     all_vowels += mora['vowel']
                    
                    if consonant_length is None:
        #                         speech_seconds += vowel_length
                        duration = vowel_length
                    else:
        #                         speech_seconds += consonant_length + vowel_length
                        duration = vowel_length + consonant_length
                    
                    segment = {
                        "vowel_type": vowel_type,
                        "length": duration
                    }
                    all_segments.append(segment)# データを配列形式に整形

                pause = 0.0
                # "pause_mora" フィールドが辞書である場合の処理を追加
                if isinstance(accent_phrase['pause_mora'], dict):
                    pause_mora = accent_phrase['pause_mora']
                    pause_consonant = pause_mora['consonant']
                    pause_consonant_length = pause_mora['consonant_length']
                    pause_vowel_length = pause_mora['vowel_length']
        #                     print(pause_consonant)
                    
                    if pause_consonant_length is not None:
        #                         speech_seconds += pause_consonant_length
                        pause += pause_consonant_length
                    if pause_vowel_length is not None:
        #                         speech_seconds += pause_vowel_length
                        pause += pause_vowel_length

                    pause_segment = {
                        "vowel_type": "silent_vowel",
                        "length": pause
                    }
                    all_segments.append(pause_segment)# データを配列形式に整形
            wrapped_data = {"data": all_segments}
            json_str = json.dumps(wrapped_data)
            if DEBUG:print(json_str)
            ###---###

            ### Unityから読み込める形式でjsonファイルに保存 ###
            #Use the current date and time to create a unique file name
            current_time = datetime.now().strftime("%Y%m%d%H%M%S")
            
            json_file = './tmp/' + str(current_time) + '.json'
            with open(json_file, 'w', encoding='utf-8') as f:
                json.dump(wrapped_data, f, ensure_ascii=False, indent=4)
            ###---###

            headers = {'Content-Type': 'application/json',}
            response2 = requests.post(
                f'http://{host}:{port}/synthesis',
                headers=headers,
                params=params,
                data=modified_json_str.encode('utf-8')
            )

            ### jsonファイルが先に作成されるのでjsonファイルが作成された時刻に名前を合わせる
            input_file = './{}/input_{}.wav'.format(self.TMP_DIR, current_time)

            wf = wave.open(input_file, 'wb')
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(24000)
            wf.writeframes(response2.content)
            wf.close()

            if DEBUG:print("Length of audio data: ", len(response2.content))
            if DEBUG:print("Status code: ", response2.status_code)

            tts_file = './tmp/' + str(current_time) + '.wav'
            self.trim_wav(input_file, tts_file)

            # ここで音声ファイルを再生
            # try:
            #     self.play_sound(tts_file, True)
            # except Exception as e:
            #     print(f"音声再生エラー: {e}")

            # 終了時間を記録
            end_time = datetime.now()
            # 処理時間を計算して表示
            elapsed_time = end_time - start_time
            # if DEBUG:sys.stdout.write("\n\n"+f"音声合成にかかった処理時間（秒）:" + str(elapsed_time) + "\n\n")
            # if DEBUG:sys.stdout.flush()
            # #現在時刻を表示
            # if DEBUG:print("応答文再生開始刻："+datetime.now().strftime('%Y/%m/%d %H:%M:%S.%f')[:-3])
            
            # if self.sound_available == False:
            #     if datetime.now() - self.prev_response_time > timedelta(seconds=self.response_pause_length):
            #         self.play_sound(tts_file, True)
            #         self.prev_response_time = datetime.now() 
            # os.remove(tts_file)
            self.speak_end = True
            self.last_tts_file = tts_file  # 直近の合成ファイル名を記録

            # Unityに口形素列を送信
            # if self.sound_available == False:
            #     if datetime.now() - self.prev_response_time > timedelta(seconds=self.response_pause_length):
                    # Unityから読み込める形式でファイルに保存
                    # with open('data.json', 'w', encoding='utf-8') as f:
                    #     json.dump(json_str, f, ensure_ascii=False, indent=4)
                    # dummy_signal = "animation start"
                    # client.sendto(dummy_signal.encode('utf-8'),(HOST,PORT))
                    # self.prev_response_time = datetime.now() 

            return tts_file
        except Exception as e:
            print('VOICEVOXerror: VOICEVOX sound is not generated. Do you launch VOICEVOX?')
            print(e.args)
            self.last_tts_file = ""
            return None

if __name__ == "__main__":
    tts = SpeechSynthesis()

    while True:
        text = input('input: ')
        tts.run(text)