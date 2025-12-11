""""
仕様
200ms以上の無音でバッファ削除 フラグを建てる
200ms以上のバッファ && 200ms以上の無音 -> 音声をモデルに入力
フラグが立っている状態で音声が入力される -> フラグを消してバッファに音声を貯める
200ms未満のバッファ && 200ms以上の無音 -> バッファ削除 && フラグを建てる
"""

import pyaudio
import matplotlib.pyplot as plt
import numpy as np
import webrtcvad
import time
import librosa
import librosa.display
from model_turntaking import ModelTurntaking

import threading
import queue

import sys

# グローバル変数を定義
audio_queue = queue.Queue()  # マイクからの音声データを保存するキュー


# 設定 ######################################################################
mic_sample_rate = 48000
sample_rate     = 16000
frame_duration  = 30  # ms
CHUNK           = int(mic_sample_rate * frame_duration / 1000)

TurnJudgeThreshold = 0.650


# audio start ###############################################################
def audiostart():
    audio = pyaudio.PyAudio() 

    # Sennheiser USB headset のデバイスIDを探す -----------------------------
    device_id = None
    for i in range(audio.get_device_count()):
        info = audio.get_device_info_by_index(i)
        if 'default' == info['name']:
            device_id = info['index']
            break

    # Sennheiser USB headset が見つからなかったら終了 ----------------------
    if device_id is None:
        print("defaultのマイク が見つかりませんでした。")
        exit()

    print(f"[default] index:{device_id}")

    # 見つかったデバイスIDを使用して、ストリームを開く ---------------------
    stream = audio.open(format = pyaudio.paInt16,
                        rate = mic_sample_rate,
                        channels = 1, 
                        input = True, 
                        frames_per_buffer = CHUNK,
                        input_device_index=device_id)

    return audio, stream


# audio stop #############################################################
def audiostop(audio, stream):
    stream.stop_stream()
    stream.close()
    audio.terminate()


# read plot data #########################################################
def read_plot_data(stream):
    data = stream.read(1024, exception_on_overflow = False)
    audiodata = np.frombuffer(data, dtype='int16')
    
    plt.plt(audiodata)
    plt.ylim(-10,10)
    plt.draw()
    plt.pause(0.001)
    plt.cla()


# draw bar ##############################################################
def draw_bar(volume, max_volume=32768, bar_length=50):
    """
    音量に応じたバーをCUIに表示します。
    volume: 現在の音量
    max_volume: 最大音量値
    bar_length: バーの最大の長さ
    """
    normalized_volume = int((volume / max_volume) * bar_length)
    bar = "■" * normalized_volume + " " * (bar_length - normalized_volume)
    sys.stdout.write("\r" + bar)
    sys.stdout.flush()


# マイク入力を別スレッドで実行 ##########################################
def mic_input_thread(sample_rate, CHUNK):
    (audio, stream) = audiostart()  # スレッド内でaudiostart()を実行

    while True:
        data = stream.read(CHUNK)
        audiodata = np.frombuffer(data, dtype='int16')
        audio_queue.put(audiodata)
    
    # スレッド終了時にaudiostop()を実行
    audiostop(audio, stream)


##########################################################################
# main 
##########################################################################
if __name__ == '__main__':
    # model load --------------------------------------------------------
    model = ModelTurntaking()
    print("model load successful")

    # WebRTC start ------------------------------------------------------
    vad = webrtcvad.Vad()
    vad.set_mode(3)

    # settings ----------------------------------------------------------
    sound_available = False
    sound_count = 0
    silent_count = 0
    sound = np.empty(0) # モデルに入力する音声まとめ(長い)

    # mic スレッドを開始 ------------------------------------------------
    mic_thread = threading.Thread(target=mic_input_thread, args=(sample_rate, CHUNK))
    mic_thread.start()

    # 設定値の確認 ------------------------------------------------------
    print(f"判定しきい値: {TurnJudgeThreshold}")


    ################################
    # main loop                    #
    ################################
    while True:
        try:
            # read data is connected to sound data ----------------------
            audiodata = audio_queue.get() # バッファから音声データを取得

            audiodata = np.array(audiodata, dtype='float')
            audiodata = librosa.resample(y=audiodata, orig_sr=mic_sample_rate, target_sr=sample_rate)
            audiodata = np.array(audiodata, dtype='int16')
            sound = np.concatenate([sound, audiodata])

            # マイクからの音量を計算して表示
            audiodata = np.array(audiodata, dtype='int32') # or dtype='float32'
            volume = np.sqrt(np.mean(audiodata**2))
            draw_bar(volume)

            # 音声を5秒に整形(5.1秒保持しておき、モデルに入力する際に無音0.1秒を消去)
            if sound.shape[0] >= int(5.1 * sample_rate):
                sound = sound[-int(5.1 * sample_rate):]
                assert sound.shape[0] == int(5.1 * sample_rate), 'sound length is illegal.'
            

            # 今micから読み込んだ音が「音声」かどうかチェック ----------
            audio_checkvad = sound[-int(sample_rate * frame_duration / 1000):] # 最新音声を取り出す
            audio_checkvad_bytes = np.array(audio_checkvad, dtype=np.int16).tobytes()

            if vad.is_speech(audio_checkvad_bytes, sample_rate):
                # 音声セグメント長の計測
                silent_count = 0
                sound_count += 1
                process_start_time = 0

                if sound_count >= (200 / frame_duration): # 200msより短い音声区間ならノイズとして無視
                    sound_available = True

            elif sound.shape[0] >= 5 * sample_rate:
                # 無音長の計測
                process_start_time = time.perf_counter()
                sound_count = 0
                silent_count += 1
                if silent_count >= (100 / frame_duration):
                    if sound_available:
                        # 音声を正規化 -----------------------------------------------
                        sound_comp = sound / np.abs(sound).max()

                        # WAVファイルを保存 ------------------------------------------
                        from scipy.io.wavfile import write
                        write('model_input_sound.wav', sample_rate, sound_comp[:int(5 * sample_rate)])

                        # rawファイルを保存 ------------------------------------------
                        file_path = "model_input_sound.raw"  # 書き込むファイルのパス
                        sound_comp16 = np.array(sound_comp*32767, dtype='int16')
                        sound_comp16[:int(5 * sample_rate)].tofile(file_path)

                        # モデルに音声を入力 -----------------------------------------
                        result, output = model.judge_turntaking(sound_comp[:int(5 * sample_rate)]) # 入力は float64 らしい。どうやら
                        result = 1 if output[0] > TurnJudgeThreshold else 0

                        # 結果 -------------------------------------------------------
                        if result == 0:
                            label = "STOP (waiting user utterance)"
                            mark = "  "
                        elif result == 1:
                            label = "SPEAK (system can utter it)"
                            mark = "###"
                        else:
                            print("ERROR:モデル構成間違えたorz")
                        
                        # 処理時間 -----------------------------------------------
                        process_end_time = time.perf_counter()
                        process_time = process_end_time - process_start_time
                        process_time = process_time * 1000 # 単位をmsになおす

                        # 表示 ----------------------------------------------------
                        # print(output[0][0])
                        print(f'\r[{label:<30}]\t{output[0][0]:.3f} (STOP/SPEAK) {mark:<3} ({process_time:3.0f}ms)')

                        sound_available = False


        except KeyboardInterrupt:
            print() 
            break

    mic_thread.join()  # スレッドが終了するのを待つ