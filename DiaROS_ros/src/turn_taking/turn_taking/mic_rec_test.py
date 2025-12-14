import os
os.environ['PYAUDIO_IGNORE_ALSA_PLUGHW'] = '1'

import pyaudio
import wave
import time
import sys

# パラメータ #######################################################################
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 48000
CHUNK = 1024
RECORD_SECONDS = 5
OUTPUT_FILENAME = "output.wav"

# pyaudio #########################################################################
audio = pyaudio.PyAudio()

# Sennheiser USB headset のデバイスIDを探す #######################################
device_id = None
for i in range(audio.get_device_count()):
    info = audio.get_device_info_by_index(i)
    if 'Sennheiser USB headset' in info['name']:
        device_id = info['index']
        break

# Senmheiser USB headset が見つからなかったら終了
if device_id is None:
    print("Sennheiser USB headset が見つかりませんでした。")
    exit()

print(f"[Sennheiser USB headset] index:{device_id}")

# 見つかったデバイスIDを使用して、ストリームを開く ###############################
stream = audio.open(format=FORMAT, channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK,
                    input_device_index=device_id)

# 録音開始 ########################################################################
print("録音中...", end='')
frames = []
start_time = time.time()

try:
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
        
        elapsed_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        sys.stdout.write(f"\r録音中... {elapsed_time:.0f} ms/ {RECORD_SECONDS*1000} ms")
        sys.stdout.flush()

except KeyboardInterrupt:
    pass  # Allow manual interruption with Ctrl+C

print("\n録音終了")

# 録音の停止とストリームのクローズ ############################################
stream.stop_stream()
stream.close()
audio.terminate()

# 音声データをファイルに保存 ###################################################
with wave.open(OUTPUT_FILENAME, 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(audio.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
