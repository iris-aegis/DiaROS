import pyaudio
import wave

# マイクの設定
FORMAT = pyaudio.paInt16  # オーディオフォーマット (16-bit)
CHANNELS = 1              # チャンネル数 (モノラル)
RATE = 44100              # サンプルレート (Hz)
CHUNK = 1024              # チャンクサイズ (データを小さなブロックに分割)

# 録音時間（秒）
RECORD_SECONDS = 5

# 録音用のオブジェクトを作成
p = pyaudio.PyAudio()

# マイクからのオーディオデータをストリームでキャプチャ
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("録音中...")

frames = []

# オーディオデータを録音
for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("録音終了")

# ストリームを停止
stream.stop_stream()
stream.close()

# PyAudioオブジェクトを閉じる
p.terminate()

# 録音したデータを保存
with wave.open("output.wav", 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))

print("データをoutput.wavに保存しました")

