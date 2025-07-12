# TurtTaking,back_channelのプログラムをDiaROS内に入れる開発
import numpy as np
from aubio import pitch
import sys
import time
import socket
import json

# 定数
tolerance = 0.8
win_s = 2048
# hop_s = 220
hop_s = 240
f0_upper = 600.0
VAD_THRES_DB = 150
TAIL_MARGIN = 500  # 発話終了判定のマージン（ミリ秒）

class AcousticAnalysis:
    def __init__(self, rate):
        sys.stdout.write('acousticAnalysis start\n')
        self.rate = rate
        self.pitch_o = pitch("yinfast", win_s, hop_s, rate)
        self.pitch_o.set_unit("Hz")
        self.pitch_o.set_tolerance(tolerance)
        self.f0_list = []
        self.zc_list = []
        self.count = 0
        self.grad = 0.0
        self.speaking_start_time = None
        self.speaking_end_time = None
        self.elapsed_time = 0  # 発話継続時間の初期化
        self.input_data = None
        self.update_flag = False
        self.last_result = None

        # # TCPソケットの初期化
        # self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.tcp_ip = "localhost"  # 送信先のIPアドレス
        # self.tcp_port = 55555      # 送信先のポート番号

        # for i in range(5):
        #     try:
        #         self.tcp_socket.connect((self.tcp_ip, self.tcp_port))
        #         print(f"Connected to {self.tcp_ip}:{self.tcp_port}")
        #         break
        #     except ConnectionRefusedError:
        #         print("back_channelのサーバーの起動を待っています...")
        #         time.sleep(1)
        # else:
        #     print("back_channelのサーバーが起動していません。")
        #     sys.exit(1)

    def __del__(self):
        """オブジェクト破棄時にソケットを閉じる"""
        # self.tcp_socket.close()

    def zeroCrossing(self, signal):
        return np.count_nonzero(np.diff(np.signbit(signal)))

    def calculate_decibels(self, audio_signal, reference_pressure=20e-6):
        signal = np.array(audio_signal, dtype='float32')
        rms = np.sqrt(np.mean(np.square(signal)))
        if rms == 0:
            return 0
        decibels = 20 * np.log10(rms / reference_pressure)
        return decibels

    def update_speaking_status(self, power, current_time):
        """発話状態を更新し、発話継続時間を管理"""
        if power > VAD_THRES_DB:  # 音声がしきい値を超えた場合
            if self.speaking_start_time is None:
                self.speaking_start_time = current_time  # 発話開始時刻を記録
            self.speaking_end_time = None  # 発話中は終了タイマーをリセット
        else:  # 音声がしきい値以下の場合
            if self.speaking_end_time is None:
                self.speaking_end_time = current_time  # 終了タイマー開始
            elif current_time - self.speaking_end_time > TAIL_MARGIN / 1000:
                # 発話終了が確定した場合
                self.speaking_start_time = None
                self.speaking_end_time = None
                self.elapsed_time = 0  # 継続時間をリセット

        # 継続時間を計算
        if self.speaking_start_time is not None:
            self.elapsed_time = current_time - self.speaking_start_time
        else:
            self.elapsed_time = 0

    def resample(self, data, original_rate, target_rate):
        """
        音声データを異なるサンプルレートにリサンプリングする
        """
        # 固定サイズの出力を保証する
        desired_samples = hop_s  # 必要なサンプル数
        
        # リサンプリング比率を計算
        ratio = float(target_rate) / original_rate
        number_of_samples = round(len(data) * ratio)
        
        # 数が少なすぎる場合は、ゼロパディングする
        if number_of_samples < desired_samples:
            resampled_data = np.zeros(desired_samples, dtype=np.float32)
            if number_of_samples > 0:
                # 計算したサンプル数でリサンプリング
                temp_data = np.interp(
                    np.linspace(0.0, len(data), number_of_samples, endpoint=False),
                    np.arange(len(data)),
                    data
                )
                # 結果をコピー
                resampled_data[:number_of_samples] = temp_data
            return resampled_data.astype(np.float32)  # 明示的にfloat32として返す
        else:
            # 十分なサンプル数がある場合は、通常通りリサンプリング
            return np.interp(
                np.linspace(0.0, len(data), desired_samples, endpoint=False),
                np.arange(len(data)),
                data
            ).astype(np.float32)  # 明示的にfloat32として返す

    def update(self, input_data):
        self.input_data = input_data
        self.update_flag = True

    def run(self):
        while True:
            if self.update_flag and self.input_data is not None:
                result = self._analyze(self.input_data)
                self.last_result = result
                self.update_flag = False
            time.sleep(0.01)

    def _analyze(self, inputs):
        self.count += 1
        signal = np.frombuffer(inputs, dtype=np.int16).astype(np.float32)

        # パワー計算
        power = float(self.calculate_decibels(signal))

        # リサンプリング処理
        # 常に240サンプルになるよう修正したリサンプリング関数を使用
        # 明示的にfloat32型にキャストする
        chunk_resampled = self.resample(signal, self.rate, self.rate).astype(np.float32)
        
        # ピッチ計算
        if power > VAD_THRES_DB:
            # データの型と形状を確認するデバッグ出力を追加
            # sys.stdout.write(f"DEBUG: chunk_resampled type: {chunk_resampled.dtype}, shape: {chunk_resampled.shape}\n")
            f0 = float(self.pitch_o(chunk_resampled)[0])
        else:
            f0 = float(0)
            power = float(0)

        # ゼロ交差計算
        zerocross = self.zeroCrossing(signal)
        self.zc_list.append(zerocross)

        #リサンプリングなし
        # # ピッチ計算
        # if power > VAD_THRES_DB:
        #     f0 = float(self.pitch_o(signal)[0])
        # else:
        #     f0 = 0
        #     power = 0

        self.f0_list.append(f0)

        # 勾配計算
        if self.count == 30:
            x = np.array(list(range(30)))
            y = np.array(self.f0_list)
            self.grad = np.polyfit(x, y, 1)[0]
            self.f0_list = []
            self.count = 0

        # 発話状態を更新
        current_time = time.time()  # 現在時刻を取得
        self.update_speaking_status(power, current_time)

        # 音響分析結果を作成
        result = {
            "f0": f0,
            "grad": self.grad,
            "power": power,
            "zerocross": zerocross,
            "speak_time": self.elapsed_time
        }
        # sys.stdout.write('\n'+f"{result}を送信した\n")

        # # TCPで送信
        # try:
        #     # 送信データをJSON形式にする
        #     # sys.stdout.write('\n'+f"{result}を送信した\n")
        #     message = json.dumps(result).encode("utf-8")
        #     self.tcp_socket.sendall(message)
        #     # print(f"Sent: {message}")  # 送信データを確認
        # except socket.error as e:
        #     print(f"Error sending data: {e}")

        return result
