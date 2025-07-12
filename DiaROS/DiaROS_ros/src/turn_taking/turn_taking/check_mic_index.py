import pyaudio  #録音機能を使うためのライブラリ

#######################################################################
# オーディオデバイスの情報を取得
# マイクのインデックス番号を入手する。
import pyaudio

def list_audio_devices():
    p = pyaudio.PyAudio()
    
    # 最大のデバイス名の長さを初期値として0に設定
    max_name_length = 0

    # すべてのデバイスの名前をチェックして、最大の長さを見つける
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name_length = len(info['name'])
        if name_length > max_name_length:
            max_name_length = name_length
    
    # ヘッダーを表示
    print(f"{'id':<2} {'name':<{max_name_length}} {'in':<3} {'out':<3} {'Hz':<8}")
    print("-" * (2 + max_name_length + 3 + 3 + 8 + 4))  # 各項目の幅と間のスペース合計

    # 各デバイスの情報を表示
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        name = info['name']
        max_input_channels = info['maxInputChannels']
        max_output_channels = info['maxOutputChannels']
        default_sample_rate = info['defaultSampleRate']

        print(f"{i:<2} {name:<{max_name_length}} {max_input_channels:<3} {max_output_channels:<3} {default_sample_rate:<8}")

    p.terminate()

list_audio_devices()