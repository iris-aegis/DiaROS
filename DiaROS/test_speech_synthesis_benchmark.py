import time
from diaros.speechSynthesis import SpeechSynthesis

def main():
    tts = SpeechSynthesis()
    test_text = "これは音声合成のベンチマークテストです。"  # 20文字
    print(f"テスト用テキスト: {test_text}（{len(test_text)}文字）")

    start = time.time()
    wav_path = tts.run(test_text)
    end = time.time()

    elapsed_sec = end - start
    elapsed_ms = elapsed_sec * 1000

    print(f"音声合成ファイル: {wav_path}")
    print(f"音声合成にかかった時間: {elapsed_sec:.3f}秒 ({elapsed_ms:.1f}ms)")

if __name__ == "__main__":
    main()
