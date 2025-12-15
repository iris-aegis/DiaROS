import os

file_path = "/workspace/test_vad_comparison_fixed_audio copy.py"

try:
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    old_string = """        min_silence_duration_ms=96,
        speech_pad_ms=0"""
    new_string = """        min_silence_duration_ms=96,
        min_speech_duration_ms=32,
        speech_pad_ms=0"""

    if old_string not in content:
        print(f"Error: Target string not found in {file_path}")
    else:
        content = content.replace(old_string, new_string)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"Successfully added min_speech_duration_ms to {file_path}")

except Exception as e:
    print(f"Error: {e}")
