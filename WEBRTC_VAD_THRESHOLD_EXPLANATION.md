# WebRTC VAD の閾値メカニズム

## 質問: WebRTC VADに有声か無声かを判定するための閾値はあるか？

### 結論

**WebRTC VADにはユーザーが設定可能な閾値パラメータは存在しません。**

代わりに、`mode` パラメータ（0〜3）が内部的に閾値を制御します。

---

## WebRTC VAD と SileroVAD の比較

| 項目 | SileroVAD | WebRTC VAD |
|------|-----------|------------|
| **閾値の設定方法** | `threshold=0.5` のように直接指定 | 設定不可 |
| **閾値の制御** | ユーザーが自由に設定可能 | `mode` (0〜3) で間接的に制御 |
| **出力形式** | `float` (0.0〜1.0) の確率値 | `bool` (True/False) の二値のみ |
| **確信度の取得** | 可能（確率値として） | 不可能 |
| **調整の柔軟性** | 高い（任意の閾値を設定可能） | 低い（4段階のみ） |

---

## WebRTC VAD の API

### 利用可能なメソッド

```python
import webrtcvad

vad = webrtcvad.Vad()

# モードを設定（0〜3）
vad.set_mode(3)  # 0: 最も寛容, 3: 最も厳格

# 音声判定
is_speech = vad.is_speech(audio_bytes, sample_rate)  # 戻り値: bool
```

### 存在しないメソッド

以下のようなメソッドは**存在しません**：

```python
# ❌ これらのメソッドは存在しない
vad.set_threshold(0.5)  # 閾値を直接設定
vad.get_probability()   # 確率値を取得
vad.get_confidence()    # 確信度を取得
```

---

## mode パラメータの効果

### 実験結果（test_webrtc_vad_threshold_modes_v2.py より）

```
テスト音声                          | mode 0 | mode 1 | mode 2 | mode 3
--------------------------------------------------------------------------------
完全な無音                          |   無音   |   無音   |   無音   |   無音
極小ノイズ (0.01)                   |   音声   |   音声   |   音声   |   無音   ← 違いが見える！
小ノイズ (0.05)                    |   音声   |   音声   |   音声   |   音声
中ノイズ (0.1)                     |   音声   |   音声   |   音声   |   音声
大ノイズ (0.3)                     |   音声   |   音声   |   音声   |   音声
```

**重要な観察**：
- 「極小ノイズ (0.01)」のケースで、mode 3 のみが「無音」と判定
- これは mode 3 の内部閾値が mode 0〜2 より高いことを示す
- しかし、この閾値の具体的な値をユーザーが知ることも変更することもできない

---

## WebRTC VAD の制限

### 1. 閾値の細かい調整ができない

```python
# SileroVAD なら可能
vad_iterator = VADIterator(model, threshold=0.3)  # 0.3に設定
vad_iterator = VADIterator(model, threshold=0.5)  # 0.5に設定
vad_iterator = VADIterator(model, threshold=0.7)  # 0.7に設定

# WebRTC VAD では不可能
vad = webrtcvad.Vad(mode=0)  # mode 0〜3の4段階のみ
```

### 2. 判定の確信度が得られない

```python
# SileroVAD なら可能
speech_prob = model(audio_chunk, 16000).item()  # 例: 0.834
if speech_prob > 0.5:
    print(f"音声（確信度: {speech_prob:.2%}）")

# WebRTC VAD では不可能
is_speech = vad.is_speech(audio_bytes, 16000)  # True or False のみ
# → 「どのくらい確信を持って判定したか」は不明
```

### 3. 4段階の固定的な調整のみ

| mode | 説明 | 用途 |
|------|------|------|
| 0 | 最も寛容（音声と判定しやすい） | ノイズが少ない環境 |
| 1 | やや寛容 | - |
| 2 | やや厳格 | - |
| 3 | 最も厳格（無音と判定しやすい） | ノイズが多い環境 |

---

## SileroVAD の利点

### 1. 閾値を自由に設定可能

```python
# 任意の値を設定できる
VADIterator(model, threshold=0.3)   # 音声を検出しやすい
VADIterator(model, threshold=0.5)   # 標準
VADIterator(model, threshold=0.7)   # 音声を検出しにくい
VADIterator(model, threshold=0.85)  # 非常に厳格
```

### 2. 確率値が得られる

```python
speech_prob = model(audio_chunk, 16000).item()
# 例: 0.234, 0.567, 0.891 など

# これにより、以下が可能:
# - 判定の確信度を評価
# - グラデーションのある処理（完全に二値ではない）
# - ログ記録や分析に有用
```

### 3. より柔軟なチューニング

アプリケーションの要件に応じて細かく調整可能：

```python
# ノイズが多い環境 → 高い閾値
vad_iterator = VADIterator(model, threshold=0.7)

# 静かな環境 → 低い閾値
vad_iterator = VADIterator(model, threshold=0.3)

# 中間
vad_iterator = VADIterator(model, threshold=0.5)
```

---

## 実装上の比較

### WebRTC VAD の使用例

```python
import webrtcvad

vad = webrtcvad.Vad(mode=3)
is_speech = vad.is_speech(audio_bytes, 16000)

if is_speech:
    print("音声")  # ただし、確信度は不明
else:
    print("無音")  # ただし、確信度は不明
```

### SileroVAD の使用例

```python
import torch

model, utils = torch.hub.load(
    repo_or_dir='snakers4/silero-vad',
    model='silero_vad'
)

speech_prob = model(audio_tensor, 16000).item()

if speech_prob > 0.5:  # threshold は自由に設定可能
    print(f"音声（確信度: {speech_prob:.1%}）")
else:
    print(f"無音（確信度: {1-speech_prob:.1%}）")
```

---

## まとめ

### WebRTC VAD の閾値について

1. **ユーザーが設定可能な閾値パラメータは存在しない**
2. **mode (0〜3) が内部的に閾値を制御する**
3. **出力は True/False の二値のみで、確率値は得られない**
4. **4段階の固定的な調整のみ可能**

### いつ WebRTC VAD を使うべきか

- **超高速処理が必要**（1フレームあたり1〜2μs）
- **シンプルな二値判定で十分**
- **レガシーシステムとの互換性が必要**
- **軽量なライブラリが必要**（依存関係が少ない）

### いつ SileroVAD を使うべきか

- **閾値を細かく調整したい**
- **判定の確信度を知りたい**
- **ニューラルネットワークベースの高精度な判定が必要**
- **処理時間に余裕がある**（それでも200μs以下で十分高速）

---

## 検証スクリプト

以下のスクリプトで実際に確認可能：

```bash
# WebRTC VAD の閾値メカニズムを検証
python3 /workspace/test_webrtc_vad_threshold_modes_v2.py
```

このスクリプトは以下を示します：
- WebRTC VAD には閾値を設定するAPIが存在しないこと
- mode パラメータが内部閾値を制御すること
- 出力が bool型の二値であること
- SileroVAD との違い

---

## 参考資料

- WebRTC VAD Python ラッパー: https://github.com/wiseman/py-webrtcvad
- Silero VAD: https://github.com/snakers4/silero-vad
- DiaROS VAD比較テスト: `/workspace/test_vad_comparison_fixed_audio.py`
- WebRTC VAD タイミング検証: `/workspace/README_VAD_TESTS.md`

---

作成日: 2025-11-17
