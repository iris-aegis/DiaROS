# WebRTC VAD 検証スクリプト

WebRTC VADの動作を検証するための3つのスクリプトを作成しました。

## 作成したスクリプト

### 1. test_webrtc_vad_timing.py
**目的**: WebRTC VADが100ms無声区間を検出するタイミングを検証

**実行方法**:
```bash
cd /workspace/DiaROS
python3 test_webrtc_vad_timing.py
```

**検証内容**:
- 30msフレームサイズでの無声検出
- 設定値100msに対して実際には120msで検出されることを確認
- 離散化による誤差（+20ms）の確認

**出力例**:
```
【検出イベントまとめ】
    810ms: 音声区間開始
   7440ms: 100ms無声検出
           → 無声継続時間: 120ms (フレーム数: 4)
           → 設定値100msに対して実際の検出: 120ms
```

---

### 2. test_webrtc_batch_vs_stream.py
**目的**: バッチ処理とストリーミング処理で判定結果が変わるか検証

**実行方法**:
```bash
cd /workspace/DiaROS
python3 test_webrtc_batch_vs_stream.py
```

**検証内容**:
- バッチ処理（一気に全フレーム処理）
- ストリーミング処理（1フレームずつ逐次処理）
- 判定結果の一致性確認
- 処理速度の比較

**出力例**:
```
【判定結果の比較】
✓ バッチ処理とストリーミング処理の判定結果は完全に一致
  → WebRTC VADはステートレス（フレーム間で状態を保持しない）

【処理速度の比較】
平均処理時間/フレーム: 2.38μs
リアルタイム係数: 0.0001x
✓ リアルタイム処理可能！（余裕: 29.998ms = 100.0%）
```

---

### 3. test_webrtc_fastest_streaming.py
**目的**: 最速設定（10msフレーム）での全フレーム出力確認

**実行方法**:
```bash
cd /workspace/DiaROS
python3 test_webrtc_fastest_streaming.py
```

**検証内容**:
- 10msフレームサイズ（WebRTC VADの最小値）
- 全フレームの判定結果を表示
- 正確に100msで無声検出できることを確認
- 処理速度の測定

**出力例**:
```
【ストリーミング処理: 全フレーム出力】
時刻(ms) | フレーム# | VAD判定 | sound_count | silent_count | 処理時間(μs) | イベント
7330ms   | 733      | 無音    | 1           |
7340ms   | 734      | 無音    | 2           |
...
7420ms   | 742      | 無音    | 10          | ★ 100ms無声検出！（実際: 100ms）

【結論】
100ms無声検出の実際の平均時間: 100.0ms
フレーム離散化による誤差: 0.0ms
```

---

## 必要な依存関係

すべてのスクリプトで必要なPythonパッケージ:

```bash
pip install numpy scipy librosa webrtcvad
```

**確認コマンド**:
```bash
python3 -c "import numpy, scipy, librosa, webrtcvad; print('All dependencies installed!')"
```

---

## 入力ファイル

すべてのスクリプトは `/workspace/DiaROS/script1.wav` を使用します。

**ファイルの確認**:
```bash
ls -lh /workspace/DiaROS/script1.wav
```

**別のWAVファイルを使用する場合**:

各スクリプトの以下の行を編集してください：

```python
wav_file = '/workspace/DiaROS/script1.wav'  # ← ここを変更
```

---

## クイックスタート

すべてのスクリプトを順番に実行:

```bash
cd /workspace/DiaROS

echo "=== Test 1: 100ms無声検出タイミング ==="
python3 test_webrtc_vad_timing.py

echo ""
echo "=== Test 2: バッチ vs ストリーミング ==="
python3 test_webrtc_batch_vs_stream.py

echo ""
echo "=== Test 3: 最速設定での全フレーム出力 ==="
python3 test_webrtc_fastest_streaming.py
```

---

## 出力の保存

結果をファイルに保存する場合:

```bash
# タイムスタンプ付きで保存
python3 test_webrtc_vad_timing.py > vad_timing_$(date +%Y%m%d_%H%M%S).log 2>&1

# 全スクリプトの結果を1つのファイルに保存
{
  echo "=== Test 1: 100ms無声検出タイミング ==="
  python3 test_webrtc_vad_timing.py
  echo ""
  echo "=== Test 2: バッチ vs ストリーミング ==="
  python3 test_webrtc_batch_vs_stream.py
  echo ""
  echo "=== Test 3: 最速設定での全フレーム出力 ==="
  python3 test_webrtc_fastest_streaming.py
} > all_vad_tests_$(date +%Y%m%d_%H%M%S).log 2>&1
```

---

## トラブルシューティング

### エラー: `No such file or directory: '/workspace/DiaROS/script1.wav'`

**解決策**: WAVファイルのパスを確認
```bash
find /workspace/DiaROS -name "*.wav" -type f | head -5
```

### エラー: `ModuleNotFoundError: No module named 'webrtcvad'`

**解決策**: 依存パッケージをインストール
```bash
pip install webrtcvad
```

### エラー: `ModuleNotFoundError: No module named 'librosa'`

**解決策**: librosaをインストール
```bash
pip install librosa
```

---

## 検証結果のまとめ

### 主要な発見

1. **WebRTC VADはステートレス**
   - バッチ処理とストリーミング処理で完全に同じ結果
   - 各フレームは独立して処理される

2. **離散化による誤差**
   - 30msフレーム: 100ms設定 → 実際120ms（誤差+20ms）
   - 10msフレーム: 100ms設定 → 実際100ms（誤差0ms）

3. **超高速処理**
   - 1フレームあたり1.42μs～2.38μs
   - リアルタイム処理に十分な余裕（99.99%）

4. **最適な設定**
   - 正確な100ms検出には10msフレームが最適
   - 処理速度とのトレードオフはほぼなし

---

## 関連ファイル

- `mic_input_original.py`: WebRTC VADを使用した元のコード（30msフレーム）
- `DiaROS_py/diaros/turnTaking.py`: VADIteratorを使用したターンテイキング実装

---

## ライセンス

これらのスクリプトはDiaROSプロジェクトの一部として、同じライセンスの下で提供されます。

---

## 更新履歴

- 2025-11-16: 初版作成
  - test_webrtc_vad_timing.py
  - test_webrtc_batch_vs_stream.py
  - test_webrtc_fastest_streaming.py
