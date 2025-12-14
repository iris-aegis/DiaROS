# WebRTC VAD 検証スクリプト - クイックスタート

## 📋 準備

### 1. 依存関係の確認

```bash
python3 -c "import numpy, scipy, librosa, webrtcvad; print('✓ OK')"
```

**エラーが出る場合**:
```bash
pip install numpy scipy librosa webrtcvad
```

### 2. 入力ファイルの確認

```bash
ls -lh /workspace/DiaROS/script1.wav
```

---

## 🚀 実行方法

### 方法1: 一括実行（推奨）

すべてのテストを順番に実行:

```bash
cd /workspace/DiaROS
./run_all_vad_tests.sh
```

### 方法2: 個別実行

#### テスト1: 100ms無声検出タイミング検証

```bash
cd /workspace/DiaROS
python3 test_webrtc_vad_timing.py
```

**確認内容**: 30msフレームで100ms設定が実際には120msになる

---

#### テスト2: バッチ vs ストリーミング比較

```bash
cd /workspace/DiaROS
python3 test_webrtc_batch_vs_stream.py
```

**確認内容**: 処理方式で結果が変わらないこと、処理速度

---

#### テスト3: 最速設定での全フレーム出力

```bash
cd /workspace/DiaROS
python3 test_webrtc_fastest_streaming.py
```

**確認内容**: 10msフレームで正確に100ms検出できること

---

## 📊 結果の保存

### ログファイルに保存

```bash
cd /workspace/DiaROS
./run_all_vad_tests.sh > vad_test_results_$(date +%Y%m%d_%H%M%S).log 2>&1
```

### 結果の確認

```bash
# 最新のログファイルを表示
ls -lt vad_test_results_*.log | head -1 | awk '{print $9}' | xargs cat
```

---

## 🔍 期待される結果

### テスト1の結果

```
100ms無声検出の実際の平均時間: 120.0ms
フレーム離散化による誤差: 20.0ms
```

### テスト2の結果

```
✓ バッチ処理とストリーミング処理の判定結果は完全に一致
平均処理時間/フレーム: 2.38μs
リアルタイム係数: 0.0001x
```

### テスト3の結果

```
100ms無声検出の実際の平均時間: 100.0ms
フレーム離散化による誤差: 0.0ms
平均処理時間: 1.42μs
```

---

## 📁 作成されるファイル

実行後、以下のファイルが作成されます:

```
/workspace/DiaROS/
├── test_webrtc_vad_timing.py           # テストスクリプト1
├── test_webrtc_batch_vs_stream.py      # テストスクリプト2
├── test_webrtc_fastest_streaming.py    # テストスクリプト3
├── run_all_vad_tests.sh                # 一括実行スクリプト
├── README_VAD_TESTS.md                 # 詳細ドキュメント
├── QUICKSTART_VAD_TESTS.md             # このファイル
└── vad_test_results_*.log              # 結果ログ（任意）
```

---

## 🛠️ トラブルシューティング

### エラー: `No such file or directory: script1.wav`

**原因**: 入力ファイルが見つからない

**解決策**:
```bash
# WAVファイルを探す
find /workspace/DiaROS -name "*.wav" -type f

# スクリプト内のパスを変更
# 各スクリプトの wav_file = '...' の行を編集
```

### エラー: `ModuleNotFoundError`

**原因**: 必要なパッケージがインストールされていない

**解決策**:
```bash
pip install numpy scipy librosa webrtcvad
```

### エラー: `Permission denied`

**原因**: 実行権限がない

**解決策**:
```bash
chmod +x /workspace/DiaROS/run_all_vad_tests.sh
chmod +x /workspace/DiaROS/test_webrtc*.py
```

---

## 💡 ヒント

### 処理速度を詳しく見たい

テスト2とテスト3の結果に詳細な統計が含まれています

### 特定のWAVファイルを使いたい

各スクリプトの先頭付近にある以下の行を編集:

```python
wav_file = '/workspace/DiaROS/script1.wav'  # ← ここを変更
```

### 出力を絞りたい

```bash
# 重要な情報のみ表示
python3 test_webrtc_vad_timing.py | grep -E "(検出イベント|結論|平均)"
```

---

## 📞 サポート

詳細なドキュメントは `README_VAD_TESTS.md` を参照してください。

```bash
cat /workspace/DiaROS/README_VAD_TESTS.md
```
