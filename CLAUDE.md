# CLAUDE.md

このファイルはClaude Code (claude.ai/code) がこのリポジトリのコードで作業する際のガイダンスを提供します。

## 🔴 最重要事項 / CRITICAL REQUIREMENTS

### 日本語対応 / Japanese Language Support
**必ず日本語で対話してください。** ユーザーとのすべてのコミュニケーションは日本語で行う必要があります。
- コメント、説明、エラーメッセージなど、すべて日本語で記述
- 技術用語は必要に応じて英語併記可
- コード内のコメントも可能な限り日本語で記述

**ALWAYS communicate in Japanese.** All communication with users must be in Japanese.
- Comments, explanations, error messages should all be in Japanese
- Technical terms can include English when necessary
- Code comments should also be in Japanese whenever possible

## プロジェクト概要

このローカルリポジトリは、**DiaROS分散音声対話システムのNLG（自然言語生成）コンポーネント**です。
**重要**: このNLGコンポーネントは、メインのDiaROSシステムから分離された独立したノードとして動作し、別のPC上で実行されます。

### 分散アーキテクチャ構成
```
[メインPC: DiaROSシステム]
音声入力 → 音響解析 → 音声認識 → 対話管理 → 音声合成 → ターン管理
                                    ↓ ROS2トピック通信
[このPC: NLGコンポーネント]          
                          自然言語生成 (ChatGPT/Local LLM)
                                    ↑ ROS2トピック通信
```

### 分散実行の目的
- **処理負荷の分散**: 大型LLMの推論処理を別PCで実行
- **スケーラビリティ**: 複数のNLGノードを並列実行可能
- **専用GPU利用**: NLG専用PCでGPUリソースを効率的に活用
- **システム安定性**: メインシステムへの影響を最小化

### 通信フロー
1. **入力**: `DMtoNLG`トピック（メインPCから音声認識結果リスト受信）
2. **処理**: 音声認識結果を統合して自然な対話文を生成
3. **出力**: `NLGtoSS`トピック（生成された応答文をメインPCへ送信）
4. **タイミング情報**: メインPCでの詳細なタイミング分析のため時刻データも送信

## 必須コマンド

### 環境セットアップ
```bash
# ROS2環境設定
cd DiaROS_ros
source /opt/ros/foxy/setup.bash
source ./install/local_setup.bash

# Pythonパッケージのインストール
cd ../DiaROS_py
python -m pip install . --user

# ROS2パッケージのビルド
cd ../DiaROS_ros
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash
```

### NLGコンポーネントの起動
```bash
# 基本起動（ChatGPT API優先、未設定時はローカルLLM）
ros2 launch diaros_package sdsmod.launch.py

# ChatGPT APIモード
./scripts/launch/launch_diaros_chatgpt.sh

# ローカルLLMモード（軽量モデル）
./scripts/launch/launch_diaros_local.sh

# NLGノード単体起動
ros2 run diaros_package ros2_natural_language_generation
```

### 設定スクリプト
```bash
# モデル事前ダウンロード（推奨 - 起動時間短縮）
./scripts/setup/predownload_all_models.sh

# API設定
./scripts/setup/setup_api.sh
./scripts/setup/setup_chatgpt_api.sh

# ローカルモデルダウンロード（個別）
./scripts/setup/download_gemma_model.sh

# ROS2環境自動設定
./scripts/setup/setup_ros2_env.sh
```

## API設定

### 高速応答生成API (推奨)
DiaROSでは対話リズム維持のため、1500ms以内の応答が必要です。

**推奨API（優先順位順）:**
- **OpenAI API (ChatGPT)**: ~500-1000ms、最も高速で安定
- **Anthropic API (Claude)**: ~800-1200ms、高品質応答  
- **ローカルLLM**: ~2000-5000ms、オフライン動作可能

**API設定:**
```bash
# 環境変数設定
export OPENAI_API_KEY="sk-your-openai-api-key"
export ANTHROPIC_API_KEY="sk-ant-your-anthropic-api-key"

# または設定スクリプト使用
./scripts/setup/setup_api.sh
```

### ローカルLLMオプション
```bash
# 軽量高速モデル（推奨）
export DIAROS_LLM_MODEL=rinna-small

# 高品質モデル
export DIAROS_LLM_MODEL=gemma2

# デバイス設定
export DIAROS_DEVICE=cuda  # または 'cpu', 'mps' (macOS)
```

## 主要ファイル構成

### NLGコア実装
- **`DiaROS_py/diaros/naturalLanguageGeneration.py`**
  - 自然言語生成のメインクラス
  - ChatGPT APIとローカルLLM（Gemma、Rinna）の両対応
  - 音声認識結果リストの統合処理機能

### ROS2統合レイヤー
- **`DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`**
  - ROS2ノードラッパー
  - `DMtoNLG`トピック購読（音声認識結果受信）
  - `NLGtoSS`トピック配信（生成応答送信）

### メッセージ定義
- **`interfaces/msg/Idm.msg`**: Dialog Management→NLG（音声認識結果リスト）
- **`interfaces/msg/Inlg.msg`**: NLG→Speech Synthesis（生成応答文）

### 起動設定
- **`launch/sdsmod.launch.py`**: ROS2システム起動定義
- **`scripts/launch/`**: クロスプラットフォーム起動スクリプト

## 開発・デバッグ

### ROS2トピック監視
```bash
# トピック一覧表示
ros2 topic list

# NLG入力監視
ros2 topic echo /DMtoNLG

# NLG出力監視  
ros2 topic echo /NLGtoSS

# ノード関係図表示
ros2 run rqt_graph rqt_graph
```

### デバッグスクリプト
```bash
# NLG単体テスト
./scripts/debug/test_nlg_response.py

# API接続テスト
./scripts/test/test_api_nlg.py
./scripts/test/quick_api_test.py

# システム全体フロー確認
./scripts/debug/debug_diaros_flow.py
```

### ログ出力例
```
[14:23:45.123][NLG] 音声認識結果受信時刻: 14:23:45.123
[14:23:45.123][NLG] 受信した音声認識結果リスト: ['今日は', '良い天気', 'ですね']
[14:23:46.456][NLG] 対話生成結果送信時刻: 14:23:46.456  
[14:23:46.456][NLG] 送信する対話生成内容: そうですね、とても気持ちの良い日です。
```

## 技術仕様

### 音声認識結果の統合処理
- CER20%程度の音声認識結果を複数受信
- 雑音・無音区間を除去して自然な文章に復元
- リスト形式の断片的な認識結果から文脈を理解

### LLMモデル選択
- **rinna-small**: 500ms目標、軽量高速
- **calm-small**: 1000ms、中品質
- **gemma-2-2b**: 2000ms、高品質
- **ChatGPT-3.5/4**: 500-1000ms、クラウドAPI

### 分散アーキテクチャ
- ROS2ベースのリアルタイム通信
- 独立したプロセスとして実行可能
- 他の対話システムコンポーネントとの疎結合

## 依存関係

### 必須パッケージ
```bash
# ROS2関連
rclpy
interfaces (カスタムメッセージ)

# AI/ML関連  
openai
langchain-ollama
torch
transformers

# Python標準
requests
json
datetime
```

### システム要件
- **OS**: Ubuntu 20.04+ または macOS
- **ROS2**: Foxy以上
- **Python**: 3.8.13+
- **GPU**: CUDA対応（ローカルLLM使用時推奨）

## 重要な注意事項

### 改行コードの統一
**すべてのシェルスクリプトはLF（Unix形式）で作成すること。**
```bash
# 改行コードの確認・修正
file /path/to/script.sh
sed -i '' 's/\r$//' /path/to/script.sh  # macOS
dos2unix /path/to/script.sh           # Linux
chmod +x /path/to/script.sh
```

### パスの汎用性
- 絶対パスは使用禁止
- 相対パスで記述
- ユーザー固有のパスをハードコードしない

### 応答時間最適化
- 1500ms以内の応答を目標
- API選択の優先順位: OpenAI → Anthropic → ローカルLLM
- 応答時間超過時は警告メッセージを表示

## テストコマンド
現在、特定のテストフレームワークは設定されていません。手動テストスクリプトを使用：
```bash
./scripts/test/test_diaros_response.py
./scripts/test/test_api_nlg.sh
```

## リント・型チェックコマンド
プロジェクト設定にリント・型チェックコマンドは見つかりませんでした。

## Gemma 3 実装実験の記録

### 実験概要
2024年にGemma 3 (27B/12B) モデルを4ビット量子化でローカル実行するNLG実装を試行しました。

### 実装内容
- **モデル**: Gemma 3 27B/12B（4ビット量子化）
- **実装**: transformersライブラリベースの直接実装
- **GPU最適化**: 
  - BitsAndBytesConfig 4bit量子化
  - bfloat16データ型での数値安定性向上
  - Flash Attention 2による高速化（利用可能な場合）
  - 並列推論システム（2つのワーカースレッド）

### 技術的成果
- ✅ Gemma 3 27B/12Bの4ビット量子化でのローカル実行成功
- ✅ 事前ダウンロードスクリプトによる高速起動実現
- ✅ inf/nanエラーの解決（複数フォールバック機能）
- ✅ GPU メモリ使用量の最適化
- ✅ 並列推論による応答性向上

### パフォーマンス測定結果
- **Gemma 3 27B（4ビット量子化）**: 1100-1800ms
- **Ollama実装**: より高速（推論時間明記なし）

### 問題点と最終判断
1. **パフォーマンス**: transformers実装がOllama実装より遅い
2. **複雑性**: 実装とメンテナンスの複雑さ
3. **安定性**: Ollama実装の方が安定している

### 最終決定
**Ollama実装に戻すことを決定**

理由:
- パフォーマンスの優位性
- 実装の簡潔性
- 安定性の高さ
- ユーザーからの明確な要望

### アーカイブファイル
- `DiaROS_py/diaros/naturalLanguageGeneration_gemma3_archive.py`: Gemma 3 transformers実装
- `scripts/setup/predownload_all_models.py`: Gemma 3モデル事前ダウンロードスクリプト
- `scripts/setup/predownload_all_models.sh`: ダウンロードスクリプトのシェルラッパー

### 教訓
- 最新技術の採用は性能向上を保証するとは限らない
- 実装の複雑さは運用コストに直結する
- 既存の安定したソリューションの価値は高い
- ユーザーの要望と技術的最適性のバランスが重要

### 現在のNLG実装
元のOllama + LangChainベースの実装に戻し、`gemma3:12b`モデルを使用。

### ⚠️ プロンプト変更禁止事項
**重要**: `DiaROS_py/diaros/prompts/asr_dialogue_prompt.txt`に格納されているプロンプトは、ユーザーが明示的に変更許可を出すまで変更してはいけません。

理由:
- 音声認識結果の統合処理が複雑で、微細な変更でも品質に大きく影響する
- 20文字程度の応答制約や口調設定が対話システムの要件として確定している
- プロンプトの簡略化は推論速度向上に寄与するが、対話品質の劣化リスクが高い

現在のプロンプト構成:
- ASR結果の統合処理ルール
- 雑音・無音タグの処理指示  
- アンドロイドペルソナ設定
- 20文字応答制約
- 友達口調指定

変更する場合は必ずユーザーの許可を得てから実施すること。