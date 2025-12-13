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
[DMPC: DiaROSメインシステム]
音声入力 → 音響解析 → 音声認識 → 対話管理 → 音声合成 → ターン管理
                                    ↓ ROS2トピック通信
[NLGPC: このPC - NLG専用コンポーネント]
                          自然言語生成 (ChatGPT/Local LLM)
                                    ↑ ROS2トピック通信
```

### PC役割分担
- **DMPC (Dialog Management PC)**:
  - 音声入力、音響解析、音声認識、対話管理、音声合成、ターン管理を実行
  - DiaROSのメインシステムが稼働
  - **リポジトリブランチ**: `main`
  - **ソースコード参照**: [DiaROS main ブランチ](https://github.com/iris-aegis/DiaROS/tree/main/DiaROS)

- **NLGPC (Natural Language Generation PC) - このPC**:
  - **NLGモジュールのみを起動**
  - その他のモジュール（音声認識、対話管理など）は起動しない
  - LLM推論処理に特化
  - **リポジトリブランチ**: `local_nlg`
  - **ソースコード参照**: [DiaROS local_nlg ブランチ](https://github.com/iris-aegis/DiaROS/tree/local_nlg)

### 分散実行の目的
- **処理負荷の分散**: 大型LLMの推論処理を別PCで実行
- **スケーラビリティ**: 複数のNLGノードを並列実行可能
- **専用GPU利用**: NLG専用PCでGPUリソースを効率的に活用
- **システム安定性**: メインシステムへの影響を最小化
- **役割分離**: DMPCとNLGPCで明確に役割を分離し、効率的なシステム運用を実現

### ⚠️ DMPC側コードの確認方法
DMPC側のコード（音声認識、対話管理など）を確認する必要がある場合：
1. GitHub上の[main ブランチ](https://github.com/iris-aegis/DiaROS/tree/main/DiaROS)を参照
2. WebFetch ツールを使用してGitHub上のファイルを直接読み取り
3. 特に以下のモジュールはDMPC側で動作：
   - `DiaROS_py/diaros/automaticSpeechRecognition.py` - 音声認識
   - `DiaROS_py/diaros/dialogManagement.py` - 対話管理
   - `DiaROS_py/diaros/backChannel.py` - 相槌生成
   - `DiaROS_py/diaros/turnTaking.py` - ターンテイキング

### 🔧 DMPC側の修正が必要な場合
DMPC側（mainブランチ）のコードに問題や修正が必要な場合：
1. **NLGPC側（このPC）で可能な修正があれば実施**
2. **DMPC側での修正が必要な場合**：
   - 修正が必要な理由と詳細を説明
   - DMPC側のClaude Codeに渡すプロンプトを生成してユーザーに提示
   - ユーザーがDMPC側で修正を実施

**プロンプト生成の形式：**
```
【DMPC側で修正が必要です】

修正対象ファイル: DiaROS_py/diaros/dialogManagement.py
修正理由: [理由の詳細]
修正内容: [具体的な修正内容]

以下のプロンプトをDMPC側のClaude Codeに入力してください：
---
[ファイル名]の[行番号]付近で[問題の説明]。
[修正内容の詳細]に変更してください。

理由: [修正理由]
期待される動作: [修正後の期待される動作]
---
```

### 通信フロー
1. **入力**: `DMtoNLG`トピック（DMPCから音声認識結果リスト受信）
2. **処理**: NLGPC上で音声認識結果を統合して自然な対話文を生成
3. **出力**: `NLGtoSS`トピック（生成された応答文をDMPCへ送信）
4. **タイミング情報**: DMPCでの詳細なタイミング分析のため時刻データも送信

## 🎯 NLGPCでの作業範囲

このPCは**NLGモジュール専用**です。以下の作業のみを実施してください：

### ✅ 修正対象（NLGPCで実施）
- **`DiaROS_py/diaros/naturalLanguageGeneration.py`**: 推論ロジック
- **`DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`**: ROS2ラッパー
- **`DiaROS_py/diaros/prompts/`**: プロンプトテンプレート修正
- 応答生成パラメータ調整
- 推論エンジン（Ollama、OpenAI APIなど）の設定
- NLG関連の設定スクリプト

### ❌ 修正対象外（DMPCで実施 - 修正しないこと）
以下のモジュールはDMPC側で動作するため、**このPCでは修正しない**：
- **音声入力**: `speechInput.py`
- **音響分析**: `acousticAnalysis.py`
- **音声認識**: `automaticSpeechRecognition.py`
- **対話管理**: `dialogManagement.py`
- **相槌生成**: `backChannel.py`
- **ターンテイキング**: `turnTaking.py`
- **音声合成**: `speechSynthesis.py`

**重要**: DMPC側のモジュールに問題がある場合は、DMPC側で確認・修正してください。NLGPCでは修正できません。

## 🔄 ソースコード変更時の自動コミット

NLGモジュールの修正後、セキュリティ上の確認が必要でない限り、**自動的にgitにコミット**してください。

### 自動コミット対象
- NLG関連のすべてのPythonモジュール修正
- プロンプトテンプレート修正
- 設定ファイル修正
- ROS2メッセージ定義の修正（NLG関連）
- 起動スクリプト修正

### 自動コミット対象外
- セキュリティ認証情報（APIキーなど）
- 本番環境に直接影響する重大な変更
- ユーザーが明示的に確認を求めた場合

### コミットメッセージの形式
```
Fix/Feature/Refactor/Docs: 変更内容の簡潔な説明

詳細な説明（必要に応じて）

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

### コミット例
```bash
git add DiaROS_py/diaros/naturalLanguageGeneration.py
git commit -m "$(cat <<'EOF'
Fix: Second stage応答生成時の空データ処理を改善

- 空のASR結果でもfirst_stage_responseがあれば処理続行
- デバッグログを追加して受信データを可視化

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
EOF
)"
```

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

## 📡 通信インターフェース

### NLGPCが受け取るメッセージ（Idm - DMPC→NLGPC）
`/DMtoNLG` トピック経由で受信：
- **`words`**: 音声認識結果のリスト（例: `['今日は', '良い', '天気']`）
- **`session_id`**: セッション識別子
- **`stage`**: 処理段階（`'first'` = 相槌生成、`'second'` = 本応答生成）
- **`request_id`**: リクエスト識別子
- **`turn_taking_decision_timestamp_ns`**: TurnTaking判定時刻（ナノ秒）
- **`first_stage_backchannel_at_tt`**: TurnTaking判定時の相槌内容（Second stage用）
- **`asr_history_2_5s`**: 2.5秒間隔の音声認識結果リスト（Second stage用）

### NLGPCが返送するメッセージ（Inlg - NLGPC→DMPC）
`/NLGtoSS` トピック経由で送信：
- **`reply`**: 生成された応答テキスト（例: `"そうですね、良い天気ですね"`）
- **`stage`**: 完了したステージ（`'first'` または `'second'`）
- **`request_id`**: 対応するリクエスト識別子
- **`source_words`**: 対話生成の元にした音声認識結果
- **`worker_name`**: ワーカー名（並列処理時の識別用）
- **`start_timestamp_ns`**: 推論開始時刻（ナノ秒）
- **`completion_timestamp_ns`**: 推論完了時刻（ナノ秒）
- **`inference_duration_ms`**: 推論処理時間（ミリ秒）

### メッセージフロー例
```
1. DMPC → NLGPC (First stage):
   words=['今日は', '良い', '天気'], stage='first', request_id=1

2. NLGPC → DMPC:
   reply='うんうん', stage='first', request_id=1, inference_duration_ms=150

3. DMPC → NLGPC (Second stage):
   words=[], stage='second', request_id=1,
   first_stage_backchannel_at_tt='うんうん',
   asr_history_2_5s=['今日は', '良い', '天気', 'ですね']

4. NLGPC → DMPC:
   reply='そうですね、とても気持ちの良い日ですね',
   stage='second', request_id=1, inference_duration_ms=1200
```

## ⚙️ 環境設定（NLGPC）

### ROS2通信設定
DMPCと**同一のドメインID**を設定してください：
```bash
# ROS2ドメイン設定（DMPCと一致させる）
export ROS_DOMAIN_ID=0

# ROS2実装設定
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ログ設定（エラーのみ表示）
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'
```

### LLM設定
```bash
# 使用するモデルの指定
export DIAROS_LLM_MODEL=gemma3:4b  # または gemma3:12b, gemma3:27b

# デバイス設定
export DIAROS_DEVICE=cuda          # または 'cpu', 'mps' (macOS)

# OpenAI API使用時
export OPENAI_API_KEY="sk-your-openai-api-key"
```

### DiaROS Pythonモジュールのパス設定
```bash
export PYTHONPATH="/workspace/DiaROS/DiaROS_py:$PYTHONPATH"
```

## ✅ 修正後の確認項目

NLGモジュール修正後は以下を確認してください：

### 応答生成の確認
- ✅ **First stage応答時間**: 200ms以内（目標）
- ✅ **Second stage応答時間**: 1-2秒以内（推奨）
- ✅ メッセージの送受信が正常に行われているか
- ✅ DMPC側のログで「応答生成完了」が表示されるか

### トピック通信の確認
```bash
# DMPCからのメッセージ確認
ros2 topic echo /DMtoNLG

# NLGPCからの応答確認
ros2 topic echo /NLGtoSS

# トピック接続状態確認
ros2 topic info /DMtoNLG
ros2 topic info /NLGtoSS
```

### ノード状態の確認
```bash
# ノード一覧表示
ros2 node list

# NLGノードの詳細情報
ros2 node info /natural_language_generation
```

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