# CLAUDE.md

このファイルはこのリポジトリで Claude Code (claude.ai/code) を使用する際のガイダンスを提供します。

## 🔴 最重要事項

### 実行環境
**この環境はDockerコンテナ内で実行されています。**
- すべてのコマンドはDockerコンテナ内で実行される前提
- ホストOSとは独立した環境で動作
- プロセス管理（Ollama停止など）もコンテナ内のプロセスが対象

#### Dockerコンテナ環境の制約
- **sudo権限**: コンテナ内では既にroot権限で実行されているため、`sudo` コマンドは不要（かつ使用できない場合がある）
- **systemd/systemctl**: Dockerコンテナ内ではsystemdが動作していないため、`systemctl` コマンドは使用不可
- **サービス管理**: サービスの起動・停止は直接プロセスを操作（`pkill`, `&` でバックグラウンド実行など）

#### Dockerコンテナ内でのOllama制御
```bash
# Ollamaプロセスを確認
ps aux | grep ollama | grep -v grep

# Ollamaを停止（強制終了）
pkill -9 ollama

# または穏やかに停止
pkill ollama

# Ollamaを起動（バックグラウンド）
ollama serve &
```

#### 注意事項
- `sudo` を使わずに直接コマンドを実行
- `systemctl start/stop/restart` の代わりに、プロセスを直接起動・停止
- サービスの自動起動設定（systemd unit）は使用不可

### 日本語対応
**必ず日本語で対話してください。** ユーザーとのすべてのコミュニケーションは日本語で行う必要があります。
- コメント、説明、エラーメッセージなど、すべて日本語で記述
- 技術用語は必要に応じて英語併記可
- コード内のコメントも可能な限り日本語で記述

### スクリプト・コマンド実行の厳格なルール
**既存のスクリプトやツールを必ず確認・活用すること。** 新規作成前に徹底的な調査が必要です。
1. **必ず既存実装を探す**: コマンドやスクリプトを実行する前に、同じ機能のものが既に存在しないか十分に確認
2. **既存ツールを精査**: 見つかった場合は内容を精査し、目的に合致すれば必ずそれを使用
3. **新規作成は最終手段**: 既存のものがない場合のみ新規作成を検討
4. **スクリプトの配置ルール**:
   - **scripts/ルートディレクトリには直接ファイルを置かない**
   - 必ず適切なサブディレクトリに配置する:
     - `debug/`: デバッグ・モニタリング
     - `launch/`: 起動スクリプト
     - `setup/`: セットアップ・設定
     - `test/`: テストスクリプト
     - `utils/`: その他ユーティリティ

### 改行コードの統一
**すべてのシェルスクリプトはLF（Unix形式）で作成すること。**
- Windowsの改行コード（CRLF）は使用禁止
- 新規作成時は必ずLFを使用
- エディタの設定を確認してLFに統一

**重要：スクリプト作成時の手順**
1. 必ずWriteツールで作成すること（Editツールは改行コードが不正になる場合がある）
2. 作成後、以下のコマンドで改行コードを確認・修正：
   ```bash
   # 改行コードの確認
   file /path/to/script.sh

   # CRLFをLFに変換（macOS）
   sed -i '' 's/\r$//' /path/to/script.sh

   # または dos2unix を使用
   dos2unix /path/to/script.sh
   ```
3. 実行権限を付与：
   ```bash
   chmod +x /path/to/script.sh
   ```

### ソースコード変更時の自動コミット
**ソースコード（.py, .sh, .launch.py など）に変更を加えた場合、セキュリティ上の確認が必要でない限り、自動的に git にコミットしてください。**

#### 自動コミット対象
- `DiaROS_py/diaros/` 内の Python モジュール（.py ファイル）
- `DiaROS_ros/src/` 内の ROS2 パッケージファイル
- `scripts/` 内のユーティリティスクリプト
- PromptTemplate や設定ファイル（dialog_*.txt など）

#### 自動コミット対象外（手動確認推奨）
- セキュリティ認証情報（API キー、パスワードなど）を含むファイル
- 依存関係の大幅な変更（requirements.txt、package.xml など大幅な修正）
- 本番環境に直接影響する重大な変更（ユーザーに確認を取ってから実施）

#### コミットメッセージの形式
```
Fix/Feature/Refactor/Docs: 変更内容の簡潔な説明

詳細な説明（必要に応じて）

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
```

このルールにより、開発サイクルを効率化し、変更履歴を常に最新に保ちます。

### 現在の運用構成（NLG分散実行）
**このリポジトリは、NLGモジュール（自然言語生成）を別PCで実行する構成で運用されています。**

#### システム構成
- **DMPC（このPC）**: このリポジトリの main ブランチを使用
  - NLGノードなしで起動（`nlg:=false`）
  - 音声入力、音響分析、音声認識、対話管理、相槌生成、ターンテイキング、相槌予測を実行
  - DM から NLGPC へ Second stage リクエストを ROS2 トピック経由で送信

- **NLGPC（NLG専用PC）**: 別リポジトリの `local_nlg` ブランチを使用
  - リポジトリ: https://github.com/iris-aegis/DiaROS/tree/local_nlg
  - `ros2 run diaros_package ros2_natural_language_generation` で単独実行
  - DM からのリクエストを受け取り、Second stage 応答を生成
  - 生成した応答を DMPC へ返送

#### NLGPC で実行されるモジュール
NLGPC では以下のソースコードが使用されます（別リポジトリ `local_nlg` ブランチから）：
- `DiaROS_py/diaros/naturalLanguageGeneration.py`: コア NLG エンジン
- `DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`: ROS2 ラッパー
- `DiaROS_py/diaros/prompts/`: プロンプトテンプレート

##### 重要：NLGPC ブランチの確認

このリポジトリ（DMPC）と NLGPC では**異なるブランチ**を使用しています。NLGモジュールに関する以下の作業を行う場合は、**NLGPC のブランチを確認・修正してください**：

- NLGモジュールの実装詳細確認
- 推論ロジックの変更
- プロンプトテンプレートの修正
- 応答生成パラメータ調整

**NLGPC で使用されるブランチ（必要に応じて参照・修正）：**
```
https://github.com/iris-aegis/DiaROS/tree/local_nlg
```

DMPC での作業（音声入力、対話管理、相槌生成など）は、このリポジトリの main ブランチで実施してください。

#### 通信プロトコル
- **ROS2 トピック**: DMtoNLG / NLGtoDM 経由でメッセージ交換
- **メッセージ型**: Idm（DM→NLG）、Inlg（NLG→DM）
  - `first_stage_backchannel_at_tt`: TurnTaking判定時の相槌
  - `asr_history_2_5s`: 2.5秒間隔の音声認識結果リスト

#### 前提条件
- 両PC間で同一の `ROS_DOMAIN_ID` を設定
- ROS2 マルチキャスト通信が可能なネットワーク環境
- 両PC間の時刻同期（NTP推奨）

このセットアップにより、応答生成処理をスケールアウト可能にし、推論処理の遅延をDMPC 音声処理から分離します。

## NLGPC での Claude Code 実行ガイダンス

**以下の指示は、NLGPC（別リポジトリの `local_nlg` ブランチ）で Claude Code を使用する場合に適用されます。**

### NLGPC での作業範囲
NLGPC では以下の作業を実施してください：

#### 修正対象（NLGPC）
- NLGモジュールの実装詳細確認と修正
- `DiaROS_py/diaros/naturalLanguageGeneration.py`: 推論ロジック
- `DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`: ROS2 ラッパー
- `DiaROS_py/diaros/prompts/`: プロンプトテンプレート修正
- 応答生成パラメータ調整
- 推論エンジン（Ollama、OpenAI API など）の設定

#### 修正対象外（DMPC で実施）
**以下の作業は DMPC（main ブランチ）で実施してください。修正しないでください：**
- 音声入力（speechInput.py）
- 音響分析（acousticAnalysis.py）
- 音声認識（automaticSpeechRecognition.py）
- 対話管理（dialogManagement.py）
- 相槌生成（backChannel.py）
- ターンテイキング（turnTaking.py）
- 音声合成（speechSynthesis.py）

DMPC での変更がある場合は、DMPC 側で確認してください。NLGPC では DMPC からのメッセージを受け取るインターフェース部分のみを修正してください。

### NLGPC でのルール

#### 1. ブランチ確認
- 使用ブランチ：`local_nlg`
- DMPC との違い：
  - DMPC（main ブランチ）とは異なるコードベース
  - NLG固有の実装に集中
  - 音声入力・対話管理は含まない

#### 2. ソースコード変更時の自動コミット
NLGPC でも同じルールを適用：
- **自動コミット対象**: NLG関連のすべてのファイル変更
- **自動コミット対象外**: セキュリティ認証情報、本番環境に影響する重大な変更

#### 3. 通信インターフェースの確認
NLGPC が受け取るメッセージ：
- **Idm メッセージ**（DMPC→NLGPC）:
  - `first_stage_backchannel_at_tt`: TurnTaking判定時の相槌
  - `asr_history_2_5s`: 2.5秒間隔の音声認識結果
  - `stage`: 処理段階（'first' または 'second'）
  - `request_id`: リクエスト識別子

NLGPC が返送するメッセージ：
- **Inlg メッセージ**（NLGPC→DMPC）:
  - `reply`: 生成された応答テキスト
  - `stage`: 完了したステージ
  - `request_id`: リクエスト識別子
  - タイミング情報

#### 4. 環境設定（NLGPC）
NLGPC で設定すべき環境変数：
```bash
# ROS2通信設定（DMPC と同一）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2ログ設定
export RCL_LOGGING_SYSLOG=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'

# LLM設定
export DIAROS_LLM_MODEL=gemma2  # または ollama, openai など
export DIAROS_DEVICE=cuda       # または cpu, mps など

# API設定（OpenAI 使用時）
export OPENAI_API_KEY="sk-your-key"
```

#### 5. NLG固有の確認項目
修正後は以下を確認してください：
- 応答生成の遅延（推奨: 1-2秒以内）
- メッセージの送受信が正常に行われているか
- DMPC 側のログで「応答生成完了」が表示されるか
- Second stage のタイムアウト（デフォルト: 5秒）内に応答が生成されるか

#### 6. デバッグ時の推奨コマンド
```bash
# NLGPCでの実行
ros2 run diaros_package ros2_natural_language_generation

# DMPCからのメッセージ確認
ros2 topic echo /DMtoNLG

# NLGPCからの応答確認
ros2 topic echo /NLGtoDM

# ノード状態確認
ros2 node list
ros2 topic list
```

## NLGPC に修正が必要な場合

**Second stage の応答生成に問題がある場合や、推論ロジック・プロンプトを修正する必要がある場合、NLGPC（別リポジトリ `local_nlg` ブランチ）の Claude Code に以下のプロンプトを設定して修正してください。**

### NLGPC 用プロンプト（コピーしてNLGPC側に設定）

```
# NLGPC での Claude Code 実行プロンプト

このプロンプトを NLGPC（別リポジトリ `local_nlg` ブランチ）の Claude Code に設定してください。

## 🔴 最重要事項

### NLGPC での作業範囲

このPC は **NLGモジュール専用** です。以下の作業のみを実施してください：

#### ✅ 修正対象（NLGPC で実施）
- `DiaROS_py/diaros/naturalLanguageGeneration.py`: 推論ロジック
- `DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`: ROS2 ラッパー
- `DiaROS_py/diaros/prompts/`: プロンプトテンプレート修正
- 応答生成パラメータ調整
- 推論エンジン（Ollama、OpenAI API など）の設定

#### ❌ 修正対象外（DMPC で実施 - 修正しないこと）
- 音声入力（speechInput.py）
- 音響分析（acousticAnalysis.py）
- 音声認識（automaticSpeechRecognition.py）
- 対話管理（dialogManagement.py）
- 相槌生成（backChannel.py）
- ターンテイキング（turnTaking.py）
- 音声合成（speechSynthesis.py）

DMPC 側で変更がある場合は DMPC で確認してください。

### ブランチ確認
- **使用ブランチ**: `local_nlg`
- **リポジトリ**: https://github.com/iris-aegis/DiaROS/tree/local_nlg
- DMPC（main ブランチ）とは異なるコードベースです

### ソースコード変更時の自動コミット

NLGモジュールの修正後、セキュリティ上の確認が必要でない限り、自動的に git にコミットしてください。

#### 自動コミット対象
- NLG関連のすべての Python モジュール修正
- プロンプトテンプレート修正
- 設定ファイル修正

#### 自動コミット対象外
- セキュリティ認証情報（API キーなど）
- 本番環境に直接影響する重大な変更

## 通信インターフェース

### NLGPC が受け取るメッセージ（Idm - DMPC→NLGPC）
- `first_stage_backchannel_at_tt`: TurnTaking判定時の相槌内容
- `asr_history_2_5s`: 2.5秒間隔の音声認識結果リスト
- `stage`: 処理段階（'first' または 'second'）
- `request_id`: リクエスト識別子
- `turn_taking_decision_timestamp_ns`: TurnTaking判定時刻

### NLGPC が返送するメッセージ（Inlg - NLGPC→DMPC）
- `reply`: 生成された応答テキスト
- `stage`: 完了したステージ
- `request_id`: リクエスト識別子

## 修正後の確認項目

修正後は以下を確認してください：
- ✅ 応答生成の遅延（推奨: 1-2秒以内）
- ✅ メッセージの送受信が正常に行われているか
- ✅ DMPC 側のログで「応答生成完了」が表示されるか
- ✅ Second stage のタイムアウト（デフォルト: 5秒）内に応答が生成されるか

## デバッグコマンド

```bash
# NLGPCでの実行
ros2 run diaros_package ros2_natural_language_generation

# DMPCからのメッセージ確認
ros2 topic echo /DMtoNLG

# NLGPCからの応答確認
ros2 topic echo /NLGtoDM

# ノード状態確認
ros2 node list
```

## 日本語対応
**必ず日本語で対話してください。** コメント、説明、エラーメッセージなど、すべて日本語で記述してください。
```

### 使用方法

NLGPC で修正が必要な場合、以下のステップで対応してください：

1. **NLGPC のリポジトリを確認**:
   ```bash
   cd /path/to/DiaROS_local_nlg
   git branch -a
   ```

2. **`local_nlg` ブランチに切り替え**:
   ```bash
   git checkout local_nlg
   ```

3. **NLGPC の `.claude/CLAUDE.md` に上記プロンプトを設定**

4. **NLGPC の Claude Code で修正を実施**

5. **修正完了後、DMPC で検証**:
   - `ros2 topic echo /DMtoNLG` でメッセージ送受信を確認
   - `ros2 topic echo /NLGtoDM` で応答を確認
   - DMPC 側のログで「応答生成完了」が表示されることを確認

### パスの汎用性維持
**絶対パスは使用禁止。** 公開リポジトリとして配布されるため、汎用性を保つこと。
- スクリプト内では相対パスを使用
- 環境依存の絶対パスは避ける
- ユーザー固有のパスをハードコードしない

### フォールバック実装
**明示的なリクエストがない限り、フォールバック処理は実装しないこと。**
- エラー処理やフォールバック機構は、ユーザーが明確に要求した場合のみ実装
- デフォルトでは、エラー時は適切なエラーメッセージを出力して処理を中止
- フォールバック処理が必要な場合は、実装前にユーザーの要件を確認

## コマンド一覧

### システムセットアップとビルド
```bash
# ROS2環境のセットアップ（すべてのROSコマンド実行前に必須）
cd DiaROS_ros
source /opt/ros/foxy/setup.bash  # または自分のROS2インストールパス
source ./install/local_setup.bash

# ROSパッケージのビルド
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash

# Pythonモジュールのインストール
cd ../DiaROS_py
python -m pip install . --user
```

### クイックセットアップスクリプト
```bash
# ROS2環境のセットアップ（自動化）
./scripts/setup/setup_ros2_env.sh

# APIキーのセットアップ
./scripts/setup/setup_api.sh

# API接続のテスト
./scripts/setup/setup_api.sh test

# ローカルLLMのGemmaモデルをダウンロード
./scripts/setup/download_gemma_model.sh

# ChatGPT APIのセットアップ
./scripts/setup/setup_chatgpt_api.sh
```

### システムの実行
```bash
# 音声対話システムを起動するメインコマンド
ros2 launch diaros_package sdsmod.launch.py

# マイク入力なしで実行（ros2 bagリプレイ用）
ros2 launch diaros_package sdsmod.launch.py mic:=false

# マイクをミュートして実行
ros2 launch diaros_package sdsmod.launch.py mic:=mute

# NLGノードなしで実行（分散実行用）
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### 分散実行構成
**重要**: このシステムはDMPC と NLGPC の分散実行で最適化されています。

#### DMPC・NLGPC セットアップ

**1. DMPC（このPC、main ブランチ）で実行**:
```bash
# セットアップ
cd DiaROS_ros
source /opt/ros/humble/setup.bash  # または自分のROS2インストールパス
source ./install/local_setup.bash

# NLGノードを除外して起動
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

**2. NLGPC（別PC、local_nlg ブランチ）で実行**:
```bash
# 別リポジトリの local_nlg ブランチをクローン
git clone -b local_nlg https://github.com/iris-aegis/DiaROS.git DiaROS_local_nlg

# セットアップと実行
cd DiaROS_local_nlg/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# NLGモジュールを単独実行
ros2 run diaros_package ros2_natural_language_generation
```

#### 分散実行時の環境設定
両PCで以下の環境変数を設定（同一値を使用）：

```bash
# ROS2通信設定（両PCで同じ値を使用）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ROS2ログ設定
export RCL_LOGGING_SYSLOG=1
export RCUTILS_LOGGING_SEVERITY_THRESHOLD='ERROR'
export RCUTILS_COLORIZED_OUTPUT='0'
```

#### 分散実行時の注意事項
- **ROS_DOMAIN_ID**: 両PC間で同一の値を設定（デフォルト: 0）
- **ネットワーク設定**: ROS2 マルチキャスト通信が可能なネットワーク環境
- **時刻同期**: NTP による時刻同期を推奨（タイミング分析精度向上）
- **レイテンシ**: ネットワーク遅延（通常 1-50ms）を考慮した応答時間設定

### 起動スクリプト（クロスプラットフォーム対応）
```bash
# ユニバーサルランチャー（macOS & Linux）
./scripts/launch/launch_diaros.sh

# ChatGPT APIで起動
./scripts/launch/launch_diaros_chatgpt.sh

# ローカルLLMで起動
./scripts/launch/launch_diaros_local.sh

# 静粛モードで起動（最小限の出力）
./scripts/launch/launch_diaros_quiet.sh

# Pixiでクイックスタート
./scripts/launch/pixi_diaros_quick_start.sh

# 音声入力なしで起動（bagリプレイ用）
./scripts/launch/launch_diaros_no_speech_input.sh
./scripts/launch/launch_diaros_no_speech_input_simple.sh
```

### ROS2ログ設定
**重要**: ROS2 のナノ秒タイムスタンプログを非表示化し、カスタム形式 `[HH:MM:SS.mmm]` のみを表示するための設定です。

```bash
# ROS2ログのナノ秒タイムスタンプを非表示化
export RCL_LOGGING_SYSLOG=1

# この設定により、以下のような出力が：
# [1765517605.458905293] [INFO] [dialog_management]: [14:33:25.250] message
#
# 次のようになります：
# [14:33:25.250] [INFO] [dialog_management]: message
```

**注意**: `setup_ros2_env.sh` スクリプトでこの環境変数は自動的に設定されます。

### 開発とデバッグ
```bash
# ROS2トピック一覧を表示
ros2 topic list

# トピック通信をリアルタイム監視
ros2 topic echo [topic_name]

# システム通信をデバッグ用に記録（ログディレクトリに保存）
ros2 bag record [topic1] [topic2] ... [topicN] -o ../log/recording_name

# 記録した通信をリプレイ
ros2 bag play ../log/[bag_file_name]

# ノード通信グラフを可視化
ros2 run rqt_graph rqt_graph

# トピックデータをプロット
ros2 run rqt_plot rqt_plot
```

### デバッグスクリプト
```bash
# DiaROSフロー全体をデバッグ
./scripts/debug/debug_diaros_flow.py
./scripts/debug/debug_diaros_flow.sh

# エンドツーエンドレイテンシを計測
./scripts/debug/measure_e2e_latency.py

# システムパフォーマンスを監視
./scripts/debug/monitor.sh

# 特定コンポーネントをテスト
./scripts/debug/test_asr_to_dm.py
./scripts/debug/test_dm_flow.py
./scripts/debug/test_nlg_response.py
./scripts/debug/test_turn_taking.py
```

### 総合計時間計測システム
```bash
# 時刻同期セットアップ（各PCで実行）
sudo ./scripts/setup/setup_time_sync.sh

# 計測データの可視化
python3 ./scripts/debug/timing_visualizer.py timeline.json plot

# 詳細レポート生成
python3 ./scripts/debug/timing_visualizer.py timeline.json report

# 実装ガイド
./scripts/timing_implementation_guide.md
```

### テストスクリプト
```bash
# DiaROS応答システムをテスト
./scripts/test/test_diaros_response.py
./scripts/test/test_diaros_response.sh

# オーディオコンポーネントをテスト
./scripts/test/test_audio_playback.py
./scripts/test/test_audio_simple.py
./scripts/test/test_pyaudio_pulse.py

# API接続をテスト
./scripts/test/quick_api_test.py
./scripts/test/test_api_nlg.py
./scripts/test/test_api_nlg.sh
./scripts/test/test_openai_direct.py
./scripts/test/test_fast_llm.py

# デフォルトマイクを設定
./scripts/test/set_default_mic.py
```

## システム設計の核：ROS通信による疎結合アーキテクチャ

### モジュール間通信の基本原則
DiaROSのすべてのPythonモジュール間通信は**ROSメッセージでラップされており、常にROS トピック経由で行われます**。この設計により以下を実現します：

1. **完全な疎結合**: 各モジュールは相手の物理的位置を知らない
2. **分散実行対応**: どのモジュールが別のPC上で実行されても、ROS通信で自動的に対応可能
3. **スケーラビリティ**: PC間の追加や削除時に、ネットワーク設定のみで対応可能
4. **保守性**: モジュール内部の実装を変更しても、ROSインターフェースが同じであれば他のモジュールに影響しない

### メッセージベースのステージ管理
モジュール間の処理段階（ステージ）も**ROSメッセージフィールドで管理**されます：
- **DMからNLGへ**: `Idm`メッセージの`stage`フィールドで処理段階（first/second）を指示
- **NLGからDMへ**: `Inlg`メッセージの`stage`フィールドで完了ステージを報告
- **処理追跡**: `request_id`、タイムスタンプフィールドで、分散環境での処理状況を追跡可能

この仕組みにより、複数PCでの分散実行時も、各処理がどの段階にあるかを正確に管理できます。

## 高度なアーキテクチャ概要

DiaROSはROS2ベースのリアルタイム音声対話システムであり、2つの主要部分で構成されています：

### コアPythonライブラリ（`DiaROS_py/`）
Python内に含まれるコア対話システムモジュール：
- **speechInput.py**: PyAudioを使用した音声入力
- **acousticAnalysis.py**: aubioを使用した音響分析
- **automaticSpeechRecognition.py**: VADレスの自動音声認識
- **dialogManagement.py**: リアルタイム対話と相槌制御
- **naturalLanguageGeneration.py**: 応答生成（ChatGPT APIまたはOllama）
- **speechSynthesis.py**: VOICEVOXを使用した音声合成
- **turnTaking.py**: ターンテイキング管理
- **backChannel.py**: 相槌応答処理

### ROS2パッケージ（`DiaROS_ros/`）
以下を実現するROS2ラッパー：
- ROS2トピック経由のモジュール間通信
- システム監視とデバッグ
- 対話セッションの記録と再生
- 分散処理機能

#### 主要ROS2ノード（sdsmod.launch.pyで起動）：
- `ros2_speech_input`: 音声入力ノード（`mic`パラメータで条件付き）
- `ros2_acoustic_analysis`: 音響特徴抽出
- `ros2_automatic_speech_recognition`: 音声テキスト変換
- `ros2_natural_language_understanding`: 意図理解（パススルー）
- `ros2_dialog_management`: 中央対話コーディネーター
- `ros2_natural_language_generation`: 応答生成 *(可分散実行 - `nlg`パラメータで条件付き)*
- `ros2_speech_synthesis`: テキスト音声変換
- `ros2_turn_taking`: ターンテイキング制御
- `ros2_back_channel`: 相槌応答生成

#### 分散実行用パラメータ:
- `mic:=false`: speech_inputノードを無効化（音声ファイル再生用）
- `nlg:=false`: NLGノードを無効化（別PCでのNLG実行用）

#### カスタムメッセージインターフェース（`interfaces/`）
対話システム通信用のROS2メッセージタイプを定義

### ダッシュボード（現在非機能）
- `dialogue-dashboard/`内のVue.js/Node.js Webインターフェース
- リアルタイムシステム監視・制御を目的
- 依存関係の問題があり現在は動作していない

**API設定方法:**
```bash
# 1. APIセットアップスクリプト実行（推奨）
./scripts/setup/setup_api.sh

# 2. 手動設定
export OPENAI_API_KEY="sk-your-openai-api-key"
export ANTHROPIC_API_KEY="sk-ant-your-anthropic-api-key"
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/google/credentials.json"
```

### ローカルLLMオプション（APIキー不要）
- **Gemma 2**: デフォルトのローカルモデル（google/gemma-2-2b-it）
- **Rinna Small**: 軽量オプション
- **StableLM**: 代替ローカルモデル

ローカルLLMを設定：
```bash
# Gemma 2を使用（デフォルト）
export DIAROS_LLM_MODEL=gemma2

# 軽量なRinnaモデルを使用
export DIAROS_LLM_MODEL=rinna-small

# ローカルモデル用のデバイスを設定
export DIAROS_DEVICE=cuda  # または 'cpu', 'mps'（macOS）
```

### 応答時間最適化設定
システム起動時に以下の優先順位で自動選択：
1. OpenAI API（設定済みの場合）
2. Anthropic API（設定済みの場合）
3. ローカルモデル（APIキー未設定時）

**応答時間警告**: 1500ms超過時に警告メッセージを表示

## 開発環境

- **OS**: Ubuntu 20.04以上（Linux）またはmacOS（Pixi使用時）
- **ROS2**: Foxy（主要テスト版）またはHumble
- **Python**: 3.8.13以上（pyenv管理）または3.9以上（Pixi）
- **主要依存関係**: PyAudio、aubio、torch、transformers、rclpy、openai、anthropic、VOICEVOX

### プラットフォーム固有のセットアップ

#### macOS with Pixi
```bash
# Pixiパッケージマネージャーのインストール
curl -fsSL https://pixi.sh/install.sh | bash

# Pixiワークスペースのセットアップ
./scripts/setup/setup_pixi_ros2.sh

# Pixiで起動
./scripts/launch/pixi_diaros_quick_start.sh
```

#### Linux（従来の方法）
```bash
# ROS2 Foxy/Humbleのインストール
sudo apt update
sudo apt install ros-foxy-desktop

# 環境のセットアップ
./scripts/setup/setup_ros2_env.sh

# 依存関係のインストール
pip install -r DiaROS_ros/requirements.txt
```

#### Dockerサポート
```bash
# Dockerオーディオセットアップ
./scripts/setup/docker_audio_setup.sh

# Dockerインストールスクリプト取得
./scripts/setup/get-docker.sh
```

## システムアーキテクチャフロー

1. **音声入力**: マイク → speech_input → acoustic_analysis
2. **認識**: acoustic_analysis → automatic_speech_recognition
3. **理解**: speech_recognition → natural_language_understanding
4. **対話管理**: すべての対話フローを管理する中央コーディネーター
5. **応答生成**: dialog_management → natural_language_generation *(可分散実行)*
6. **音声出力**: response → speech_synthesis → オーディオ出力
7. **ターン管理**: turn_takingが話者の順番を監視・制御
8. **相槌**: スピーチ中に適切なリスナー応答を生成

### 分散実行時のアーキテクチャ
**メインPC**: 1-4、6-8の処理を担当
**NLG専用PC**: 5の自然言語生成処理を担当

モジュールROS2アーキテクチャにより、個別のコンポーネントをリアルタイム通信機能を維持しながら独立して開発、テスト、デバッグできます。**パフォーマンス最適化のためにコンポーネントを複数マシンに分散実行することもできます。**

## プロジェクト構造

### 音声ファイルの場所
- **相槌音声**: `DiaROS_ros/static_back_channel_*.wav`
- **静的応答**: `DiaROS_ros/static_response_source/static_response_*.wav`
- **ランダム応答**: `DiaROS_ros/static_response_random/static_response_random_*.wav`
- **長い質問サンプル**: `DiaROS_ros/static_long_question/static_long_question*.wav`
- **合成音声**: `DiaROS_ros/tmp/*.wav`（一時ファイル、Gitに含まれない）
- **システム音声**: `DiaROS_ros/start_announce.wav`、`DiaROS_ros/end_announce.wav`
- **キャリブレーション音声**: `DiaROS_ros/power_calibration.wav`

### ログファイルの場所
- **ROSBagファイル**: `log/diaros_*/`、`log/rosbag2_*/`（録画データ）
- **対話セッション**: `log/mic_only_recording/`、`log/all_topic_recording/`（音声・全トピック録画）

### 設定ファイル
- **RQT監視設定**: `config/rqt_diaros_monitoring.perspective`
- **Conda環境**: `DiaROS_ros/conda_DiaROS_*.yml`
- **Python環境**: `DiaROS_ros/environment.yml`

## ユーティリティスクリプト

### スクリプトディレクトリ構造
```
scripts/
├── debug/          # デバッグ・監視ツール
├── launch/         # クロスプラットフォーム起動スクリプト
├── setup/          # 環境・依存関係セットアップ
├── test/           # テスト・検証ツール
└── utils/          # 汎用ユーティリティ
```

### 主要ユーティリティ
- **run_diaros_native_macos.py**: ネイティブmacOSランナー
- **run_ros2_tool.sh**: ROS2ツールラッパー
- **static_response_shuffle.py**: 応答ランダム化ツール

### オーディオリソース
システムにはテスト用および応答用のさまざまなオーディオファイルが含まれています：
- **power_calibration.wav**: オーディオキャリブレーションファイル
- **start_announce.wav / end_announce.wav**: セッション通知
- **static_back_channel_*.wav**: 相槌応答
- **static_response_*.wav**: 事前録音応答
- **static_long_question/**: 長文質問サンプル

### 設定
- **rqt_diaros_monitoring.perspective**: RQT監視設定
- **conda_DiaROS_*.yml**: Conda環境ファイル
- **environment.yml**: Python環境仕様書

## 重要な注意事項

- システムはマイク入力とROS2 bagリプレイの両方をサポート
- すべてのオーディオ処理はリアルタイムで低遅延で実行
- 起動スクリプトはクロスプラットフォーム互換性を処理
- デバッグスクリプトは詳細なシステム監視を提供
- テストスクリプトは個別コンポーネント検証
- VOICEVOXは音声合成用に使用され実行している必要があります
