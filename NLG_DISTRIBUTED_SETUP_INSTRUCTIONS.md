# DiaROS分散NLG実行セットアップ手順

## 概要
このドキュメントは、DiaROSシステムのNLG（自然言語生成）コンポーネントを別PCで実行するための完全なセットアップ手順を記載しています。

## システム構成

```
[メインPC: DiaROSシステム]
音声入力 → 音響解析 → 音声認識 → 対話管理 → TurnTaking → 音声合成
                                    ↓ DMtoNLG (ROS2トピック)
[NLG PC: このPC]
                          自然言語生成 (ChatGPT/Ollama)
                                    ↑ NLGtoSS (ROS2トピック)
```

## 前提条件

### 両PC共通
- Ubuntu 20.04以上
- ROS2 Foxy/Humble
- 同一ネットワーク内に接続
- ROS_DOMAIN_IDが統一されていること

### NLG PC
- Ollama インストール済み（ローカルLLM使用時）
- OpenAI API キー設定済み（ChatGPT使用時）
- GPU推奨（ローカルLLM使用時）

## セットアップ手順

### 1. リポジトリのクローン（両PC）

```bash
git clone https://github.com/iris-aegis/DiaROS.git
cd DiaROS
git checkout local_nlg
```

### 2. NLG PCでの初期セットアップ

```bash
# ビルドスクリプトを実行
bash /workspace/DiaROS/scripts/build/rebuild_nlg_distributed.sh
```

このスクリプトは以下を自動実行します：
- Pythonパッケージのインストール
- interfacesパッケージのビルド
- diaros_packageのビルド

### 3. メインPCでの初期セットアップ

メインPCでも同様にinterfacesパッケージをビルド：

```bash
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/foxy/setup.bash
rm -rf build/interfaces install/interfaces
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash
```

### 4. メッセージ定義の確認

両PCで以下のコマンドを実行して、メッセージ定義が一致していることを確認：

```bash
ros2 interface show interfaces/msg/Idm
```

期待される出力：
```
string[] words
string session_id
string stage
int64 turn_taking_decision_timestamp_ns
```

```bash
ros2 interface show interfaces/msg/Inlg
```

期待される出力：
```
string reply
string[] source_words
string stage
int32 request_id
string worker_name
int64 start_timestamp_ns
int64 completion_timestamp_ns
float64 inference_duration_ms
string session_id
```

## 実行手順

### 1. メインPCでDiaROSシステムを起動（NLG除外）

```bash
cd /workspace/DiaROS
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

または：

```bash
bash /workspace/DiaROS/scripts/launch/start_diaros.sh
```

### 2. NLG PCでNLGノードを起動

```bash
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/foxy/setup.bash
source ./install/local_setup.bash
ros2 run diaros_package ros2_natural_language_generation
```

または：

```bash
bash /workspace/DiaROS/scripts/launch/start_nlg_node.sh
```

## 動作確認

### 1. トピック確認

両PCで以下のコマンドでトピックが見えることを確認：

```bash
ros2 topic list | grep -E "DMtoNLG|NLGtoSS"
```

### 2. テストメッセージ送信（メインPC）

```bash
ros2 topic pub /DMtoNLG interfaces/msg/Idm \
  "{words: ['こんにちは', '元気ですか'], \
    session_id: 'test123', \
    stage: 'first', \
    turn_taking_decision_timestamp_ns: 1733660000000000000}" --once
```

### 3. NLG PC側の期待ログ出力

```
[14:XX:XX.XXX][NLG DEBUG] dm_updateコールバック呼び出し
[14:XX:XX.XXX][NLG] 音声認識結果受信時刻: 14:XX:XX.XXX
[14:XX:XX.XXX][NLG] 受信した音声認識結果リスト: ['こんにちは', '元気ですか']
[14:XX:XX.XXX][NLG] リクエストステージ: first
[14:XX:XX.XXX][NLG FIRST_STAGE] 🤖 相槌生成開始
[14:XX:XX.XXX][NLG FIRST_STAGE] ✅ 相槌生成完了 (XXXms): 'うんうん'
[14:XX:XX.XXX][NLG] First stage相槌送信（テキストのみ）: 'うんうん'
```

### 4. メインPC側の期待ログ出力

```
[DM→NLG] stage=first, words=['こんにちは', '元気ですか']
[NLG←SS] stage=first, reply='うんうん'
[SS] First stage相槌を音声合成開始: 'うんうん'
```

## 二段階応答フロー

### First Stage（相槌生成）

1. **メインPC**: ユーザーの発話を検出し、DMがfirst stageリクエストを送信
   ```
   DMtoNLG: {stage: "first", words: ["音声認識結果"]}
   ```

2. **NLG PC**: 相槌を即座に生成（200ms目標）
   ```python
   self.naturalLanguageGeneration.generate_first_stage(words)
   ```

3. **NLG PC**: 相槌をメインPCに送信
   ```
   NLGtoSS: {stage: "first", reply: "うんうん"}
   ```

4. **メインPC**: 相槌を音声合成して即座に再生

### Second Stage（本応答生成）

1. **メインPC**: TurnTakingモデルが応答判定を出す
   ```
   DMtoNLG: {stage: "second", words: ["完全なASR履歴"], turn_taking_decision_timestamp_ns: XXXXXXXXX}
   ```

2. **NLG PC**: TurnTaking判定時刻を受信・保存
   ```python
   self.turn_taking_decision_timestamp_ns = turn_taking_ts
   ```

3. **NLG PC**: 本応答を生成（first stage相槌を参照）
   ```python
   self.naturalLanguageGeneration.generate_second_stage(words)
   ```

4. **NLG PC**: 本応答をメインPCに送信
   ```
   NLGtoSS: {stage: "second", reply: "それは良かったですね"}
   ```

5. **メインPC**: 本応答を音声合成して再生

## トラブルシューティング

### トピックが見えない

**原因**: ROS_DOMAIN_IDが異なる

**解決策**:
```bash
# 両PCで同じDOMAIN IDを設定
export ROS_DOMAIN_ID=0
```

### メッセージ型エラー

**原因**: interfacesパッケージのビルドバージョンが異なる

**解決策**:
```bash
# 両PCでinterfacesを再ビルド
cd /workspace/DiaROS/DiaROS_ros
rm -rf build/interfaces install/interfaces
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
```

### NLG PCでメッセージが受信できない

**原因1**: ネットワークファイアウォール

**解決策**:
```bash
# DDS通信ポートを開放
sudo ufw allow proto udp from any to any port 7400:7500
```

**原因2**: DDS設定

**解決策**: `~/.bashrc` に以下を追加：
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Ollama接続エラー

**原因**: Ollamaサーバーが起動していない

**解決策**:
```bash
# Ollamaを起動
ollama serve
```

別ターミナルでモデルをプル：
```bash
ollama pull gemma3:4b
```

## 設定ファイル

### モデル設定

[DiaROS_py/diaros/naturalLanguageGeneration.py](DiaROS_py/diaros/naturalLanguageGeneration.py) の冒頭：

```python
# ChatGPT使用時
MODEL_NAME = "gpt-4.1-nano"

# ローカルLLM使用時
MODEL_NAME = "gemma3:4b"
```

### プロンプト設定

```python
# First stage用（相槌）
PROMPT_FILE_NAME = "dialog_first_stage.txt"

# Second stage用（本応答）
# プロンプトファイルはgenerate_second_stage()内で自動読み込み
```

## パフォーマンス目標

- **First stage応答時間**: 200ms以内
- **Second stage応答時間**: 1500ms以内
- **ネットワーク遅延**: 10ms以内（LAN内）

## 実装詳細

### タイムスタンプフィールド

```python
turn_taking_decision_timestamp_ns: int64  # ナノ秒単位
```

変換例：
```python
# ナノ秒 → ミリ秒
ms = turn_taking_decision_timestamp_ns / 1_000_000

# ナノ秒 → 秒
sec = turn_taking_decision_timestamp_ns / 1_000_000_000

# ナノ秒 → datetime
dt = datetime.fromtimestamp(turn_taking_decision_timestamp_ns / 1_000_000_000)
```

### Stage情報

```python
stage: str  # "first" または "second"
```

- `"first"`: 相槌生成リクエスト（ユーザー発話中）
- `"second"`: 本応答生成リクエスト（ユーザー発話終了後）

## 参考資料

- [CLAUDE.md](CLAUDE.md) - プロジェクト全体のガイダンス
- [DiaROS_py/diaros/naturalLanguageGeneration.py](DiaROS_py/diaros/naturalLanguageGeneration.py) - NLGコア実装
- [DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py](DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py) - ROS2統合レイヤー
- [DiaROS_ros/src/interfaces/msg/Idm.msg](DiaROS_ros/src/interfaces/msg/Idm.msg) - Dialog Management → NLG メッセージ定義
- [DiaROS_ros/src/interfaces/msg/Inlg.msg](DiaROS_ros/src/interfaces/msg/Inlg.msg) - NLG → Speech Synthesis メッセージ定義
