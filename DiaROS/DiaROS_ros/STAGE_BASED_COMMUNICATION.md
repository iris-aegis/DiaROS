# ステージベース通信実装ドキュメント

## 概要

DiaROSの核となるシステム設計として、**すべてのPythonモジュール間通信はROSメッセージでラップされており、常にROS トピック経由で行われます**。これにより、どのモジュールが別のPC上で実行されても、ROS通信で自動的に対応可能な完全に疎結合されたアーキテクチャを実現しています。

本ドキュメントでは、特に**ステージベース通信**（Dialog Management ↔ Natural Language Generation）の実装について説明します。

## システム設計の原則

### 1. 完全な疎結合
- 各モジュールは相手の物理的位置（同じPC上か別PCか）を知らない
- ROSトピック経由の通信のみに依存

### 2. 分散実行対応
- NLGを別PCで実行する場合も、ROS_DOMAIN_IDの設定のみで対応可能
- 追加のコード変更不要

### 3. スケーラビリティ
- 新しいモジュールを追加する際は、ROSメッセージインターフェースを定義するだけ
- 既存モジュールへの影響最小化

## ステージベース通信の仕組み

### メッセージフロー

```
[DM（Dialog Management）]
        ↓ (Idm メッセージ)
    stage=first/second
    request_id=1,2,3...
        ↓
[NLG（Natural Language Generation）]
        ↓ (Inlg メッセージ)
    stage=first/second (リクエストを反映)
    request_id=同じID
        ↓
[SS（Speech Synthesis）]
```

### 処理段階（ステージ）

DiaROSの応答生成は2段階で進行します：

1. **First Stage（相槌生成）**
   - ユーザーの発言中に、システムが「そうですね」「なるほど」などの相槌を生成
   - 潜在期間が短い（素早く生成される）
   - stage: `"first"`

2. **Second Stage（応答生成）**
   - ユーザーの発言完了後、実際の応答を生成
   - より詳細な応答を作成可能
   - stage: `"second"`

## 実装詳細

### 1. Idm メッセージ（DM → NLG）

```python
# fields in Idm message
words: list[str]           # ASR結果の単語列
stage: str                 # "first" または "second"
request_id: int            # リクエストの一意識別子
session_id: str            # セッション識別子
turn_taking_decision_timestamp_ns: int  # ターンテイキング判定時刻
```

**送信タイミング**:
- First stage: ユーザーの発言中、ターンテイキングが「発話継続」と判定した時
- Second stage: ユーザーの発言が終了し、「ターン交代」と判定した時

### 2. Inlg メッセージ（NLG → DM/SS）

```python
# fields in Inlg message
reply: str                 # 生成された応答テキスト
stage: str                 # 受け取ったrequest_idのstageを返す
request_id: int            # 受け取ったrequest_idを返す
source_words: list[str]    # 元となったASR結果
worker_name: str           # LLMワーカー名
start_timestamp_ns: int    # NLG処理開始時刻
completion_timestamp_ns: int  # NLG処理完了時刻
inference_duration_ms: float  # 推論処理時間
```

### 3. リクエストID管理

**DMノード（ros2_dialog_management.py）**:
```python
class RosDialogManagement(Node):
    def __init__(self, ...):
        self.request_id_counter = 0      # グローバルカウンター
        self.current_request_stage = None # 現在のステージ

    def callback(self):
        # ステージが変わったらrequest_idを増加
        if stage != self.current_request_stage:
            self.request_id_counter += 1
            self.current_request_stage = stage

        # リクエストメッセージに request_id を設定
        dm.request_id = self.request_id_counter
        self.pub_dm.publish(dm)
```

**NLGノード（ros2_natural_language_generation.py）**:
```python
class RosNaturalLanguageGeneration(Node):
    def __init__(self, ...):
        self.current_stage = None       # 現在処理中のステージ
        self.current_request_id = None  # 現在処理中のリクエストID
        self.stage_start_timestamp_ns = 0  # ステージ開始時刻

    def dm_update(self, msg):
        # 新しいリクエストの開始を記録
        if msg.request_id != self.current_request_id:
            self.current_stage = msg.stage
            self.current_request_id = msg.request_id
            self.stage_start_timestamp_ns = time.time_ns()

    def ping(self):
        # 応答生成完了時、同じ request_id と stage を返す
        nlg_msg.stage = self.current_stage
        nlg_msg.request_id = self.current_request_id
        self.pub_nlg.publish(nlg_msg)
```

## ステージ管理フロー（詳細）

### シナリオ例：ユーザーが "今日の天気は?" と発言

```
時刻 0ms: ユーザーが発言開始
    ↓
時刻 50ms: 発言途中でASR結果「今日の」が認識
    - DM が TT（ターンテイキング）に問い合わせ
    - TT: 「まだ発話継続中」と判定
    ↓
時刻 50ms: DM → NLG へ first stage リクエスト送信
    - request_id = 1
    - stage = "first"
    - words = ["今日の"]
    ↓
時刻 100ms: NLG が相槌「そうですね」を生成
    - NLG → DM へ応答送信
    - request_id = 1  ★ DM と同じID
    - stage = "first" ★ DM のステージを反映
    - reply = "そうですね"
    ↓
時刻 150ms: ユーザー発言完了「...天気は?」
    - ASR 最終結果「今日の天気は？」
    ↓
時刻 150ms: DM が TT に再び問い合わせ
    - TT: 「ターン交代」と判定
    ↓
時刻 150ms: DM → NLG へ second stage リクエスト送信
    - request_id = 2  ★ 新しいステージなので ID も増加
    - stage = "second" ★ ステージ変更
    - words = ["今日の", "天気は？"]
    ↓
時刻 500ms: NLG が応答「今日は晴れです」を生成
    - NLG → DM へ応答送信
    - request_id = 2  ★ DM と同じID
    - stage = "second" ★ DM のステージを反映
    - reply = "今日は晴れです"
```

## タイミング計測

各ステージでの処理時間を計測し、システムパフォーマンスの分析に使用できます。

### NLGノードでの計測

```python
# ステージ開始時刻の記録
self.stage_start_timestamp_ns = time.time_ns()

# ステージ完了時の処理時間計算
stage_duration_ms = (time.time_ns() - self.stage_start_timestamp_ns) / 1_000_000

# ログ出力（例）
# [NLG] 相槌生成ステージ完了 (request_id=1, 処理時間=234.5ms)
```

### DMノードでのタイムスタンプ伝播

NLGからのタイミング情報をDMが受信し、さらに下流モジュール（SS）に伝播：

```python
nlg_data = {
    'nlg_start_timestamp_ns': msg.start_timestamp_ns,
    'nlg_completion_timestamp_ns': msg.completion_timestamp_ns,
    'nlg_inference_duration_ms': msg.inference_duration_ms
}
self.dialogManagement.updateNLG(nlg_data)
```

## テスト方法

### 環境準備

```bash
# ROS2環境を起動
cd DiaROS_ros
source /opt/ros/foxy/setup.bash
source ./install/local_setup.bash

# DiaROSシステムを起動
ros2 launch diaros_package sdsmod.launch.py
```

### テストスクリプト実行

```bash
# 別のターミナルでテストを実行
cd DiaROS_ros
python3 test_stage_based_communication.py
```

### テスト内容

テストスクリプトは以下を検証します：

1. **First stage リクエスト**
   - DM → NLG へステージ情報付きでリクエスト送信
   - NLGがステージ情報を反映した応答を返す

2. **Second stage リクエスト**
   - 異なるステージのリクエスト送信
   - request_id が正しく更新される

3. **複数リクエストの処理**
   - request_id の連続性確認
   - ステージ情報の正確性確認

4. **レスポンス確認**
   - NLGからの応答がrequest_id と stage を正しく含んでいるか
   - ステージ間での情報漏洩がないか

## 分散実行での利用

NLGを別PCで実行する場合も、このステージベース通信は変わりません。

### メインPC 上での実行

```bash
# NLGノードなしで起動
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### NLG専用PC 上での実行

```bash
# NLGノードのみを実行
ros2 run diaros_package ros2_natural_language_generation
```

**重要**: 両PCで同じ `ROS_DOMAIN_ID` を設定してください。

```bash
export ROS_DOMAIN_ID=0  # 両PCで同一
```

## ログ出力例

### DMノード

```
[DM] 相槌生成リクエスト送信 (request_id=1, 入力数=3)
[DM] NLGから相槌生成応答受信 (request_id=1): 'そうですね'
[DM] 応答生成リクエスト送信 (request_id=2, 入力数=5)
[DM] NLGから応答生成応答受信 (request_id=2): '今日は晴れです。...'
```

### NLGノード

```
[NLG] 相槌生成ステージ開始 (request_id=1, 入力数=3)
[NLG] 相槌生成ステージ完了 (request_id=1, 処理時間=234.5ms, 応答='そうですね')
[NLG] 応答生成ステージ開始 (request_id=2, 入力数=5)
[NLG] 応答生成ステージ完了 (request_id=2, 処理時間=1250.3ms, 応答='今日は晴れです...')
```

## トラブルシューティング

### NLGからの応答が受信できない

1. **ROS通信の確認**
   ```bash
   ros2 topic echo NLGtoSS
   ```
   メッセージが流れているか確認

2. **ログの確認**
   ```bash
   # NLGノードのログを確認
   ros2 node info /natural_language_generation
   ```

3. **request_id の確認**
   - DMが送信した request_id とNLGが返した request_id が一致しているか確認

### ステージが混在している

1. **DMノードのステージ管理確認**
   ```python
   # ros2_dialog_management.py の current_request_stage が正しく更新されているか
   ```

2. **タイムスタンプの確認**
   - ステージ開始・完了のタイミングは正確か

## まとめ

このステージベース通信実装により：

- ✓ DM と NLG間の完全な疎結合を実現
- ✓ リクエスト ID による処理の追跡が可能
- ✓ 複数PC での分散実行に対応
- ✓ 処理時間の計測・分析が可能
- ✓ スケーラビリティの高いシステムアーキテクチャを構築

このデザインは、将来的に他のモジュール（ASR、SS、TT等）を別のPCに移行する際も応用可能です。
