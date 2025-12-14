# NLG分散実行の指示ドキュメント

## 概要
このドキュメントは、別PCで実行するNLG（自然言語生成）モジュールをClaude Codeで修正する際の指示です。

## メインPC側の変更内容

メインPC（dialogManagement.py）で以下の変更が加えられました：

### 1. TurnTaking判定時刻の記録
```python
self.turn_taking_decision_timestamp_ns = 0  # 新規メンバ変数
```
- TurnTakingモデルが応答判定を出した時刻を記録
- ナノ秒単位で保存（タイムスタンプベースの処理用）

### 2. pubDM_second_stage()の修正
- TurnTaking判定時刻以降で最初のASR結果をフィルタリング
- NLGへ送信するdictに`turn_taking_decision_timestamp_ns`を追加

```json
{
    "words": ["ASR結果"],
    "update": true,
    "stage": "second",
    "turn_taking_decision_timestamp_ns": 1702000000000000000  // ナノ秒
}
```

## 別PC（NLG）側の修正方針

### ファイル位置
- 修正対象：`DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`
- または: `DiaROS_py/diaros/naturalLanguageGeneration.py`

### 修正内容

#### 1. メンバ変数の追加
dialogManagementから受け取った時刻情報を保存

```python
self.turn_taking_decision_timestamp_ns = 0  # TurnTaking判定時刻
```

#### 2. second_stage受信時の処理修正
メインPCからのメッセージを受ける部分で、新しいフィールドを解析

```python
# 修正前：
def update_with_second_stage_words(self, words_list):
    self.second_stage_text = words_list
    # 応答生成処理...

# 修正後：
def update_with_second_stage_words(self, message_data):
    words_list = message_data.get("words", [])
    self.turn_taking_decision_timestamp_ns = message_data.get("turn_taking_decision_timestamp_ns", 0)

    timestamp_ms = self.turn_taking_decision_timestamp_ns / 1_000_000
    now = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f"[NLG] TurnTaking判定時刻受信: {timestamp_ms:.1f}ms @ {now}")

    self.second_stage_text = words_list
    # 応答生成処理...
```

#### 3. ログ出力の追加（推奨）
デバッグ用に、受け取ったASR結果と時刻情報をログ出力

```python
now = datetime.now()
timestamp = now.strftime('%H:%M:%S.%f')[:-3]
print(f"[NLG-second] TurnTaking判定後の最初のASR: '{words_list[0] if words_list else 'なし'}' @ {timestamp}")
print(f"[NLG-second] TurnTaking判定時刻: {self.turn_taking_decision_timestamp_ns}ns")
```

## Claude Code実装指示例

別PCのClaude Codeに以下のように指示してください：

```
## 修正リクエスト

以下のファイルをDiaROS分散実行対応として修正してください：
- ファイル: `ros2_natural_language_generation.py`

### 修正内容

1. **メンバ変数追加**
   - `self.turn_taking_decision_timestamp_ns = 0` を__init__に追加

2. **second_stage受信処理の修正**
   - dialogManagementから送信される新しいフィールド `turn_taking_decision_timestamp_ns` を解析
   - メッセージ構造:
     ```json
     {
         "words": ["ASR結果"],
         "update": true,
         "stage": "second",
         "turn_taking_decision_timestamp_ns": 1702000000000000000
     }
     ```
   - この時刻情報を `self.turn_taking_decision_timestamp_ns` に保存

3. **ログ出力追加**
   - TurnTaking判定時刻とASR結果を以下フォーマットで出力:
     ```
     [NLG-second] TurnTaking判定後の最初のASR: '認識結果' @ HH:MM:SS.mmm
     [NLG-second] TurnTaking判定時刻: XXXXXXXXXXXXXXXXns
     ```

4. **テスト**
   - メインPCで `ros2 launch diaros_package sdsmod.launch.py nlg:=false` を実行
   - 別PCで `ros2 run diaros_package ros2_natural_language_generation` を実行
   - メインPC側で TurnTaking判定時刻が出力されることを確認
   - 別PCのNLG側で受け取った時刻情報がログに出力されることを確認

## タイムスタンプ単位
- 全てナノ秒（ns）単位
- ミリ秒に変換：ns / 1_000_000
- 秒に変換：ns / 1_000_000_000
```

## 統合テスト方法

### メインPC側
```bash
cd DiaROS_ros
source /opt/ros/foxy/setup.bash
source ./install/local_setup.bash

# NLGノードを除外して起動
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### 別PC側
```bash
cd DiaROS_ros
source /opt/ros/foxy/setup.bash
source ./install/local_setup.bash

# NLGノードのみ実行
ros2 run diaros_package ros2_natural_language_generation
```

### 確認ポイント
1. メインPC側：
   ```
   [TT] TurnTaking判定時刻を記録: HH:MM:SS.mmm (ns: 1702000000000000000)
   [DM-second] TurnTaking判定後の最初のASR結果を使用: '認識結果' @ HH:MM:SS.mmm
   ```

2. 別PC側：
   ```
   [NLG] TurnTaking判定時刻受信: 1702000.0ms @ HH:MM:SS.mmm
   [NLG-second] TurnTaking判定後の最初のASR: '認識結果' @ HH:MM:SS.mmm
   ```

## 技術詳細

### なぜこの方法を採用？
1. **時刻同期不要**：両PC間で個別に時刻を記録するため、時刻同期が不要
2. **ナノ秒精度**：ミリ秒の誤差を最小化
3. **シンプルな履歴管理**：ASR履歴から特定時刻以降の最初の結果を抽出

### トラブルシューティング

**問題: NLG側で時刻情報が受け取られない**
- ROS2トピックの接続確認：`ros2 topic echo /dm_second_stage`
- メッセージ形式が正しいか確認

**問題: TurnTaking判定時刻と実際のASR時刻がずれている**
- 両PC間のネットワーク遅延を考慮
- ログで実際の時刻差を測定

**問題: 異なるASR結果が使用されている**
- ASR履歴のタイムスタンプを確認
- TurnTaking判定時刻がASR結果よりも前の時刻になっているか確認
