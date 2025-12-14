# ステージベース通信システムセットアップガイド

## セットアップ完了状態

以下のステージベース通信システムのセットアップが完了しました。

## 📋 実装内容

### 1. メッセージインターフェース更新
- **Idm.msg**: `request_id` フィールド追加
- **Inlg.msg**: 既存（request_id を含む）
- **Iss.msg**: 既存（request_id を含む）

### 2. ノード実装更新
- **ros2_dialog_management.py**: request_id 生成・管理機構を実装
- **ros2_natural_language_generation.py**: ステージ管理機構を実装

### 3. テスト・ドキュメント
- **test_stage_based_communication.py**: ステージベース通信テストスクリプト
- **STAGE_BASED_COMMUNICATION.md**: 詳細ドキュメント
- **IMPLEMENTATION_SUMMARY.md**: 実装サマリー
- **MESSAGE_INTERFACE_UPDATE.md**: メッセージ定義変更ドキュメント

## 🚀 システム起動方法

### ステップ1: ROS2環境のセットアップ

```bash
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
```

### ステップ2: DiaROSシステムの起動

```bash
# 通常起動（すべてのモジュールが同一PC上で実行）
ros2 launch diaros_package sdsmod.launch.py

# 分散実行：NLGを別PCで実行する場合
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### ステップ3: ステージベース通信のテスト

**別のターミナルで実行**:

```bash
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# テストスクリプト実行
python3 test_stage_based_communication.py
```

## 📊 期待されるログ出力

### DM ノードのログ例

```
[DM] 相槌生成リクエスト送信 (request_id=1, 入力数=3)
[DM] NLGから相槌生成応答受信 (request_id=1): 'そうですね'
[DM] 応答生成リクエスト送信 (request_id=2, 入力数=5)
[DM] NLGから応答生成応答受信 (request_id=2): '今日は晴れです'
```

### NLG ノードのログ例

```
[NLG] 相槌生成ステージ開始 (request_id=1, 入力数=3)
[NLG] 相槌生成ステージ完了 (request_id=1, 処理時間=234.5ms, 応答='そうですね')
[NLG] 応答生成ステージ開始 (request_id=2, 入力数=5)
[NLG] 応答生成ステージ完了 (request_id=2, 処理時間=1250.3ms, 応答='今日は晴れです')
```

## ✓ 検証チェックリスト

実装後、以下の項目が正常に動作しているか確認してください：

### 単一PC での実行

- [ ] DM が NLG にリクエストを送信できる
- [ ] NLG が DM からリクエストを受け取れる
- [ ] NLG がステージ情報を正しく処理できる
- [ ] DM が NLG からの応答を受け取れる
- [ ] request_id が DM と NLG で対応している
- [ ] ログに「request_id=N」が正しく表示されている

### request_id の確認

```bash
# ターミナルでログを監視
ros2 topic echo NLGtoSS
```

出力例:
```
---
reply: 'そうですね'
stage: first
request_id: 1
source_words:
- ユーザーの発言
worker_name: ''
start_timestamp_ns: 1738867200000000000
completion_timestamp_ns: 1738867200234000000
inference_duration_ms: 234.5
session_id: ''
```

## 🔄 分散実行での確認

NLGを別PCで実行する場合：

### メインPC（DM、ASR、SSなどを実行）

```bash
export ROS_DOMAIN_ID=0
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### NLG専用PC

```bash
export ROS_DOMAIN_ID=0
ros2 run diaros_package ros2_natural_language_generation
```

**重要**: 両PCで同じ `ROS_DOMAIN_ID` を設定してください。

### 確認方法

メインPCのターミナルで：

```bash
# NLG トピックがメインPCで受け取れるか確認
ros2 topic echo NLGtoSS
```

NLG がテキストを生成すれば、分散実行が正常に動作しています。

## 🔧 トラブルシューティング

### エラー1: AttributeError: 'Idm' object has no attribute 'request_id'

**原因**: メッセージ定義が更新されていない

**解決策**:
```bash
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash
```

### エラー2: NLGからのメッセージが受信されない

**確認事項**:
1. ROS トピックが正常に動作しているか確認:
   ```bash
   ros2 topic list | grep NLG
   ```

2. NLG ノードが起動しているか確認:
   ```bash
   ros2 node list | grep natural
   ```

3. request_id が正しく設定されているか確認:
   ```bash
   ros2 topic echo NLGtoSS
   ```

### エラー3: request_id が 0 のままで増加しない

**原因**: ステージが変わっていない

**確認事項**:
1. DM ノードが異なるステージのリクエストを送信しているか確認
2. ログで「request_id=N」が正しく増加しているか確認

## 📚 関連ドキュメント

- [CLAUDE.md](CLAUDE.md): システム全体の設計ガイド
- [STAGE_BASED_COMMUNICATION.md](STAGE_BASED_COMMUNICATION.md): ステージベース通信の詳細説明
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md): 実装内容の総括
- [MESSAGE_INTERFACE_UPDATE.md](MESSAGE_INTERFACE_UPDATE.md): メッセージインターフェース変更詳細

## 💡 次のステップ

### 追加実装の検討

現在のステージベース通信設計は、以下のモジュールにも適用可能です：

1. **ASR（自動音声認識）**
   - request_id を含める
   - 音声認識の段階を管理

2. **SS（音声合成）**
   - request_id をさらに下流に伝播
   - TTS 処理時間を計測

3. **TT（ターンテイキング）**
   - request_id で判定タイミングを追跡

4. **BC（相槌生成）**
   - request_id で相槌応答を管理

### パフォーマンス最適化

1. **キャッシング**: 同じ request_id での重複処理を避ける
2. **パイプライング**: 複数リクエストの並行処理
3. **タイムアウト**: 処理時間が長すぎる場合の検出

## 🎯 成功指標

システムが正常に動作している場合：

✓ DM と NLG が request_id で正確に通信している
✓ ステージ遷移時に request_id が自動的に更新される
✓ 複数の同時リクエストで干渉が発生しない
✓ 分散実行（別PC での NLG 実行）が可能
✓ 処理時間が計測可能

---

**作成日**: 2025-12-08
**バージョン**: 1.0
**ステータス**: 実装完了、テスト準備完了

