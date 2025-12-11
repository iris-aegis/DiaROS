# ステージベース通信検証ガイド

## 検証概要

このドキュメントでは、DiaROSのステージベース通信実装が正常に動作しているかを検証する手順を説明します。

実際のマイクデータを使用した検証により、以下を確認できます：

- ✓ DMノードがNLGノードにリクエストを送信している
- ✓ NLGノードがDMからのリクエストを受け取っている
- ✓ request_id が正しく管理されている
- ✓ ステージ（first/second）の遷移が正確である
- ✓ NLGノードがDMに応答を返している
- ✓ タイムスタンプが正しく計測されている

## 環境準備

### 必要なデータファイル
```
/workspace/DiaROS/log/diaros_20250713_151409/diaros_20250713_151409_0.db3
```

このROSバッグファイルには、実際のマイクデータが格納されています。

### システム要件
- ROS2 Humble
- VOICEVOX（音声合成エンジン）
- すべてのDiaROS Pythonモジュール

## 検証手順

### ステップ1: ターミナル1 でDiaROSシステムを起動

```bash
bash /workspace/DiaROS/scripts/launch/launch_diaros_no_speech_input_simple.sh
```

このコマンド実行後、以下のが起動されます：

```
✅ VOICEVOX環境確認
✅ ROS2環境セットアップ
✅ Pythonモジュール再インストール
✅ interfacesパッケージビルド
✅ diaros_packageビルド
✅ すべてのノード起動

📝 起動ノード一覧:
  - acoustic_analysis
  - automatic_speech_recognition
  - natural_language_understanding
  - dialog_management
  - natural_language_generation ⭐
  - speech_synthesis
  - turn_taking
  - back_channel
```

**起動完了の目安**:
```
🚀 DiaROSを起動します (speech_inputノード除外)...
🎵 ros2 bag playで音声データを再生してください
```

このメッセージが表示されたら、ターミナル1での起動は完了です。

### ステップ2: ターミナル2 でマイクデータを再生

```bash
ros2 bag play /workspace/DiaROS/log/diaros_20250713_151409/diaros_20250713_151409_0.db3 --topics /mic_audio_float32
```

このコマンドは、ROSバッグファイルから `/mic_audio_float32` トピックのデータを再生します。

**実行例**:
```
[INFO] [player]: Playback of [/workspace/DiaROS/log/diaros_20250713_151409] paused [0.000000s/X.XXXs]
[INFO] [player]: Playback of [/workspace/DiaROS/log/diaros_20250713_151409] started
```

## 検証ポイント

### ターミナル1 のログを監視

バッグプレイが開始されると、以下のようなログが流れます：

#### 期待されるログ出力

**ASR結果の受信**:
```
[NLUtoDM] ASR受信: 'ユーザーの発言' (is_final: False)
[NLUtoDM] ASR受信: 'ユーザーの発言1' (is_final: True)
```

**DMからのNLGリクエスト**:
```
[DM] 相槌生成リクエスト送信 (request_id=1, 入力数=2)
```

**NLGでのステージ処理**:
```
[NLG] 相槌生成ステージ開始 (request_id=1, 入力数=2)
[NLG] 相槌生成ステージ完了 (request_id=1, 処理時間=234.5ms, 応答='そうですね')
```

**DMでのNLG応答受信**:
```
[DM] NLGから相槌生成応答受信 (request_id=1): 'そうですね'
```

**Second stageへの遷移**:
```
[DM] 応答生成リクエスト送信 (request_id=2, 入力数=3)
[NLG] 応答生成ステージ開始 (request_id=2, 入力数=3)
[NLG] 応答生成ステージ完了 (request_id=2, 処理時間=1250.3ms, 応答='今日は晴れです')
[DM] NLGから応答生成応答受信 (request_id=2): '今日は晴れです'
```

### ログの分析チェック項目

| チェック項目 | 期待値 | 確認方法 |
|-----------|--------|---------|
| **request_id の増加** | 1 → 2 → 3... | ログで request_id が単調増加 |
| **ステージ遷移** | first → second | ログで stage が正しく遷移 |
| **タイムスタンプ** | ナノ秒精度 | 処理時間が計測されている |
| **処理時間** | 100-2000ms程度 | 応答の処理時間が合理的 |
| **応答テキスト** | 非空 | reply フィールドに応答が含まれている |

### ROS トピック監視（オプション）

ターミナル3 でトピックの詳細を確認：

```bash
# NLG応答メッセージの確認
ros2 topic echo NLGtoSS

# DM→NLGリクエストメッセージの確認（フォーマット: Idm）
ros2 topic echo DMtoNLG
```

期待される `NLGtoSS` メッセージ:
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
---
reply: '今日は晴れです'
stage: second
request_id: 2
source_words:
- ユーザーの発言
- さらに詳細な入力
worker_name: ''
start_timestamp_ns: 1738867201000000000
completion_timestamp_ns: 1738867202250000000
inference_duration_ms: 1250.3
session_id: ''
```

## トラブルシューティング

### 症状1: NLGノードが起動しない

**原因**: launch ファイルでNLGがコメントアウトされている

**解決策**: sdsmod.launch.py を確認
```bash
cat /workspace/DiaROS/DiaROS_ros/src/diaros_package/launch/sdsmod.launch.py | grep -A 3 natural_language_generation
```

出力例（正常）:
```
Node(
    package='diaros_package',
    executable='ros2_natural_language_generation',
    output='screen'
),
```

### 症状2: request_id が 0 のままで増加しない

**原因**: ステージが遷移していない

**確認**:
1. ログで「request_id」を検索
2. 値が増加しているか確認

**解決策**:
- ASR結果が正常に流入しているか確認
- DM ノードが稼働しているか確認（プロセスリスト確認）

### 症状3: NLG応答がない

**原因1**: NLGノードが起動していない
```bash
ros2 node list | grep natural_language_generation
```

**原因2**: ROSバッグファイルが再生されていない
```bash
ros2 topic list | grep mic_audio
```

**原因3**: NLGの推論が失敗している
ログで「[NLG]」で検索、エラーがないか確認

## 検証の完了

以下がすべて確認できれば、ステージベース通信の実装は成功です：

- [x] request_id が 1, 2, 3... と増加している
- [x] stage が "first" → "second" と遷移している
- [x] 各ステージの処理時間が計測されている
- [x] NLG応答が DM で受信されている
- [x] 複数のリクエストが干渉なく処理されている

## パフォーマンス指標（参考値）

| 処理段階 | 期待処理時間 |
|---------|-----------|
| First stage（相槌） | 100-500ms |
| Second stage（応答） | 500-2000ms |
| 音声合成（TTS） | 500-2000ms |
| **全体（end-to-end）** | **1000-5000ms** |

## ドキュメント参照

詳細な実装情報は以下を参照してください：

- [CLAUDE.md](CLAUDE.md): システム設計全体
- [STAGE_BASED_COMMUNICATION.md](STAGE_BASED_COMMUNICATION.md): ステージベース通信詳細
- [STAGE_COMMUNICATION_SETUP.md](STAGE_COMMUNICATION_SETUP.md): セットアップガイド
- [MESSAGE_INTERFACE_UPDATE.md](MESSAGE_INTERFACE_UPDATE.md): メッセージ仕様

---

**検証日**: 2025-12-08
**ステータス**: 検証準備完了

