# test_dm_to_nlg_send.py 使用方法

## 概要
音声認識結果のJSONファイルを指定して、DM to NLGトピックに送信するテストツールです。

## 基本的な使用方法

### 1. ヘルプ表示
```bash
python3 test_dm_to_nlg_send.py
```

### 2. JSONファイルを指定して実行
```bash
# ダブルクォート形式のJSONファイル
python3 test_dm_to_nlg_send.py asr_result_lists/asr_result_list_1s.json

# シングルクォート形式のJSONファイル（自動変換対応）
python3 test_dm_to_nlg_send.py asr_result_lists/sample_single_quotes.json
```

### 3. ROS2環境での実行
```bash
# ROS2環境をセットアップしてから実行
source install/local_setup.bash
python3 test_dm_to_nlg_send.py asr_result_lists/asr_result_list_1s.json
```

## JSONファイル形式

### 対応形式1: 配列形式（推奨）
```json
[
    "",
    "こんにちは",
    "今日は良い天気ですね",
    "音声認識のテストです"
]
```

### 対応形式2: シングルクォート形式（自動変換）
```json
[
    '',
    'こんにちは',
    '今日は良い天気ですね',
    '音声認識のテストです'
]
```

### 対応形式3: 辞書形式
```json
{
    "words": [
        "",
        "こんにちは",
        "今日は良い天気ですね"
    ]
}
```

自動的に以下のキーを探索します：
- `words`
- `results`
- `asr_results`
- `texts`
- `data`

## テスト手順

### 1. JSON読み込みテスト（ROS2環境不要）
```bash
python3 test_json_loading.py asr_result_lists/asr_result_list_1s.json
```

### 2. ROS2トピック送信テスト
```bash
# ターミナル1: DiaROSシステム起動
ros2 launch diaros_package sdsmod.launch.py

# ターミナル2: トピック監視
ros2 topic echo /DMtoNLG

# ターミナル3: テスト送信実行
source install/local_setup.bash
python3 test_dm_to_nlg_send.py asr_result_lists/asr_result_list_1s.json
```

## 出力例

### 成功時の出力
```
🚀 DM to NLG送信テスト開始
📂 対象JSONファイル: /workspace/DiaROS_ros/asr_result_lists/asr_result_list_1s.json
==================================================
✅ JSONファイル 'asr_result_lists/asr_result_list_1s.json' から14件のASR結果を読み込みました
📄 JSONファイル形式: リスト（14項目）
DMtoNLGトピックに送信中...
送信内容（全14件）:
  [1] 
  [2] 
  [3] えっえっ[雑音]
  [4] 今どの
  [5] 今度の休日なんだけど
  ...
送信完了！
```

### エラー時の出力
```
❌ エラー: ファイル 'invalid_file.json' が見つかりません
```

## トラブルシューティング

### Q1: "No module named 'rclpy'" エラー
**A:** ROS2環境が設定されていません。以下を実行してください：
```bash
source install/local_setup.bash
```

### Q2: JSONパースエラー
**A:** ファイル形式を確認してください。シングルクォートは自動変換されますが、複雑な形式は手動修正が必要です。

### Q3: トピックが送信されない
**A:** DiaROSシステムが起動していることを確認してください：
```bash
ros2 topic list | grep DMtoNLG
```

## ファイル構成
- `test_dm_to_nlg_send.py`: メインのテストプログラム
- `test_json_loading.py`: JSON読み込みテスト（ROS2環境不要）
- `asr_result_lists/`: JSONファイル格納ディレクトリ
  - `asr_result_list_1s.json`: メインのテストデータ
  - `sample_single_quotes.json`: シングルクォートテスト用