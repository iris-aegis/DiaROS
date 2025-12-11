# ステージベース通信実装サマリー

## 実装日時
2025-12-08

## 実装概要

DiaROSシステムの核となる設計思想「ROS通信による疎結合アーキテクチャ」を実現するため、Dialog Management（DM）と Natural Language Generation（NLG）間の**ステージベース通信**を実装しました。

これにより、どのモジュールが別のPCに移行されても、システムが継続して動作可能な設計を実現しました。

## 変更内容

### 1. ドキュメント更新

#### CLAUDE.md
**場所**: `/workspace/DiaROS/CLAUDE.md`

追加セクション:
```markdown
## システム設計の核：ROS通信による疎結合アーキテクチャ

### モジュール間通信の基本原則
- 完全な疎結合
- 分散実行対応
- スケーラビリティ
- 保守性

### メッセージベースのステージ管理
- DMからNLGへ: Idm メッセージの stage フィールドで処理段階を指示
- NLGからDMへ: Inlg メッセージの stage フィールドで完了ステージを報告
- 処理追跡: request_id、タイムスタンプフィールドで分散環境での処理状況を追跡
```

### 2. NLGノードの改善

**ファイル**: `/workspace/DiaROS/DiaROS_ros/src/diaros_package/diaros_package/ros2_natural_language_generation.py`

**変更点**:
- `time` モジュールをインポート
- ステージ管理フィールドを追加:
  - `current_stage`: 現在処理中のステージ
  - `current_request_id`: 現在処理中のリクエストID
  - `stage_start_timestamp_ns`: ステージ開始時刻

- `dm_update()` メソッドを改善:
  - `stage` フィールドの取得
  - `request_id` フィールドの取得
  - 新規リクエストの開始を記録（ログ出力）
  - ステージ開始タイムスタンプの記録

- `ping()` メソッドを改善:
  - NLGメッセージに `stage` を設定
  - NLGメッセージに `request_id` を設定
  - ステージ処理時間の計測
  - ステージ完了時のログ出力

### 3. DMノードの改善

**ファイル**: `/workspace/DiaROS/DiaROS_ros/src/diaros_package/diaros_package/ros2_dialog_management.py`

**変更点**:
- リクエストID管理機構を追加:
  - `request_id_counter`: グローバルリクエストIDカウンター
  - `current_request_stage`: 現在のステージ

- `nlg_callback()` メソッドを改善:
  - `request_id` フィールドの取得と処理
  - ステージ完了ログの出力
  - NLGタイミング情報の伝播（NLGの推論時間を下流に渡す）

- `callback()` メソッド（First stage）を改善:
  - ステージ変更時のリクエストID自動更新
  - リクエストメッセージへの request_id 設定
  - DM→NLG送信ログの追加

- `callback()` メソッド（Second stage）を改善:
  - Second stage リクエスト時のリクエストID管理
  - ステージ変更の検出と request_id 更新

### 4. テストスクリプト新規作成

**ファイル**: `/workspace/DiaROS/DiaROS_ros/test_stage_based_communication.py`

**機能**:
- First stage リクエストのテスト
- Second stage リクエストのテスト
- 複数リクエスト処理のテスト
- request_id の連続性確認
- ステージ情報の正確性確認
- テスト結果のサマリー表示

**実行方法**:
```bash
python3 /workspace/DiaROS/DiaROS_ros/test_stage_based_communication.py
```

### 5. ドキュメント新規作成

#### STAGE_BASED_COMMUNICATION.md
**ファイル**: `/workspace/DiaROS/DiaROS_ros/STAGE_BASED_COMMUNICATION.md`

内容:
- システム設計原則の詳細説明
- ステージベース通信の仕組み
- Idm/Inlg メッセージ仕様
- リクエストID管理の詳細
- ステージ管理フロー（シナリオ例含む）
- タイミング計測方法
- テスト方法
- 分散実行での利用方法
- ログ出力例
- トラブルシューティング

## メッセージフロー図

```
[DM]
  |
  | Idm メッセージ
  | - stage: "first" or "second"
  | - request_id: 1, 2, 3, ...
  | - words: ["音声認識結果"]
  |
  ↓
[NLG]
  |
  | Inlg メッセージ
  | - stage: 受け取ったstageを返す
  | - request_id: 受け取ったrequest_idを返す
  | - reply: "生成された応答"
  |
  ↓
[SS（Speech Synthesis）]
```

## ステージ定義

### First Stage（相槌生成）
- **用途**: ユーザー発言中に相槌を生成
- **特徴**: 潜在期間が短い
- **例**: "そうですね"、"なるほど"

### Second Stage（応答生成）
- **用途**: ユーザー発言終了後に実際の応答を生成
- **特徴**: より詳細な応答を作成可能
- **例**: "今日は晴れです"、"そのためには..."

## request_id の役割

- **一意性**: 各リクエストを一意に識別
- **追跡**: DMが送信したリクエストとNLGからの応答を対応付け
- **分散実行対応**: 複数PCでの処理状況を追跡可能

### request_id の割り当てルール

1. DMが最初のリクエストを送信: request_id = 1
2. ステージが変わったら request_id をインクリメント
3. NLGはDMから受け取った request_id をそのまま返す
4. DMはNLG応答の request_id を使用して処理を対応付け

## タイミング計測

各処理段階での時間を計測可能：

- `stage_start_timestamp_ns`: ステージ開始時刻
- `stage_duration_ms`: ステージ処理時間
- `nlg_inference_duration_ms`: NLGの推論時間

これらはシステムパフォーマンス分析の基礎となります。

## 分散実行への対応

このステージベース通信設計により、以下のシナリオで対応可能：

### シナリオ1: メインPC で DM、別PC で NLG を実行

```bash
# メインPC
export ROS_DOMAIN_ID=0
ros2 launch diaros_package sdsmod.launch.py nlg:=false

# NLG専用PC
export ROS_DOMAIN_ID=0
ros2 run diaros_package ros2_natural_language_generation
```

### シナリオ2: 将来的に他のモジュールを別PCに移行

同じ設計パターンをASR、SS、TT等にも適用可能。

## 検証チェックリスト

- [x] CLAUDE.md に設計思想を記載
- [x] NLGノードでステージ管理を実装
- [x] DMノードでリクエストID生成・管理を実装
- [x] NLGとDM間のメッセージ交換でステージ情報を伝播
- [x] テストスクリプトを作成
- [x] ドキュメントを作成

## 使用技術

- ROS2 (Foxy/Humble)
- Python 3.8+
- カスタムROSメッセージインターフェース (Idm, Inlg)

## 今後の拡張可能性

1. **他のモジュールへの拡張**
   - ASR（Automatic Speech Recognition）
   - SS（Speech Synthesis）
   - TT（Turn Taking）
   - BC（Back Channel）

2. **タイミング分析システムとの統合**
   - 各ステージの処理時間をDB に記録
   - パフォーマンスボトルネックの特定

3. **分散実行トレーシング**
   - 複数PC での処理フロー可視化
   - ネットワーク遅延の計測

4. **エラーハンドリングの拡張**
   - タイムアウト検出
   - 再試行メカニズム
   - フェイルオーバー対応

## 参考資料

- `CLAUDE.md`: システム全体の設計ガイドライン
- `STAGE_BASED_COMMUNICATION.md`: ステージベース通信の詳細ドキュメント
- `test_stage_based_communication.py`: テスト実装例
- `ros2_dialog_management.py`: DMノード実装
- `ros2_natural_language_generation.py`: NLGノード実装

## 注意事項

- 分散実行時は `ROS_DOMAIN_ID` を両PCで統一すること
- タイムスタンプはナノ秒単位で記録されるため、システム時刻同期（NTP）を推奨
- ログ出力の詳細度は、本番環境でのパフォーマンス影響を考慮して調整してください

---

**実装者**: Claude Code
**実装日**: 2025-12-08
**バージョン**: 1.0
