# メッセージインターフェース更新ドキュメント

## 更新日時
2025-12-08

## 更新概要

ステージベース通信の実装に伴い、DM（Dialog Management）からNLG（Natural Language Generation）へのリクエストメッセージ `Idm` に、**`request_id` フィールドを追加**しました。

これにより、DM と NLG、さらに SS（Speech Synthesis）間で、処理リクエストを一貫して追跡できるようになります。

## 変更内容

### Idm メッセージ（Dialog Management → Natural Language Generation）

**ファイル**: `/workspace/DiaROS/DiaROS_ros/src/interfaces/msg/Idm.msg`

#### 変更前
```
string[] words
string session_id
string stage
int64 turn_taking_decision_timestamp_ns
```

#### 変更後
```
string[] words
string session_id
string stage
int32 request_id
int64 turn_taking_decision_timestamp_ns
```

**追加フィールド**:
- `request_id` (int32): リクエストの一意識別子

## メッセージの設計一貫性

現在、DiaROS の主要メッセージインターフェースは以下の構造になっています：

### request_id を含むメッセージ

| メッセージ | 用途 | request_id | stage |
|-----------|------|-----------|-------|
| **Idm** | DM → NLG リクエスト | ✓ **新規追加** | ✓ |
| **Inlg** | NLG → DM/SS 応答 | ✓ 既存 | ✓ |
| **Iss** | SS → DM 応答 | ✓ 既存 | ✗ |

### 処理フロー上での request_id の流れ

```
DM (request_id=1)
  ↓
NLG メッセージ受信 (request_id=1 を取得)
  ↓ NLG 処理実行
  ↓
NLG → SS メッセージ送信 (request_id=1 を設定)
  ↓
SS メッセージ受信 (request_id=1 を処理)
  ↓
SS 処理実行（音声合成）
  ↓
SS → DM メッセージ送信 (request_id=1 を設定)
```

この設計により、**複数の並行リクエストがあっても、各処理ステージで正確に対応付け可能**になります。

## ビルドとデプロイ

メッセージ定義を変更したため、以下のビルドコマンドを実行してください：

```bash
cd DiaROS_ros
source /opt/ros/humble/setup.bash

# Interfaceパッケージをリビルド
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces

# ローカルセットアップを更新
source ./install/local_setup.bash

# diaros_package をリビルド
colcon build --packages-select diaros_package

# 環境をリロード
source ./install/local_setup.bash
```

## コード実装への影響

### DMノード（ros2_dialog_management.py）

`request_id` の設定が可能になりました：

```python
dm = Idm()
dm.words = words
dm.stage = stage
dm.request_id = self.request_id_counter  # ★ 新しく設定可能
self.pub_dm.publish(dm)
```

### NLGノード（ros2_natural_language_generation.py）

`request_id` を受け取れるようになりました：

```python
def dm_update(self, msg):
    request_id = getattr(msg, 'request_id', 0)  # ★ 新しく取得可能
    words = list(msg.words)
    # ...
```

## 互換性への考慮

### 既存コードとの互換性

- **後方互換性**: 既存の実装で `request_id` を設定していなくても、デフォルト値 `0` で動作します
- **前方互換性**: NLG側では `getattr(msg, 'request_id', 0)` で安全に取得できます

### マイグレーション時の注意

古いメッセージ定義を使用していた外部モジュール（カスタムノード等）がある場合：

1. メッセージ定義を更新
2. パッケージをリビルド
3. コード内で `request_id` を適切に処理（設定または取得）

## テスト確認項目

メッセージ定義の変更後、以下をテストしてください：

- [ ] DM が `Idm` メッセージに `request_id` を設定できる
- [ ] NLG が `Idm` メッセージから `request_id` を取得できる
- [ ] NLG が `Inlg` メッセージに `request_id` を設定できる
- [ ] DM が `Inlg` メッセージから `request_id` を取得できる
- [ ] 複数の同時リクエストで `request_id` が正確に対応付けられている

## ビルド結果

```
Starting >>> interfaces
Finished <<< interfaces [2.94s]

Starting >>> diaros_package
Finished <<< diaros_package [0.46s]

Summary: 2 packages finished [3.54s]
```

✓ ビルド成功

## 関連ドキュメント

- [CLAUDE.md](CLAUDE.md): システム設計の全体像
- [STAGE_BASED_COMMUNICATION.md](STAGE_BASED_COMMUNICATION.md): ステージベース通信の詳細
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md): 実装内容の総括

---

**更新者**: Claude Code
**更新日**: 2025-12-08
**バージョン**: 1.1
