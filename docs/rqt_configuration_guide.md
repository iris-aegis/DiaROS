# RQT設定永続化ガイド - DiaROS版

## 概要

DiaROSの監視に最適化されたRQT設定を保存・再利用できる機能をmonitor.shに追加しました。

## 機能

### 1. DiaROS専用RQT監視
- **メニューオプション**: `1a. DiaROS専用rqt監視（プリセット設定）`
- **説明**: 保存済みの設定でRQTを起動

### 2. 設定管理
- **36a. RQT設定を保存**: 現在の設定をプリセットとして保存
- **36b. RQT設定を初期化**: デフォルト設定に戻す

## 使用方法

### 初回設定手順

1. **monitor.shを起動**
   ```bash
   cd /workspace/DiaROS
   ./scripts/debug/monitor.sh
   ```

2. **通常のRQTを起動**
   ```
   # メニューで「1」を選択
   1. rqt (Full GUI Dashboard)
   ```

3. **監視したいプラグインを設定**
   - **Plot**: `Plugins` → `Visualization` → `Plot`
   - **Topic**: `Plugins` → `Topics` → `Topic Monitor`  
   - **Graph**: `Plugins` → `Visualization` → `ROS Graph`

4. **Plotで監視するトピックを設定**
   ```
   /mic_audio_float32/data[0]    # 音声入力レベル
   /ASRtoNLU/you                 # 音声認識結果
   /DMtoNLG/response            # 対話管理応答
   /SStoDM/result               # 音声合成結果
   ```

5. **ウィンドウレイアウトを調整**
   - プラグインのサイズと位置を調整
   - 必要に応じて分割表示を設定

6. **設定を保存**
   ```
   # RQTのメニューで: File → Save Perspective
   # 保存場所: /workspace/config/rqt_diaros_monitoring.perspective
   
   # または monitor.sh で「36a」を選択
   ```

### 日常的な使用

1. **DiaROS専用RQTを起動**
   ```bash
   ./scripts/debug/monitor.sh
   # メニューで「1a」を選択
   ```

2. **設定が自動的に復元される**
   - 保存したプラグイン構成
   - ウィンドウレイアウト
   - 監視対象トピック

## 監視推奨トピック

### 音声処理フロー
```
/mic_audio_float32/data[0]           # マイク入力レベル
/AAtoDM/frequency                    # 音響解析結果
/ASRtoNLU/you                        # 音声認識結果
/ASRtoNLU/is_final                   # 認識確定フラグ
```

### 対話処理フロー
```
/NLUtoDM/intent                      # 意図理解結果
/DMtoNLG/response                    # 対話管理応答
/NLGtoSS/synthesis_text              # 音声合成テキスト
/SStoDM/result                       # 音声合成結果
```

### システム状態
```
/TTtoDM/speaker_state                # 話者状態
/BCtoDM/backchannel_type             # 相槌タイプ
/MM/mod                              # モード情報
```

## 環境対応

### Docker環境
- Docker内での実行に対応
- X11転送によるGUI表示

### ネイティブ環境
- 直接Linux環境での実行に対応
- ROS2 Humble環境での動作

## 設定ファイル

### 場所
```
/workspace/config/rqt_diaros_monitoring.perspective
```

### 内容
- JSON形式のRQT設定ファイル
- プラグイン構成とレイアウト情報
- 監視対象トピック設定

## トラブルシューティング

### 設定が読み込まれない
1. 設定ファイルの存在確認
   ```bash
   ls -la /workspace/config/rqt_diaros_monitoring.perspective
   ```
2. 「36b」で設定を初期化
3. 再度設定を作成

### プラグインが表示されない
1. DiaROSシステムが起動していることを確認
2. トピックが正しく配信されていることを確認
   ```bash
   ros2 topic list
   ```
3. 必要に応じてDiaROSを再起動

### GUI表示エラー（Docker環境）
1. X11転送の設定確認
2. monitor.shの「42. X11環境診断」を実行
3. 必要に応じて「43. X11接続修復」を実行

## 高度な使用方法

### 複数の設定を作成
```bash
# 設定ファイルを複数作成
cp /workspace/config/rqt_diaros_monitoring.perspective /workspace/config/rqt_audio_only.perspective
cp /workspace/config/rqt_diaros_monitoring.perspective /workspace/config/rqt_dialog_only.perspective
```

### カスタムメニューの追加
monitor.shを編集して、カスタム設定オプションを追加可能です。

## 注意事項

- 設定保存前にRQTを正しく設定する必要があります
- Docker環境では適切なX11転送設定が必要です
- トピック名の変更がある場合は、設定ファイルの更新が必要です
- 大きなウィンドウサイズでの設定は、小さな画面では適切に表示されない場合があります

## 関連ファイル

- `/workspace/scripts/debug/monitor.sh` - メイン監視スクリプト
- `/workspace/config/rqt_diaros_monitoring.perspective` - RQT設定ファイル
- `/workspace/docs/rqt_configuration_guide.md` - このガイド