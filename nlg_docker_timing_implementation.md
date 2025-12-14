# DiaROS分散時間計測 - NLG側Docker実装指示書

## 🎯 実装目的
NLG専用Docker環境で対話生成の時間計測を行い、メインPCと協調した総合計時間計測を実現します。

## 📋 実装すべきタスク

### 1. Docker専用時間計測ライブラリの確認
```bash
# Docker専用timeTrackerが利用可能か確認
python3 -c "from diaros.timeTracker_docker import get_time_tracker; print('✅ Docker TimeTracker OK')"

# エラーが出る場合は以下を実行
cd /workspace/DiaROS/DiaROS_py
python -m pip install . --user
```

### 2. NLG時間計測の統合実装

#### A. ros2_natural_language_generation.py の修正

以下のコードを **ros2_natural_language_generation.py** ファイルに追加してください：

```python
# インポート部分に追加
from diaros.timing_integration import get_timing_logger, log_nlg_start, log_nlg_complete

class RosNaturalLanguageGeneration(Node):
    def __init__(self, naturalLanguageGeneration):
        super().__init__('natural_language_generation')
        # 既存の初期化コード...
        
        # 時間計測用の追加
        self.timing_logger = get_timing_logger()
        self.current_session_id = None
        self.nlg_start_time = None
        
    def callback_for_nlg(self, dm_msg):
        """DM（対話管理）からのメッセージを受信した際のコールバック"""
        
        # 時間計測: セッションID取得とNLG開始
        if self.current_session_id is None:
            sessions = self.timing_logger.session_data
            if sessions:
                self.current_session_id = list(sessions.keys())[-1]  # 最新セッション
        
        # 新しいセッションの場合は開始
        if self.current_session_id is None:
            from diaros.timing_integration import start_timing_session
            self.current_session_id = start_timing_session()
        
        # 時間計測: 対話生成開始
        if self.current_session_id:
            self.nlg_start_time = time.time()
            log_nlg_start(self.current_session_id)
        
        # 既存のNLG処理
        dialogue_context = dm_msg.dialogue_context  # または適切なフィールド
        response = self.naturalLanguageGeneration.generate_response(dialogue_context)
        
        # 時間計測: 対話生成完了
        if self.current_session_id and self.nlg_start_time:
            nlg_duration_ms = (time.time() - self.nlg_start_time) * 1000
            log_nlg_complete(self.current_session_id, response, nlg_duration_ms)
        
        # NLG応答をメインPCに送信
        nlg_msg = Inlg()
        nlg_msg.reply = response
        nlg_msg.session_id = self.current_session_id  # セッションIDを追加
        # 他の必要なフィールドも設定...
        
        self.publisher.publish(nlg_msg)
```

### 3. 時間計測データの可視化

#### A. Docker内での計測結果確認
```bash
# 計測データファイル確認
ls -la /tmp/diaros_timing/timeline_*.json

# 最新セッション分析
latest=$(ls -t /tmp/diaros_timing/timeline_*.json | head -1)
python3 /workspace/DiaROS/scripts/debug/timing_visualizer.py "$latest" analyze
```

#### B. NLG専用の性能監視
```bash
# NLG処理時間の監視
python3 << 'EOF'
import json
import glob
import time

print("🐳 NLG Docker 性能監視開始")
print("=" * 40)

while True:
    files = glob.glob("/tmp/diaros_timing/timeline_*.json")
    if files:
        latest_file = sorted(files)[-1]
        
        try:
            with open(latest_file, 'r') as f:
                data = json.load(f)
                timeline = data['timeline']
                
                # NLG関連のイベントを抽出
                nlg_events = [event for event in timeline if event['node_name'] == 'NLG']
                
                if len(nlg_events) >= 2:
                    nlg_start = next((e for e in nlg_events if e['event_type'] == 'generation_start'), None)
                    nlg_complete = next((e for e in nlg_events if e['event_type'] == 'generation_complete'), None)
                    
                    if nlg_start and nlg_complete:
                        nlg_duration = nlg_complete['elapsed_ms'] - nlg_start['elapsed_ms']
                        response_text = nlg_complete.get('data', {}).get('generated_response', '')
                        
                        print(f"🧠 NLG処理時間: {nlg_duration:.1f}ms")
                        print(f"📝 応答: {response_text[:50]}...")
                        print("-" * 40)
        except Exception as e:
            pass
    
    time.sleep(2)
EOF
```

### 4. 分散環境での協調設定

#### A. ROS2環境変数の確認
```bash
# ROS2ドメイン設定確認
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# 未設定の場合は以下を実行
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

#### B. メインPCとの通信確認
```bash
# メインPCからのトピック受信確認
ros2 topic echo /DMtoNLG --timeout 10

# メインPCへのトピック送信確認
ros2 topic pub /NLGtoSS interfaces/msg/Inlg "{reply: 'テスト応答', session_id: 'test_123'}" --once
```

### 5. 統合テスト実行

#### A. NLGノード単体起動
```bash
cd /workspace/DiaROS/DiaROS_ros
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# NLGノード起動
ros2 run diaros_package ros2_natural_language_generation
```

#### B. 計測データの統合確認
```bash
# メインPCで実行されるコマンド
# bash /workspace/DiaROS/scripts/launch/launch_diaros_no_speech_input_simple.sh

# Docker内で計測データ確認
watch -n 3 'ls -t /tmp/diaros_timing/timeline_*.json | head -3'
```

### 6. 性能最適化の監視

#### A. NLG処理時間のベンチマーク
```bash
# NLG性能分析スクリプト
python3 << 'EOF'
import json
import glob
import numpy as np
from datetime import datetime

files = glob.glob("/tmp/diaros_timing/timeline_*.json")
nlg_times = []

for file in files:
    try:
        with open(file, 'r') as f:
            data = json.load(f)
            timeline = data['timeline']
            
            nlg_events = [e for e in timeline if e['node_name'] == 'NLG']
            if len(nlg_events) >= 2:
                start = next((e for e in nlg_events if 'start' in e['event_type']), None)
                complete = next((e for e in nlg_events if 'complete' in e['event_type']), None)
                
                if start and complete:
                    duration = complete['elapsed_ms'] - start['elapsed_ms']
                    nlg_times.append(duration)
    except:
        continue

if nlg_times:
    print(f"📊 NLG Docker 性能統計 (N={len(nlg_times)}):")
    print(f"平均生成時間: {np.mean(nlg_times):.1f}ms")
    print(f"最短生成時間: {np.min(nlg_times):.1f}ms")
    print(f"最長生成時間: {np.max(nlg_times):.1f}ms")
    print(f"標準偏差: {np.std(nlg_times):.1f}ms")
    
    # 性能判定
    fast_count = sum(1 for t in nlg_times if t < 500)
    good_count = sum(1 for t in nlg_times if 500 <= t < 1000)
    slow_count = sum(1 for t in nlg_times if t >= 1000)
    
    print(f"\nNLG性能分布:")
    print(f"  高速 (<500ms): {fast_count} ({fast_count/len(nlg_times)*100:.1f}%)")
    print(f"  良好 (500-1000ms): {good_count} ({good_count/len(nlg_times)*100:.1f}%)")
    print(f"  要改善 (>1000ms): {slow_count} ({slow_count/len(nlg_times)*100:.1f}%)")
else:
    print("❌ NLG計測データが見つかりません")
EOF
```

### 7. トラブルシューティング

#### A. セッションIDが伝達されない場合
```bash
# ROS2メッセージ確認
ros2 interface show interfaces/msg/Inlg

# トピック通信確認
ros2 topic echo /DMtoNLG
ros2 topic echo /NLGtoSS
```

#### B. 時間計測データが生成されない場合
```bash
# Python環境確認
python3 -c "from diaros.timing_integration import get_timing_logger; print('OK')"

# ログディレクトリ確認
ls -la /tmp/diaros_timing/

# 権限確認
chmod 777 /tmp/diaros_timing/
```

## 🎯 期待される結果

### NLG処理時間の可視化
- **対話生成開始時刻**: DMからのメッセージ受信時
- **対話生成完了時刻**: NLG応答生成完了時
- **対話生成にかかった時間**: ms単位で正確に計測

### 統合時間計測
- **メインPCとの時刻同期**: 1ms以内の精度
- **セッションID伝達**: 各ノード間で一貫したセッション管理
- **総合計時間**: 音声入力から音声再生完了まで

### 性能分析
- **NLG処理時間**: 平均、最短、最長の統計
- **ボトルネック特定**: 処理段階別の時間分析
- **継続的監視**: リアルタイムでの性能追跡

この実装により、分散環境でのNLG処理時間を正確に計測し、DiaROSシステム全体の性能最適化に貢献します。

## 📞 サポート

実装中に問題が発生した場合は、以下の情報を共有してください：
- エラーメッセージ
- 実行したコマンド
- `/tmp/diaros_timing/`内のファイル一覧
- ROS2トピックの通信状況

これらの情報により、迅速な問題解決が可能です。