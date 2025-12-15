# DiaROS総合計時間計測実装ガイド

## 🎯 概要

分散マルチプロセス環境でのマイクから音声合成終了までの総合計時間を高精度で計測するシステムの実装手順です。

## 📋 実装手順

### Step 1: 時刻同期セットアップ

```bash
# 各PCで実行
chmod +x /workspace/scripts/setup/setup_time_sync.sh
sudo /workspace/scripts/setup/setup_time_sync.sh
```

### Step 2: インターフェース再ビルド

```bash
cd /workspace/DiaROS_ros
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
```

### Step 3: 各ノードに計測コード追加

#### A. 音声入力ノード (speech_input)
```python
from diaros.timeTracker import start_timing, checkpoint

def detect_speech_start(self, audio_data):
    # セッション開始
    session_id = f"dialog_{int(time.time() * 1000)}"
    start_timing(session_id, "speech_input")
    
    # 音声データと一緒にsession_idを送信
    return session_id, audio_data
```

#### B. 音声認識ノード (asr)
```python
from diaros.timeTracker import checkpoint

def process_audio(self, session_id, audio_data):
    checkpoint(session_id, "asr", "recognition_start")
    
    # 既存処理
    result = self.recognize_speech(audio_data)
    
    checkpoint(session_id, "asr", "recognition_complete", 
              {"text": result, "confidence": 0.95})
    
    return session_id, result
```

#### C. 対話管理ノード (dm)
```python
from diaros.timeTracker import checkpoint

def process_dialogue(self, session_id, asr_result):
    checkpoint(session_id, "dm", "dialogue_start")
    
    # 既存処理
    response = self.generate_response(asr_result)
    
    checkpoint(session_id, "dm", "dialogue_complete", 
              {"response": response})
    
    return session_id, response
```

#### D. 自然言語生成ノード (nlg) - 別PC
```python
from diaros.timeTracker import get_time_tracker, checkpoint

def __init__(self):
    self.time_tracker = get_time_tracker("nlg_pc")  # PC名指定

def generate_response(self, session_id, dialogue_context):
    checkpoint(session_id, "nlg", "generation_start")
    
    # 既存処理
    response = self.call_llm_api(dialogue_context)
    
    checkpoint(session_id, "nlg", "generation_complete", 
              {"response": response, "model": "gpt-4"})
    
    return session_id, response
```

#### E. 音声合成ノード (ss)
```python
from diaros.timeTracker import checkpoint, end_timing

def synthesize_and_play(self, session_id, text):
    checkpoint(session_id, "ss", "synthesis_start")
    
    # 音声合成
    audio_file = self.generate_audio(text)
    
    checkpoint(session_id, "ss", "synthesis_complete", 
              {"audio_file": audio_file})
    
    # 音声再生
    checkpoint(session_id, "ss", "playback_start")
    self.play_sound(audio_file)
    
    # 🎯 総計時間計測終了
    end_timing(session_id, "ss")
```

### Step 4: システム起動

```bash
# メインPC
cd /workspace/DiaROS_ros
bash /workspace/scripts/launch/launch_diaros_no_speech_input_simple.sh

# NLG専用PC
ros2 run diaros_package ros2_natural_language_generation
```

### Step 5: 計測結果の確認

```bash
# リアルタイム監視
tail -f /tmp/diaros_timing.log

# 可視化
python3 /workspace/scripts/debug/timing_visualizer.py timeline_dialog_123.json plot

# 詳細レポート
python3 /workspace/scripts/debug/timing_visualizer.py timeline_dialog_123.json report
```

## 📊 計測結果例

### 典型的な処理時間分布
```
📊 セッション dialog_1234567890 分析結果:
==================================================
総計時間: 1247.3ms
イベント数: 8
平均段階時間: 178.2ms

🔍 各段階の処理時間:
  speech_input→asr: 45.2ms (3.6%)
  asr→dm: 123.7ms (9.9%)
  dm→nlg: 15.3ms (1.2%)
  nlg→ss: 890.1ms (71.4%)  ← ボトルネック
  ss→playback: 173.0ms (13.9%)

⚠️  ボトルネック: nlg→ss (890.1ms)
```

### 性能評価基準
- **優秀**: 1000ms以内 ✅
- **良好**: 2000ms以内 🟡
- **要改善**: 2000ms超過 🔴

## 🔧 トラブルシューティング

### 1. 時刻同期エラー
```bash
# NTP状態確認
ntpq -p

# 手動同期
sudo ntpdate -s ntp.nict.jp
```

### 2. セッションIDが伝達されない
```bash
# ROS2トピック確認
ros2 topic echo /TimingSession

# メッセージ形式確認
ros2 interface show interfaces/msg/TimingSession
```

### 3. 計測データが表示されない
```bash
# Python環境確認
python3 -c "from diaros.timeTracker import get_time_tracker; print('OK')"

# ログファイル確認
ls -la /tmp/diaros_timing_*.json
```

## 🚀 高度な機能

### 1. 連続セッション分析
```python
# 複数セッションの統計分析
from diaros.timeTracker import get_time_tracker

tracker = get_time_tracker()
sessions = ["dialog_001", "dialog_002", "dialog_003"]

for session_id in sessions:
    tracker.export_timeline(session_id, f"timeline_{session_id}.json")
```

### 2. リアルタイム性能監視
```python
# 性能監視ダッシュボード
class PerformanceMonitor:
    def __init__(self):
        self.recent_times = []
    
    def on_session_complete(self, session_id, total_time_ms):
        self.recent_times.append(total_time_ms)
        
        # 直近10セッションの平均
        if len(self.recent_times) >= 10:
            avg_time = sum(self.recent_times[-10:]) / 10
            print(f"平均応答時間: {avg_time:.1f}ms")
```

### 3. 自動アラート
```python
# 性能劣化時の自動アラート
def check_performance_alert(total_time_ms):
    if total_time_ms > 2000:
        print(f"🚨 性能アラート: 応答時間 {total_time_ms:.1f}ms")
        # Slack通知、メール送信等
```

## 📈 最適化のポイント

1. **ボトルネック特定**: 各段階の処理時間を可視化
2. **並列処理**: 可能な処理の並列化
3. **ネットワーク最適化**: 分散環境での通信最適化
4. **キャッシュ活用**: 頻繁に使用するデータのキャッシュ
5. **モデル最適化**: LLMモデルの推論速度向上

## 🎯 期待される効果

- **精密な性能測定**: ナノ秒精度での計測
- **ボトルネック特定**: 最適化すべき箇所の明確化
- **継続的改善**: 性能変化の追跡
- **分散環境対応**: 複数PC間での統一計測
- **自動化**: 手動計測の自動化

この実装により、分散マルチプロセス環境での音声対話システムの総合計時間を高精度で計測し、継続的な性能改善が可能になります。