#!/usr/bin/env python3
"""
DiaROS時間計測統合例
既存ノードへの統合方法のサンプル
"""

# 1. 音声入力ノード（speech_input）での統合例
class SpeechInputWithTiming:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker("main_pc")
        
    def detect_speech_start(self, audio_data):
        """音声開始検出時"""
        # セッションIDを生成（音声開始時刻ベース）
        session_id = f"dialog_{int(time.time() * 1000)}"
        
        # 計測開始
        self.time_tracker.start_session(session_id, "speech_input")
        
        # 音声データと一緒にsession_idを次のノードに送信
        return session_id, audio_data

# 2. 音声認識ノード（ASR）での統合例
class ASRWithTiming:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker("main_pc")
        
    def process_audio(self, session_id, audio_data):
        """音声認識処理"""
        # チェックポイント追加
        self.time_tracker.add_checkpoint(session_id, "asr", "recognition_start")
        
        # 既存の音声認識処理
        result = self.recognize_speech(audio_data)
        
        # 認識完了チェックポイント
        self.time_tracker.add_checkpoint(
            session_id, "asr", "recognition_complete", 
            {"text": result, "confidence": 0.95}
        )
        
        return session_id, result

# 3. 対話管理ノード（DM）での統合例
class DialogManagementWithTiming:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker("main_pc")
        
    def process_dialogue(self, session_id, asr_result):
        """対話管理処理"""
        self.time_tracker.add_checkpoint(session_id, "dm", "dialogue_start")
        
        # 既存の対話管理処理
        response = self.generate_response(asr_result)
        
        self.time_tracker.add_checkpoint(
            session_id, "dm", "dialogue_complete", 
            {"response": response}
        )
        
        return session_id, response

# 4. 自然言語生成ノード（NLG）での統合例（別PC）
class NLGWithTiming:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker("nlg_pc")  # 別PC
        
    def generate_response(self, session_id, dialogue_context):
        """応答生成処理"""
        self.time_tracker.add_checkpoint(session_id, "nlg", "generation_start")
        
        # 既存のNLG処理
        response = self.call_llm_api(dialogue_context)
        
        self.time_tracker.add_checkpoint(
            session_id, "nlg", "generation_complete", 
            {"response": response, "model": "gpt-4"}
        )
        
        return session_id, response

# 5. 音声合成ノード（SS）での統合例
class SpeechSynthesisWithTiming:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker("main_pc")
        
    def synthesize_speech(self, session_id, text):
        """音声合成処理"""
        self.time_tracker.add_checkpoint(session_id, "ss", "synthesis_start")
        
        # 既存の音声合成処理
        audio_file = self.generate_audio(text)
        
        self.time_tracker.add_checkpoint(
            session_id, "ss", "synthesis_complete", 
            {"audio_file": audio_file}
        )
        
        return session_id, audio_file
    
    def play_audio(self, session_id, audio_file):
        """音声再生開始"""
        self.time_tracker.add_checkpoint(session_id, "ss", "playback_start")
        
        # 音声再生処理
        self.play_sound(audio_file)
        
        # 🎯 計測終了（総計時間計算）
        self.time_tracker.end_session(session_id, "ss")

# 6. 統合計測レポート生成
class TimingReporter:
    def __init__(self):
        from diaros.timeTracker import get_time_tracker
        self.time_tracker = get_time_tracker()
    
    def generate_performance_report(self, session_id):
        """性能レポート生成"""
        timeline = self.time_tracker.get_session_timeline(session_id)
        
        if not timeline:
            return "セッションが見つかりません"
        
        # 各段階の処理時間計算
        stages = {}
        for i in range(len(timeline) - 1):
            current = timeline[i]
            next_event = timeline[i + 1]
            
            stage_name = f"{current.node_name}→{next_event.node_name}"
            duration_ms = (next_event.timestamp_ns - current.timestamp_ns) / 1_000_000
            stages[stage_name] = duration_ms
        
        # 総計時間
        total_time_ms = (timeline[-1].timestamp_ns - timeline[0].timestamp_ns) / 1_000_000
        
        # レポート出力
        report = f"""
📊 音声対話システム性能レポート
セッションID: {session_id}
総計時間: {total_time_ms:.1f}ms

🔍 各段階の処理時間:
"""
        for stage, duration in stages.items():
            report += f"  {stage}: {duration:.1f}ms\n"
        
        # 性能評価
        if total_time_ms < 1000:
            report += "\n✅ 優秀: 1秒以内で応答完了"
        elif total_time_ms < 2000:
            report += "\n🟡 良好: 2秒以内で応答完了"
        else:
            report += "\n🔴 要改善: 応答時間が長すぎます"
        
        return report

# 使用例
if __name__ == "__main__":
    # セッション開始から終了まで
    session_id = "dialog_1234567890"
    
    # 1. 音声入力
    speech_input = SpeechInputWithTiming()
    session_id, audio = speech_input.detect_speech_start(b"audio_data")
    
    # 2. 音声認識
    asr = ASRWithTiming()
    session_id, text = asr.process_audio(session_id, audio)
    
    # 3. 対話管理
    dm = DialogManagementWithTiming()
    session_id, response = dm.process_dialogue(session_id, text)
    
    # 4. 自然言語生成（別PC）
    nlg = NLGWithTiming()
    session_id, final_response = nlg.generate_response(session_id, response)
    
    # 5. 音声合成・再生
    ss = SpeechSynthesisWithTiming()
    session_id, audio_file = ss.synthesize_speech(session_id, final_response)
    ss.play_audio(session_id, audio_file)
    
    # 6. レポート生成
    reporter = TimingReporter()
    report = reporter.generate_performance_report(session_id)
    print(report)