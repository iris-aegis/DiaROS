#!/usr/bin/env python3
"""
DiaROS統合時間計測システム
全ノードで共通利用可能な時間計測ユーティリティ
"""

import time
import json
import threading
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, Optional

class DiaROSTimingLogger:
    """DiaROS統合時間計測ロガー"""
    
    def __init__(self):
        self.session_data = {}
        self.lock = threading.Lock()
        self.log_dir = Path("/tmp/diaros_timing")
        self.log_dir.mkdir(exist_ok=True)
        
        # ログファイル設定
        self.stdout_log = True
        self.file_log = True
        self.current_session_id = None
        
        print("🕐 DiaROS統合時間計測システム初期化完了")
    
    def start_session(self, session_id: str = None) -> str:
        """セッション開始"""
        if session_id is None:
            session_id = f"dialog_{int(time.time_ns() / 1000000)}"
        
        with self.lock:
            self.current_session_id = session_id
            self.session_data[session_id] = {
                "session_id": session_id,
                "start_time_ns": time.time_ns(),
                "events": [],
                "metrics": {}
            }
        
        self._log_event(session_id, "SESSION", "start", "セッション開始")
        return session_id
    
    def log_timing(self, session_id: str, node_name: str, event_type: str, 
                   message: str, data: Dict[str, Any] = None):
        """時間計測ログ追加"""
        timestamp_ns = time.time_ns()
        
        with self.lock:
            if session_id not in self.session_data:
                # セッションが存在しない場合は自動作成
                self.session_data[session_id] = {
                    "session_id": session_id,
                    "start_time_ns": timestamp_ns,
                    "events": [],
                    "metrics": {}
                }
            
            # 経過時間計算
            start_time = self.session_data[session_id]["start_time_ns"]
            elapsed_ms = (timestamp_ns - start_time) / 1_000_000
            
            event = {
                "timestamp_ns": timestamp_ns,
                "elapsed_ms": elapsed_ms,
                "node_name": node_name,
                "event_type": event_type,
                "message": message,
                "data": data or {}
            }
            
            self.session_data[session_id]["events"].append(event)
        
        self._log_event(session_id, node_name, event_type, message, elapsed_ms, data)
    
    def _log_event(self, session_id: str, node_name: str, event_type: str, 
                   message: str, elapsed_ms: float = None, data: Dict = None):
        """イベントログ出力"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        if elapsed_ms is not None:
            log_msg = f"[{timestamp}][{node_name}] {event_type}: {message} ({elapsed_ms:.1f}ms)"
        else:
            log_msg = f"[{timestamp}][{node_name}] {event_type}: {message}"
        
        # 標準出力
        if self.stdout_log:
            print(log_msg)
        
        # ファイル出力
        if self.file_log:
            log_file = self.log_dir / f"timing_{session_id}.log"
            with open(log_file, 'a', encoding='utf-8') as f:
                f.write(f"{log_msg}\n")
                if data:
                    f.write(f"    データ: {json.dumps(data, ensure_ascii=False)}\n")
    
    def end_session(self, session_id: str, final_message: str = "セッション終了"):
        """セッション終了"""
        timestamp_ns = time.time_ns()
        
        with self.lock:
            if session_id in self.session_data:
                start_time = self.session_data[session_id]["start_time_ns"]
                total_ms = (timestamp_ns - start_time) / 1_000_000
                
                self.session_data[session_id]["end_time_ns"] = timestamp_ns
                self.session_data[session_id]["total_duration_ms"] = total_ms
                
                self._log_event(session_id, "SESSION", "end", f"{final_message} (総計: {total_ms:.1f}ms)")
                
                # セッションサマリー出力
                self._export_session_summary(session_id)
                
                return total_ms
        
        return None
    
    def _export_session_summary(self, session_id: str):
        """セッション概要出力"""
        if session_id not in self.session_data:
            return
        
        session = self.session_data[session_id]
        events = session["events"]
        
        if len(events) < 2:
            return
        
        # 詳細タイムライン出力
        timeline_file = self.log_dir / f"timeline_{session_id}.json"
        with open(timeline_file, 'w', encoding='utf-8') as f:
            json.dump(session, f, indent=2, ensure_ascii=False)
        
        # サマリー出力
        total_ms = session.get("total_duration_ms", 0)
        
        print(f"\n📊 セッション {session_id} 完了")
        print("=" * 50)
        print(f"総計時間: {total_ms:.1f}ms")
        print(f"イベント数: {len(events)}")
        
        # 各段階の処理時間
        if len(events) > 1:
            print("\n🔍 処理段階:")
            for i in range(len(events) - 1):
                current = events[i]
                next_event = events[i + 1]
                stage_duration = next_event["elapsed_ms"] - current["elapsed_ms"]
                print(f"  {current['node_name']}→{next_event['node_name']}: {stage_duration:.1f}ms")
        
        print(f"\n📁 詳細ログ: {timeline_file}")
        print("=" * 50)

# グローバルインスタンス
_timing_logger = None
_logger_lock = threading.Lock()

def get_timing_logger() -> DiaROSTimingLogger:
    """統合時間計測ロガー取得"""
    global _timing_logger
    
    with _logger_lock:
        if _timing_logger is None:
            _timing_logger = DiaROSTimingLogger()
        return _timing_logger

# 便利関数
def start_timing_session(session_id: str = None) -> str:
    """タイミングセッション開始"""
    logger = get_timing_logger()
    return logger.start_session(session_id)

def log_timing(session_id: str, node_name: str, event_type: str, 
               message: str, data: Dict[str, Any] = None):
    """タイミングログ記録"""
    logger = get_timing_logger()
    logger.log_timing(session_id, node_name, event_type, message, data)

def end_timing_session(session_id: str, final_message: str = "セッション終了") -> Optional[float]:
    """タイミングセッション終了"""
    logger = get_timing_logger()
    return logger.end_session(session_id, final_message)

# 各ノード専用の便利関数
def log_audio_frame_received(session_id: str, frame_count: int = None):
    """音声フレーム受信ログ"""
    data = {"frame_count": frame_count} if frame_count else None
    log_timing(session_id, "SPEECH_INPUT", "audio_frame", "最新音声フレーム取得", data)

def log_asr_start(session_id: str):
    """音声認識開始ログ"""
    log_timing(session_id, "ASR", "recognition_start", "音声認識開始")

def log_asr_complete(session_id: str, text: str, duration_ms: float):
    """音声認識完了ログ"""
    data = {"recognized_text": text, "asr_duration_ms": duration_ms}
    log_timing(session_id, "ASR", "recognition_complete", f"音声認識完了: {text}", data)

def log_nlg_start(session_id: str):
    """対話生成開始ログ"""
    log_timing(session_id, "NLG", "generation_start", "対話生成開始")

def log_nlg_complete(session_id: str, response: str, duration_ms: float):
    """対話生成完了ログ"""
    data = {"generated_response": response, "nlg_duration_ms": duration_ms}
    log_timing(session_id, "NLG", "generation_complete", f"対話生成完了: {response}", data)

def log_tts_start(session_id: str, text: str):
    """音声合成開始ログ"""
    data = {"synthesis_text": text}
    log_timing(session_id, "TTS", "synthesis_start", f"音声合成開始: {text}", data)

def log_tts_complete(session_id: str, audio_file: str, duration_ms: float):
    """音声合成完了ログ"""
    data = {"audio_file": audio_file, "tts_duration_ms": duration_ms}
    log_timing(session_id, "TTS", "synthesis_complete", f"音声合成完了: {audio_file}", data)

def log_audio_playback_start(session_id: str, audio_file: str):
    """音声再生開始ログ"""
    data = {"audio_file": audio_file}
    log_timing(session_id, "PLAYBACK", "playback_start", f"音声再生開始: {audio_file}", data)

def log_audio_playback_end(session_id: str):
    """音声再生終了ログ"""
    log_timing(session_id, "PLAYBACK", "playback_end", "音声再生終了")

if __name__ == "__main__":
    # テスト実行
    print("🧪 DiaROS統合時間計測システムテスト")
    
    # テストセッション
    session_id = start_timing_session()
    
    log_audio_frame_received(session_id, 1024)
    time.sleep(0.1)
    
    log_asr_start(session_id)
    time.sleep(0.2)
    log_asr_complete(session_id, "こんにちは", 200.5)
    
    log_nlg_start(session_id)
    time.sleep(0.3)
    log_nlg_complete(session_id, "はい、こんにちは", 300.2)
    
    log_tts_start(session_id, "はい、こんにちは")
    time.sleep(0.15)
    log_tts_complete(session_id, "response.wav", 150.8)
    
    log_audio_playback_start(session_id, "response.wav")
    time.sleep(0.05)
    
    total_ms = end_timing_session(session_id)
    print(f"\n✅ テスト完了: 総計時間 {total_ms:.1f}ms")