#!/usr/bin/env python3
"""
DiaROS分散時間計測 - タイミング統合モジュール
NLG Docker環境とメインPCの時間計測を統合管理
"""

import json
import time
import os
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any
from .timeTracker_docker import get_time_tracker

class TimingLogger:
    """統合時間計測ロガー"""
    
    def __init__(self):
        self.timing_dir = "/tmp/diaros_timing"
        os.makedirs(self.timing_dir, exist_ok=True)
        
        self.docker_tracker = get_time_tracker("nlg_docker")
        self.session_data = {}
        self.current_session_id = None
        
        print(f"[TimingLogger] 初期化完了: {self.timing_dir}")
    
    def create_timeline_file(self, session_id: str) -> str:
        """タイムライン計測ファイルを作成"""
        timeline_file = os.path.join(self.timing_dir, f"timeline_{session_id}.json")
        
        initial_data = {
            "session_id": session_id,
            "session_type": "nlg_docker",
            "start_time": datetime.now().isoformat(),
            "timeline": [],
            "metadata": {
                "docker_environment": True,
                "node_name": "NLG",
                "pc_name": "nlg_docker"
            }
        }
        
        with open(timeline_file, 'w', encoding='utf-8') as f:
            json.dump(initial_data, f, ensure_ascii=False, indent=2)
        
        return timeline_file
    
    def add_timeline_event(self, session_id: str, event_type: str, data: Optional[Dict] = None):
        """タイムラインイベントを追加"""
        timeline_file = os.path.join(self.timing_dir, f"timeline_{session_id}.json")
        
        if not os.path.exists(timeline_file):
            self.create_timeline_file(session_id)
        
        # イベントデータ作成
        event = {
            "timestamp": datetime.now().isoformat(),
            "timestamp_ns": time.time_ns(),
            "elapsed_ms": 0,  # 後で計算
            "node_name": "NLG",
            "event_type": event_type,
            "data": data or {}
        }
        
        # ファイル読み込み、イベント追加、保存
        try:
            with open(timeline_file, 'r', encoding='utf-8') as f:
                timeline_data = json.load(f)
            
            # 経過時間計算（初回イベントからの相対時間）
            if timeline_data["timeline"]:
                first_event_ns = timeline_data["timeline"][0]["timestamp_ns"]
                event["elapsed_ms"] = (event["timestamp_ns"] - first_event_ns) / 1_000_000
            
            timeline_data["timeline"].append(event)
            
            with open(timeline_file, 'w', encoding='utf-8') as f:
                json.dump(timeline_data, f, ensure_ascii=False, indent=2)

            # print(f"[TimingLogger] イベント記録: {session_id[:8]} - {event_type}")
            
        except Exception as e:
            print(f"[TimingLogger] イベント記録エラー: {e}")

# グローバルインスタンス
_timing_logger_instance = None

def get_timing_logger() -> TimingLogger:
    """統合タイミングロガーのシングルトンインスタンスを取得"""
    global _timing_logger_instance
    
    if _timing_logger_instance is None:
        _timing_logger_instance = TimingLogger()
    
    return _timing_logger_instance

def start_timing_session() -> str:
    """新しいタイミングセッションを開始"""
    session_id = str(uuid.uuid4())[:8]
    
    logger = get_timing_logger()
    logger.current_session_id = session_id
    
    # Docker TimeTrackerにもセッション開始を通知
    logger.docker_tracker.start_session(session_id, "nlg")
    
    # タイムラインファイル作成
    logger.create_timeline_file(session_id)

    # print(f"[TimingIntegration] セッション開始: {session_id}")
    return session_id

def log_nlg_start(session_id: str, dialogue_context: str = ""):
    """NLG処理開始をログ"""
    logger = get_timing_logger()
    
    # Docker TimeTrackerにチェックポイント追加
    logger.docker_tracker.add_checkpoint(session_id, "nlg", "generation_start")
    
    # タイムラインイベント追加
    data = {
        "dialogue_context": dialogue_context,
        "processing_stage": "nlg_start"
    }
    logger.add_timeline_event(session_id, "generation_start", data)

def log_nlg_complete(session_id: str, generated_response: str, processing_time_ms: float):
    """NLG処理完了をログ"""
    logger = get_timing_logger()
    
    # Docker TimeTrackerにチェックポイント追加
    metadata = {
        "generated_response": generated_response,
        "processing_time_ms": processing_time_ms
    }
    logger.docker_tracker.add_checkpoint(session_id, "nlg", "generation_complete", metadata)
    
    # タイムラインイベント追加
    data = {
        "generated_response": generated_response,
        "processing_time_ms": processing_time_ms,
        "response_length": len(generated_response),
        "processing_stage": "nlg_complete"
    }
    logger.add_timeline_event(session_id, "generation_complete", data)

def log_nlg_send(session_id: str, message_data: Dict):
    """NLGメッセージ送信をログ"""
    logger = get_timing_logger()
    
    # Docker TimeTrackerにチェックポイント追加
    logger.docker_tracker.add_checkpoint(session_id, "nlg", "message_sent", message_data)
    
    # タイムラインイベント追加
    logger.add_timeline_event(session_id, "message_sent", message_data)

def end_timing_session(session_id: str):
    """タイミングセッションを終了"""
    logger = get_timing_logger()
    
    # Docker TimeTrackerのセッション終了
    logger.docker_tracker.end_session(session_id, "nlg")
    
    # タイムラインイベント追加
    logger.add_timeline_event(session_id, "session_end", {"final_stage": "nlg_docker_complete"})
    
    print(f"[TimingIntegration] セッション終了: {session_id}")

def get_session_summary(session_id: str) -> Optional[Dict]:
    """セッションサマリーを取得"""
    timeline_file = f"/tmp/diaros_timing/timeline_{session_id}.json"
    
    if not os.path.exists(timeline_file):
        return None
    
    try:
        with open(timeline_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        timeline = data["timeline"]
        if len(timeline) < 2:
            return None
        
        # NLG処理時間計算
        nlg_events = [e for e in timeline if e["node_name"] == "NLG"]
        
        start_event = next((e for e in nlg_events if e["event_type"] == "generation_start"), None)
        complete_event = next((e for e in nlg_events if e["event_type"] == "generation_complete"), None)
        
        if start_event and complete_event:
            nlg_duration = complete_event["elapsed_ms"] - start_event["elapsed_ms"]
            
            return {
                "session_id": session_id,
                "nlg_processing_time_ms": nlg_duration,
                "generated_response": complete_event.get("data", {}).get("generated_response", ""),
                "total_events": len(timeline),
                "session_duration_ms": timeline[-1]["elapsed_ms"]
            }
    
    except Exception as e:
        print(f"[TimingIntegration] サマリー取得エラー: {e}")
    
    return None