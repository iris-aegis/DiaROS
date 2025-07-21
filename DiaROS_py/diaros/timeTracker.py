#!/usr/bin/env python3
"""
DiaROS分散時間計測システム - タイムトラッカー
高精度な時間計測とセッション管理を提供
"""

import json
import time
import threading
from datetime import datetime
from typing import Dict, List, Optional, Any
import uuid
import os

class TimeTracker:
    """分散環境での高精度時間計測を管理するクラス"""
    
    def __init__(self, pc_name: str):
        self.pc_name = pc_name
        self.sessions: Dict[str, Dict] = {}
        self.lock = threading.Lock()
        self.output_dir = "/tmp"
        
        # 出力ファイルの設定
        self.timing_file = os.path.join(self.output_dir, f"diaros_timing_{pc_name}.json")
        
        print(f"[TimeTracker] 初期化完了: PC={pc_name}, 出力ファイル={self.timing_file}")
    
    def create_session(self, session_type: str = "dialogue") -> str:
        """新しいセッションを作成"""
        session_id = str(uuid.uuid4())
        
        with self.lock:
            self.sessions[session_id] = {
                "session_id": session_id,
                "session_type": session_type,
                "pc_name": self.pc_name,
                "start_time": time.time_ns(),
                "checkpoints": [],
                "created_at": datetime.now().isoformat()
            }
        
        print(f"[TimeTracker] セッション作成: {session_id}")
        return session_id
    
    def add_checkpoint(self, session_id: str, component: str, event: str, metadata: Optional[Dict] = None):
        """チェックポイントを追加"""
        timestamp_ns = time.time_ns()
        
        with self.lock:
            if session_id not in self.sessions:
                print(f"[TimeTracker] 警告: セッションID {session_id} が見つかりません")
                return
            
            checkpoint = {
                "component": component,
                "event": event,
                "timestamp_ns": timestamp_ns,
                "pc_name": self.pc_name,
                "metadata": metadata or {}
            }
            
            self.sessions[session_id]["checkpoints"].append(checkpoint)
            
            # デバッグ出力
            timestamp_str = datetime.fromtimestamp(timestamp_ns / 1_000_000_000).strftime('%H:%M:%S.%f')[:-3]
            print(f"[TimeTracker] チェックポイント追加: {session_id[:8]} - {component}.{event} @ {timestamp_str}")
    
    def get_session_data(self, session_id: str) -> Optional[Dict]:
        """セッションデータを取得"""
        with self.lock:
            return self.sessions.get(session_id)
    
    def save_session(self, session_id: str):
        """セッションデータをファイルに保存"""
        with self.lock:
            if session_id not in self.sessions:
                return
            
            session_data = self.sessions[session_id].copy()
            session_data["completed_at"] = datetime.now().isoformat()
            
            # ファイルに追記
            try:
                with open(self.timing_file, 'a', encoding='utf-8') as f:
                    json.dump(session_data, f, ensure_ascii=False)
                    f.write('\n')
                
                print(f"[TimeTracker] セッション保存完了: {session_id}")
            except Exception as e:
                print(f"[TimeTracker] セッション保存エラー: {e}")
    
    def cleanup_session(self, session_id: str):
        """セッションをクリーンアップ"""
        with self.lock:
            if session_id in self.sessions:
                del self.sessions[session_id]
                print(f"[TimeTracker] セッションクリーンアップ: {session_id}")

# グローバルインスタンス管理
_time_tracker_instance = None
_time_tracker_lock = threading.Lock()

def get_time_tracker(pc_name: str) -> TimeTracker:
    """タイムトラッカーのシングルトンインスタンスを取得"""
    global _time_tracker_instance
    
    with _time_tracker_lock:
        if _time_tracker_instance is None:
            _time_tracker_instance = TimeTracker(pc_name)
        return _time_tracker_instance

def create_session(session_type: str = "dialogue") -> str:
    """新しいセッションを作成（便利関数）"""
    tracker = get_time_tracker("default")
    return tracker.create_session(session_type)

def add_checkpoint(session_id: str, component: str, event: str, metadata: Optional[Dict] = None):
    """チェックポイントを追加（便利関数）"""
    tracker = get_time_tracker("default")
    tracker.add_checkpoint(session_id, component, event, metadata)