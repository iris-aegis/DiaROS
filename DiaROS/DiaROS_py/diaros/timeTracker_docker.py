#!/usr/bin/env python3
"""
DiaROS Docker環境専用時間計測システム
Docker環境ではNTPサーバーアクセスが制限されるため、
ローカル時刻ベースの高精度計測を実装
"""

import time
import json
import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Any
from pathlib import Path
import uuid

@dataclass
class TimingEvent:
    session_id: str
    node_name: str
    pc_name: str
    timestamp_ns: int
    event_type: str
    checkpoint_name: str = ""
    data: Dict[str, Any] = None

class DiaROSDockerTimeTracker:
    """Docker環境専用タイムトラッカー"""
    
    def __init__(self, pc_name: str = "docker_container"):
        self.pc_name = pc_name
        self.sessions: Dict[str, List[TimingEvent]] = {}
        self.lock = threading.Lock()
        self.start_time_ns = time.time_ns()
        
        # Docker環境用設定
        self.timing_dir = Path("/tmp/diaros_timing")
        self.timing_dir.mkdir(exist_ok=True)
        
        print(f"🐳 DiaROS Docker TimeTracker 初期化: {pc_name}")
    
    def get_precise_time_ns(self) -> int:
        """高精度時刻取得（Docker環境用）"""
        return time.time_ns()
    
    def start_session(self, session_id: str, node_name: str, 
                     checkpoint_name: str = "start", data: Dict = None) -> bool:
        """セッション開始"""
        try:
            with self.lock:
                event = TimingEvent(
                    session_id=session_id,
                    node_name=node_name,
                    pc_name=self.pc_name,
                    timestamp_ns=self.get_precise_time_ns(),
                    event_type="start",
                    checkpoint_name=checkpoint_name,
                    data=data or {}
                )
                
                if session_id not in self.sessions:
                    self.sessions[session_id] = []
                
                self.sessions[session_id].append(event)
                
                print(f"🕐 セッション開始: {session_id} @ {node_name}")
                return True
                
        except Exception as e:
            print(f"❌ セッション開始エラー: {e}")
            return False
    
    def add_checkpoint(self, session_id: str, node_name: str, 
                      checkpoint_name: str, data: Dict = None) -> bool:
        """チェックポイント追加"""
        try:
            with self.lock:
                if session_id not in self.sessions:
                    print(f"⚠️ セッション {session_id} が見つかりません")
                    return False
                
                event = TimingEvent(
                    session_id=session_id,
                    node_name=node_name,
                    pc_name=self.pc_name,
                    timestamp_ns=self.get_precise_time_ns(),
                    event_type="checkpoint",
                    checkpoint_name=checkpoint_name,
                    data=data or {}
                )
                
                self.sessions[session_id].append(event)
                
                # 経過時間計算
                start_time = self.sessions[session_id][0].timestamp_ns
                elapsed_ms = (event.timestamp_ns - start_time) / 1_000_000
                
                print(f"📍 チェックポイント: {session_id} @ {node_name}:{checkpoint_name} ({elapsed_ms:.1f}ms)")
                return True
                
        except Exception as e:
            print(f"❌ チェックポイントエラー: {e}")
            return False
    
    def end_session(self, session_id: str, node_name: str, 
                   checkpoint_name: str = "end", data: Dict = None) -> bool:
        """セッション終了"""
        try:
            with self.lock:
                if session_id not in self.sessions:
                    print(f"⚠️ セッション {session_id} が見つかりません")
                    return False
                
                event = TimingEvent(
                    session_id=session_id,
                    node_name=node_name,
                    pc_name=self.pc_name,
                    timestamp_ns=self.get_precise_time_ns(),
                    event_type="end",
                    checkpoint_name=checkpoint_name,
                    data=data or {}
                )
                
                self.sessions[session_id].append(event)
                
                # 総計時間計算
                start_time = self.sessions[session_id][0].timestamp_ns
                total_ms = (event.timestamp_ns - start_time) / 1_000_000
                
                print(f"🏁 セッション終了: {session_id} - 総計時間: {total_ms:.1f}ms")
                
                # タイムライン自動エクスポート
                self.export_timeline(session_id)
                
                return True
                
        except Exception as e:
            print(f"❌ セッション終了エラー: {e}")
            return False
    
    def export_timeline(self, session_id: str, filename: str = None) -> bool:
        """タイムライン出力"""
        try:
            if session_id not in self.sessions:
                print(f"⚠️ セッション {session_id} が見つかりません")
                return False
            
            events = self.sessions[session_id]
            if not events:
                return False
            
            start_time_ns = events[0].timestamp_ns
            timeline = []
            
            for event in events:
                elapsed_ms = (event.timestamp_ns - start_time_ns) / 1_000_000
                timeline.append({
                    "session_id": event.session_id,
                    "node_name": event.node_name,
                    "pc_name": event.pc_name,
                    "timestamp_ns": event.timestamp_ns,
                    "elapsed_ms": elapsed_ms,
                    "event_type": event.event_type,
                    "checkpoint_name": event.checkpoint_name,
                    "data": event.data
                })
            
            # ファイル出力
            if filename is None:
                filename = f"timeline_{session_id}.json"
            
            filepath = self.timing_dir / filename
            
            export_data = {
                "session_id": session_id,
                "pc_name": self.pc_name,
                "export_time": time.time(),
                "timeline": timeline
            }
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=2, ensure_ascii=False)
            
            print(f"📁 タイムライン出力: {filepath}")
            return True
            
        except Exception as e:
            print(f"❌ タイムライン出力エラー: {e}")
            return False
    
    def get_session_timeline(self, session_id: str) -> Optional[List[TimingEvent]]:
        """セッションタイムライン取得"""
        with self.lock:
            return self.sessions.get(session_id, None)
    
    def get_session_summary(self, session_id: str) -> Optional[Dict]:
        """セッション概要取得"""
        events = self.get_session_timeline(session_id)
        if not events:
            return None
        
        start_time = events[0].timestamp_ns
        end_time = events[-1].timestamp_ns
        total_ms = (end_time - start_time) / 1_000_000
        
        return {
            "session_id": session_id,
            "total_ms": total_ms,
            "event_count": len(events),
            "start_node": events[0].node_name,
            "end_node": events[-1].node_name
        }
    
    def print_session_summary(self, session_id: str):
        """セッション概要表示"""
        summary = self.get_session_summary(session_id)
        if not summary:
            print(f"❌ セッション {session_id} が見つかりません")
            return
        
        print(f"""
📊 セッション概要: {session_id}
総計時間: {summary['total_ms']:.1f}ms
イベント数: {summary['event_count']}
開始ノード: {summary['start_node']}
終了ノード: {summary['end_node']}
""")

# グローバルインスタンス
_docker_tracker_instance = None
_docker_tracker_lock = threading.Lock()

def get_time_tracker(pc_name: str = "docker_container") -> DiaROSDockerTimeTracker:
    """Docker環境用タイムトラッカー取得"""
    global _docker_tracker_instance
    
    with _docker_tracker_lock:
        if _docker_tracker_instance is None:
            _docker_tracker_instance = DiaROSDockerTimeTracker(pc_name)
        return _docker_tracker_instance

# 便利関数
def start_timing(session_id: str, node_name: str, checkpoint_name: str = "start", data: Dict = None):
    """計測開始"""
    tracker = get_time_tracker()
    return tracker.start_session(session_id, node_name, checkpoint_name, data)

def checkpoint(session_id: str, node_name: str, checkpoint_name: str, data: Dict = None):
    """チェックポイント追加"""
    tracker = get_time_tracker()
    return tracker.add_checkpoint(session_id, node_name, checkpoint_name, data)

def end_timing(session_id: str, node_name: str, checkpoint_name: str = "end", data: Dict = None):
    """計測終了"""
    tracker = get_time_tracker()
    return tracker.end_session(session_id, node_name, checkpoint_name, data)

if __name__ == "__main__":
    # テスト実行
    print("🧪 DiaROS Docker TimeTracker テスト")
    
    tracker = get_time_tracker("test_container")
    
    # テストセッション
    session_id = f"test_{int(time.time() * 1000)}"
    
    tracker.start_session(session_id, "speech_input")
    time.sleep(0.1)
    tracker.add_checkpoint(session_id, "asr", "recognition_complete")
    time.sleep(0.2)
    tracker.add_checkpoint(session_id, "dm", "dialogue_complete")
    time.sleep(0.3)
    tracker.end_session(session_id, "ss")
    
    tracker.print_session_summary(session_id)