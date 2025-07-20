#!/usr/bin/env python3
"""
DiaROS統合時間計測システム
分散マルチプロセス環境での総合計時間計測
"""

import time
import json
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import threading
import queue

@dataclass
class TimeEvent:
    """時刻イベント情報"""
    event_id: str
    event_type: str  # 'start', 'process', 'end'
    node_name: str
    pc_name: str
    timestamp_ns: int
    local_time: str
    data: Dict = None

class DiaROSTimeTracker:
    """DiaROS統合時間計測器"""
    
    def __init__(self, pc_name: str = "unknown"):
        self.pc_name = pc_name
        self.events: List[TimeEvent] = []
        self.active_sessions: Dict[str, TimeEvent] = {}
        self.lock = threading.Lock()
        
        # 時刻同期補正値（NTPサーバーとの差分）
        self.time_offset_ns = 0
        
    def sync_time_with_ntp(self):
        """NTPサーバーとの時刻同期（オプション）"""
        try:
            import ntplib
            client = ntplib.NTPClient()
            response = client.request('pool.ntp.org')
            ntp_time = response.tx_time
            local_time = time.time()
            self.time_offset_ns = int((ntp_time - local_time) * 1_000_000_000)
            print(f"時刻同期完了: オフセット {self.time_offset_ns/1_000_000:.2f}ms")
        except Exception as e:
            print(f"時刻同期失敗: {e}")
            self.time_offset_ns = 0
    
    def get_synchronized_time_ns(self) -> int:
        """同期済み時刻を取得（ナノ秒）"""
        return int(time.time() * 1_000_000_000) + self.time_offset_ns
    
    def start_session(self, session_id: str, node_name: str) -> str:
        """セッション開始"""
        with self.lock:
            event = TimeEvent(
                event_id=session_id,
                event_type='start',
                node_name=node_name,
                pc_name=self.pc_name,
                timestamp_ns=self.get_synchronized_time_ns(),
                local_time=datetime.now().strftime('%H:%M:%S.%f')[:-3]
            )
            self.events.append(event)
            self.active_sessions[session_id] = event
            return session_id
    
    def add_checkpoint(self, session_id: str, node_name: str, checkpoint_name: str, data: Dict = None):
        """チェックポイント追加"""
        with self.lock:
            event = TimeEvent(
                event_id=f"{session_id}_{checkpoint_name}",
                event_type='process',
                node_name=node_name,
                pc_name=self.pc_name,
                timestamp_ns=self.get_synchronized_time_ns(),
                local_time=datetime.now().strftime('%H:%M:%S.%f')[:-3],
                data=data
            )
            self.events.append(event)
    
    def end_session(self, session_id: str, node_name: str):
        """セッション終了"""
        with self.lock:
            event = TimeEvent(
                event_id=f"{session_id}_end",
                event_type='end',
                node_name=node_name,
                pc_name=self.pc_name,
                timestamp_ns=self.get_synchronized_time_ns(),
                local_time=datetime.now().strftime('%H:%M:%S.%f')[:-3]
            )
            self.events.append(event)
            
            # 総計時間計算
            if session_id in self.active_sessions:
                start_event = self.active_sessions[session_id]
                total_time_ms = (event.timestamp_ns - start_event.timestamp_ns) / 1_000_000
                
                print(f"🕐 [総計時間] セッション {session_id}: {total_time_ms:.1f}ms")
                print(f"   開始: {start_event.local_time} ({start_event.node_name}@{start_event.pc_name})")
                print(f"   終了: {event.local_time} ({event.node_name}@{event.pc_name})")
                
                del self.active_sessions[session_id]
    
    def get_session_timeline(self, session_id: str) -> List[TimeEvent]:
        """セッションの時系列データ取得"""
        with self.lock:
            return [e for e in self.events if e.event_id.startswith(session_id)]
    
    def export_timeline(self, session_id: str, filename: str = None):
        """タイムライン出力"""
        timeline = self.get_session_timeline(session_id)
        
        if not timeline:
            print(f"セッション {session_id} が見つかりません")
            return
        
        # 時系列ソート
        timeline.sort(key=lambda x: x.timestamp_ns)
        
        # 詳細出力
        print(f"\n📊 セッション {session_id} 詳細タイムライン:")
        print("=" * 80)
        
        start_time = timeline[0].timestamp_ns
        for i, event in enumerate(timeline):
            elapsed_ms = (event.timestamp_ns - start_time) / 1_000_000
            print(f"{i+1:2d}. [{event.local_time}] {event.node_name}@{event.pc_name}")
            print(f"    イベント: {event.event_type} | 経過時間: {elapsed_ms:.1f}ms")
            if event.data:
                print(f"    データ: {event.data}")
        
        # JSON出力
        if filename:
            export_data = {
                'session_id': session_id,
                'timeline': [
                    {
                        'event_id': e.event_id,
                        'event_type': e.event_type,
                        'node_name': e.node_name,
                        'pc_name': e.pc_name,
                        'timestamp_ns': e.timestamp_ns,
                        'local_time': e.local_time,
                        'elapsed_ms': (e.timestamp_ns - start_time) / 1_000_000,
                        'data': e.data
                    } for e in timeline
                ]
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=2, ensure_ascii=False)
            print(f"\n📁 タイムライン出力: {filename}")

# グローバルインスタンス
_time_tracker = None

def get_time_tracker(pc_name: str = "unknown") -> DiaROSTimeTracker:
    """時間計測器のシングルトンインスタンス取得"""
    global _time_tracker
    if _time_tracker is None:
        _time_tracker = DiaROSTimeTracker(pc_name)
    return _time_tracker

def start_timing(session_id: str, node_name: str) -> str:
    """計測開始"""
    tracker = get_time_tracker()
    return tracker.start_session(session_id, node_name)

def checkpoint(session_id: str, node_name: str, checkpoint_name: str, data: Dict = None):
    """チェックポイント"""
    tracker = get_time_tracker()
    tracker.add_checkpoint(session_id, node_name, checkpoint_name, data)

def end_timing(session_id: str, node_name: str):
    """計測終了"""
    tracker = get_time_tracker()
    tracker.end_session(session_id, node_name)

def export_session(session_id: str, filename: str = None):
    """セッション出力"""
    tracker = get_time_tracker()
    tracker.export_timeline(session_id, filename)

if __name__ == "__main__":
    # テスト用
    tracker = DiaROSTimeTracker("test_pc")
    
    session_id = "test_session_001"
    tracker.start_session(session_id, "speech_input")
    
    time.sleep(0.1)
    tracker.add_checkpoint(session_id, "asr", "recognition_complete", {"text": "こんにちは"})
    
    time.sleep(0.2)
    tracker.add_checkpoint(session_id, "nlg", "generation_complete", {"response": "こんにちは！"})
    
    time.sleep(0.05)
    tracker.end_session(session_id, "speech_synthesis")
    
    tracker.export_timeline(session_id, "test_timeline.json")