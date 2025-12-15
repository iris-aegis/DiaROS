#!/usr/bin/env python3
"""
DiaROS分散時間計測システム - Docker環境専用タイムトラッカー
Docker環境での高精度な時間計測とセッション管理を提供
"""

import json
import time
import threading
from datetime import datetime
from typing import Dict, List, Optional, Any
import uuid
import os

class DockerTimeTracker:
    """Docker環境での高精度時間計測を管理するクラス"""
    
    def __init__(self, pc_name: str):
        self.pc_name = f"{pc_name}_docker"
        self.sessions: Dict[str, Dict] = {}
        self.lock = threading.Lock()
        
        # Docker環境での出力ディレクトリ設定
        self.output_dir = "/workspace/logs"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 出力ファイルの設定
        self.timing_file = os.path.join(self.output_dir, f"diaros_timing_{self.pc_name}.json")
        
        # Docker環境情報を記録
        self.docker_info = {
            "container_id": os.environ.get("HOSTNAME", "unknown"),
            "is_docker": os.path.exists("/.dockerenv"),
            "output_dir": self.output_dir
        }
        
        print(f"[DockerTimeTracker] 初期化完了: PC={self.pc_name}")
        print(f"[DockerTimeTracker] Docker環境: {self.docker_info['is_docker']}")
        print(f"[DockerTimeTracker] 出力ファイル: {self.timing_file}")
    
    def start_session(self, session_id: str, session_type: str = "dialogue"):
        """セッションを開始"""
        with self.lock:
            self.sessions[session_id] = {
                "session_id": session_id,
                "session_type": session_type,
                "pc_name": self.pc_name,
                "start_time_ns": time.time_ns(),
                "start_time_iso": datetime.now().isoformat(),
                "checkpoints": [],
                "docker_info": self.docker_info,
                "status": "active"
            }
        
        # timestamp_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        # print(f"[DockerTimeTracker] セッション開始: {session_id} @ {timestamp_str}")
    
    def add_checkpoint(self, session_id: str, component: str, event: str, metadata: Optional[Dict] = None):
        """チェックポイントを追加（Docker環境最適化）"""
        timestamp_ns = time.time_ns()
        
        with self.lock:
            if session_id not in self.sessions:
                print(f"[DockerTimeTracker] 警告: セッションID {session_id} が見つかりません")
                return
            
            # セッション開始時刻からの相対時間を計算（Docker環境では相対精度が重要）
            start_time_ns = self.sessions[session_id]["start_time_ns"]
            relative_time_ns = timestamp_ns - start_time_ns
            
            checkpoint = {
                "component": component,
                "event": event,
                "timestamp_ns": timestamp_ns,
                "relative_time_ns": relative_time_ns,
                "relative_time_ms": relative_time_ns / 1_000_000,
                "pc_name": self.pc_name,
                "metadata": metadata or {}
            }
            
            self.sessions[session_id]["checkpoints"].append(checkpoint)
            
            # デバッグ出力（相対時間も表示）
            # timestamp_str = datetime.fromtimestamp(timestamp_ns / 1_000_000_000).strftime('%H:%M:%S.%f')[:-3]
            # print(f"[DockerTimeTracker] チェックポイント: {session_id[:8]} - {component}.{event}")
            # print(f"                      絶対時刻: {timestamp_str}, 相対時間: {relative_time_ns/1_000_000:.3f}ms")
    
    def end_session(self, session_id: str, component: str):
        """セッションを終了"""
        timestamp_ns = time.time_ns()
        
        with self.lock:
            if session_id not in self.sessions:
                print(f"[DockerTimeTracker] 警告: セッションID {session_id} が見つかりません")
                return
            
            # 終了チェックポイントを追加
            self.add_checkpoint(session_id, component, "session_end")
            
            # セッション情報を更新
            session = self.sessions[session_id]
            session["end_time_ns"] = timestamp_ns
            session["end_time_iso"] = datetime.now().isoformat()
            session["total_duration_ns"] = timestamp_ns - session["start_time_ns"]
            session["total_duration_ms"] = session["total_duration_ns"] / 1_000_000
            session["status"] = "completed"
            
            # ファイルに保存
            self.save_session(session_id)
            
            timestamp_str = datetime.fromtimestamp(timestamp_ns / 1_000_000_000).strftime('%H:%M:%S.%f')[:-3]
            print(f"[DockerTimeTracker] セッション終了: {session_id} @ {timestamp_str}")
            print(f"                      総処理時間: {session['total_duration_ms']:.3f}ms")
    
    def get_session_data(self, session_id: str) -> Optional[Dict]:
        """セッションデータを取得"""
        with self.lock:
            return self.sessions.get(session_id)
    
    def save_session(self, session_id: str):
        """セッションデータをファイルに保存（Docker環境対応）"""
        with self.lock:
            if session_id not in self.sessions:
                return
            
            session_data = self.sessions[session_id].copy()
            session_data["saved_at"] = datetime.now().isoformat()
            
            # Docker環境での永続化
            try:
                with open(self.timing_file, 'a', encoding='utf-8') as f:
                    json.dump(session_data, f, ensure_ascii=False, indent=2)
                    f.write('\n')
                
                print(f"[DockerTimeTracker] セッション保存完了: {session_id}")
                print(f"                      保存先: {self.timing_file}")
            except Exception as e:
                print(f"[DockerTimeTracker] セッション保存エラー: {e}")
    
    def cleanup_session(self, session_id: str):
        """セッションをクリーンアップ"""
        with self.lock:
            if session_id in self.sessions:
                del self.sessions[session_id]
                print(f"[DockerTimeTracker] セッションクリーンアップ: {session_id}")
    
    def get_precision_info(self) -> Dict:
        """Docker環境での時刻精度情報を取得"""
        # 時刻精度テスト
        times = []
        for _ in range(100):
            times.append(time.time_ns())
        
        diffs = [times[i+1] - times[i] for i in range(len(times)-1) if times[i+1] > times[i]]
        
        if diffs:
            min_diff = min(diffs)
            avg_diff = sum(diffs) / len(diffs)
        else:
            min_diff = avg_diff = 0
        
        return {
            "clock_resolution": time.get_clock_info('time').resolution,
            "min_time_diff_ns": min_diff,
            "avg_time_diff_ns": avg_diff,
            "min_time_diff_ms": min_diff / 1_000_000,
            "avg_time_diff_ms": avg_diff / 1_000_000,
            "docker_environment": self.docker_info
        }

# グローバルインスタンス管理（Docker環境専用）
_docker_time_tracker_instance = None
_docker_time_tracker_lock = threading.Lock()

def get_time_tracker(pc_name: str) -> DockerTimeTracker:
    """Docker環境用タイムトラッカーのシングルトンインスタンスを取得"""
    global _docker_time_tracker_instance
    
    with _docker_time_tracker_lock:
        if _docker_time_tracker_instance is None:
            _docker_time_tracker_instance = DockerTimeTracker(pc_name)
        return _docker_time_tracker_instance

def start_session(session_id: str, session_type: str = "dialogue"):
    """新しいセッションを開始（便利関数）"""
    tracker = get_time_tracker("nlg")
    tracker.start_session(session_id, session_type)

def add_checkpoint(session_id: str, component: str, event: str, metadata: Optional[Dict] = None):
    """チェックポイントを追加（便利関数）"""
    tracker = get_time_tracker("nlg")
    tracker.add_checkpoint(session_id, component, event, metadata)

def end_session(session_id: str, component: str):
    """セッションを終了（便利関数）"""
    tracker = get_time_tracker("nlg")
    tracker.end_session(session_id, component)