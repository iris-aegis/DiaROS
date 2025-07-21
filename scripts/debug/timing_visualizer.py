#!/usr/bin/env python3
"""
DiaROS統合タイムスタンプシステムのデータ可視化ツール
"""

import sys
import json
import argparse
from datetime import datetime
from pathlib import Path

def load_timing_data(file_path):
    """タイミングデータを読み込み"""
    sessions = []
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                if line.strip():
                    sessions.append(json.loads(line))
        return sessions
    except Exception as e:
        print(f"❌ ファイル読み込みエラー: {e}")
        return []

def analyze_session(session):
    """セッションの分析"""
    checkpoints = session.get('checkpoints', [])
    if not checkpoints:
        return {}
    
    # 時刻の計算
    start_time = min(cp['timestamp_ns'] for cp in checkpoints)
    end_time = max(cp['timestamp_ns'] for cp in checkpoints)
    total_duration = (end_time - start_time) / 1_000_000  # ms
    
    # コンポーネント別の時間分析
    component_times = {}
    for i, cp in enumerate(checkpoints):
        component = cp['component']
        event = cp['event']
        
        if component not in component_times:
            component_times[component] = {}
        
        if event.endswith('_start'):
            component_times[component]['start'] = cp['timestamp_ns']
        elif event.endswith('_complete'):
            component_times[component]['end'] = cp['timestamp_ns']
    
    # 各コンポーネントの処理時間を計算
    durations = {}
    for comp, times in component_times.items():
        if 'start' in times and 'end' in times:
            duration = (times['end'] - times['start']) / 1_000_000  # ms
            durations[comp] = duration
    
    return {
        'session_id': session['session_id'],
        'total_duration_ms': total_duration,
        'component_durations': durations,
        'checkpoint_count': len(checkpoints),
        'pc_name': session.get('pc_name', 'unknown')
    }

def print_timeline(sessions):
    """タイムライン表示"""
    print("\n🕐 タイムライン分析")
    print("=" * 80)
    
    for session in sessions:
        checkpoints = session.get('checkpoints', [])
        if not checkpoints:
            continue
            
        print(f"\nセッション: {session['session_id'][:8]}... (PC: {session.get('pc_name', 'unknown')})")
        print("-" * 60)
        
        # 開始時刻を基準とする
        base_time = min(cp['timestamp_ns'] for cp in checkpoints)
        
        for cp in checkpoints:
            rel_time = (cp['timestamp_ns'] - base_time) / 1_000_000  # ms
            timestamp = datetime.fromtimestamp(cp['timestamp_ns'] / 1_000_000_000)
            print(f"  +{rel_time:6.1f}ms | {cp['component']}.{cp['event']} @ {timestamp.strftime('%H:%M:%S.%f')[:-3]}")

def print_summary(sessions):
    """サマリー表示"""
    print("\n📊 処理時間サマリー")
    print("=" * 80)
    
    all_durations = {}
    
    for session in sessions:
        analysis = analyze_session(session)
        
        print(f"\nセッション: {analysis['session_id'][:8]}... (PC: {analysis['pc_name']})")
        print(f"  総処理時間: {analysis['total_duration_ms']:.1f}ms")
        print(f"  チェックポイント数: {analysis['checkpoint_count']}")
        
        for comp, duration in analysis['component_durations'].items():
            print(f"  {comp}: {duration:.1f}ms")
            
            if comp not in all_durations:
                all_durations[comp] = []
            all_durations[comp].append(duration)
    
    # 全体統計
    if all_durations:
        print(f"\n📈 全体統計 ({len(sessions)} sessions)")
        print("-" * 60)
        
        for comp, durations in all_durations.items():
            avg_duration = sum(durations) / len(durations)
            min_duration = min(durations)
            max_duration = max(durations)
            
            print(f"  {comp}:")
            print(f"    平均: {avg_duration:.1f}ms")
            print(f"    最小: {min_duration:.1f}ms")
            print(f"    最大: {max_duration:.1f}ms")

def main():
    parser = argparse.ArgumentParser(description='DiaROS統合タイムスタンプシステムの可視化')
    parser.add_argument('file', help='タイミングデータファイル')
    parser.add_argument('mode', choices=['timeline', 'summary', 'both'], 
                       help='表示モード')
    
    args = parser.parse_args()
    
    # データの読み込み
    sessions = load_timing_data(args.file)
    
    if not sessions:
        print("❌ データが見つかりませんでした")
        return
    
    print(f"✅ {len(sessions)} セッションのデータを読み込みました")
    
    # 表示モードに応じて処理
    if args.mode in ['timeline', 'both']:
        print_timeline(sessions)
    
    if args.mode in ['summary', 'both']:
        print_summary(sessions)
    
    print(f"\n🎉 分析完了! ({len(sessions)} sessions)")

if __name__ == "__main__":
    main()