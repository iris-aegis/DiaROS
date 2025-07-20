#!/usr/bin/env python3
"""
DiaROS時間計測可視化ツール
総合計時間の分析・可視化
"""

import json
import sys
import os
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
from pathlib import Path

# 日本語フォント設定
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

class TimingVisualizer:
    def __init__(self):
        self.sessions = {}
        
    def load_timeline(self, filename: str):
        """タイムラインJSONファイルを読み込み"""
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            session_id = data['session_id']
            self.sessions[session_id] = data['timeline']
            print(f"✅ セッション {session_id} を読み込みました")
            return session_id
        except Exception as e:
            print(f"❌ ファイル読み込みエラー: {e}")
            return None
    
    def analyze_session(self, session_id: str):
        """セッション分析"""
        if session_id not in self.sessions:
            print(f"❌ セッション {session_id} が見つかりません")
            return
        
        timeline = self.sessions[session_id]
        
        # 基本統計
        total_time = timeline[-1]['elapsed_ms']
        num_events = len(timeline)
        
        # 各段階の処理時間
        stages = {}
        for i in range(len(timeline) - 1):
            current = timeline[i]
            next_event = timeline[i + 1]
            
            stage_name = f"{current['node_name']}→{next_event['node_name']}"
            duration = next_event['elapsed_ms'] - current['elapsed_ms']
            stages[stage_name] = duration
        
        # 結果出力
        print(f"\n📊 セッション {session_id} 分析結果:")
        print("=" * 50)
        print(f"総計時間: {total_time:.1f}ms")
        print(f"イベント数: {num_events}")
        print(f"平均段階時間: {total_time/max(1, num_events-1):.1f}ms")
        
        print("\n🔍 各段階の処理時間:")
        for stage, duration in stages.items():
            percentage = (duration / total_time) * 100
            print(f"  {stage}: {duration:.1f}ms ({percentage:.1f}%)")
        
        # ボトルネック特定
        if stages:
            bottleneck = max(stages.items(), key=lambda x: x[1])
            print(f"\n⚠️  ボトルネック: {bottleneck[0]} ({bottleneck[1]:.1f}ms)")
        
        return {
            'total_time': total_time,
            'stages': stages,
            'timeline': timeline
        }
    
    def plot_timeline(self, session_id: str, save_file: str = None):
        """タイムライン可視化"""
        if session_id not in self.sessions:
            print(f"❌ セッション {session_id} が見つかりません")
            return
        
        timeline = self.sessions[session_id]
        
        # データ準備
        nodes = [event['node_name'] for event in timeline]
        times = [event['elapsed_ms'] for event in timeline]
        pcs = [event['pc_name'] for event in timeline]
        
        # 色分け（PC別）
        unique_pcs = list(set(pcs))
        colors = plt.cm.Set3(np.linspace(0, 1, len(unique_pcs)))
        pc_colors = {pc: colors[i] for i, pc in enumerate(unique_pcs)}
        
        # グラフ作成
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
        
        # 1. タイムライン（時系列）
        for i, (node, time_ms, pc) in enumerate(zip(nodes, times, pcs)):
            ax1.barh(i, time_ms, color=pc_colors[pc], alpha=0.7)
            ax1.text(time_ms + 5, i, f"{node}@{pc}", 
                    va='center', fontsize=9)
        
        ax1.set_xlabel('経過時間 (ms)')
        ax1.set_ylabel('処理順序')
        ax1.set_title(f'DiaROS処理タイムライン - セッション {session_id}')
        ax1.grid(True, alpha=0.3)
        
        # 2. 各段階の処理時間
        stages = {}
        for i in range(len(timeline) - 1):
            current = timeline[i]
            next_event = timeline[i + 1]
            stage_name = f"{current['node_name']}→{next_event['node_name']}"
            duration = next_event['elapsed_ms'] - current['elapsed_ms']
            stages[stage_name] = duration
        
        if stages:
            stage_names = list(stages.keys())
            durations = list(stages.values())
            
            bars = ax2.bar(range(len(stage_names)), durations, 
                          color='skyblue', alpha=0.7)
            ax2.set_xlabel('処理段階')
            ax2.set_ylabel('処理時間 (ms)')
            ax2.set_title('各段階の処理時間')
            ax2.set_xticks(range(len(stage_names)))
            ax2.set_xticklabels(stage_names, rotation=45, ha='right')
            
            # 値をバーの上に表示
            for bar, duration in zip(bars, durations):
                ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                        f'{duration:.1f}ms', ha='center', va='bottom')
        
        plt.tight_layout()
        
        if save_file:
            plt.savefig(save_file, dpi=300, bbox_inches='tight')
            print(f"📁 グラフ保存: {save_file}")
        
        plt.show()
    
    def plot_comparison(self, session_ids: list, save_file: str = None):
        """複数セッションの比較"""
        valid_sessions = [sid for sid in session_ids if sid in self.sessions]
        
        if not valid_sessions:
            print("❌ 有効なセッションがありません")
            return
        
        # 総計時間比較
        total_times = []
        labels = []
        
        for session_id in valid_sessions:
            timeline = self.sessions[session_id]
            total_time = timeline[-1]['elapsed_ms']
            total_times.append(total_time)
            labels.append(f"Session {session_id}")
        
        # グラフ作成
        fig, ax = plt.subplots(figsize=(10, 6))
        
        bars = ax.bar(labels, total_times, color='lightcoral', alpha=0.7)
        ax.set_ylabel('総計時間 (ms)')
        ax.set_title('セッション別総計時間比較')
        ax.grid(True, alpha=0.3)
        
        # 平均線
        avg_time = np.mean(total_times)
        ax.axhline(y=avg_time, color='red', linestyle='--', 
                  label=f'平均: {avg_time:.1f}ms')
        
        # 値をバーの上に表示
        for bar, time_ms in zip(bars, total_times):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 10,
                   f'{time_ms:.1f}ms', ha='center', va='bottom')
        
        ax.legend()
        plt.xticks(rotation=45, ha='right')
        plt.tight_layout()
        
        if save_file:
            plt.savefig(save_file, dpi=300, bbox_inches='tight')
            print(f"📁 比較グラフ保存: {save_file}")
        
        plt.show()
    
    def generate_report(self, session_id: str, output_file: str = None):
        """詳細レポート生成"""
        analysis = self.analyze_session(session_id)
        if not analysis:
            return
        
        timeline = analysis['timeline']
        total_time = analysis['total_time']
        stages = analysis['stages']
        
        # レポート生成
        report = f"""
# DiaROS音声対話システム性能レポート

## 基本情報
- **セッションID**: {session_id}
- **総計時間**: {total_time:.1f}ms
- **測定日時**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

## 処理フロー
"""
        
        for i, event in enumerate(timeline):
            report += f"{i+1}. **{event['node_name']}@{event['pc_name']}** "
            report += f"({event['elapsed_ms']:.1f}ms) - {event['event_type']}\n"
        
        report += "\n## 各段階の処理時間\n"
        for stage, duration in stages.items():
            percentage = (duration / total_time) * 100
            report += f"- **{stage}**: {duration:.1f}ms ({percentage:.1f}%)\n"
        
        # 性能評価
        report += "\n## 性能評価\n"
        if total_time < 1000:
            report += "✅ **優秀**: 1秒以内で応答完了\n"
        elif total_time < 2000:
            report += "🟡 **良好**: 2秒以内で応答完了\n"
        else:
            report += "🔴 **要改善**: 応答時間が長すぎます\n"
        
        # 改善提案
        if stages:
            bottleneck = max(stages.items(), key=lambda x: x[1])
            report += f"\n## 改善提案\n"
            report += f"- **ボトルネック**: {bottleneck[0]} ({bottleneck[1]:.1f}ms)\n"
            report += f"- この段階の最適化を優先的に検討してください\n"
        
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f"📁 レポート保存: {output_file}")
        
        print(report)

def main():
    """メイン関数"""
    if len(sys.argv) < 2:
        print("使用方法: python timing_visualizer.py <timeline.json> [command]")
        print("コマンド:")
        print("  analyze  - 分析結果表示")
        print("  plot     - グラフ表示")
        print("  report   - 詳細レポート生成")
        return
    
    visualizer = TimingVisualizer()
    
    # タイムラインファイル読み込み
    timeline_file = sys.argv[1]
    session_id = visualizer.load_timeline(timeline_file)
    
    if not session_id:
        return
    
    # コマンド実行
    command = sys.argv[2] if len(sys.argv) > 2 else "analyze"
    
    if command == "analyze":
        visualizer.analyze_session(session_id)
    elif command == "plot":
        visualizer.plot_timeline(session_id, f"timeline_{session_id}.png")
    elif command == "report":
        visualizer.generate_report(session_id, f"report_{session_id}.md")
    else:
        print(f"❌ 不明なコマンド: {command}")

if __name__ == "__main__":
    main()