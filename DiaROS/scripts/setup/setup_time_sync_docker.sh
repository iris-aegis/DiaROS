#!/bin/bash
# DiaROS Docker環境用時刻同期セットアップ

echo "🐳 DiaROS Docker環境時刻同期セットアップ"
echo "========================================="

# 1. 必要パッケージのインストール
echo "1. 必要パッケージインストール中..."
apt-get update -qq
apt-get install -y python3-pip curl

# 2. Python時刻同期ライブラリインストール
echo "2. Python時刻同期ライブラリインストール中..."
pip3 install ntplib

# 3. Docker用時刻同期テスト
echo "3. Docker環境時刻同期テスト実行中..."
python3 << 'EOF'
import time
import ntplib
from datetime import datetime

try:
    # NTPサーバーとの時刻差を計算
    client = ntplib.NTPClient()
    response = client.request('ntp.nict.jp', timeout=10)
    
    ntp_time = response.tx_time
    local_time = time.time()
    offset_ms = (ntp_time - local_time) * 1000
    
    print(f"✅ Docker環境時刻同期テスト完了")
    print(f"   コンテナ時刻: {datetime.fromtimestamp(local_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"   NTP時刻: {datetime.fromtimestamp(ntp_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"   オフセット: {offset_ms:.2f}ms")
    
    # Docker環境での時刻同期状況を判定
    if abs(offset_ms) < 50:
        print("   🟢 Docker環境での時刻同期は良好です")
    elif abs(offset_ms) < 200:
        print("   🟡 Docker環境での時刻同期は許容範囲内です")
    else:
        print("   🔴 Docker環境での時刻同期に問題があります")
        print("   💡 ホストマシンでの時刻同期を確認してください")
        
except Exception as e:
    print(f"❌ 時刻同期テスト失敗: {e}")
    print("💡 ネットワーク接続またはファイアウォール設定を確認してください")
EOF

# 4. DiaROS時間計測システム用の環境変数設定
echo "4. DiaROS時間計測環境変数設定中..."
export DIAROS_TIMING_ENABLED=true
export DIAROS_TIMING_NTP_SERVER="ntp.nict.jp"

# 5. 時間計測ディレクトリ作成
echo "5. 時間計測ディレクトリ作成中..."
mkdir -p /tmp/diaros_timing
chmod 777 /tmp/diaros_timing

echo ""
echo "🎉 Docker環境時刻同期セットアップ完了"
echo "========================================="
echo "📋 Docker環境での注意事項:"
echo "1. Docker コンテナの時刻はホストマシンに依存"
echo "2. ホストマシンでNTP同期が有効であることを確認"
echo "3. 分散環境では各ホストマシンで時刻同期設定"
echo "4. DiaROS時間計測システムはNTPオフセット補正機能付き"
echo ""
echo "📝 次の手順:"
echo "1. 他のPCでも同様の設定を実行"
echo "2. DiaROS時間計測システムを起動"
echo "3. bash /workspace/DiaROS/scripts/launch/launch_diaros_no_speech_input_simple.sh"
echo ""