#!/bin/bash
# DiaROS分散環境時刻同期セットアップ

echo "🕐 DiaROS分散環境時刻同期セットアップ"
echo "=================================="

# 1. NTPサーバーの設定
echo "1. NTPサーバー設定中..."
sudo apt-get update -qq
sudo apt-get install -y ntp ntpdate

# 2. NTP設定ファイルの作成
echo "2. NTP設定ファイル作成中..."
sudo tee /etc/ntp.conf > /dev/null << 'EOF'
# DiaROS専用NTP設定
driftfile /var/lib/ntp/ntp.drift

# 日本のNTPサーバー
server ntp.nict.jp
server ntp1.jst.mfeed.ad.jp
server ntp2.jst.mfeed.ad.jp
server ntp3.jst.mfeed.ad.jp

# ローカルネットワーク同期（マスターPC設定）
server 127.127.1.0
fudge 127.127.1.0 stratum 10

# 統計収集
statistics loopstats peerstats clockstats
filegen loopstats file loopstats type day enable
filegen peerstats file peerstats type day enable
filegen clockstats file clockstats type day enable
EOF

# 3. NTPサービス再起動
echo "3. NTPサービス再起動中..."
sudo systemctl restart ntp
sudo systemctl enable ntp

# 4. 時刻同期強制実行
echo "4. 時刻同期強制実行中..."
sudo ntpdate -s ntp.nict.jp

# 5. 同期状態確認
echo "5. 同期状態確認中..."
ntpq -p

# 6. Python時刻同期ライブラリインストール
echo "6. Python時刻同期ライブラリインストール中..."
pip install ntplib

# 7. DiaROS時刻同期テスト
echo "7. DiaROS時刻同期テスト実行中..."
python3 << 'EOF'
import time
import ntplib
from datetime import datetime

try:
    # NTPサーバーとの時刻差を計算
    client = ntplib.NTPClient()
    response = client.request('ntp.nict.jp')
    
    ntp_time = response.tx_time
    local_time = time.time()
    offset_ms = (ntp_time - local_time) * 1000
    
    print(f"✅ 時刻同期テスト完了")
    print(f"   ローカル時刻: {datetime.fromtimestamp(local_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"   NTP時刻: {datetime.fromtimestamp(ntp_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"   オフセット: {offset_ms:.2f}ms")
    
    if abs(offset_ms) < 10:
        print("   🟢 時刻同期は良好です")
    elif abs(offset_ms) < 50:
        print("   🟡 時刻同期は許容範囲内です")
    else:
        print("   🔴 時刻同期に問題があります")
        
except Exception as e:
    print(f"❌ 時刻同期テスト失敗: {e}")
EOF

echo ""
echo "🎉 時刻同期セットアップ完了"
echo "=================================="
echo "📋 次の手順:"
echo "1. 各PCでこのスクリプトを実行"
echo "2. 全PCで同期状態を確認: ntpq -p"
echo "3. DiaROS時間計測システムを起動"
echo ""