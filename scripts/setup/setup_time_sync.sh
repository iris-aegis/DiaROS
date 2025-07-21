#!/bin/bash
# DiaROS分散時間同期設定スクリプト

echo "🕐 DiaROS分散時間同期設定を開始します..."

# 色付きの出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 現在の時刻確認
echo -e "${YELLOW}現在の時刻:${NC}"
date

# NTP時刻同期の設定
echo -e "${YELLOW}NTP時刻同期を設定中...${NC}"

# chronyd（推奨）またはntpサービスの確認
if command -v chronyd > /dev/null 2>&1; then
    echo -e "${GREEN}chronydを使用して時刻同期を設定します${NC}"
    
    # chronyd設定
    sudo systemctl stop chronyd
    sudo chrony sources -v
    sudo systemctl start chronyd
    sudo systemctl enable chronyd
    
    # 同期状態の確認
    echo -e "${YELLOW}同期状態を確認中...${NC}"
    sudo chronyc sources -v
    sudo chronyc tracking
    
elif command -v ntpd > /dev/null 2>&1; then
    echo -e "${GREEN}ntpdを使用して時刻同期を設定します${NC}"
    
    # ntpd設定
    sudo systemctl stop ntp
    sudo ntpd -gq
    sudo systemctl start ntp
    sudo systemctl enable ntp
    
    # 同期状態の確認
    echo -e "${YELLOW}同期状態を確認中...${NC}"
    ntpq -p
    
else
    echo -e "${RED}時刻同期サービスが見つかりません。手動でインストールしてください:${NC}"
    echo "Ubuntu/Debian: sudo apt-get install chrony"
    echo "CentOS/RHEL: sudo yum install chrony"
    exit 1
fi

# 高精度時刻の確認
echo -e "${YELLOW}高精度時刻機能を確認中...${NC}"
python3 -c "
import time
print(f'現在のナノ秒時刻: {time.time_ns()}')
print(f'システムクロック分解能: {time.get_clock_info(\"time\").resolution}秒')
"

# タイムゾーン設定確認
echo -e "${YELLOW}タイムゾーン設定:${NC}"
timedatectl

# 時刻同期完了
echo -e "${GREEN}✅ 時刻同期設定が完了しました${NC}"
echo -e "${YELLOW}💡 分散環境の全PCで同じ設定を実行してください${NC}"

# 最終確認
echo -e "${YELLOW}最終確認:${NC}"
date
echo "システム時刻が正確に同期されていることを確認してください"