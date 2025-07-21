#!/bin/bash
# DiaROS Docker環境時間同期設定スクリプト

echo "🐳 DiaROS Docker環境時間同期設定を開始します..."

# 色付きの出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 現在の時刻確認
echo -e "${YELLOW}現在のDocker環境時刻:${NC}"
date

# Docker環境であることの確認
echo -e "${BLUE}Docker環境の確認:${NC}"
if [ -f /.dockerenv ]; then
    echo -e "${GREEN}✅ Docker環境内で実行されています${NC}"
else
    echo -e "${YELLOW}⚠️  Docker環境外で実行されている可能性があります${NC}"
fi

# カーネル情報表示
echo -e "${YELLOW}カーネル情報:${NC}"
uname -r

# Docker環境での時刻同期説明
echo -e "${BLUE}📝 Docker環境での時刻同期について:${NC}"
echo "• Docker環境はホストマシンのシステムクロックを共有します"
echo "• 時刻精度はホストマシンの設定に依存します"
echo "• Docker内でのNTP設定は限定的です"

# 利用可能な時刻同期ツールの確認
echo -e "${YELLOW}利用可能な時刻同期ツールを確認中...${NC}"

# chronyd確認
if command -v chronyd > /dev/null 2>&1; then
    echo -e "${GREEN}chronydが利用可能です${NC}"
    chronyc sources -v 2>/dev/null || echo "chronyd情報取得に制限があります（Docker環境のため）"
else
    echo "chronydは利用できません"
fi

# ntpd確認  
if command -v ntpd > /dev/null 2>&1; then
    echo -e "${GREEN}ntpdが利用可能です${NC}"
    ntpq -p 2>/dev/null || echo "ntpd情報取得に制限があります（Docker環境のため）"
else
    echo "ntpdは利用できません"
fi

# timedatectl確認
if command -v timedatectl > /dev/null 2>&1; then
    echo -e "${YELLOW}タイムゾーン設定:${NC}"
    timedatectl 2>/dev/null || echo "timedatectl機能が制限されています（Docker環境のため）"
else
    echo "timedatectlは利用できません"
fi

# 高精度時刻の確認
echo -e "${YELLOW}高精度時刻機能を確認中...${NC}"
python3 -c "
import time
import sys

try:
    current_time = time.time()
    ns_time = time.time_ns()
    clock_info = time.get_clock_info('time')
    
    print(f'現在時刻: {current_time}')
    print(f'ナノ秒時刻: {ns_time}')
    print(f'クロック分解能: {clock_info.resolution}秒')
    print(f'クロック精度: {clock_info.resolution * 1000:.3f}ms')
    
    # Docker環境での相対精度確認
    times = []
    for i in range(10):
        times.append(time.time_ns())
    
    diffs = [times[i+1] - times[i] for i in range(len(times)-1)]
    min_diff = min(diffs)
    
    print(f'最小時刻差: {min_diff}ns ({min_diff/1000000:.3f}ms)')
    
    if min_diff < 1000000:  # 1ms以下
        print('✅ 高精度時刻測定が可能です')
    else:
        print('⚠️  時刻測定精度が制限されています')
        
except Exception as e:
    print(f'❌ 時刻機能確認エラー: {e}')
"

# Docker環境での推奨事項
echo -e "${BLUE}🔧 Docker環境での推奨設定:${NC}"
echo "1. ホストマシンで適切な時刻同期を設定してください："
echo "   sudo systemctl enable chronyd && sudo systemctl start chronyd"
echo ""
echo "2. Dockerコンテナ起動時のオプション（必要に応じて）："
echo "   docker run --cap-add SYS_TIME ..."
echo ""
echo "3. DiaROS分散計測では相対時刻精度が重要です"
echo "   ホストマシンで1ms精度が達成されていれば十分です"

# 時刻同期確認完了
echo -e "${GREEN}✅ Docker環境時刻同期確認が完了しました${NC}"
echo -e "${YELLOW}💡 ホストマシンの時刻同期設定も確認してください${NC}"

# 最終確認
echo -e "${YELLOW}最終確認 - Docker環境時刻:${NC}"
date

echo -e "${BLUE}📊 次のステップ:${NC}"
echo "1. Python時刻同期精度テストを実行してください"
echo "2. DiaROSタイムトラッカーの動作確認を行ってください"
echo "3. ホストマシンでNTP同期が有効であることを確認してください"