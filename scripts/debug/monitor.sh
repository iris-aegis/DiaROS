#!/bin/bash

# DiaROS Monitoring Script
# ネイティブ環境およびDocker環境の両方に対応

set -e

# 実行環境の検出
is_docker=false
if [ -f /.dockerenv ] || grep -qa docker /proc/1/cgroup 2>/dev/null; then
    is_docker=true
fi

# Docker実行環境の自動検出
is_running_in_docker=false
if [ -f /.dockerenv ]; then
    is_running_in_docker=true
fi

# ========== 現在の環境設定 ==========
DIAROS_DIR="/workspace/DiaROS_ros"

# ROS2環境自動セットアップ関数
setup_ros2_environment() {
    # ROS2 Humbleの自動検出とセットアップ
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo "✅ ROS2 Humble環境をセットアップしました"
    else
        echo "❌ ROS2 Humbleが見つかりません: /opt/ros/humble/setup.bash"
        return 1
    fi
    
    # DiaROSローカル環境のセットアップ
    if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
        source "$DIAROS_DIR/install/local_setup.bash"
        echo "✅ DiaROSローカル環境をセットアップしました"
    else
        echo "❌ DiaROSビルドが見つかりません: $DIAROS_DIR/install/local_setup.bash"
        return 1
    fi
}

# GUI環境設定関数
setup_gui_environment() {
    if [ "$is_running_in_docker" = true ]; then
        echo "🖥️  Docker内GUI環境をセットアップ中..."
        
        # 現在の設定を表示
        echo "  現在のDISPLAY: $DISPLAY"
        echo "  現在のXAUTHORITY: $XAUTHORITY"
        
        # XAUTHORITY設定の修正
        if [ -z "$XAUTHORITY" ]; then
            # 一般的なXAUTHORITYパスを試行
            for auth_path in "/root/.Xauthority" "/tmp/.X11-unix/X${DISPLAY#:}" "/run/user/1000/gdm/Xauthority" "/home/*/.Xauthority"; do
                if [ -f "$auth_path" ]; then
                    export XAUTHORITY="$auth_path"
                    echo "  XAUTHORITY=$auth_path に設定しました"
                    break
                fi
            done
        fi
        
        # DISPLAY設定の確認（変更せずに現在の設定を使用）
        echo "  現在のDISPLAY設定: $DISPLAY を使用します"
        
        # Qt環境設定
        export QT_X11_NO_MITSHM=1
        export QT_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins"
        export QT_QPA_PLATFORM=xcb
        
        # X11転送テスト
        echo "  X11接続をテスト中..."
        if xset q >/dev/null 2>&1; then
            echo "  ✅ X11接続が確認できました"
        else
            echo -e "${YELLOW}  ⚠️  X11サーバーに接続できません${NC}"
            echo "  対処法:"
            echo "    1. ホスト側で: xhost +local:docker"
            echo "    2. または: xhost +localhost"  
            echo "    3. または: xhost +local:"
            echo ""
            read -p "  GUIなしで続行しますか？ (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "  終了します..."
                exit 1
            fi
        fi
        
        echo "  最終設定:"
        echo "    DISPLAY=$DISPLAY"
        echo "    XAUTHORITY=$XAUTHORITY"
        echo "    QT_X11_NO_MITSHM=$QT_X11_NO_MITSHM"
    fi
}

# 実行環境の表示
echo "==================================="
echo "DiaROS Monitoring & Debug Tools"
if [ "$is_running_in_docker" = true ]; then
    echo "🐳 実行環境: Docker内"
    setup_gui_environment
    setup_ros2_environment
elif [ "$is_docker" = true ]; then
    echo "🖥️  実行環境: Docker外→Docker内"
else
    echo "🖥️  実行環境: ネイティブ"
    setup_ros2_environment
fi
echo "==================================="

# コマンド実行関数（現在の環境対応）
run_command() {
    if [ "$is_running_in_docker" = true ]; then
        # Docker内で直接実行
        setup_ros2_environment && cd "$DIAROS_DIR" && eval "$1"
    elif [ "$is_docker" = true ]; then
        # Docker外からDocker内のコンテナに対してコマンド実行
        docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && source /DiaROS_ros/install/local_setup.bash && $1"
    else
        # ネイティブ環境で実行
        setup_ros2_environment && cd "$DIAROS_DIR" && eval "$1"
    fi
}

# GUI表示対応コマンド実行関数（現在の環境対応）
run_gui_command() {
    if [ "$is_running_in_docker" = true ]; then
        # Docker内で直接実行（GUI対応）
        # GUI環境変数は既にsetup_gui_environment()で設定済み
        setup_ros2_environment && cd "$DIAROS_DIR" && eval "$1"
    elif [ "$is_docker" = true ]; then
        # Docker外からDocker内のコンテナに対してGUIコマンド実行
        docker exec -it -e DISPLAY=host.docker.internal:0 diaros_container bash -c "source /opt/ros/humble/setup.bash && source /DiaROS_ros/install/local_setup.bash && $1"
    else
        # ネイティブ環境で実行（GUI対応）
        setup_ros2_environment && cd "$DIAROS_DIR" && eval "$1"
    fi
}

# DiaROSシステムヘルスチェック関数
check_diaros_health() {
    echo "DiaROSシステムの稼働状況を確認中..."
    run_command "echo '稼働中のDiaROSノード:' && ros2 node list | grep -E '(speech_input|acoustic_analysis|automatic_speech_recognition|dialog_management|speech_synthesis|turn_taking|back_channel)' && echo '' && echo 'トピック周期:' && timeout 5 ros2 topic hz /mic_audio_float32 2>/dev/null | tail -1 && timeout 5 ros2 topic hz /AAtoDM 2>/dev/null | tail -1"
}

# X11診断関数
x11_diagnosis() {
    echo "🔍 X11環境診断:"
    echo "  DISPLAY: $DISPLAY"
    echo "  XAUTHORITY: $XAUTHORITY"
    
    # X11プロセス確認
    echo ""
    echo "📋 X11関連プロセス:"
    ps aux | grep -E "(Xorg|X11|xorg)" | grep -v grep | head -5 || echo "  X11プロセスが見つかりません"
    
    # X11ソケット確認
    echo ""
    echo "📁 X11ソケット:"
    ls -la /tmp/.X11-unix/ 2>/dev/null || echo "  /tmp/.X11-unix/ が見つかりません"
    
    # X認証ファイル確認
    echo ""
    echo "🔑 X認証ファイル:"
    if [ -n "$XAUTHORITY" ] && [ -f "$XAUTHORITY" ]; then
        ls -la "$XAUTHORITY"
        echo "  認証エントリ数: $(xauth list 2>/dev/null | wc -l)"
    else
        echo "  XAUTHORITY設定なし、または存在しません"
        find /root /home -name ".Xauth*" -o -name "Xauth*" 2>/dev/null | head -3
    fi
    
    # Xクライアントテスト
    echo ""
    echo "🧪 Xクライアントテスト:"
    if command -v xwininfo >/dev/null 2>&1; then
        if timeout 2 xwininfo -root >/dev/null 2>&1; then
            echo "  ✅ xwininfo 成功"
        else
            echo "  ❌ xwininfo 失敗"
        fi
    else
        echo "  xwininfo が見つかりません"
    fi
    
    echo ""
    echo "Enterキーを押して続行..."
    read -r
}

# X11接続修復関数
fix_x11_connection() {
    echo "🔧 X11接続修復を開始..."
    
    # DISPLAY設定の修復
    echo "1. DISPLAY設定の確認・修復"
    if [ "$DISPLAY" = ":1" ]; then
        echo "  DISPLAY=:1 から DISPLAY=:0 に変更"
        export DISPLAY=:0
    fi
    
    # XAUTHORITY設定の修復
    echo "2. XAUTHORITY設定の確認・修復"
    if [ -z "$XAUTHORITY" ]; then
        # 候補パスを検索
        for auth_path in "/root/.Xauthority" "/run/user/1000/gdm/Xauthority" "/home/*/.Xauthority"; do
            if [ -f "$auth_path" ]; then
                export XAUTHORITY="$auth_path"
                echo "  XAUTHORITY=$auth_path に設定"
                break
            fi
        done
    fi
    
    # Qt環境変数の設定
    echo "3. Qt環境変数の設定"
    export QT_X11_NO_MITSHM=1
    export QT_QPA_PLATFORM=xcb
    echo "  QT_X11_NO_MITSHM=1"
    echo "  QT_QPA_PLATFORM=xcb"
    
    # X11テスト
    echo "4. X11接続テスト"
    if xset q >/dev/null 2>&1; then
        echo "  ✅ X11接続成功！"
    else
        echo "  ❌ まだ接続できません"
        echo ""
        echo "  追加の対処法:"
        echo "    ホスト側で以下のコマンドを実行してください:"
        echo "    xhost +local:docker"
        echo "    または"
        echo "    xhost +localhost"
    fi
    
    echo ""
    echo "修復完了。Enterキーを押して続行..."
    read -r
}

# Function to display menu
show_menu() {
    echo ""
    echo "==================================="
    echo "DiaROS Monitoring & Debug Tools"
    echo "==================================="
    echo "=== 基本ROS2ツール ==="
    echo "1. rqt (Full GUI Dashboard)"
    echo "1a. rqt_bag専用起動（正常なレコーディング保証）"
    echo "2. rqt_graph (Node Communication Graph)"
    echo "3. rqt_plot (Real-time Data Plotting)"
    echo "4. rqt_topic (Topic Monitor)"
    echo "5. rqt_bag (Bag File Viewer)"
    echo "6. rqt_console (Log Console)"
    echo "7. ros2 topic list (Command Line)"
    echo "8. ros2 bag record (Start Recording)"
    echo ""
    echo "=== DiaROS専用モニタリング ==="
    echo "9. DiaROSシステムヘルスチェック"
    echo "10. 対話フロー監視 (リアルタイム)"
    echo "11. DiaROS対話フローデバッガー (debug_diaros_flow.py)"
    echo "12. 音声入力モニター (mic_audio_float32)"
    echo "13. 音声認識モニター (ASR出力)"
    echo "14. 対話状態総合モニター (全対話トピック)"
    echo "15. ターンテイキングモニター (話者交代管理)"
    echo "16. バックチャネルモニター (相槌応答)"
    echo "17. 対話セッション録画 (全DiaROSトピック)"
    echo "18. DiaROS対話フローグラフ表示"
    echo ""
    echo "=== 個別モジュールテスト ==="
    echo "19. ASR→DM対話フローテスト (test_asr_to_dm.py)"
    echo "20. DM→NLG対話フローテスト (test_dm_flow.py)"
    echo "21. NLG応答テスト (test_nlg_response.py)"
    echo "22. ターンテイキングテスト (test_turn_taking.py)"
    echo ""
    echo "=== 音声デバイステスト ==="
    echo "23. 簡易音声デバイステスト (test_audio_simple.py)"
    echo "24. マイク設定・テスト (set_default_mic.py)"
    echo "25. PyAudio PulseAudioテスト (test_pyaudio_pulse.py)"
    echo ""
    echo "=== システムテスト ==="
    echo "26. DiaROS応答フローテスト (test_diaros_response.py)"
    echo "27. API設定と応答生成テスト (test_api_nlg.py)"
    echo "28. 高速日本語LLMベンチマーク (test_fast_llm.py)"
    echo "29. OpenAI API直接テスト (test_openai_direct.py)"
    echo ""
    echo "=== 性能監視・デバッグツール ==="
    echo "30. トピック周期監視 (ros2 topic hz)"
    echo "31. 複数トピック同時周期監視"
    echo "32. エンドツーエンド遅延測定 (詳細フロー解析)"
    echo "33. エンドツーエンド遅延測定 (専用統計ツール)"
    echo "34. Plotjuggler起動 (リアルタイムグラフ)"
    echo "35. 性能トレース記録 (ros2 trace)"
    echo "36. システムリソース監視 (CPU/メモリ)"
    echo ""
    echo "=== RQT設定管理 ==="
    echo "36a. RQT設定を保存（現在の設定をプリセットとして保存）"
    echo "36b. RQT設定を初期化（デフォルト設定に戻す）"
    echo ""
    echo "=== ビルド・起動ツール ==="
    echo "37. DiaROSビルド (build_diaros.sh)"
    echo "38. DiaROSクリーンビルド (build_diaros.sh clean)"
    echo "39. DiaROS起動 - 標準 (launch_diaros.sh)"
    echo "40. DiaROS起動 - 静音版 (launch_diaros_quiet.sh)"
    echo "41. DiaROS起動 - ChatGPT優先 (launch_diaros_chatgpt.sh)"
    echo ""
    echo "=== Docker/X11診断ツール ==="
    echo "42. X11環境診断"
    echo "43. X11接続修復"
    echo ""
    echo "=== rosbag2修復ツール ==="
    echo "44. rosbag2レコーディング問題修正"
    echo "45. rqt_bag専用セットアップ（バックグラウンド監視付き）"
    echo ""
    echo ""
    echo "46. Exit"
    echo "==================================="
    echo -n "選択してください [1-46, 1a, 36a, 36b]: "
}

# 環境に応じた準備
if [ "$is_running_in_docker" = true ]; then
    # Docker内で実行中の場合
    echo "🔍 Docker内環境を確認中..."
    
    # DiaROSディレクトリの確認
    if [ ! -d "$DIAROS_DIR" ]; then
        echo "❌ DiaROSディレクトリが見つかりません: $DIAROS_DIR"
        exit 1
    fi
    
    # DiaROSビルドの確認
    if [ ! -d "$DIAROS_DIR/install" ]; then
        echo "❌ DiaROSがビルドされていません: $DIAROS_DIR/install"
        echo "start_diaros.shを先に実行してDiaROSをビルドしてください。"
        exit 1
    fi
    
    # ROS2環境の確認
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "❌ ROS2環境が見つかりません"
        echo "ROS2 Humbleがインストールされていることを確認してください。"
        exit 1
    fi
    
    echo "✅ Docker内環境とDiaROSビルドを確認しました"
    
elif [ "$is_docker" = false ]; then
    # ネイティブ環境の場合
    echo "🔍 ネイティブ環境を確認中..."
    
    # DiaROSディレクトリの確認
    if [ ! -d "$DIAROS_DIR" ]; then
        echo "❌ DiaROSディレクトリが見つかりません: $DIAROS_DIR"
        exit 1
    fi
    
    # DiaROSビルドの確認
    if [ ! -d "$DIAROS_DIR/install" ]; then
        echo "❌ DiaROSがビルドされていません: $DIAROS_DIR/install"
        echo "start_diaros.shを先に実行してDiaROSをビルドしてください。"
        exit 1
    fi
    
    # ROS2環境の確認
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "❌ ROS2環境が見つかりません"
        echo "ROS2 Humbleがインストールされていることを確認してください。"
        exit 1
    fi
    
    echo "✅ ネイティブ環境とDiaROSビルドを確認しました"
else
    # Docker環境の場合（外部からDocker内を操作）
    if ! command -v docker >/dev/null 2>&1; then
        echo "❌ Dockerが見つかりません"
        exit 1
    fi
    
    if ! docker ps | grep -q diaros_container; then
        echo "❌ DiaROSコンテナが起動していません。"
        echo "DiaROSコンテナを起動してから再実行してください。"
        exit 1
    fi
    
    echo "✅ Docker環境を確認しました"
fi

# macOSでのGUI表示設定
if [[ "$OSTYPE" == "darwin"* ]]; then
    if [ "$is_docker" = true ]; then
        # Docker環境でのXQuartz設定
        if ! pgrep -f "XQuartz|X11\.bin|Xquartz" > /dev/null; then
            echo "Warning: XQuartz may not be running."
            echo "Please ensure XQuartz is installed and running:"
            echo "  1. Install XQuartz from https://www.xquartz.org/"
            echo "  2. Open XQuartz"
            echo "  3. In XQuartz preferences, go to Security tab"
            echo "  4. Check 'Allow connections from network clients'"
            echo "  5. Run: /opt/X11/bin/xhost +localhost"
            echo ""
        else
            # Try to set xhost if XQuartz is running
            if [ -x "/opt/X11/bin/xhost" ]; then
                /opt/X11/bin/xhost +localhost 2>/dev/null || true
            fi
        fi
    fi
fi

# Main loop
while true; do
    show_menu
    read -r choice
    
    case $choice in
        1)
            echo "rqtを起動中..."
            
            # rqt_bag専用の環境設定
            export QT_X11_NO_MITSHM=1
            export ROSBAG2_STORAGE_PLUGIN=rosbag2_storage_sqlite3
            export ROSBAG2_CONVERTER=rosbag2_converter_default
            
            echo "🔧 rqt_bag対応設定を適用しました"
            run_gui_command "rqt"
            ;;
        "1a")
            echo "🚀 rqt_bag専用起動を開始中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/utils/launch_rqt_bag_proper.sh"
            ;;
        2)
            echo "rqt_graphを起動中..."
            run_gui_command "rqt_graph"
            ;;
        3)
            echo "rqt_plotを起動中..."
            run_gui_command "rqt_plot"
            ;;
        4)
            echo "rqt_topicを起動中..."
            run_gui_command "rqt_topic"
            ;;
        5)
            echo "rqt_bagを起動中..."
            run_gui_command "rqt_bag"
            ;;
        6)
            echo "rqt_consoleを起動中..."
            run_gui_command "rqt_console"
            ;;
        7)
            echo "ROS2トピックを一覧表示中..."
            run_command "ros2 topic list -v"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        8)
            echo "bag録画を開始します..."
            echo "録画するトピックを入力してください (スペース区切り、'all'で全トピック):"
            read -r topics
            
            timestamp=$(date +%Y%m%d_%H%M%S)
            if [ "$topics" = "all" ]; then
                echo "全トピックを録画中: ../log/diaros_$timestamp"
                run_command "ros2 bag record -a -o ../log/diaros_$timestamp"
            else
                echo "指定トピックを録画中: ../log/diaros_$timestamp"
                run_command "ros2 bag record $topics -o ../log/diaros_$timestamp"
            fi
            ;;
        9)
            echo "DiaROSシステムヘルスチェックを実行中..."
            check_diaros_health
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        10)
            echo "DiaROS対話フローを監視中..."
            run_command "echo '主要DiaROSトピックを監視:' && echo '============================' && echo '音声入力周波数:' && timeout 3 ros2 topic hz /mic_audio_float32 2>/dev/null | tail -1 && echo '' && echo '最新の音声認識結果:' && timeout 2 ros2 topic echo /ASRtoNLU --once 2>/dev/null && echo '' && echo '最新の対話管理出力:' && timeout 2 ros2 topic echo /DMtoNLG --once 2>/dev/null && echo '' && echo '音声合成ステータス:' && timeout 2 ros2 topic echo /SStoDM --once 2>/dev/null"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        11)
            echo "DiaROS対話フローデバッガーを起動中..."
            # スクリプトのパスを取得
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            # debug_diaros_flow.shを実行
            "$SCRIPT_DIR/debug_diaros_flow.sh"
            ;;
        12)
            echo "音声入力を監視中... (Ctrl+Cで終了)"
            run_command "ros2 topic hz /mic_audio_float32"
            ;;
        13)
            echo "音声認識出力を監視中... (Ctrl+Cで終了)"
            run_command "ros2 topic echo /ASRtoNLU"
            ;;
        14)
            echo "総合対話モニターを起動中..."
            run_command "tmux new-session -d -s dialog_monitor && tmux split-window -h && tmux split-window -v && tmux select-pane -t 0 && tmux split-window -v && tmux send-keys -t 0 'ros2 topic echo /ASRtoNLU' C-m && tmux send-keys -t 1 'ros2 topic echo /DMtoNLG' C-m && tmux send-keys -t 2 'ros2 topic echo /TTtoDM' C-m && tmux send-keys -t 3 'ros2 topic echo /BCtoDM' C-m && tmux attach -t dialog_monitor"
            ;;
        15)
            echo "ターンテイキングを監視中... (Ctrl+Cで終了)"
            run_command "ros2 topic echo /TTtoDM"
            ;;
        16)
            echo "バックチャネル応答を監視中... (Ctrl+Cで終了)"
            run_command "ros2 topic echo /BCtoDM"
            ;;
        17)
            echo "DiaROS対話セッションを録画中..."
            timestamp=$(date +%Y%m%d_%H%M%S)
            echo "録画ファイル: ../log/diaros_dialog_$timestamp"
            echo "Ctrl+Cで録画を停止"
            run_command "ros2 bag record /mic_audio_float32 /AAtoDM /ASRtoNLU /NLUtoDM /DMtoNLG /NLGtoSS /SStoDM /TTtoDM /BCtoDM -o ../log/diaros_dialog_$timestamp"
            ;;
        18)
            echo "DiaROS対話フローグラフを生成中..."
            run_gui_command "rqt_graph"
            ;;
        19)
            echo "ASR→DM対話フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            if [ "$is_running_in_docker" = true ]; then
                cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "$SCRIPT_DIR/test_asr_to_dm.py"
            else
                cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "$SCRIPT_DIR/test_asr_to_dm.py"
            fi
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        20)
            echo "DM→NLG対話フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "$SCRIPT_DIR/test_dm_flow.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        21)
            echo "NLG応答テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "$SCRIPT_DIR/test_nlg_response.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        22)
            echo "ターンテイキングテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "$SCRIPT_DIR/test_turn_taking.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        23)
            echo "簡易音声デバイステストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_audio_simple.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        24)
            echo "マイク設定・テストツールを起動中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/set_default_mic.py"
            ;;
        25)
            echo "PyAudio PulseAudioテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_pyaudio_pulse.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        26)
            echo "DiaROS応答フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_diaros_response.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        27)
            echo "API設定と応答生成テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_api_nlg.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        28)
            echo "高速日本語LLMベンチマークを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_fast_llm.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        29)
            echo "OpenAI API直接テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_DIR" && source /opt/ros/humble/setup.bash && source ./install/local_setup.bash && python3 "/workspace/scripts/test/test_openai_direct.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        30)
            echo "複数トピックの周期を同時監視します。"
            echo "主要DiaROSトピックの周期を監視中... (Ctrl+Cで終了)"
            run_command "echo '=== 音声入力周期 ===' && timeout 5 ros2 topic hz /mic_audio_float32 & echo '' && echo '=== 音響解析周期 ===' && timeout 5 ros2 topic hz /AAtoDM & echo '' && echo '=== 音声認識周期 ===' && timeout 5 ros2 topic hz /ASRtoNLU & echo '' && echo '=== 音声合成周期 ===' && timeout 5 ros2 topic hz /SStoDM & wait"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        32)
            echo "エンドツーエンド遅延測定を開始します。"
            echo "音声認識から音声合成までの遅延をリアルタイム測定..."
            echo "debug_diaros_flow.pyを使用した詳細な遅延分析 (Ctrl+Cで終了)"
            run_command "python3 /workspace/scripts/debug/debug_diaros_flow.py"
            ;;
        33)
            echo "エンドツーエンド遅延測定 (専用統計ツール)を開始します。"
            echo "ASR→SS間の遅延をリアルタイム統計測定中..."
            echo "平均・最小・最大・P95パーセンタイルを表示 (Ctrl+Cで終了)"
            run_command "python3 /workspace/scripts/debug/measure_e2e_latency.py"
            ;;
        34)
            echo "Plotjugglerを起動します..."
            if [[ "$OSTYPE" == "darwin"* ]]; then
                echo "❌ PlotjugglerはmacOSでは利用できません。"
                echo "代替案: rqt_plotを使用してください（選択肢3）"
            else
                echo "注意: plotjuggler-rosがインストールされている必要があります"
                run_gui_command "if command -v plotjuggler >/dev/null 2>&1; then plotjuggler; else echo 'Plotjugglerがインストールされていません。'; echo 'インストールコマンド: sudo apt install ros-humble-plotjuggler-ros'; fi"
            fi
            ;;
        35)
            echo "性能トレースを開始します。"
            echo "セッション名を入力してください:"
            read -r session_name
            echo "トレースを開始中... (Ctrl+Cで停止)"
            run_command "if command -v ros2 trace >/dev/null 2>&1; then ros2 trace start $session_name; else echo 'ros2-tracingがインストールされていません。'; echo 'インストールコマンド: sudo apt install ros-humble-tracing-tools-trace'; fi"
            ;;
        36)
            echo "システムリソースを監視中..."
            if [ "$is_docker" = true ]; then
                run_command "echo 'DiaROSノードのCPU/メモリ使用状況:' && echo '================================' && ps aux | grep -E '(ros2|speech_input|acoustic_analysis|automatic_speech_recognition|dialog_management|speech_synthesis|turn_taking|back_channel)' | grep -v grep && echo '' && echo 'コンテナ全体のリソース使用状況:' && echo '================================' && top -b -n 1 | head -20"
            else
                echo "DiaROSノードのCPU/メモリ使用状況:"
                echo "================================"
                ps aux | grep -E '(ros2|speech_input|acoustic_analysis|automatic_speech_recognition|dialog_management|speech_synthesis|turn_taking|back_channel)' | grep -v grep
                echo ""
                echo "システム全体のリソース使用状況:"
                echo "================================"
                top -b -n 1 | head -20
            fi
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        36a)
            echo "RQT設定を保存します..."
            echo "注意: rqtを先に起動し、Plugins→Plot等で監視したいトピックを設定してから実行してください。"
            echo ""
            echo "現在のrqt設定を保存しますか？ (y/n):"
            read -r confirm
            if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
                SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
                DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
                PERSPECTIVE_FILE="$DIAROS_ROOT/config/rqt_diaros_monitoring.perspective"
                
                echo "RQT設定を保存中..."
                echo "設定を保存するには、rqtのメニューで以下を実行してください:"
                echo "  1. File -> Save Perspective"
                echo "  2. 保存場所: $PERSPECTIVE_FILE"
                echo "  3. 次回は「1a. DiaROS専用rqt監視」を選択してください。"
                echo ""
                echo "または、現在の設定をデフォルトプリセットで上書きしますか？ (y/n):"
                read -r overwrite
                if [ "$overwrite" = "y" ] || [ "$overwrite" = "Y" ]; then
                    echo "プリセット設定を更新しました。"
                    # 現在の設定ファイルはそのまま使用
                fi
            else
                echo "保存をキャンセルしました。"
            fi
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        36b)
            echo "RQT設定を初期化します..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            PERSPECTIVE_FILE="$DIAROS_ROOT/config/rqt_diaros_monitoring.perspective"
            
            echo "プリセット設定ファイルを削除しますか？ (y/n):"
            read -r confirm
            if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
                if [ -f "$PERSPECTIVE_FILE" ]; then
                    rm "$PERSPECTIVE_FILE"
                    echo "設定ファイルを削除しました: $PERSPECTIVE_FILE"
                else
                    echo "設定ファイルが見つかりませんでした。"
                fi
                
                # デフォルト設定ファイルを再作成
                cat > "$PERSPECTIVE_FILE" << 'EOF'
{
  "keys": {},
  "groups": {
    "mainwindow": {
      "keys": {
        "geometry": "AdnQywADAAAAAAAAAAAAAAeAAAAAoAAAAAAAAAAAAeAAAACoAAAAAAAAAAAeAAAAA==",
        "state": "AAAA/wAAAAD9AAAAAgAAAAAAAAKrAAACoHP9AgAAAAL9AAAAAAAAAssAAAKgc/wCAAAAA/sAAAAUAFQAbwBwAGkAYwBzAQAAAAAAAACpAAAAVQD////7AAAAFABQAGwAbwB0AEAAVQA3ADkAYQBlAQAAAKsAAAH+AAAAggD////7AAAAHABHAHIAYQBwAGgAQAA1ADcAOQBhAGUALQAyAQAABKsAAAF9AAAA6gD///8AAAACAAAAAAAAAAAAAPwBAAAAA/sAAAAYAE0AZQB0AGEARABhAHQAYQBAADEAMwA1AAAAAAAAAGYAAABmAP////sAAAAaAEwAbwBnAGcAaQBuAGcAQAAxADMANQAtADIBAAAAAAAAZwAAAGYA////+wAAABgAUgBlAGMAbwBuAGYAaQBnAEAAMQAzADUBAAAAZwAAAGYA////AAAAAwAAB4AAAACE/AEAAAAB+wAAACAAVABvAG8AbABCAGEAcgBBAHIAZQBhAEAAMQAzADUBAAAAAAAAB4AAAAA6AP///wAAAiYAAAKgcwAAAAEAAAACAAAACAAAAAL8AAAAAA=="
      },
      "groups": {
        "dock_widget_PlotWidget": {
          "keys": {
            "dockable": "true",
            "parent": "mainwindow",
            "dock_widget_title": "Plot@579ae"
          },
          "groups": {
            "plugin": {
              "keys": {
                "instance_id": "579ae",
                "plugin_name": "rqt_plot/Plot"
              },
              "groups": {
                "plugin_settings": {
                  "keys": {
                    "autoscroll": "true",
                    "topics": "/mic_audio_float32/data[0];/ASRtoNLU/you;/DMtoNLG/response;/SStoDM/result"
                  }
                }
              }
            }
          }
        },
        "dock_widget_TopicWidget": {
          "keys": {
            "dockable": "true", 
            "parent": "mainwindow",
            "dock_widget_title": "Topics"
          },
          "groups": {
            "plugin": {
              "keys": {
                "instance_id": "135",
                "plugin_name": "rqt_topic/TopicPlugin"
              }
            }
          }
        },
        "dock_widget_GraphWidget": {
          "keys": {
            "dockable": "true",
            "parent": "mainwindow", 
            "dock_widget_title": "Graph@579ae-2"
          },
          "groups": {
            "plugin": {
              "keys": {
                "instance_id": "579ae-2",
                "plugin_name": "rqt_graph/RosGraph"
              }
            }
          }
        }
      }
    }
  }
}
EOF
                echo "デフォルト設定ファイルを再作成しました。"
            else
                echo "初期化をキャンセルしました。"
            fi
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        37)
            echo "DiaROSをビルドします..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/build/build_diaros.sh"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        38)
            echo "DiaROSをクリーンビルドします..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/build/build_diaros.sh" clean
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        39)
            echo "DiaROSを起動します（標準）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros.sh"
            ;;
        40)
            echo "DiaROSを起動します（静音版）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros_quiet.sh"
            ;;
        41)
            echo "DiaROSを起動します（ChatGPT優先）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros_chatgpt.sh"
            ;;
        42)
            echo "🔍 X11環境診断を実行中..."
            x11_diagnosis
            ;;
        43)
            echo "🔧 X11接続修復を実行中..."
            fix_x11_connection
            ;;
        44)
            echo "🔧 rosbag2レコーディング問題を修正中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_ROOT/DiaROS_ros"
            "$DIAROS_ROOT/scripts/utils/fix_rosbag_recording.sh"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        45)
            echo "🚀 rqt_bag専用セットアップを起動中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$DIAROS_ROOT/DiaROS_ros"
            "$DIAROS_ROOT/scripts/utils/setup_rqt_bag_recording.sh"
            ;;
        46)
            echo "終了します..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done