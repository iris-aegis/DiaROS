#!/bin/bash

# DiaROS Monitoring Script
# ネイティブ環境およびDocker環境の両方に対応

set -e

# 実行環境の検出
is_docker=false
if [ -f /.dockerenv ] || grep -qa docker /proc/1/cgroup 2>/dev/null; then
    is_docker=true
fi

# ========== Pixi環境対応の環境設定 ==========
PIXI_DIR="$HOME/_data/_DiaROS_mac/DiaROS_pixi"
PIXI_WS="$PIXI_DIR/diaros_workspace"
DIAROS_DIR="$PIXI_DIR/DiaROS_imamoto/DiaROS_ros"

# コマンド実行関数（Pixi環境対応）
run_command() {
    if [ "$is_docker" = true ]; then
        docker exec -it diaros_container bash -c "source /opt/ros/humble/setup.bash && source /DiaROS_ros/install/local_setup.bash && $1"
    else
        # Pixi環境で実行
        cd "$PIXI_WS" && pixi run bash -c "cd \"$DIAROS_DIR\" && export AMENT_PREFIX_PATH=\"$DIAROS_DIR/install/diaros_package:$DIAROS_DIR/install/interfaces:\${AMENT_PREFIX_PATH:-}\" && export PYTHONPATH=\"$DIAROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_DIR/install/interfaces/lib/python3.9/site-packages:\${PYTHONPATH:-}\" && export DYLD_LIBRARY_PATH=\"$DIAROS_DIR/install/interfaces/lib:\${DYLD_LIBRARY_PATH:-}\" && $1"
    fi
}

# GUI表示対応コマンド実行関数（Pixi環境対応）
run_gui_command() {
    if [ "$is_docker" = true ]; then
        docker exec -it -e DISPLAY=host.docker.internal:0 diaros_container bash -c "source /opt/ros/humble/setup.bash && source /DiaROS_ros/install/local_setup.bash && $1"
    else
        # Pixi環境で実行（GUI対応）- macOSの場合はgraphvizのPATHを追加
        if [[ "$OSTYPE" == "darwin"* ]]; then
            cd "$PIXI_WS" && pixi run bash -c "cd \"$DIAROS_DIR\" && export PATH=\"/opt/homebrew/bin:\$PATH\" && export AMENT_PREFIX_PATH=\"$DIAROS_DIR/install/diaros_package:$DIAROS_DIR/install/interfaces:\${AMENT_PREFIX_PATH:-}\" && export PYTHONPATH=\"$DIAROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_DIR/install/interfaces/lib/python3.9/site-packages:\${PYTHONPATH:-}\" && export DYLD_LIBRARY_PATH=\"$DIAROS_DIR/install/interfaces/lib:\${DYLD_LIBRARY_PATH:-}\" && $1"
        else
            cd "$PIXI_WS" && pixi run bash -c "cd \"$DIAROS_DIR\" && export AMENT_PREFIX_PATH=\"$DIAROS_DIR/install/diaros_package:$DIAROS_DIR/install/interfaces:\${AMENT_PREFIX_PATH:-}\" && export PYTHONPATH=\"$DIAROS_DIR/install/diaros_package/lib/python3.9/site-packages:$DIAROS_DIR/install/interfaces/lib/python3.9/site-packages:\${PYTHONPATH:-}\" && export DYLD_LIBRARY_PATH=\"$DIAROS_DIR/install/interfaces/lib:\${DYLD_LIBRARY_PATH:-}\" && $1"
        fi
    fi
}

# DiaROSシステムヘルスチェック関数
check_diaros_health() {
    echo "DiaROSシステムの稼働状況を確認中..."
    run_command "echo '稼働中のDiaROSノード:' && ros2 node list | grep -E '(speech_input|acoustic_analysis|automatic_speech_recognition|dialog_management|speech_synthesis|turn_taking|back_channel)' && echo '' && echo 'トピック周期:' && timeout 5 ros2 topic hz /mic_audio_float32 2>/dev/null | tail -1 && timeout 5 ros2 topic hz /AAtoDM 2>/dev/null | tail -1"
}

# Function to display menu
show_menu() {
    echo ""
    echo "==================================="
    echo "DiaROS Monitoring & Debug Tools"
    echo "==================================="
    echo "=== 基本ROS2ツール ==="
    echo "1. rqt (Full GUI Dashboard)"
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
    echo "32. エンドツーエンド遅延測定"
    echo "33. Plotjuggler起動 (リアルタイムグラフ)"
    echo "34. 性能トレース記録 (ros2 trace)"
    echo "35. システムリソース監視 (CPU/メモリ)"
    echo ""
    echo "=== ビルド・起動ツール ==="
    echo "36. DiaROSビルド (build_diaros.sh)"
    echo "37. DiaROSクリーンビルド (build_diaros.sh clean)"
    echo "38. DiaROS起動 - 標準 (launch_diaros.sh)"
    echo "39. DiaROS起動 - 静音版 (launch_diaros_quiet.sh)"
    echo "40. DiaROS起動 - ChatGPT優先 (launch_diaros_chatgpt.sh)"
    echo ""
    echo "41. Exit"
    echo "==================================="
    echo -n "選択してください [1-41]: "
}

# 環境に応じた準備
if [ "$is_docker" = false ]; then
    # ネイティブ環境の場合
    echo "🔍 Pixi/ネイティブ環境を確認中..."
    
    # Pixiワークスペースの確認
    if [ ! -d "$PIXI_WS" ]; then
        echo "❌ Pixiワークスペースが見つかりません: $PIXI_WS"
        echo "launch_diaros.shを先に実行してください。"
        exit 1
    fi
    
    # DiaROSビルドの確認
    if [ ! -d "$DIAROS_DIR/install" ]; then
        echo "❌ DiaROSがビルドされていません: $DIAROS_DIR/install"
        echo "DiaROSをビルドしてから再実行してください。"
        exit 1
    fi
    
    echo "✅ Pixi環境とDiaROSビルドを確認しました"
else
    # Docker環境の場合
    if ! docker ps | grep -q diaros_container; then
        echo "エラー: DiaROSコンテナが起動していません。"
        echo "./scripts/run.sh を実行してください。"
        exit 1
    fi
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
            run_gui_command "rqt"
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
                echo "全トピックを録画中: diaros_$timestamp"
                run_command "ros2 bag record -a -o diaros_$timestamp"
            else
                echo "指定トピックを録画中: diaros_$timestamp"
                run_command "ros2 bag record $topics -o diaros_$timestamp"
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
            echo "録画ファイル: diaros_dialog_$timestamp"
            echo "Ctrl+Cで録画を停止"
            run_command "ros2 bag record /mic_audio_float32 /AAtoDM /ASRtoNLU /NLUtoDM /DMtoNLG /NLGtoSS /SStoDM /TTtoDM /BCtoDM -o diaros_dialog_$timestamp"
            ;;
        18)
            echo "DiaROS対話フローグラフを生成中..."
            run_gui_command "rqt_graph"
            ;;
        19)
            echo "ASR→DM対話フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$SCRIPT_DIR/test_asr_to_dm.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        20)
            echo "DM→NLG対話フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$SCRIPT_DIR/test_dm_flow.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        21)
            echo "NLG応答テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$SCRIPT_DIR/test_nlg_response.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        22)
            echo "ターンテイキングテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$SCRIPT_DIR/test_turn_taking.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        23)
            echo "簡易音声デバイステストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_audio_simple.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        24)
            echo "マイク設定・テストツールを起動中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/set_default_mic.py"
            ;;
        25)
            echo "PyAudio PulseAudioテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_pyaudio_pulse.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        26)
            echo "DiaROS応答フローテストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_diaros_response.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        27)
            echo "API設定と応答生成テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_api_nlg.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        28)
            echo "高速日本語LLMベンチマークを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_fast_llm.py"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        29)
            echo "OpenAI API直接テストを実行中..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            cd "$PIXI_WS" && pixi run python3 "$DIAROS_ROOT/scripts/test/test_openai_direct.py"
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
            echo "音声入力から音声出力までの遅延を測定中..."
            echo "簡易的な監視を開始します (Ctrl+Cで終了)"
            run_command "ros2 topic echo /mic_audio_float32 --once && echo '音声認識結果:' && ros2 topic echo /ASRtoNLU --once"
            ;;
        33)
            echo "Plotjugglerを起動します..."
            if [[ "$OSTYPE" == "darwin"* ]]; then
                echo "❌ PlotjugglerはmacOSでは利用できません。"
                echo "代替案: rqt_plotを使用してください（選択肢3）"
            else
                echo "注意: plotjuggler-rosがインストールされている必要があります"
                run_gui_command "if command -v plotjuggler >/dev/null 2>&1; then plotjuggler; else echo 'Plotjugglerがインストールされていません。'; echo 'インストールコマンド: sudo apt install ros-humble-plotjuggler-ros'; fi"
            fi
            ;;
        34)
            echo "性能トレースを開始します。"
            echo "セッション名を入力してください:"
            read -r session_name
            echo "トレースを開始中... (Ctrl+Cで停止)"
            run_command "if command -v ros2 trace >/dev/null 2>&1; then ros2 trace start $session_name; else echo 'ros2-tracingがインストールされていません。'; echo 'インストールコマンド: sudo apt install ros-humble-tracing-tools-trace'; fi"
            ;;
        35)
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
        36)
            echo "DiaROSをビルドします..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/build/build_diaros.sh"
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        37)
            echo "DiaROSをクリーンビルドします..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/build/build_diaros.sh" clean
            echo ""
            echo "Enterキーを押して続行..."
            read -r
            ;;
        38)
            echo "DiaROSを起動します（標準）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros.sh"
            ;;
        39)
            echo "DiaROSを起動します（静音版）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros_quiet.sh"
            ;;
        40)
            echo "DiaROSを起動します（ChatGPT優先）..."
            SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
            DIAROS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
            "$DIAROS_ROOT/scripts/launch/launch_diaros_chatgpt.sh"
            ;;
        41)
            echo "終了します..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please try again."
            ;;
    esac
done