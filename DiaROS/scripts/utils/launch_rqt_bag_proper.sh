#!/bin/bash

# rqt_bagプラグインが正しく動作するための専用起動スクリプト
# metadata.yamlが自動的に正しく生成されるように環境を設定

echo "====================================="
echo "RQT Bag Proper Setup & Launch"
echo "====================================="

# 現在のディレクトリを保存
ORIGINAL_DIR=$(pwd)
DIAROS_DIR="/workspace/DiaROS/DiaROS_ros"

# DiaROSディレクトリに移動
if [ -d "$DIAROS_DIR" ]; then
    cd "$DIAROS_DIR"
    echo "📁 作業ディレクトリ: $DIAROS_DIR"
else
    echo "❌ DiaROSディレクトリが見つかりません: $DIAROS_DIR"
    exit 1
fi

# 1. ROS2環境のセットアップ
echo "1. ROS2環境をセットアップ中..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "   ✅ ROS2 Humble環境をロード"
else
    echo "   ❌ ROS2 Humble環境が見つかりません"
    exit 1
fi

# 2. DiaROSローカル環境のセットアップ
if [ -f "$DIAROS_DIR/install/local_setup.bash" ]; then
    source "$DIAROS_DIR/install/local_setup.bash"
    echo "   ✅ DiaROSローカル環境をロード"
fi

# 3. rosbag2専用環境変数のロード
if [ -f "/root/.ros/rosbag2_environment.sh" ]; then
    source /root/.ros/rosbag2_environment.sh
else
    echo "   ⚠️  rosbag2環境変数ファイルが見つかりません。デフォルト設定を使用"
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROSBAG2_STORAGE_PLUGIN=rosbag2_storage_sqlite3
    export ROSBAG2_CONVERTER=rosbag2_converter_default
    export QT_X11_NO_MITSHM=1
fi

# 4. rosbag2の設定確認
echo "2. rosbag2設定を確認中..."
echo "   ROSBAG2_STORAGE_PLUGIN: $ROSBAG2_STORAGE_PLUGIN"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "   QT_X11_NO_MITSHM: $QT_X11_NO_MITSHM"

# 5. 既存の不完全なrosbagファイルがあれば警告
echo "3. 既存のrosbagファイルをチェック中..."
incomplete_bags=0
for bag_dir in 20*/; do
    if [[ -d "$bag_dir" && -f "${bag_dir}"*.db3 && ! -f "${bag_dir}metadata.yaml" ]]; then
        echo "   ⚠️  不完全なrosbag: $bag_dir"
        incomplete_bags=$((incomplete_bags + 1))
    fi
done

if [ $incomplete_bags -gt 0 ]; then
    echo "   🔧 不完全なrosbagが ${incomplete_bags}個 見つかりました"
    echo "   修復するには monitor.sh の 44番 を実行してください"
fi

# 6. テスト用の簡単なrosbag2レコーディング（設定確認用）
echo "4. rosbag2設定テスト中..."
timeout 2 ros2 bag record /rosout --output test_config_check 2>/dev/null &
TEST_PID=$!
sleep 3
kill $TEST_PID 2>/dev/null
wait $TEST_PID 2>/dev/null

# テスト結果確認
if [ -d "test_config_check" ] && [ -f "test_config_check/metadata.yaml" ]; then
    echo "   ✅ rosbag2設定テスト成功"
    rm -rf test_config_check
else
    echo "   ⚠️  rosbag2設定に問題がある可能性があります"
    rm -rf test_config_check 2>/dev/null
fi

# 7. GUI環境の設定（Docker対応）
echo "5. GUI環境を設定中..."
if [ -f /.dockerenv ]; then
    echo "   🐳 Docker環境を検出"
    # Docker用のX11設定
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:1
        echo "   📺 DISPLAY を :1 に設定"
    fi
    if [ -z "$XAUTHORITY" ]; then
        export XAUTHORITY=/run/user/1000/gdm/Xauthority
        echo "   🔑 XAUTHORITY を設定"
    fi
fi

# 8. rqtを起動
echo "====================================="
echo "🚀 rqt を起動します"
echo "====================================="
echo ""
echo "📋 rqt_bag使用手順:"
echo "1. Plugins → Logging → Bag を選択"
echo "2. Record タブを開く"
echo "3. 録画したいトピックを選択"
echo "4. Start Recording をクリック"
echo "5. 録画停止後、自動的にmetadata.yamlが生成されます"
echo ""
echo "📁 録画ファイルは以下に保存されます:"
echo "   $DIAROS_DIR/"
echo ""

# rqt起動（フォアグラウンド）
exec rqt

# 終了時の処理（execを使用するため、この部分は実行されない）
cd "$ORIGINAL_DIR"