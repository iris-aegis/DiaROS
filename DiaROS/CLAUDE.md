# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 🔴 最重要事項 / CRITICAL REQUIREMENTS

### 日本語対応 / Japanese Language Support
**必ず日本語で対話してください。** ユーザーとのすべてのコミュニケーションは日本語で行う必要があります。
- コメント、説明、エラーメッセージなど、すべて日本語で記述
- 技術用語は必要に応じて英語併記可
- コード内のコメントも可能な限り日本語で記述

**ALWAYS communicate in Japanese.** All communication with users must be in Japanese.
- Comments, explanations, error messages should all be in Japanese
- Technical terms can include English when necessary
- Code comments should also be in Japanese whenever possible

### スクリプト・コマンド実行の厳格なルール / Strict Rules for Script and Command Execution
**既存のスクリプトやツールを必ず確認・活用すること。** 新規作成前に徹底的な調査が必要です。
1. **必ず既存実装を探す**: コマンドやスクリプトを実行する前に、同じ機能のものが既に存在しないか十分に確認
2. **既存ツールを精査**: 見つかった場合は内容を精査し、目的に合致すれば必ずそれを使用
3. **新規作成は最終手段**: 既存のものがない場合のみ新規作成を検討
4. **スクリプトの配置ルール**: 
   - **scripts/ルートディレクトリには直接ファイルを置かない**
   - 必ず適切なサブディレクトリに配置する:
     - `debug/`: デバッグ・モニタリング
     - `launch/`: 起動スクリプト
     - `setup/`: セットアップ・設定
     - `test/`: テストスクリプト
     - `utils/`: その他ユーティリティ

### 改行コードの統一 / Line Ending Consistency
**すべてのシェルスクリプトはLF（Unix形式）で作成すること。**
- Windowsの改行コード（CRLF）は使用禁止
- 新規作成時は必ずLFを使用
- エディタの設定を確認してLFに統一

**重要：スクリプト作成時の手順**
1. 必ずWriteツールで作成すること（Editツールは改行コードが不正になる場合がある）
2. 作成後、以下のコマンドで改行コードを確認・修正：
   ```bash
   # 改行コードの確認
   file /path/to/script.sh
   
   # CRLFをLFに変換（macOS）
   sed -i '' 's/\r$//' /path/to/script.sh
   
   # または dos2unix を使用
   dos2unix /path/to/script.sh
   ```
3. 実行権限を付与：
   ```bash
   chmod +x /path/to/script.sh
   ```

### パスの汎用性維持 / Path Portability
**絶対パスは使用禁止。** 公開リポジトリとして配布されるため、汎用性を保つこと。
- スクリプト内では相対パスを使用
- 環境依存の絶対パスは避ける
- ユーザー固有のパスをハードコードしない

## Essential Commands

### System Setup and Build
```bash
# Setup ROS2 environment (required before any ROS commands)
cd DiaROS_ros
source /opt/ros/foxy/setup.bash  # or your ROS2 installation path
source ./install/local_setup.bash

# Build the ROS packages
colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
source ./install/local_setup.bash
colcon build --packages-select diaros_package
source ./install/local_setup.bash

# Install Python modules
cd ../DiaROS_py
python -m pip install . --user
```

### Quick Setup Scripts
```bash
# Setup ROS2 environment (automated)
./scripts/setup/setup_ros2_env.sh

# Setup API keys
./scripts/setup/setup_api.sh

# Test API connection
./scripts/setup/setup_api.sh test

# Download Gemma model for local LLM
./scripts/setup/download_gemma_model.sh

# Setup ChatGPT API
./scripts/setup/setup_chatgpt_api.sh
```

### Running the System
```bash
# Primary command to launch the spoken dialog system
ros2 launch diaros_package sdsmod.launch.py

# Run without microphone input (for ros2 bag replay)
ros2 launch diaros_package sdsmod.launch.py mic:=false

# Run with muted microphone
ros2 launch diaros_package sdsmod.launch.py mic:=mute

# Run without NLG node (for distributed setup)
ros2 launch diaros_package sdsmod.launch.py nlg:=false
```

### 分散実行構成 / Distributed Execution Configuration
**重要**: このシステムは分散実行に対応しています。特にNLG（自然言語生成）を別PCで実行することが可能です。

#### NLG分散実行セットアップ
1. **メインPC（音声処理・対話管理）**:
   ```bash
   # NLGノードを除外して起動
   ros2 launch diaros_package sdsmod.launch.py nlg:=false
   ```

2. **NLG専用PC（対話生成処理）**:
   ```bash
   # NLGノードのみを実行
   ros2 run diaros_package ros2_natural_language_generation
   ```

#### 分散実行時の注意事項
- **ROS_DOMAIN_ID**: 両PC間で同一のROS_DOMAIN_IDを設定
- **ネットワーク設定**: ROS2のマルチキャスト通信が可能なネットワーク環境
- **同期**: 時刻同期（NTP）を推奨（タイミング分析の精度向上）
- **レイテンシ**: ネットワーク遅延を考慮した応答時間設定

#### 環境変数設定例
```bash
# 両PCで同一設定
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# NLG専用PCでAPI設定
export OPENAI_API_KEY="sk-your-openai-api-key"
```

### Launch Scripts (Cross-Platform)
```bash
# Universal launcher (macOS & Linux)
./scripts/launch/launch_diaros.sh

# Launch with ChatGPT API
./scripts/launch/launch_diaros_chatgpt.sh

# Launch with local LLM
./scripts/launch/launch_diaros_local.sh

# Launch in quiet mode (minimal output)
./scripts/launch/launch_diaros_quiet.sh

# Quick start with Pixi
./scripts/launch/pixi_diaros_quick_start.sh

# Launch without speech input (for bag replay)
./scripts/launch/launch_diaros_no_speech_input.sh
./scripts/launch/launch_diaros_no_speech_input_simple.sh
```

### Development and Debugging
```bash
# View ROS2 topics
ros2 topic list

# Monitor topic communication in real-time
ros2 topic echo [topic_name]

# Record system communication for debugging (saved to log directory)
ros2 bag record [topic1] [topic2] ... [topicN] -o ../log/recording_name

# Replay recorded communication
ros2 bag play ../log/[bag_file_name]

# Visualize node communication graph
ros2 run rqt_graph rqt_graph

# Plot topic data
ros2 run rqt_plot rqt_plot
```

### Debug Scripts
```bash
# Debug full DiaROS flow
./scripts/debug/debug_diaros_flow.py
./scripts/debug/debug_diaros_flow.sh

# Measure end-to-end latency
./scripts/debug/measure_e2e_latency.py

# Monitor system performance
./scripts/debug/monitor.sh

# Test specific components
./scripts/debug/test_asr_to_dm.py
./scripts/debug/test_dm_flow.py
./scripts/debug/test_nlg_response.py
./scripts/debug/test_turn_taking.py
```

### Test Scripts
```bash
# Test DiaROS response system
./scripts/test/test_diaros_response.py
./scripts/test/test_diaros_response.sh

# Test audio components
./scripts/test/test_audio_playback.py
./scripts/test/test_audio_simple.py
./scripts/test/test_pyaudio_pulse.py

# Test API connections
./scripts/test/quick_api_test.py
./scripts/test/test_api_nlg.py
./scripts/test/test_api_nlg.sh
./scripts/test/test_openai_direct.py
./scripts/test/test_fast_llm.py

# Set default microphone
./scripts/test/set_default_mic.py
```

## High-Level Architecture

DiaROS is a ROS2-based real-time spoken dialog system composed of two main parts:

### Core Python Library (`DiaROS_py/`)
Contains the core dialog system modules in Python:
- **speechInput.py**: Audio input using PyAudio
- **acousticAnalysis.py**: Acoustic analysis using aubio
- **automaticSpeechRecognition.py**: VAD-less ASR
- **dialogManagement.py**: Real-time dialog and backchannel control
- **naturalLanguageGeneration.py**: Response generation (ChatGPT API)
- **speechSynthesis.py**: Speech synthesis using VOICEVOX
- **turnTaking.py**: Turn-taking management
- **backChannel.py**: Backchannel response handling

### ROS2 Package (`DiaROS_ros/`)
ROS2 wrappers that enable:
- Inter-module communication via ROS2 topics
- System monitoring and debugging
- Recording and replay of dialog sessions
- Distributed processing capabilities

#### Key ROS2 Nodes (launched by sdsmod.launch.py):
- `ros2_speech_input`: Audio input node (conditional on `mic` parameter)
- `ros2_acoustic_analysis`: Audio feature extraction
- `ros2_automatic_speech_recognition`: Speech-to-text conversion
- `ros2_natural_language_understanding`: Intent understanding (passthrough)
- `ros2_dialog_management`: Central dialog coordinator
- `ros2_natural_language_generation`: Response generation *(可分散実行 - conditional on `nlg` parameter)*
- `ros2_speech_synthesis`: Text-to-speech conversion
- `ros2_turn_taking`: Turn-taking control
- `ros2_back_channel`: Backchannel response generation

#### 分散実行用パラメータ:
- `mic:=false`: speech_inputノードを無効化（音声ファイル再生用）
- `nlg:=false`: NLGノードを無効化（別PCでのNLG実行用）

#### Custom Message Interfaces (`interfaces/`)
Defines ROS2 message types for dialog system communication.

### Dashboard (Currently Non-Functional)
- Vue.js/Node.js web interface in `dialogue-dashboard/`
- Intended for real-time system monitoring and control
- Known to have dependency issues and is not operational

## API Requirements

### 高速応答生成API (High-Speed Response Generation)
DiaROSでは対話リズム維持のため、1500ms以内の応答が必要です。以下のAPIを推奨：

**推奨API (優先順位順):**
- **OpenAI API (ChatGPT)**: ~500-1000ms、最も高速で安定
- **Anthropic API (Claude)**: ~800-1200ms、高品質応答
- **ローカルモデル**: ~2000-5000ms、オフライン動作可能（非推奨）

**API設定方法:**
```bash
# 1. APIセットアップスクリプト実行（推奨）
./scripts/setup/setup_api.sh

# 2. 手動設定
export OPENAI_API_KEY="sk-your-openai-api-key"
export ANTHROPIC_API_KEY="sk-ant-your-anthropic-api-key"
```

### 音声認識API
- **Google Speech-to-Text API**: For speech recognition

Set environment variables:
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/google/credentials.json"
```

### Local LLM Options (No API Keys Required)
- **Gemma 2**: Default local model (google/gemma-2-2b-it)
- **Rinna Small**: Lightweight option
- **StableLM**: Alternative local model

Configure local LLM:
```bash
# Use Gemma 2 (default)
export DIAROS_LLM_MODEL=gemma2

# Use lightweight Rinna model
export DIAROS_LLM_MODEL=rinna-small

# Set device for local models
export DIAROS_DEVICE=cuda  # or 'cpu', 'mps' (macOS)
```

### 応答時間最適化設定
システム起動時に以下の優先順位で自動選択：
1. OpenAI API（設定済みの場合）
2. Anthropic API（設定済みの場合）  
3. ローカルモデル（APIキー未設定時）

**応答時間警告**: 1500ms超過時に警告メッセージを表示

## Development Environment

- **OS**: Ubuntu 20.04+ (Linux) or macOS (with Pixi)
- **ROS2**: Foxy (primary tested version) or Humble
- **Python**: 3.8.13+ (managed via pyenv) or 3.9+ (Pixi)
- **Key Dependencies**: PyAudio, aubio, torch, transformers, rclpy, openai, anthropic, VOICEVOX

### Platform-Specific Setup

#### macOS with Pixi
```bash
# Install Pixi package manager
curl -fsSL https://pixi.sh/install.sh | bash

# Setup Pixi workspace
./scripts/setup/setup_pixi_ros2.sh

# Launch with Pixi
./scripts/launch/pixi_diaros_quick_start.sh
```

#### Linux (Traditional)
```bash
# Install ROS2 Foxy/Humble
sudo apt update
sudo apt install ros-foxy-desktop

# Setup environment
./scripts/setup/setup_ros2_env.sh

# Install dependencies
pip install -r DiaROS_ros/requirements.txt
```

#### Docker Support
```bash
# Setup Docker audio
./scripts/setup/docker_audio_setup.sh

# Get Docker installation script
./scripts/setup/get-docker.sh
```

## System Architecture Flow

1. **Audio Input**: Microphone → speech_input → acoustic_analysis
2. **Recognition**: acoustic_analysis → automatic_speech_recognition
3. **Understanding**: speech_recognition → natural_language_understanding  
4. **Dialog Management**: Central coordinator managing all dialog flow
5. **Response Generation**: dialog_management → natural_language_generation *(可分散実行)*
6. **Speech Output**: response → speech_synthesis → audio output
7. **Turn Management**: turn_taking monitors and controls speaking turns
8. **Backchannel**: Generates appropriate listener responses during speech

### 分散実行時のアーキテクチャ
**メインPC**: 1-4, 6-8の処理を担当  
**NLG専用PC**: 5の自然言語生成処理を担当

The modular ROS2 architecture allows individual components to be developed, tested, and debugged independently while maintaining real-time communication capabilities. **Components can also be distributed across multiple machines for performance optimization.**

## プロジェクト構造 / Project Structure

### 音声ファイルの場所 / Audio File Locations
- **相槌音声**: `DiaROS_ros/static_back_channel_*.wav`
- **静的応答**: `DiaROS_ros/static_response_source/static_response_*.wav`
- **ランダム応答**: `DiaROS_ros/static_response_random/static_response_random_*.wav`
- **長い質問サンプル**: `DiaROS_ros/static_long_question/static_long_question*.wav`
- **合成音声**: `DiaROS_ros/tmp/*.wav` (一時ファイル、Gitに含まれない)
- **システム音声**: `DiaROS_ros/start_announce.wav`, `DiaROS_ros/end_announce.wav`
- **キャリブレーション音声**: `DiaROS_ros/power_calibration.wav`

### ログファイルの場所 / Log File Locations
- **ROSBagファイル**: `log/diaros_*/`, `log/rosbag2_*/` (録画データ)
- **対話セッション**: `log/mic_only_recording/`, `log/all_topic_recording/` (音声・全トピック録画)

### 設定ファイル / Configuration Files
- **RQT監視設定**: `config/rqt_diaros_monitoring.perspective`
- **Conda環境**: `DiaROS_ros/conda_DiaROS_*.yml`
- **Python環境**: `DiaROS_ros/environment.yml`

## Utility Scripts

### Scripts Directory Structure
```
scripts/
├── debug/          # Debugging and monitoring tools
├── launch/         # Cross-platform launch scripts
├── setup/          # Environment and dependency setup
├── test/           # Testing and validation tools
└── utils/          # General utilities
```

### Key Utilities
- **run_diaros_native_macos.py**: Native macOS runner
- **run_ros2_tool.sh**: ROS2 tool wrapper
- **static_response_shuffle.py**: Response randomization tool

### Audio Resources
The system includes various audio files for testing and responses:
- **power_calibration.wav**: Audio calibration file
- **start_announce.wav / end_announce.wav**: Session notifications
- **static_back_channel_*.wav**: Backchannel responses
- **static_response_*.wav**: Pre-recorded responses
- **static_long_question/**: Long-form question samples

### Configuration
- **rqt_diaros_monitoring.perspective**: RQT monitoring configuration
- **conda_DiaROS_*.yml**: Conda environment files
- **environment.yml**: Python environment specification

## Important Notes

- The system supports both microphone input and ROS2 bag replay
- All audio processing runs in real-time with low latency
- The launch scripts handle cross-platform compatibility
- Debug scripts provide detailed system monitoring
- Test scripts validate individual components
- VOICEVOX is used for speech synthesis and must be running