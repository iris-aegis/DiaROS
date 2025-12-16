# ============================================================
# モデル設定 - ここでモデルを切り替え
# ============================================================
# 【OpenAI API モデル】クラウドAPI、高速・高品質
# MODEL_NAME = "gpt-3.5-turbo-0125"    # 587ms - 最速・最安・安定（推奨）
MODEL_NAME = "gpt-4.1-nano"          # 604ms - 最新技術・高速
# MODEL_NAME = "gpt-5-chat-latest"     # 708ms - GPT-5最速版・安定
# MODEL_NAME = "gpt-oss:20b"
# 【Ollama ローカルモデル】オフライン動作、GPU必要
<<<<<<< HEAD
MODEL_NAME = "gemma3:4b"             
# MODEL_NAME = "gemma3:12b"            
# MODEL_NAME = "gemma3:27b"            
=======
# MODEL_NAME = "gemma3:4b"
# MODEL_NAME = "gemma3:12b"
# MODEL_NAME = "gemma3:27b"
>>>>>>> 55154d4 (Fix: NLGをGPT-4.1で1-shot例示による1回応答に変更)

# ============================================================
# プロンプトファイル名の設定 - ここでプロンプトを切り替え
# ============================================================
# 【対話生成プロンプト】音声認識結果から対話応答を生成
# PROMPT_FILE_NAME = "dialog_simple.txt"       # シンプル版（ノイズタグ自動除去）
# PROMPT_FILE_NAME = "dialog_predict.txt"      # 発話予測付き（ノイズタグ自動除去）
# PROMPT_FILE_NAME = "dialog_tag.txt"          # タグ処理付き
# PROMPT_FILE_NAME = "dialog_tag_ver2.txt"          # タグ処理付き
# PROMPT_FILE_NAME = "dialog_explain.txt"      # 詳細説明付き（ノイズタグ自動除去）
# PROMPT_FILE_NAME = "dialog_example.txt"      # 例示付き（ノイズタグ自動除去）
PROMPT_FILE_NAME = "dialog_example_role.txt"      # 例示付き（ノイズタグ自動除去）
# PROMPT_FILE_NAME = "dialog_all.txt"          # 全機能版
# PROMPT_FILE_NAME = "dialog_all_1115.txt"          # 全機能版

# PROMPT_FILE_NAME = "dialog_phone.txt"        # 電話対話用

# 【音声認識結果の補正・補完プロンプト】音声認識結果の修正のみ
# PROMPT_FILE_NAME = "fix_asr_simple.txt"      # シンプル版（ノイズタグ自動除去）
# PROMPT_FILE_NAME = "fix_asr.txt"             # 標準版
# PROMPT_FILE_NAME = "fix_asr_example.txt"     # 例示付き
# PROMPT_FILE_NAME = "fix_asr_all.txt"     #
# PROMPT_FILE_NAME = "fix_asr_explain_fixed.txt"     #
# PROMPT_FILE_NAME = "fix_asr_predict.txt"     #
# PROMPT_FILE_NAME = "remdis_test_prompt.txt"     #
<<<<<<< HEAD
# PROMPT_FILE_NAME = "dialog_first_stage.txt"     # 200ms以内達成用（短いリアクションワードのみ）
=======
PROMPT_FILE_NAME = "dialog_example_role.txt"     # 1-shot例示付き（GPT-4.1推奨）
>>>>>>> 55154d4 (Fix: NLGをGPT-4.1で1-shot例示による1回応答に変更)

# 【タイミング調整プロンプト】
# PROMPT_FILE_NAME = "example_make_delay.txt"  # 遅延生成用
# PROMPT_FILE_NAME = "WebRTCVAD_timing_example.txt"   # WebRTCVADタイミング例
# PROMPT_FILE_NAME = "powerbase_timing_example.txt"   # パワーベースタイミング例

# 【テスト用プロンプト】
# PROMPT_FILE_NAME = "remdis_test.txt"         # テスト用（シンプル）

# ============================================================

import requests
import json
import sys
import os
import time
import threading
from datetime import datetime, timedelta
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor
import openai
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain_ollama import ChatOllama
from .timeTracker import get_time_tracker

class NaturalLanguageGeneration:
    def __init__(self, dm_ref=None, rnlg_ref=None):
        self.rc = { "word": "" }
        # ★DM参照を保持（2.5秒間隔ASR履歴を取得するため）
        self.dm_ref = dm_ref
        # ★ROS2NLG参照を保持（ROS2メッセージから受け取った2.5秒間隔ASR履歴を取得するため）
        self.rnlg_ref = rnlg_ref

        self.query = ""
        self.update_flag = False
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納
        self.last_source_words = []  # 対話生成の元にした音声認識結果を格納

        # 二段階応答生成用の変数
        self.first_stage_response = ""  # first_stageで生成したリアクションワードを保存
        self.current_stage = "first"  # first または second（DMからのstage指定で切り替わる）
        self.turn_taking_decision_timestamp_ns = 0  # TurnTaking判定時刻（ナノ秒）
        self.first_stage_response_cached = ""  # first_stageリアクションワードキャッシュ
        self.asr_history_2_5s = []  # 2.5秒間隔のASR結果リスト（Second stage生成用）

        # Second stage 処理中の first stage リクエスト管理
        self.is_generating_second_stage = False  # Second stage 生成中フラグ
        self.pending_first_stage_request = None  # 保留中の first_stage リクエスト（最新のみ保持）

        # ROS2 bag記録用の追加情報
        self.last_request_id = 0
        self.last_worker_name = ""
        self.last_start_timestamp_ns = 0
        self.last_completion_timestamp_ns = 0
        self.last_inference_duration_ms = 0.0
        
        # 新しい時刻情報フィールド初期化
        self.request_id = 0
        self.worker_name = ""
        self.start_timestamp_ns = 0
        self.completion_timestamp_ns = 0
        self.inference_duration_ms = 0.0
        
        # 接続エラー制御用
        self.connection_error_count = 0
        self.last_connection_error_time = None
        self.connection_error_suppress_until = None
        
        # タイムトラッカー初期化
        self.time_tracker = get_time_tracker("nlg_pc")
        self.current_session_id = None
        
        # 並列処理設定をコメントアウト（単一プロセス版）
        # self.inference_queue = Queue()  # 推論リクエストのキュー
        # self.result_queue = Queue()     # 推論結果のキュー
        # self.request_counter = 0        # リクエストカウンター
        self.last_request_time = None   # 最後のリクエスト時刻
        self.last_inference_time = None # 最後の推論実行時刻
        self.inference_interval = 2.5   # 推論間隔（秒）
        # self.executor = ThreadPoolExecutor(max_workers=3, thread_name_prefix="NLG-Worker")
        
        # モデル初期化（ファイル上部のMODEL_NAMEを使用）
        self.model_name = MODEL_NAME

        if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
            # Ollama モデルの初期化（gemma3系、gpt-oss系）
            sys.stdout.write(f'[NLG] Ollama {self.model_name}モデルを初期化中...\n')
            sys.stdout.flush()

            # gpt-ossは推論モデルなので非常に大きなトークン数が必要
            if self.model_name.startswith("gpt-oss:"):
                num_predict = 2000  # 推論トークン + 応答トークン（複雑なプロンプト対応）
                sys.stdout.write(f'[NLG] ⚠️  gpt-oss:20bは推論モデルのため、応答に時間がかかります (num_predict={num_predict})\n')
                sys.stdout.flush()
            else:
                num_predict = 10  # gemma3系は10トークンで統一（短いリアクションワード用）

            # gpt-oss:20bの高速化設定（推論を最小限に）
            if self.model_name.startswith("gpt-oss:"):
                temperature = 0.3  # より決定的に（推論を減らす）
                additional_kwargs = {
                    "verbose": True,
                    "num_ctx": 4096,
                    "num_batch": 3072,
                    "think": "low"  # 推論モード最小化（高速化）
                }
            else:
                temperature = 0.7  # gemma3系は標準設定
                additional_kwargs = {
                    "verbose": True,
                    "num_ctx": 4096,
                    "num_batch": 3072
                }

            self.ollama_model = ChatOllama(
                model=self.model_name,
                verbose=True,  # 詳細ログ有効化
                temperature=temperature,  # 応答の多様性
                top_p=0.9,  # サンプリング設定
                num_predict=num_predict,  # 最大生成トークン数
                keep_alive="10m",  # モデルをメモリに保持する時間（延長）
                additional_kwargs=additional_kwargs
            )
            sys.stdout.write(f'[NLG] ✅ {self.model_name}モデル初期化完了 (num_predict={num_predict})\n')
            sys.stdout.flush()

        elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1") or self.model_name.startswith("chatgpt-"):
            # OpenAI API設定（GPT-5, GPT-4, GPT-3.5, o1など全てのOpenAIモデル）
            sys.stdout.write(f'[NLG] OpenAI {self.model_name}モデルを初期化中...\n')
            sys.stdout.flush()

            # OpenAI APIキーを環境変数から設定
            openai.api_key = os.environ.get("OPENAI_API_KEY")
            if not openai.api_key:
                sys.stdout.write('[NLG ERROR] OPENAI_API_KEY が設定されていません\n')
                sys.stdout.flush()
                raise ValueError("OPENAI_API_KEY が設定されていません")
            else:
                sys.stdout.write(f'[NLG] ✅ {self.model_name}モデル初期化完了\n')
                sys.stdout.flush()

        else:
            raise ValueError(f"未対応のモデル: {self.model_name}")

        # プロンプトファイルの存在確認（複数パスの試行）
        self.prompt_file_name = PROMPT_FILE_NAME

        # 複数のプロンプトパスを試行（優先順）
        possible_paths = [
            # 1. 開発ディレクトリ（ローカル開発用）
            os.path.join(os.path.dirname(__file__), 'prompts', self.prompt_file_name),
            # 2. site-packagesにインストール済み（pip install実行後）
            os.path.join(os.path.dirname(__file__), 'prompts', self.prompt_file_name),
            # 3. 環境変数で指定されたディレクトリ
            os.path.join(os.environ.get('DIAROS_PROMPTS_DIR', ''), self.prompt_file_name) if os.environ.get('DIAROS_PROMPTS_DIR') else None,
        ]

        self.prompt_file_path = None
        for path in possible_paths:
            if path and os.path.exists(path):
                self.prompt_file_path = path
                sys.stdout.write(f'[NLG] ✅ プロンプトファイル確認: {self.prompt_file_name} ({path})\n')
                sys.stdout.flush()
                break

        if not self.prompt_file_path:
            sys.stdout.write(f'[NLG WARNING] ⚠️  プロンプトファイルが見つかりません: {self.prompt_file_name}\n')
            sys.stdout.write(f'[NLG WARNING]    試行パス: {possible_paths}\n')
            sys.stdout.flush()
            # デフォルトパスを設定（ファイルなくても続行）
            self.prompt_file_path = os.path.join(os.path.dirname(__file__), 'prompts', self.prompt_file_name)

        sys.stdout.write('NaturalLanguageGeneration (単一プロセス) start up.\n')
        sys.stdout.write(f'使用モデル: {self.model_name}\n')
        sys.stdout.write('=====================================================\n')

    def update(self, words, stage='first', turn_taking_decision_timestamp_ns=0, first_stage_backchannel_at_tt=None, asr_history_2_5s=None):
        """
        メインPCからのリクエストを処理
        words: 音声認識結果のリスト
        stage: 'first' または 'second'
        turn_taking_decision_timestamp_ns: TurnTaking判定時刻（ナノ秒）
        first_stage_backchannel_at_tt: TurnTaking判定時に再生予定のリアクションワード内容（Second stage用）
        asr_history_2_5s: 2.5秒間隔のASR結果リスト（Second stage生成用）
        """
        now = datetime.now()

        # 接続エラー抑制中は新しいリクエストを受け付けない
        if self.connection_error_suppress_until and now < self.connection_error_suppress_until:
            return

        # ★【重要】Second stage 生成中に first_stage のリクエストが来た場合は保留
        if self.is_generating_second_stage and stage == 'first':
            # Second stage 生成中の first_stage リクエストを保留
            timestamp = now.strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}] ⏸️  Second stage 生成中のため、first_stage をリクエストキューに保存\n")
            sys.stdout.flush()

            # 最新の first_stage リクエストだけを保持（上書き）
            self.pending_first_stage_request = {
                'words': words,
                'turn_taking_decision_timestamp_ns': turn_taking_decision_timestamp_ns,
                'first_stage_backchannel_at_tt': first_stage_backchannel_at_tt,
                'asr_history_2_5s': asr_history_2_5s
            }
            return

        # ★stage情報とタイムスタンプを保存
        self.current_stage = stage
        self.turn_taking_decision_timestamp_ns = turn_taking_decision_timestamp_ns
        # ★TT判定時のリアクションワードを保存（Second stage用）
        # ★修正：空文字列も含めて常に更新（パラメータが渡された場合）
        if first_stage_backchannel_at_tt is not None:
            self.first_stage_response = first_stage_backchannel_at_tt
        # ★2.5秒間隔ASR結果を保存（Second stage用）
        # ★修正：空リストも含めて常に更新（パラメータが渡された場合）
        if asr_history_2_5s is not None:
            self.asr_history_2_5s = asr_history_2_5s

        # ★性能監視: 大量履歴の受信を記録
        word_count = len(words) if isinstance(words, list) else 1
        timestamp = now.strftime('%H:%M:%S.%f')[:-3]

        # ★ログ出力を簡略化：[HH:MM:SS.mmm] 形式に統一
        sys.stdout.write(f"[{timestamp}] stage='{stage}' で更新\n")
        sys.stdout.flush()

        if word_count > 20:
            sys.stdout.write(f"[{timestamp}] 大容量履歴受信: {word_count}個\n")
            sys.stdout.flush()

        # 最初の3個と最後の3個のみを表示（中間は省略）
        if isinstance(words, list):
            if word_count > 6:
                preview_words = words[:3] + ["..."] + words[-3:]
                sys.stdout.write(f"[{timestamp}] 履歴受信（{word_count}個）\n")
            elif word_count > 0:
                sys.stdout.write(f"[{timestamp}] 履歴受信（{word_count}個）\n")
            sys.stdout.flush()

        query = words

        # 音声認識結果がリストの場合はプロンプトに埋め込む
        self.asr_results = None
        # ★Second stageの場合は空のqueryを許容（first_stage_responseから続きを生成）
        # First stageの場合は空リストまたは全て空文字列なら何もしない
        if isinstance(query, list):
            if not query or all((not x or x.strip() == "") for x in query):
                # ★Second stageの場合は処理を続ける（first_stage_responseを使用）
                if self.current_stage != 'second':
                    self.update_flag = False
                    return
                # Second stageの場合は空のqueryでも処理を続ける
            self.asr_results = query if query else []
            self.query = query
        else:
            if not query or (isinstance(query, str) and query.strip() == ""):
                # ★Second stageの場合は処理を続ける
                if self.current_stage != 'second':
                    self.update_flag = False
                    return
            self.query = query
            self.asr_results = None

        # 2.5秒間隔制御を削除: 音声認識結果が来るたびにすぐ応答生成
        # (以前の間隔制御コードはコメントアウト)

        # 単一プロセス推論に変更（並列処理をコメントアウト）
        # self.request_counter += 1
        # request_id = self.request_counter
        request_id = 1  # 単一プロセスでは固定ID

        # ★ログ形式を統一：[HH:MM:SS.mmm] のみ表示
        # sys.stdout.write(f"[{now.strftime('%H:%M:%S.%f')[:-3]}] 🚀 推論開始\n")
        # sys.stdout.flush()

        # ★ステージに応じたプロンプト選択と推論実行
        # Stage ごとに異なるプロンプトを使い分けて実行（同期処理）
        if self.current_stage == 'first':
            # First stage: dialog_first_stage.txt でリアクションワード生成
            # ★ログ出力を削除（簡略化）
            self.generate_first_stage(query)
        elif self.current_stage == 'second':
            # Second stage: dialog_second_stage.txt で本応答生成
            # ★ログ出力を削除（簡略化）
            self.generate_second_stage(query)
        else:
            # その他: 従来の _perform_simple_inference()
            self._perform_simple_inference(query)

        # 最後の推論時刻を更新
        self.last_inference_time = now
        self.last_request_time = now
        self.update_flag = True
        
    def set_session_id(self, session_id: str):
        """セッションIDを設定"""
        self.current_session_id = session_id

    def generate_first_stage(self, query):
        """First stage: リアクションワード生成（dialog_first_stage.txt + humanタグでASR結果を別口入力）"""
        start_time = datetime.now()

        try:
            asr_results = query if isinstance(query, list) else [str(query)]

            # ★修正：音声認識結果が空の場合はリアクションワード生成を行わない
            if not asr_results or all((not x or x.strip() == "") for x in asr_results):
                self.first_stage_response = ""
                timestamp = start_time.strftime('%H:%M:%S.%f')[:-3]
                sys.stdout.write(f"[{timestamp}] First stage: ASR結果が空のためスキップ\n")
                sys.stdout.flush()
                return

            # ★プロンプトファイル読み込み
            prompt_build_start = datetime.now()
            try:
                prompt_text = self._load_first_stage_prompt()
            except FileNotFoundError as e:
                sys.stdout.write(f"[NLG ERROR] first_stageプロンプトが見つかりません: {e}\n")
                sys.stdout.flush()
                self.first_stage_response = "うん"
                return

            prompt_build_end = datetime.now()
            # ★ログ出力を簡略化：プロンプト読み込みログを削除

            # LLM呼び出し
            llm_start_time = datetime.now()

            try:
                if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                    # ★Ollama API /api/chat エンドポイント（humanタグ形式）
                    import requests

                    api_start = datetime.now()

                    # humanタグで別口入力: system (プロンプト) + user (ASR結果)
                    asr_text = ', '.join(asr_results)
                    messages = [
                        {"role": "system", "content": prompt_text},
                        {"role": "user", "content": f"ぶつ切りの音声認識結果: {asr_text}"}
                    ]

                    # ★ログ出力を簡略化：messages送信ログを削除

                    response = requests.post(
                        'http://localhost:11434/api/chat',
                        json={
                            'model': self.model_name,
                            'messages': messages,
                            'stream': True,
                            'options': {
                                'temperature': 0.3,
                                'num_predict': 10,
                                'num_ctx': 512,
                                'num_batch': 256
                            }
                        },
                        stream=True,
                        timeout=30
                    )

                    res = ""
                    first_token_time = None
                    token_count = 0

                    for line in response.iter_lines():
                        if line:
                            try:
                                chunk_data = json.loads(line)
                                message_data = chunk_data.get('message', {})
                                token_fragment = message_data.get('content', '')

                                if token_fragment:
                                    token_count += 1

                                    # Time to First Token (TTFT) 計測
                                    if first_token_time is None:
                                        first_token_time = datetime.now()
                                        ttft_ms = (first_token_time - api_start).total_seconds() * 1000
                                        # ★ログ出力を簡略化：TTFT計測ログを削除

                                    res += token_fragment

                                # 完了チェック
                                if chunk_data.get('done', False):
                                    api_end = datetime.now()
                                    total_time = (api_end - api_start).total_seconds() * 1000
                                    # ★ログ出力を簡略化：中間の推論時間ログを削除
                                    break

                            except json.JSONDecodeError:
                                continue

                elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                    # OpenAI API: systemプロンプト + humanタグ（user）でASR結果
                    asr_text = ', '.join(asr_results)
                    messages = [
                        {"role": "system", "content": prompt_text},
                        {"role": "user", "content": f"ぶつ切りの音声認識結果: {asr_text}"}
                    ]

                    response = openai.chat.completions.create(
                        model=self.model_name,
                        messages=messages,
                        max_completion_tokens=20,
                        temperature=0.3
                    )
                    res = response.choices[0].message.content.strip() if response.choices[0].message.content else ""

                # リアクションワードの後処理: 改行・句読点除去
                res = res.replace('\n', '').replace('\r', '').replace('。', '').replace('、', '').strip()

                llm_end_time = datetime.now()
                llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000

                self.first_stage_response = res
                # ★ROS トピック発行用に last_reply にも格納（ROS2ラッパーが監視している）
                self.last_reply = res
                # ★簡略化：[HH:MM:SS.mmm] 形式のみ表示
                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}]\n")
                sys.stdout.flush()

            except Exception as api_error:
                sys.stdout.write(f"[NLG ERROR] first_stage生成エラー: {api_error}\n")
                sys.stdout.flush()
                self.first_stage_response = "うん"  # フォールバック

        except Exception as e:
            sys.stdout.write(f"[NLG ERROR] first_stage処理エラー: {e}\n")
            sys.stdout.flush()
            self.first_stage_response = "うん"  # フォールバック

    def _load_first_stage_prompt(self):
        """dialog_first_stage.txt を読み込みます"""
        # 複数のパスを試行
        possible_paths = [
            # 開発ディレクトリ
            os.path.join(os.path.dirname(__file__), 'prompts', 'dialog_first_stage.txt'),
            # インストール済みパッケージ
            os.path.join(os.path.dirname(__file__), '..', 'diaros', 'prompts', 'dialog_first_stage.txt'),
        ]

        # 環境変数が設定されている場合はそれを優先
        if 'DIAROS_PROMPTS_DIR' in os.environ:
            possible_paths.insert(0, os.path.join(os.environ['DIAROS_PROMPTS_DIR'], 'dialog_first_stage.txt'))

        for path in possible_paths:
            if os.path.isfile(path):
                with open(path, 'r', encoding='utf-8') as f:
                    return f.read()

        # ファイルが見つからない場合
        raise FileNotFoundError(f"dialog_first_stage.txt が見つかりません。試行パス: {possible_paths}")

    def generate_second_stage(self, query):
        """Second stage: turnTakingが応答判定を出したら実行"""
        start_time = datetime.now()

        # ★【重要】Second stage 処理開始時にフラグを設定
        self.is_generating_second_stage = True
        timestamp = start_time.strftime('%H:%M:%S.%f')[:-3]
        sys.stdout.write(f"[{timestamp}] 🔄 Second stage 処理開始\n")
        sys.stdout.flush()

        try:
            # ★修正：queryが空の場合は、2.5秒間隔ASR結果またはfirst_stageのASR結果を使用
            if isinstance(query, list) and (not query or all((not x or x.strip() == "") for x in query)):
                # query が空 → 2.5秒間隔ASR結果を優先使用（Second stage用）
                if self.asr_history_2_5s:
                    asr_results = self.asr_history_2_5s
                    sys.stdout.write(f"[{start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG SECOND_STAGE] 💾 2.5秒間隔ASR結果を使用\n")
                    sys.stdout.flush()
                else:
                    # asr_history_2_5s がない場合は前回のASR結果を再利用
                    asr_results = self.asr_results if self.asr_results else []
                    sys.stdout.write(f"[{start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG SECOND_STAGE] 💾 前回の ASR 結果を再利用\n")
                    sys.stdout.flush()
            else:
                # query が有効 → それを使用
                asr_results = query if isinstance(query, list) else [str(query)]

            # ★修正：Second stageでは空のASR結果でも処理を続ける（first_stage_responseを使用するため）
            # ただしfirst_stage_responseも空の場合は返す
            # ★ログ出力を簡略化（デバッグ情報は削除）

            if (not asr_results or all((not x or x.strip() == "") for x in asr_results)) and not self.first_stage_response:
                # ★ログ出力を簡略化
                self.last_reply = ""
                self.last_source_words = []
                return

            # プロンプトファイル読み込み（複数メッセージ方式）
            prompt_load_start = datetime.now()
            prompt_dir = os.path.join(os.path.dirname(__file__), 'prompts')

            # ★修正：dialog_second_stage_triple_input.txt を使用（placeholder なし）
            second_stage_prompt_path = os.path.join(prompt_dir, 'dialog_second_stage_triple_input_example_role.txt')

            try:
                with open(second_stage_prompt_path, 'r', encoding='utf-8') as f:
                    system_prompt = f.read()

                # ★ログ出力を簡略化（プロンプト読み込みログを削除）

                # ★修正：複数メッセージ方式 - 正しいロール構造で入力
                # 1. system: システムのタスク説明
                # 2. user: ユーザーの音声認識結果（発話）
                # 3. assistant: システムが既に出力したリアクションワード（第1段階の応答）
                # この流れにより、LLMが対話コンテキストを正しく認識できる

                # メッセージリストを構築
                asr_text = ', '.join(asr_results) if asr_results else "[音声認識結果なし]"
                backchannel_text = self.first_stage_response if self.first_stage_response else "[リアクションワードなし]"

                # ★修正：first_stage（リアクションワード）の末尾に「、」がなければ追加
                if backchannel_text and backchannel_text != "[リアクションワードなし]":
                    if not backchannel_text.endswith("、"):
                        backchannel_text = backchannel_text + "、"

                messages = [
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": f"複数のぶつ切りの音声認識結果：週末に時間ができるとついついスマホを見て, 週末に時間ができるとついついスマホを見て[無音], スマホを見て一日が終わっちゃうのが嫌で, 一日が終わっちゃうのが嫌で何か新しいことをはじめたい, 何か新しいこと始めたいんだけど家の中で一人人, 家の中で一人人でも没闘できるような趣味のアイデアった, 没闘できるような趣味のアイデアってないかな[無音][無音]"},
                    {"role": "assistant", "content": f"リアクションワード：なるほど"},
                    {"role": "assistant", "content": f"タメ口の応答：それなら読書とかプラモデル作りとかはどう？"},
                    {"role": "user", "content": f"複数のぶつ切りの音声認識結果：{asr_text}"},           # ★ユーザーの発話（ラベル付き）
                    {"role": "assistant", "content": f"リアクションワード：{backchannel_text}"},  # ★システムが既に出力したリアクションワード（ラベル付き、末尾に「、」追加）
                    {"role": "assistant", "content": f"タメ口の応答："}
                ]

            except FileNotFoundError:
                sys.stdout.write(f"[NLG ERROR] second_stageプロンプトが見つかりません: {second_stage_prompt_path}\n")
                sys.stdout.flush()
                return

            # ★確認用出力：使用するASR結果とfirst_stage結果を表示
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

            # ★2.5秒間隔ASR履歴を取得（複数の優先度で取得）
            asr_2_5s_list = []
            # 優先度1: ROS2NLG参照から取得（ROS2メッセージから受け取った値）
            if self.rnlg_ref and hasattr(self.rnlg_ref, 'asr_history_2_5s'):
                asr_2_5s_list = self.rnlg_ref.asr_history_2_5s
            # 優先度2: DM参照から取得（ローカル実行時）
            elif self.dm_ref and hasattr(self.dm_ref, 'asr_history_at_tt_decision_2_5s'):
                asr_2_5s_list = self.dm_ref.asr_history_at_tt_decision_2_5s
            # 優先度3: インスタンス変数から取得
            elif self.asr_history_2_5s:
                asr_2_5s_list = self.asr_history_2_5s

            sys.stdout.write(f"[{timestamp}] [Second Stage] 2.5秒間隔ASR結果: {asr_2_5s_list}\n")
            sys.stdout.write(f"[{timestamp}] [Second Stage] First Stage結果: '{self.first_stage_response}'\n")

            # ★修正：複数メッセージ方式のメッセージリストを表示
            sys.stdout.write(f"[{timestamp}] [Second Stage] LLMへ送信するメッセージ:\n")
            for i, msg in enumerate(messages, 1):
                sys.stdout.write(f"  {i} (role={msg['role']}): {msg['content']}\n")
            sys.stdout.flush()

            # LLM呼び出し
            llm_start_time = datetime.now()
            # ★ログ出力を簡略化（LLM開始メッセージを削除）

            try:
                if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                    # ★Second stage でも requests API を直接呼び出し（first stage と同じ方式）
                    # LangChain のオーバーヘッドを排除
                    import requests

                    api_start = datetime.now()

                    # ★修正：複数メッセージ方式で送信（system + user1 + user2）
                    response = requests.post(
                        'http://localhost:11434/api/chat',
                        json={
                            'model': self.model_name,
                            'messages': messages,  # ★複数メッセージリストを直接使用
                            'stream': True,
                            'options': {
                                'temperature': 0.3,
                                'num_predict': 50,  # second stage は最大50トークン（20文字程度の一言用）
                                'num_ctx': 512,
                                'num_batch': 256
                            }
                        },
                        stream=True,
                        timeout=30
                    )

                    res = ""
                    first_token_time = None
                    token_count = 0

                    for line in response.iter_lines():
                        if line:
                            try:
                                chunk_data = json.loads(line)
                                message_data = chunk_data.get('message', {})
                                token_fragment = message_data.get('content', '')

                                if token_fragment:
                                    token_count += 1

                                    # Time to First Token (TTFT) 計測
                                    if first_token_time is None:
                                        first_token_time = datetime.now()
                                        ttft_ms = (first_token_time - api_start).total_seconds() * 1000
                                        # ★ログ出力を簡略化（TTFT ログ削除）

                                    res += token_fragment

                                # 完了チェック
                                if chunk_data.get('done', False):
                                    api_end = datetime.now()
                                    total_time = (api_end - api_start).total_seconds() * 1000
                                    # ★ログ出力を簡略化（推論時間ログ削除）
                                    break

                            except json.JSONDecodeError:
                                continue

                elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                    # ★修正：既に構築されたmessagesリスト（複数メッセージ方式）を使用
                    response = openai.chat.completions.create(
                        model=self.model_name,
                        messages=messages,  # ★複数メッセージリストを直接使用
                        max_completion_tokens=50,
                        temperature=0.3
                    )
                    res = response.choices[0].message.content.strip() if response.choices[0].message.content else ""

                # 改行除去
                res = res.replace('\n', '').replace('\r', '')

                llm_end_time = datetime.now()
                llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                total_duration = (llm_end_time - start_time).total_seconds() * 1000

                # 最終的な応答は本応答のみ（メインPC側でfirst_stageは既に再生されている）
                final_response = res

                self.last_reply = final_response
                self.last_source_words = asr_results

                # タイミング情報を設定
                self.request_id = 1
                self.worker_name = "nlg-two-stage"
                self.start_timestamp_ns = int(start_time.timestamp() * 1_000_000_000)
                self.completion_timestamp_ns = int(llm_end_time.timestamp() * 1_000_000_000)
                self.inference_duration_ms = total_duration

                # ★簡略化：[HH:MM:SS.mmm] 形式のみ表示
                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}]\n")
                sys.stdout.flush()

                # ★【重要】Second stage 処理完了時にフラグをリセット
                self.is_generating_second_stage = False
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                sys.stdout.write(f"[{timestamp}] ✅ Second stage 処理完了\n")
                sys.stdout.flush()

                # ★保留中の first_stage リクエストがあれば処理
                if self.pending_first_stage_request:
                    sys.stdout.write(f"[{timestamp}] ▶️  保留中の first_stage リクエストを実行\n")
                    sys.stdout.flush()

                    pending_req = self.pending_first_stage_request
                    self.pending_first_stage_request = None  # 保留キューをクリア

                    # 保留されていたリクエストを実行
                    self.update(
                        words=pending_req['words'],
                        stage='first',
                        turn_taking_decision_timestamp_ns=pending_req['turn_taking_decision_timestamp_ns'],
                        first_stage_backchannel_at_tt=pending_req['first_stage_backchannel_at_tt'],
                        asr_history_2_5s=pending_req['asr_history_2_5s']
                    )

            except Exception as api_error:
                sys.stdout.write(f"[NLG ERROR] second_stage生成エラー: {api_error}\n")
                sys.stdout.flush()
                self.last_reply = self.first_stage_response  # リアクションワードのみフォールバック
                self.last_source_words = asr_results

                # ★【重要】エラー時もフラグをリセット
                self.is_generating_second_stage = False
                timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                sys.stdout.write(f"[{timestamp}] ✅ Second stage 処理完了（エラー）\n")
                sys.stdout.flush()

                # ★保留中の first_stage リクエストがあれば処理
                if self.pending_first_stage_request:
                    sys.stdout.write(f"[{timestamp}] ▶️  保留中の first_stage リクエストを実行（エラー後）\n")
                    sys.stdout.flush()

                    pending_req = self.pending_first_stage_request
                    self.pending_first_stage_request = None  # 保留キューをクリア

                    # 保留されていたリクエストを実行
                    self.update(
                        words=pending_req['words'],
                        stage='first',
                        turn_taking_decision_timestamp_ns=pending_req['turn_taking_decision_timestamp_ns'],
                        first_stage_backchannel_at_tt=pending_req['first_stage_backchannel_at_tt'],
                        asr_history_2_5s=pending_req['asr_history_2_5s']
                    )

        except Exception as e:
            sys.stdout.write(f"[NLG ERROR] second_stage処理エラー: {e}\n")
            sys.stdout.flush()
            self.last_reply = self.first_stage_response  # リアクションワードのみフォールバック
            self.last_source_words = asr_results

            # ★【重要】外側の例外ハンドラーでもフラグをリセット
            self.is_generating_second_stage = False
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            sys.stdout.write(f"[{timestamp}] ✅ Second stage 処理完了（外部エラー）\n")
            sys.stdout.flush()

            # ★保留中の first_stage リクエストがあれば処理
            if self.pending_first_stage_request:
                sys.stdout.write(f"[{timestamp}] ▶️  保留中の first_stage リクエストを実行（外部エラー後）\n")
                sys.stdout.flush()

                pending_req = self.pending_first_stage_request
                self.pending_first_stage_request = None  # 保留キューをクリア

                # 保留されていたリクエストを実行
                self.update(
                    words=pending_req['words'],
                    stage='first',
                    turn_taking_decision_timestamp_ns=pending_req['turn_taking_decision_timestamp_ns'],
                    first_stage_backchannel_at_tt=pending_req['first_stage_backchannel_at_tt'],
                    asr_history_2_5s=pending_req['asr_history_2_5s']
                )

    def _perform_simple_inference(self, query):
        """シンプルな単一スレッド推論 (gemma3:12b使用)"""
        start_time = datetime.now()

        # 推論開始チェックポイント
        if self.current_session_id:
            self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "inference_start", {
                "query_type": "list" if isinstance(query, list) else "string",
                "query_length": len(query) if isinstance(query, list) else len(str(query))
            })

        try:
            res = ""  # resを必ず初期化
            asr_results = self.asr_results

            if asr_results and isinstance(asr_results, list) and len(asr_results) >= 1:
                if all((not x or x.strip() == "") for x in asr_results):
                    self.last_reply = ""
                    self.last_source_words = []
                    return

                # 特定プロンプトファイル使用時は [雑音][無音]<unk> を除去
                noise_tag_removal_prompts = [
                    "dialog_simple.txt",
                    "fix_asr_simple.txt",
                    "dialog_predict.txt",
                    "dialog_explain.txt",
                    "dialog_example.txt"
                ]

                if self.prompt_file_name in noise_tag_removal_prompts:
                    # 不要なタグを除去
                    cleaned_asr_results = []
                    for asr in asr_results:
                        # [雑音]、[無音]、<unk>を除去
                        cleaned = asr.replace("[雑音]", "").replace("[無音]", "").replace("<unk>", "")
                        cleaned = cleaned.strip()
                        if cleaned:  # 空文字列でない場合のみ追加
                            cleaned_asr_results.append(cleaned)

                    # クリーニング後のリストを使用
                    asr_results_for_prompt = cleaned_asr_results if cleaned_asr_results else asr_results
                else:
                    # その他のプロンプトではそのまま使用
                    asr_results_for_prompt = asr_results

                # 音声認識結果をすべて列挙
                asr_lines = []
                for idx, asr in enumerate(asr_results_for_prompt):
                    asr_lines.append(f"認識結果{idx+1}: {asr}")

                # プロンプトを外部ファイルから読み込み（設定したファイル名を使用）
                try:
                    with open(self.prompt_file_path, 'r', encoding='utf-8') as f:
                        prompt = f.read()
                    if not prompt.strip():
                        sys.stdout.write(f"[NLG ERROR] プロンプトファイルが空です: {self.prompt_file_path}\n")
                        sys.stdout.flush()
                        return
                except FileNotFoundError:
                    sys.stdout.write(f"[NLG ERROR] プロンプトファイルが見つかりません: {self.prompt_file_path}\n")
                    sys.stdout.flush()
                    return
                except Exception as e:
                    sys.stdout.write(f"[NLG ERROR] プロンプトファイル読み込みエラー: {self.prompt_file_path} - {e}\n")
                    sys.stdout.flush()
                    return
                
                # LLM呼び出し
                llm_start_time = datetime.now()
                sys.stdout.write(f"[{llm_start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🤖 {self.model_name}推論開始\n")
                # LLM推論開始チェックポイント
                if self.current_session_id:
                    self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "llm_start", {
                        "model": self.model_name,
                        "prompt_type": "asr_dialogue",
                        "prompt_length": len(prompt),
                        "asr_count": len(asr_results)
                    })

                # モデルタイプに応じた推論処理
                try:
                    if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                        # Ollama モデル（API直接呼び出し、ストリーミング有効）
                        full_prompt = f"{prompt}\n\nぶつ切りの音声認識結果: {', '.join(asr_results_for_prompt)}"

                        # Ollama API直接呼び出し（ストリーミング）
                        try:
                            api_start_time = datetime.now()
                            api_response = requests.post(
                                'http://localhost:11434/api/generate',
                                json={
                                    'model': self.model_name,
                                    'prompt': full_prompt,
                                    'stream': True,  # ストリーミング有効化（TTFT最小化）
                                    'options': {
                                        'temperature': 0.7,
                                        'top_p': 0.9,
                                        'num_predict': 10,  # 短いリアクションワードで高速化
                                        'num_ctx': 4096,
                                        'num_batch': 3072
                                    }
                                },
                                timeout=30,
                                stream=True
                            )

                            res = ""
                            first_token_time = None

                            if api_response.status_code == 200:
                                for line in api_response.iter_lines():
                                    if line:
                                        try:
                                            chunk_data = json.loads(line)
                                            token_fragment = chunk_data.get('response', '')

                                            if token_fragment:
                                                # TTFT計測
                                                if first_token_time is None:
                                                    first_token_time = datetime.now()
                                                    ttft_ms = (first_token_time - api_start_time).total_seconds() * 1000
                                                    sys.stdout.write(f"[{first_token_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🎯 TTFT: {ttft_ms:.1f}ms\n")
                                                    sys.stdout.flush()

                                                res += token_fragment

                                            # 完了チェック
                                            if chunk_data.get('done', False):
                                                llm_end_time = datetime.now()
                                                llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000

                                                # メトリクス記録（詳細）
                                                load_duration = chunk_data.get('load_duration', 0) / 1e6
                                                prompt_eval_duration = chunk_data.get('prompt_eval_duration', 0) / 1e6
                                                eval_duration = chunk_data.get('eval_duration', 0) / 1e6
                                                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ⏱️ load: {load_duration:.1f}ms, prompt: {prompt_eval_duration:.1f}ms, eval: {eval_duration:.1f}ms → total: {llm_duration:.1f}ms\n")
                                                break
                                        except json.JSONDecodeError:
                                            continue
                            else:
                                res = "申し訳ありません、応答の生成に失敗しました。"
                        except Exception as api_error:
                            sys.stdout.write(f"[NLG ERROR] Ollama API呼び出しエラー: {api_error}\n")
                            sys.stdout.flush()
                            res = "申し訳ありません、応答の生成に失敗しました。"

                    elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                        # OpenAI API（GPT-5, GPT-4, o1など）
                        messages = [
                            {"role": "system", "content": prompt}
                        ]

                        # ★dialog_example_role.txt使用時は1-shot例示メッセージを追加
                        if self.prompt_file_name == "dialog_example_role.txt":
                            # 1-shot例示：例示ユーザー発話
                            messages.append({
                                "role": "user",
                                "content": "複数のぶつ切りの音声認識結果: 今日会社で新しい, 今日会社で新しいプロジェクトの話があって, プロジェクトの話があって最初はすごく面白そうでやってみ, すごく面白そうでやってみたいって思んだけどシメ, 思んだけど締め切れがかなりタイトだから頑張ら"
                            })
                            # 1-shot例示：例示応答
                            messages.append({
                                "role": "assistant",
                                "content": "アンドロイドの応答: そうなんだ、無理しないで頑張ってね！"
                            })

                        # 対話履歴/例示メッセージを含める場合
                        if hasattr(self, 'example_messages') and self.example_messages:
                            messages.extend(self.example_messages)

                        # 現在のASR結果（user）
                        messages.append({
                            "role": "user",
                            "content": f"複数のぶつ切りの音声認識結果：{', '.join(asr_results_for_prompt)}"
                        })

                        # 応答生成指示（assistant）
                        messages.append({
                            "role": "assistant",
                            "content": "アンドロイドの応答: "
                        })

                        # デバッグ用ログ
                        sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG DEBUG] プロンプト長: {len(prompt)}文字, ASR結果数: {len(asr_results_for_prompt)}, messages形式: {len(messages)}ターン\n")
                        sys.stdout.flush()

                        # モデルタイプ別の最適化設定
                        if self.model_name.startswith("gpt-4.1"):
                            # GPT-4.1系: 最速540ms
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=50,
                                temperature=0.3
                            )
                        elif "chat-latest" in self.model_name:
                            # GPT-5-chat-latest: 推論最小化版
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=50
                            )
                        elif self.model_name.startswith("gpt-5") or self.model_name.startswith("o1"):
                            # GPT-5/o1: 推論モデル
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=500,
                                reasoning_effort="low"
                            )
                        else:
                            # GPT-4o系: 標準
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_tokens=50,
                                temperature=0.3
                            )

                        res = response.choices[0].message.content.strip() if response.choices[0].message.content else ""

                        # デバッグ用ログ
                        sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG DEBUG] API応答長: {len(res)}文字\n")
                        sys.stdout.flush()

                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ {self.model_name}推論完了 (LLM時間: {llm_duration:.1f}ms)\n")
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 生成応答: '{res}'\n")
                    sys.stdout.flush()

                except Exception as api_error:
                    # API呼び出し失敗時はエラーメッセージを設定
                    res = "申し訳ありません、応答の生成に失敗しました。"
                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ❌ {self.model_name} API呼び出しエラー: {api_error}\n")
                    sys.stdout.flush()

                # LLM推論完了チェックポイント
                if self.current_session_id:
                    self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "llm_complete", {
                        "model": self.model_name,
                        "llm_duration_ms": llm_duration,
                        "response_length": len(res)
                    })
                
                source_words = asr_results
                
            else:
                if not query or (isinstance(query, list) and all((not x or x.strip() == "") for x in query)):
                    self.last_reply = ""
                    self.last_source_words = []
                    return
                elif query == "dummy":
                    res = "はい"
                    source_words = [str(query)]
                else:
                    text_input = query
                    role = "優しい性格のアンドロイドとして、ユーザーの発話に対して相手を労るような返答のみを２０文字以内でしてください。"

                    llm_start_time = datetime.now()

                    if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                        # Ollama gemma3系モデル（LangChain経由）
                        messages = [
                            ("system", role),
                            ("human", text_input)
                        ]
                        query_prompt = ChatPromptTemplate.from_messages(messages)
                        chain = query_prompt | self.ollama_model | StrOutputParser()
                        res = chain.invoke({})

                    elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                        # OpenAI API（GPT-5, GPT-4, o1など）
                        messages = [
                            {"role": "system", "content": role},
                            {"role": "user", "content": text_input}
                        ]

                        # モデルタイプ別の最適化設定
                        if self.model_name.startswith("gpt-4.1"):
                            # GPT-4.1系: 最速540ms
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=50,
                                temperature=0.3
                            )
                        elif "chat-latest" in self.model_name:
                            # GPT-5-chat-latest: 推論最小化版
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=50
                            )
                        elif self.model_name.startswith("gpt-5") or self.model_name.startswith("o1"):
                            # GPT-5/o1: 推論モデル
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_completion_tokens=500,
                                reasoning_effort="low"
                            )
                        else:
                            # GPT-4o系: 標準
                            response = openai.chat.completions.create(
                                model=self.model_name,
                                messages=messages,
                                max_tokens=50,
                                temperature=0.3
                            )

                        res = response.choices[0].message.content.strip()

                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ {self.model_name}推論完了 (LLM時間: {llm_duration:.1f}ms)\n")
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 生成応答: '{res}'\n")
                    sys.stdout.flush()

                    if ":" in res:
                        res = res.split(":", 1)[1]

                    source_words = [str(query)]
            
            # 改行を除去して1行にする
            res = res.replace('\n', '').replace('\r', '')
            
            end_time = datetime.now()
            total_duration = (end_time - start_time).total_seconds() * 1000
            
            # 結果を設定
            self.last_reply = res
            self.last_source_words = source_words
            
            # タイミング情報を設定
            self.request_id = 1
            self.worker_name = "nlg-single"
            self.start_timestamp_ns = int(start_time.timestamp() * 1_000_000_000)
            self.completion_timestamp_ns = int(end_time.timestamp() * 1_000_000_000)
            self.inference_duration_ms = total_duration
            
            # 成功時は接続エラーカウントをリセット
            self.connection_error_count = 0
            self.connection_error_suppress_until = None
            
            # 推論完了チェックポイント
            if self.current_session_id:
                self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "inference_complete", {
                    "total_duration_ms": total_duration,
                    "response": res,
                    "source_words": source_words
                })
            
            sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🏁 処理完了 (総時間: {total_duration:.1f}ms): {res}\n")
            sys.stdout.flush()
            
        except Exception as e:
            end_time = datetime.now()
            
            # 接続エラーの場合は特別処理
            error_str = str(e)
            is_connection_error = (
                "[Errno 111] Connection refused" in error_str or
                "llama runner process has terminated" in error_str or
                "broken pipe" in error_str or
                "status code: 500" in error_str
            )
            
            if is_connection_error:
                self.connection_error_count += 1
                # 連続接続エラーが5回以上なら30秒間リクエスト抑制
                if self.connection_error_count >= 5:
                    self.connection_error_suppress_until = end_time + timedelta(seconds=30)
                    if self.connection_error_count == 5:  # 初回抑制時のみログ出力
                        sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG WARNING] 🚫 連続接続エラー検出。30秒間リクエスト抑制します\n")
                        sys.stdout.flush()
                # 接続エラーは詳細ログを抑制
                if self.connection_error_count <= 3:  # 最初の3回のみログ出力
                    sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG ERROR] ❌ 接続エラー: Ollama接続失敗\n")
                    sys.stdout.flush()
            else:
                # 接続エラー以外の場合はカウントリセット
                self.connection_error_count = 0
                self.connection_error_suppress_until = None
                sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG ERROR] ❌ 推論エラー: {e}\n")
                sys.stdout.flush()
            
            # エラー時のフォールバック応答（固定応答のみ）
            if is_connection_error:
                # Ollamaサーバーエラー時は簡単な固定応答のみ
                fallback_responses = [
                    "そうですね。",
                    "なるほど。", 
                    "わかりました。",
                    "はい。",
                    "そうなんですね。"
                ]
                import random
                fallback_response = random.choice(fallback_responses)
                
                self.last_reply = fallback_response
                self.last_source_words = query if isinstance(query, list) else [str(query)]
                
                # フォールバック時のタイミング情報設定
                self.request_id = 999  # 固定応答ID
                self.worker_name = "static-fallback"
                self.start_timestamp_ns = int(start_time.timestamp() * 1_000_000_000)
                self.completion_timestamp_ns = int(end_time.timestamp() * 1_000_000_000)
                self.inference_duration_ms = (end_time - start_time).total_seconds() * 1000
                
                if self.connection_error_count <= 3:
                    sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG FALLBACK] 🔄 固定応答フォールバック: {fallback_response}\n")
                    sys.stdout.flush()
            else:
                # その他のエラー時は空の結果を設定
                self.last_reply = ""
                self.last_source_words = []


    def run(self):
        sys.stdout.write("[NLG] 単一プロセス推論システム開始 (2.5秒間隔制御)\n")
        sys.stdout.write(f"[NLG] 使用モデル: {self.model_name}\n")
        sys.stdout.flush()
        
        # 並列処理版をコメントアウト
        # while True:
        #     # 並列推論システムでは結果監視のみ
        #     try:
        #         # 完了した推論結果があれば処理
        #         if not self.result_queue.empty():
        #             result = self.result_queue.get_nowait()
        #             # 結果処理はワーカー内で完結するため、ここでは特に処理なし
        #     except:
        #         pass
        #     
        #     time.sleep(0.01)  # 10ms待機
        
        # 単一プロセス版では特に無限ループは不要
        pass

if __name__ == "__main__":
    gen = NaturalLanguageGeneration()
    gen.run()