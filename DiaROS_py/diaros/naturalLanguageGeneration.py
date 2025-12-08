# ============================================================
# モデル設定 - ここでモデルを切り替え
# ============================================================
# 【OpenAI API モデル】クラウドAPI、高速・高品質
# MODEL_NAME = "gpt-3.5-turbo-0125"    # 587ms - 最速・最安・安定（推奨）
# MODEL_NAME = "gpt-4.1-nano"          # 604ms - 最新技術・高速
# MODEL_NAME = "gpt-5-chat-latest"     # 708ms - GPT-5最速版・安定
# MODEL_NAME = "gpt-oss:20b"
# 【Ollama ローカルモデル】オフライン動作、GPU必要
MODEL_NAME = "gemma3:4b"             # 軽量・高速
# MODEL_NAME = "gemma3:12b"            # 高品質
# MODEL_NAME = "gemma3:27b"            # 最高品質

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
PROMPT_FILE_NAME = "dialog_first_stage.txt"     # 200ms以内達成用（短い相槌のみ）

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
    def __init__(self):
        self.rc = { "word": "" }

        self.query = ""
        self.update_flag = False
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納
        self.last_source_words = []  # 対話生成の元にした音声認識結果を格納

        # 二段階応答生成用の変数
        self.first_stage_response = ""  # first_stageで生成した相槌を保存
        self.current_stage = "first"  # first または second
        self.turn_taking_decision_timestamp_ns = 0  # TurnTaking判定時刻（ナノ秒）

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
                num_predict = 10  # gemma3系は10トークンで統一（短い相槌用）

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

        # プロンプトファイルの存在確認
        self.prompt_file_name = PROMPT_FILE_NAME
        prompt_dir = os.path.join(os.path.dirname(__file__), 'prompts')
        self.prompt_file_path = os.path.join(prompt_dir, self.prompt_file_name)

        if os.path.exists(self.prompt_file_path):
            sys.stdout.write(f'[NLG] ✅ プロンプトファイル確認: {self.prompt_file_name}\n')
            sys.stdout.flush()
        else:
            sys.stdout.write(f'[NLG WARNING] ⚠️  プロンプトファイルが見つかりません: {self.prompt_file_path}\n')
            sys.stdout.flush()

        sys.stdout.write('NaturalLanguageGeneration (単一プロセス) start up.\n')
        sys.stdout.write(f'使用モデル: {self.model_name}\n')
        sys.stdout.write('=====================================================\n')

    def update(self, query):
        now = datetime.now()

        # 接続エラー抑制中は新しいリクエストを受け付けない
        if self.connection_error_suppress_until and now < self.connection_error_suppress_until:
            return

        # 音声認識結果がリストの場合はプロンプトに埋め込む
        self.asr_results = None
        # 空リストまたは全て空文字列なら何もしない
        if isinstance(query, list):
            if not query or all((not x or x.strip() == "") for x in query):
                self.update_flag = False
                return
            self.asr_results = query
            self.query = query
        else:
            if not query or (isinstance(query, str) and query.strip() == ""):
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

        sys.stdout.write(f"[{now.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🚀 推論開始 (ID: {request_id}, モデル: {self.model_name})\n")
        sys.stdout.flush()

        # 単一プロセス推論を実行（並列処理をコメントアウト）
        # future = self.executor.submit(self._perform_parallel_inference, request_id, query, now)

        # シンプルな推論実行に戻す
        self._perform_simple_inference(query)

        # 最後の推論時刻を更新
        self.last_inference_time = now
        self.last_request_time = now
        self.update_flag = True
        
    def set_session_id(self, session_id: str):
        """セッションIDを設定"""
        self.current_session_id = session_id

    def generate_first_stage(self, query):
        """First stage: 常に相槌を生成（音声認識結果が来るたびに実行）"""
        start_time = datetime.now()

        try:
            asr_results = query if isinstance(query, list) else [str(query)]

            if not asr_results or all((not x or x.strip() == "") for x in asr_results):
                self.first_stage_response = ""
                return

            # LLM呼び出し
            llm_start_time = datetime.now()
            sys.stdout.write(f"[{llm_start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE] 🤖 相槌生成開始\n")

            try:
                if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                    # Ollama APIを直接呼び出してTTFT計測（ストリーミング）
                    import requests

                    prompt_build_start = datetime.now()
                    # 超シンプルなプロンプト
                    simple_prompt = f"短い相槌を一つ: {', '.join(asr_results[:2])}"
                    prompt_build_end = datetime.now()
                    sys.stdout.write(f"[{prompt_build_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] プロンプト構築: {(prompt_build_end - prompt_build_start).total_seconds() * 1000:.1f}ms\n")
                    sys.stdout.write(f"[{prompt_build_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] プロンプト長: {len(simple_prompt)}文字\n")
                    sys.stdout.flush()

                    # Ollama API直接呼び出し（ストリーミング）
                    api_start = datetime.now()
                    response = requests.post(
                        'http://localhost:11434/api/generate',
                        json={
                            'model': self.model_name,
                            'prompt': simple_prompt,
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
                                token_fragment = chunk_data.get('response', '')

                                if token_fragment:
                                    token_count += 1

                                    # Time to First Token (TTFT) 計測
                                    if first_token_time is None:
                                        first_token_time = datetime.now()
                                        ttft_ms = (first_token_time - api_start).total_seconds() * 1000
                                        sys.stdout.write(f"[{first_token_time.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] 🎯 TTFT (Time to First Token): {ttft_ms:.1f}ms\n")
                                        sys.stdout.flush()

                                    res += token_fragment

                                # 完了チェック
                                if chunk_data.get('done', False):
                                    api_end = datetime.now()
                                    total_time = (api_end - api_start).total_seconds() * 1000

                                    # 詳細メトリクス取得
                                    load_duration = chunk_data.get('load_duration', 0) / 1e6  # ns → ms
                                    prompt_eval_duration = chunk_data.get('prompt_eval_duration', 0) / 1e6
                                    eval_duration = chunk_data.get('eval_duration', 0) / 1e6
                                    prompt_eval_count = chunk_data.get('prompt_eval_count', 0)
                                    eval_count = chunk_data.get('eval_count', 0)

                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] LLM推論時間（総計）: {total_time:.1f}ms\n")
                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] トークン数: {token_count}\n")
                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] ⚙️ load_duration: {load_duration:.1f}ms\n")
                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] ⚙️ prompt_eval_duration: {prompt_eval_duration:.1f}ms ({prompt_eval_count} tokens)\n")
                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] ⚙️ eval_duration: {eval_duration:.1f}ms ({eval_count} tokens)\n")
                                    sys.stdout.write(f"[{api_end.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE DEBUG] ⚠️ オーバーヘッド分析: TTFT({ttft_ms:.1f}ms) - prompt_eval({prompt_eval_duration:.1f}ms) - load({load_duration:.1f}ms) = {ttft_ms - prompt_eval_duration - load_duration:.1f}ms\n")
                                    sys.stdout.flush()
                                    break

                            except json.JSONDecodeError:
                                continue

                elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                    # 超シンプルなプロンプト
                    simple_prompt = f"短い相槌を一つ: {', '.join(asr_results[:2])}"

                    messages = [
                        {"role": "system", "content": simple_prompt},
                        {"role": "user", "content": "上記の音声認識結果から相槌を一つ出力してください。"}
                    ]
                    response = openai.chat.completions.create(
                        model=self.model_name,
                        messages=messages,
                        max_completion_tokens=20,
                        temperature=0.3
                    )
                    res = response.choices[0].message.content.strip() if response.choices[0].message.content else ""

                # 相槌の後処理: 改行・句読点除去
                res = res.replace('\n', '').replace('\r', '').replace('。', '').replace('、', '').strip()

                llm_end_time = datetime.now()
                llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000

                self.first_stage_response = res
                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG FIRST_STAGE] ✅ 相槌生成完了 ({llm_duration:.1f}ms): '{res}'\n")
                sys.stdout.flush()

            except Exception as api_error:
                sys.stdout.write(f"[NLG ERROR] first_stage生成エラー: {api_error}\n")
                sys.stdout.flush()
                self.first_stage_response = "うん"  # フォールバック

        except Exception as e:
            sys.stdout.write(f"[NLG ERROR] first_stage処理エラー: {e}\n")
            sys.stdout.flush()
            self.first_stage_response = "うん"  # フォールバック

    def generate_second_stage(self, query):
        """Second stage: turnTakingが応答判定を出したら実行"""
        start_time = datetime.now()

        try:
            asr_results = query if isinstance(query, list) else [str(query)]

            if not asr_results or all((not x or x.strip() == "") for x in asr_results):
                self.last_reply = ""
                self.last_source_words = []
                return

            # プロンプトファイル読み込み
            prompt_dir = os.path.join(os.path.dirname(__file__), 'prompts')
            second_stage_prompt_path = os.path.join(prompt_dir, 'dialog_second_stage.txt')

            try:
                with open(second_stage_prompt_path, 'r', encoding='utf-8') as f:
                    prompt_template = f.read()

                # {first_stage_response} を実際の相槌に置換（エスケープ不要）
                # プロンプトと音声認識結果を直接結合
                prompt_with_backchannel = prompt_template.replace('{first_stage_response}', self.first_stage_response)
                prompt = f"{prompt_with_backchannel}\n\n# 先ほど打った相槌\n{self.first_stage_response}\n\n# 音声認識結果\nぶつ切りの音声認識結果: {', '.join(asr_results)}"

            except FileNotFoundError:
                sys.stdout.write(f"[NLG ERROR] second_stageプロンプトが見つかりません: {second_stage_prompt_path}\n")
                sys.stdout.flush()
                return

            # LLM呼び出し
            llm_start_time = datetime.now()
            sys.stdout.write(f"[{llm_start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG SECOND_STAGE] 🤖 本応答生成開始（相槌: '{self.first_stage_response}'）\n")

            try:
                if self.model_name.startswith("gemma3:") or self.model_name.startswith("gpt-oss:"):
                    messages = [
                        ("system", prompt),
                        ("human", "上記の音声認識結果から本応答を生成してください。")
                    ]
                    query_prompt = ChatPromptTemplate.from_messages(messages)
                    chain = query_prompt | self.ollama_model | StrOutputParser()
                    res = chain.invoke({})

                elif self.model_name.startswith("gpt-") or self.model_name.startswith("o1"):
                    messages = [
                        {"role": "system", "content": prompt},
                        {"role": "user", "content": "上記の音声認識結果から本応答を生成してください。"}
                    ]
                    response = openai.chat.completions.create(
                        model=self.model_name,
                        messages=messages,
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

                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG SECOND_STAGE] ✅ 本応答生成完了 ({llm_duration:.1f}ms): '{res}'\n")
                sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG SECOND_STAGE] 🏁 最終応答: '{final_response}'\n")
                sys.stdout.flush()

            except Exception as api_error:
                sys.stdout.write(f"[NLG ERROR] second_stage生成エラー: {api_error}\n")
                sys.stdout.flush()
                self.last_reply = self.first_stage_response  # 相槌のみフォールバック
                self.last_source_words = asr_results

        except Exception as e:
            sys.stdout.write(f"[NLG ERROR] second_stage処理エラー: {e}\n")
            sys.stdout.flush()
            self.last_reply = self.first_stage_response  # 相槌のみフォールバック
            self.last_source_words = asr_results

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
                                        'num_predict': 10,  # 短い相槌で高速化
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
                            {"role": "system", "content": prompt},
                            {"role": "user", "content": f"ぶつ切りの音声認識結果: {', '.join(asr_results_for_prompt)}"}
                        ]

                        # デバッグ用ログ
                        sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG DEBUG] プロンプト長: {len(prompt)}文字\n")
                        sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG DEBUG] 音声認識結果数: {len(asr_results_for_prompt)}\n")
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

<<<<<<< HEAD

    def run(self):
        sys.stdout.write("[NLG] 単一プロセス推論システム開始 (2.5秒間隔制御)\n")
        sys.stdout.write(f"[NLG] 使用モデル: {self.model_name}\n")
        sys.stdout.flush()
        
=======
    # _perform_parallel_inference() メソッドは現在使用されていないためコメントアウト
    # GPT-3.5-turbo版は必要になったら実装
    """
    def _perform_parallel_inference(self, request_id, query, start_time):
        \"""並列推論を実行するワーカー関数\"""
        worker_name = f"nlg-worker-{request_id % 3 + 1}"  # 3つのワーカーで交互実行
        
        sys.stdout.write(f"[{start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🔄 {worker_name} 推論開始\n")
        sys.stdout.flush()
        
        # 推論開始チェックポイント
        if self.current_session_id:
            self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "parallel_inference_start", {
                "worker_name": worker_name,
                "request_id": request_id,
                "query_type": "list" if isinstance(query, list) else "string",
                "query_length": len(query) if isinstance(query, list) else len(str(query))
            })
        
        try:
            res = ""  # resを必ず初期化
            asr_results = self.asr_results if hasattr(self, 'asr_results') else None
            
            if asr_results and isinstance(asr_results, list) and len(asr_results) >= 1:
                if all((not x or x.strip() == "") for x in asr_results):
                    self.last_reply = ""
                    self.last_source_words = []
                    return
                
                # 音声認識結果をすべて列挙
                asr_lines = []
                for idx, asr in enumerate(asr_results):
                    asr_lines.append(f"認識結果{idx+1}: {asr}")
                
                # プロンプトを外部ファイルから読み込み
                current_dir = os.path.dirname(os.path.abspath(__file__))
                prompt_file_path = os.path.join(current_dir, "prompts", "example_dialog.txt")

                if not os.path.exists(prompt_file_path):
                    workspace_path = "/workspace/DiaROS/DiaROS_py/diaros/prompts/example_dialog.txt"
                    if os.path.exists(workspace_path):
                        prompt_file_path = workspace_path
                
                try:
                    with open(prompt_file_path, 'r', encoding='utf-8') as f:
                        prompt = f.read()
                    if not prompt.strip():
                        sys.stdout.write(f"[NLG ERROR] プロンプトファイルが空です: {prompt_file_path}\n")
                        sys.stdout.flush()
                        return
                except FileNotFoundError:
                    sys.stdout.write(f"[NLG ERROR] プロンプトファイルが見つかりません: {prompt_file_path}\n")
                    sys.stdout.flush()
                    return
                except Exception as e:
                    sys.stdout.write(f"[NLG ERROR] プロンプトファイル読み込みエラー: {prompt_file_path} - {e}\n")
                    sys.stdout.flush()
                    return
                
                # プロンプト作成（音声認識結果を組み込み）
                full_prompt = f"{prompt}\n\nぶつ切りの音声認識結果: {', '.join(asr_results)}"
                
                # LLM呼び出し
                llm_start_time = datetime.now()
                sys.stdout.write(f"[{llm_start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🤖 {worker_name} Ollama推論開始\n")
                
                # LLM推論開始チェックポイント
                if self.current_session_id:
                    self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "llm_start", {
                        "model": self.model_name,
                        "worker_name": worker_name,
                        "prompt_type": "asr_dialogue",
                        "prompt_length": len(prompt),
                        "asr_count": len(asr_results)
                    })
                
                # Ollama APIを直接呼び出して対話生成と統計情報を取得（ストリーミングモード）
                try:
                    import requests
                    api_response = requests.post('http://localhost:11434/api/generate',
                        json={
                            'model': self.model_name,
                            'prompt': full_prompt,
                            'stream': True,  # ストリーミングモードを有効化
                            'options': {
                                'temperature': 0.7,
                                'top_p': 0.9,
                                'num_predict': 50,
                                'num_ctx': 4096,
                                'num_batch': 3072
                            }
                        },
                        timeout=30,
                        stream=True  # requestsのストリーミングも有効化
                    )

                    if api_response.status_code == 200:
                        res = ""
                        token_count = 0
                        first_token_time = None
                        last_token_time = llm_start_time
                        token_times = []  # 各トークンの時間差を記録

                        # ストリーミングレスポンスを逐次処理
                        for line in api_response.iter_lines():
                            if line:
                                current_time = datetime.now()
                                try:
                                    chunk_data = json.loads(line)
                                    token_fragment = chunk_data.get('response', '')

                                    if token_fragment:
                                        token_count += 1

                                        # Time to First Token (TTFT) を計測
                                        if first_token_time is None:
                                            first_token_time = current_time
                                            ttft_ms = (first_token_time - llm_start_time).total_seconds() * 1000
                                            sys.stdout.write(f"[{current_time.strftime('%H:%M:%S.%f')[:-3]}][NLG TOKEN] 🎯 {worker_name} 最初のトークン受信 (TTFT: {ttft_ms:.1f}ms)\n")
                                            sys.stdout.flush()

                                        # Inter-Token Latency (ITL) を計測
                                        itl_ms = (current_time - last_token_time).total_seconds() * 1000
                                        token_times.append(itl_ms)

                                        # トークンごとの詳細ログ出力
                                        sys.stdout.write(f"[{current_time.strftime('%H:%M:%S.%f')[:-3]}][NLG TOKEN] {worker_name} #{token_count}: '{token_fragment}' (ITL: {itl_ms:.1f}ms)\n")
                                        sys.stdout.flush()

                                        res += token_fragment
                                        last_token_time = current_time

                                    # 最終チャンクの処理
                                    if chunk_data.get('done', False):
                                        llm_end_time = current_time
                                        llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000

                                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ {worker_name} Ollama推論完了 (LLM時間: {llm_duration:.1f}ms)\n")

                                        # トークン統計情報
                                        if token_count > 0 and first_token_time:
                                            avg_itl = sum(token_times) / len(token_times) if token_times else 0
                                            ttft_ms = (first_token_time - llm_start_time).total_seconds() * 1000
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG STATS] 📊 {worker_name} トークン数: {token_count}\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG STATS] 📊 {worker_name} TTFT (最初のトークンまで): {ttft_ms:.1f}ms\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG STATS] 📊 {worker_name} 平均ITL (トークン間隔): {avg_itl:.1f}ms\n")

                                        # Verbose統計情報（Ollama提供）
                                        if 'total_duration' in chunk_data:
                                            total_duration_ms = chunk_data['total_duration'] / 1_000_000
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} かかった時間: {total_duration_ms/1000:.6f}s\n")

                                        if 'load_duration' in chunk_data:
                                            load_duration_ms = chunk_data['load_duration'] / 1_000_000
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} モデルロード時間: {load_duration_ms:.3f}ms\n")

                                        if 'prompt_eval_count' in chunk_data and 'prompt_eval_duration' in chunk_data:
                                            prompt_tokens = chunk_data['prompt_eval_count']
                                            prompt_eval_ms = chunk_data['prompt_eval_duration'] / 1_000_000
                                            tokens_per_sec = prompt_tokens / (prompt_eval_ms / 1000) if prompt_eval_ms > 0 else 0
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} 入力プロンプトのトークン数: {prompt_tokens} token(s)\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} 入力プロンプトの処理時間: {prompt_eval_ms:.3f}ms\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} 入力プロンプトの処理トークン/s: {tokens_per_sec:.2f} tokens/s\n")

                                        if 'eval_count' in chunk_data and 'eval_duration' in chunk_data:
                                            output_tokens = chunk_data['eval_count']
                                            eval_ms = chunk_data['eval_duration'] / 1_000_000
                                            output_tokens_per_sec = output_tokens / (eval_ms / 1000) if eval_ms > 0 else 0
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} モデル出力のトークン数: {output_tokens} token(s)\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} モデル出力にかかった時間: {eval_ms/1000:.6f}s\n")
                                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} モデル出力の処理トークン/s: {output_tokens_per_sec:.2f} tokens/s\n")

                                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} 生成応答: '{res}'\n")
                                        sys.stdout.flush()
                                        break

                                except json.JSONDecodeError:
                                    continue

                        # ストリーミングが完了しても llm_end_time が設定されていない場合
                        if 'llm_end_time' not in locals():
                            llm_end_time = datetime.now()
                            llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    else:
                        # API呼び出し失敗時はエラーメッセージを設定
                        res = "申し訳ありません、応答の生成に失敗しました。"
                        llm_end_time = datetime.now()
                        llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ❌ {worker_name} Ollama API呼び出し失敗 (status: {api_response.status_code})\n")
                        sys.stdout.flush()

                except Exception as api_error:
                    # API呼び出し失敗時はエラーメッセージを設定
                    res = "申し訳ありません、応答の生成に失敗しました。"
                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ❌ {worker_name} Ollama API呼び出しエラー: {api_error}\n")
                    sys.stdout.flush()
                
                # LLM推論完了チェックポイント
                if self.current_session_id:
                    self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "llm_complete", {
                        "model": self.model_name,
                        "worker_name": worker_name,
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
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ {worker_name} {self.model_name}推論完了 (LLM時間: {llm_duration:.1f}ms)\n")
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] {worker_name} 生成応答: '{res}'\n")
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
            self.request_id = request_id
            self.worker_name = worker_name
            self.start_timestamp_ns = int(start_time.timestamp() * 1_000_000_000)
            self.completion_timestamp_ns = int(end_time.timestamp() * 1_000_000_000)
            self.inference_duration_ms = total_duration
            
            # 成功時は接続エラーカウントをリセット
            self.connection_error_count = 0
            self.connection_error_suppress_until = None
            
            # 推論完了チェックポイント
            if self.current_session_id:
                self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "parallel_inference_complete", {
                    "worker_name": worker_name,
                    "request_id": request_id,
                    "total_duration_ms": total_duration,
                    "response": res,
                    "source_words": source_words
                })
            
            sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🏁 {worker_name} 処理完了 (総時間: {total_duration:.1f}ms): {res}\n")
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
                        sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG WARNING] 🚫 {worker_name} 連続接続エラー検出。30秒間リクエスト抑制します\n")
                        sys.stdout.flush()
                # 接続エラーは詳細ログを抑制
                if self.connection_error_count <= 3:  # 最初の3回のみログ出力
                    sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG ERROR] ❌ {worker_name} 接続エラー: Ollama接続失敗\n")
                    sys.stdout.flush()
            else:
                # 接続エラー以外の場合はカウントリセット
                self.connection_error_count = 0
                self.connection_error_suppress_until = None
                sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG ERROR] ❌ {worker_name} 推論エラー: {e}\n")
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
                self.worker_name = f"static-fallback-{worker_name}"
                self.start_timestamp_ns = int(start_time.timestamp() * 1_000_000_000)
                self.completion_timestamp_ns = int(end_time.timestamp() * 1_000_000_000)
                self.inference_duration_ms = (end_time - start_time).total_seconds() * 1000
                
                if self.connection_error_count <= 3:
                    sys.stdout.write(f"[{end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG FALLBACK] 🔄 {worker_name} 固定応答フォールバック: {fallback_response}\n")
                    sys.stdout.flush()
            else:
                # その他のエラー時は空の結果を設定
                self.last_reply = ""
                self.last_source_words = []
    """

    def run(self):
        sys.stdout.write("[NLG] 単一プロセス推論システム開始 (2.5秒間隔制御)\n")
        sys.stdout.write(f"[NLG] 使用モデル: {self.model_name}\n")
        sys.stdout.flush()
        
>>>>>>> 5d1bb974d10e290d00ef142d14ca452a728f451a
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