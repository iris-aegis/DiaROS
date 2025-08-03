# 一旦履歴諦め

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
        self.dialogue_history = []
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納
        self.last_source_words = []  # 対話生成の元にした音声認識結果を格納
        
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
        
        # 並列処理用の設定（一時的にコメントアウト）
        # self.inference_queue = Queue()  # 推論リクエストのキュー
        # self.result_queue = Queue()     # 推論結果のキュー
        # self.request_counter = 0        # リクエストカウンター
        # self.last_request_time = None   # 最後のリクエスト時刻
        # self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="NLG-Worker")
        
        # Ollamaモデルを初期化時に1回だけ作成（再利用）
        sys.stdout.write('[NLG] Ollama ChatOllamaモデルを初期化中...\n')
        sys.stdout.flush()
        self.model_name = "gemma3:12b"  # 使用するモデル名を指定
        self.ollama_model = ChatOllama(
            model=self.model_name,
            verbose=True,  # 詳細ログ有効化
            temperature=0.7,  # 応答の多様性
            top_p=0.9,  # サンプリング設定
            num_predict=1024,  # 最大生成トークン数
            keep_alive="5m",  # モデルをメモリに保持する時間
            # ollamaコマンドの--verboseオプションを有効化
            additional_kwargs={"verbose": True}
        )
        sys.stdout.write('[NLG] ✅ ChatOllamaモデル初期化完了\n')
        sys.stdout.flush()
        
        sys.stdout.write('NaturalLanguageGeneration (単一プロセス start up.\n')
        sys.stdout.write('=====================================================\n')
        # OpenAI APIキーを環境変数から設定
        openai.api_key = os.environ.get("OPENAI_API_KEY")

    def update(self, query):
        # 接続エラー抑制中は新しいリクエストを受け付けない
        # now = datetime.now()
        # if self.connection_error_suppress_until and now < self.connection_error_suppress_until:
        #     return
            
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

        now = datetime.now()  # ← ここを追加

        # 単一スレッドで即座に推論実行
        sys.stdout.write(f"[{now.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🚀 推論開始\n")
        sys.stdout.flush()
        
        # 直接推論を実行
        self._perform_simple_inference(query)
        
        self.update_flag = True
        
    def set_session_id(self, session_id: str):
        """セッションIDを設定"""
        self.current_session_id = session_id


    # def generate_dialogue(self, query):
    #     sys.stdout.write('対話履歴作成\n')
    #     sys.stdout.flush()
    #     response_res = self.response(query)
    #     dialogue_res = response_res
    #     if ":" in dialogue_res:
    #         dialogue_res = dialogue_res.split(":")[1]
    #     self.dialogue_history.append("usr:" + query)
    #     self.dialogue_history.append("sys:" + dialogue_res)
    #     # self.dialogue_historyの最後から４つの要素を保存
    #     if len(self.dialogue_history) > 5:
    #         self.dialogue_history = self.dialogue_history[-4:]
    #     sys.stdout.write('対話履歴作成\n')
    #     sys.stdout.flush()
    #     return response_res
    
    def _perform_simple_inference(self, query):
        """シンプルな単一スレッド推論"""
        start_time = datetime.now()
        
        # 推論開始チェックポイント
        if self.current_session_id:
            self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "inference_start", {
                "query_type": "list" if isinstance(query, list) else "string",
                "query_length": len(query) if isinstance(query, list) else len(str(query))
            })
        
        try:
            # 初期化済みのOllamaモデルを使用
            ollama_model = self.ollama_model
            
            res = ""  # resを必ず初期化
            asr_results = self.asr_results
            
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
                prompt_file_path = os.path.join(current_dir, "prompts", "example_dialog_prompt.txt")
                
                if not os.path.exists(prompt_file_path):
                    workspace_path = "/workspace/DiaROS/DiaROS_py/diaros/prompts/example_dialog_prompt.txt"
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
                
                # プロンプト作成（ASR結果を組み込み）
                full_prompt = f"{prompt}\n\nASR結果: {', '.join(asr_results)}"
                
                # LLM呼び出し
                llm_start_time = datetime.now()
                sys.stdout.write(f"[{llm_start_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] 🤖 Ollama推論開始\n")                
                # LLM推論開始チェックポイント
                if self.current_session_id:
                    self.time_tracker.add_checkpoint(self.current_session_id, "nlg", "llm_start", {
                        "model": self.model_name,
                        "prompt_type": "asr_dialogue",
                        "prompt_length": len(prompt),
                        "asr_count": len(asr_results)
                    })
                
                # Ollama APIを直接呼び出して対話生成と統計情報を取得
                try:
                    import requests
                    api_response = requests.post('http://localhost:11434/api/generate', 
                        json={
                            'model': self.model_name,
                            'prompt': full_prompt,
                            'stream': False,
                            'options': {
                                'temperature': 0.7,
                                'top_p': 0.9,
                                'num_predict': 50
                            }
                        },
                        timeout=30
                    )
                    
                    if api_response.status_code == 200:
                        api_data = api_response.json()
                        res = api_data.get('response', '')
                        
                        # 詳細統計情報をログ出力
                        llm_end_time = datetime.now()
                        llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                        
                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ Ollama推論完了 (LLM時間: {llm_duration:.1f}ms)\n")
                        
                        # Verbose統計情報
                        if 'total_duration' in api_data:
                            total_duration_ms = api_data['total_duration'] / 1_000_000
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] かかった時間: {total_duration_ms/1000:.6f}s\n")
                        
                        if 'load_duration' in api_data:
                            load_duration_ms = api_data['load_duration'] / 1_000_000
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] モデルロード時間: {load_duration_ms:.3f}ms\n")
                        
                        if 'prompt_eval_count' in api_data and 'prompt_eval_duration' in api_data:
                            prompt_tokens = api_data['prompt_eval_count']
                            prompt_eval_ms = api_data['prompt_eval_duration'] / 1_000_000
                            tokens_per_sec = prompt_tokens / (prompt_eval_ms / 1000) if prompt_eval_ms > 0 else 0
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 入力プロンプトのトークン数: {prompt_tokens} token(s)\n")
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 入力プロンプトの処理時間: {prompt_eval_ms:.3f}ms\n")
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 入力プロンプトの処理トークン/s: {tokens_per_sec:.2f} tokens/s\n")
                        
                        if 'eval_count' in api_data and 'eval_duration' in api_data:
                            output_tokens = api_data['eval_count']
                            eval_ms = api_data['eval_duration'] / 1_000_000
                            output_tokens_per_sec = output_tokens / (eval_ms / 1000) if eval_ms > 0 else 0
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] モデル出力のトークン数: {output_tokens} token(s)\n")
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] モデル出力にかかった時間: {eval_ms/1000:.6f}s\n")
                            sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] モデル出力の処理トークン/s: {output_tokens_per_sec:.2f} tokens/s\n")
                        
                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG VERBOSE] 生成応答: '{res}'\n")
                        sys.stdout.flush()
                    else:
                        # API呼び出し失敗時はエラーメッセージを設定
                        res = "申し訳ありません、応答の生成に失敗しました。"
                        llm_end_time = datetime.now()
                        llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                        sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ❌ Ollama API呼び出し失敗 (status: {api_response.status_code})\n")
                        sys.stdout.flush()
                        
                except Exception as api_error:
                    # API呼び出し失敗時はエラーメッセージを設定
                    res = "申し訳ありません、応答の生成に失敗しました。"
                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ❌ Ollama API呼び出しエラー: {api_error}\n")
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

                    messages = [
                        ("system", role),
                        ("human", text_input)
                    ]
                    query_prompt = ChatPromptTemplate.from_messages(messages)
                    chain = query_prompt | ollama_model | StrOutputParser()
                    
                    llm_start_time = datetime.now()                    
                    res = chain.invoke({})
                    
                    llm_end_time = datetime.now()
                    llm_duration = (llm_end_time - llm_start_time).total_seconds() * 1000
                    sys.stdout.write(f"[{llm_end_time.strftime('%H:%M:%S.%f')[:-3]}][NLG] ✅ Ollama推論完了 (LLM時間: {llm_duration:.1f}ms)\n")
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

    # 並列処理版（一時的にコメントアウト）
    # def _perform_inference_old(self, request):
    #     """推論を実行する並列処理関数"""
    #     # ... (省略) ...

    def run(self):
        sys.stdout.write("[NLG] 単一スレッドシステム開始\n")
        sys.stdout.flush()
        
        while True:
            # 単一スレッドシステムでは特に処理なし
            time.sleep(0.01)  # 10ms待機

if __name__ == "__main__":
    gen = NaturalLanguageGeneration()
    gen.run()