# Gemma 3モデル専用の自然言語生成システム

import requests
import json
import sys
import os
import time
import threading
from datetime import datetime
from queue import Queue, Empty
import openai
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

class NaturalLanguageGeneration:
    def __init__(self):
        self.rc = { "word": "" }
        
        self.query = ""
        self.update_flag = False
        self.dialogue_history = []
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納
        self.last_source_words = []  # 対話生成の元にした音声認識結果を格納
        
        # 並列推論用の設定
        self.inference_queue = Queue()  # 推論リクエストのキュー
        self.current_inference_start_time = None  # 現在の推論開始時刻
        self.inference_lock = threading.Lock()  # 推論状態管理用ロック
        self.is_inferencing = False  # 推論中フラグ
        self.latest_result = None  # 最新の推論結果
        self.result_lock = threading.Lock()  # 結果アクセス用ロック
        
        # Gemma 3モデル専用初期化
        sys.stdout.write("[NLG] 🚀 Gemma 3モデルを初期化中...\n")
        sys.stdout.flush()
        
        # Gemma 3モデルの初期化
        if not self._init_gemma3_models():
            sys.stdout.write("[NLG] ❌ Gemma 3モデルの初期化に失敗しました\n")
            sys.exit(1)
        
        # GPU使用状況を確認
        self._check_gpu_status()
        
        # 推論ワーカースレッドを開始
        sys.stdout.write("[NLG] 推論ワーカースレッドを開始中...\n")
        self.worker_thread_1 = threading.Thread(target=self._inference_worker, args=(1,), daemon=True)
        self.worker_thread_2 = threading.Thread(target=self._inference_worker, args=(2,), daemon=True)
        self.worker_thread_1.start()
        self.worker_thread_2.start()
        sys.stdout.write("[NLG] ✅ 推論ワーカースレッド開始完了\n")

        sys.stdout.write('NaturalLanguageGeneration start up.\n')
        sys.stdout.write('=====================================================\n')
        # OpenAI クライアントを初期化
        self.openai_client = openai.OpenAI(
            api_key=os.environ.get("OPENAI_API_KEY")
        )

    def _check_gpu_status(self):
        """GPU使用状況を確認"""
        try:
            import subprocess
            # nvidia-smiでGPU使用状況を確認
            result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total,memory.used,memory.free', '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                gpu_info = result.stdout.strip()
                sys.stdout.write(f"[NLG] 🖥️  GPU状況: {gpu_info}\n")
        except Exception as e:
            sys.stdout.write(f"[NLG] ⚠️  GPU状況確認エラー: {e}\n")
        sys.stdout.flush()

    def _log_gpu_memory_usage(self, context):
        """GPU メモリ使用量をログ出力"""
        try:
            import subprocess
            result = subprocess.run(['nvidia-smi', '--query-gpu=memory.used,memory.total', '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                memory_info = result.stdout.strip()
                if memory_info:
                    used, total = memory_info.split(', ')
                    usage_percent = (int(used) / int(total)) * 100
                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[{timestamp}][NLG] 🖥️  {context}: GPU使用率 {usage_percent:.1f}% ({used}/{total}MB)\n")
                    sys.stdout.flush()
        except Exception:
            # GPU監視エラーは無視（サイレント）
            pass

    def _check_model_completeness(self, model_path, model_size, expected_files):
        """モデルが完全にダウンロードされているかチェック"""
        try:
            if not os.path.exists(model_path):
                return False
            
            # snapshotsディレクトリ内のsafetensorsファイルを確認
            snapshots_dir = os.path.join(model_path, "snapshots")
            if not os.path.exists(snapshots_dir):
                return False
            
            # 最新のスナップショットディレクトリを取得
            snapshot_dirs = [d for d in os.listdir(snapshots_dir) if os.path.isdir(os.path.join(snapshots_dir, d))]
            if not snapshot_dirs:
                return False
            
            latest_snapshot = os.path.join(snapshots_dir, snapshot_dirs[0])
            
            # safetensorsファイルの数をカウント
            safetensors_files = [f for f in os.listdir(latest_snapshot) if f.endswith('.safetensors')]
            actual_files = len(safetensors_files)
            
            sys.stdout.write(f"[NLG] 📊 {model_size}モデル: {actual_files}/{expected_files} ファイル存在\n")
            
            if actual_files >= expected_files:
                return True
            else:
                sys.stdout.write(f"[NLG] ⚠️  {model_size}モデルが不完全です ({actual_files}/{expected_files})\n")
                return False
                
        except Exception as e:
            sys.stdout.write(f"[NLG] ❌ {model_size}モデル確認エラー: {e}\n")
            return False

    def _init_gemma3_models(self):
        """事前ダウンロード済みGemma 3モデルを4ビット量子化で読み込み"""
        try:
            import torch
            from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig
            import os
            
            # GPU使用量確認
            if torch.cuda.is_available():
                gpu_count = torch.cuda.device_count()
                total_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
                sys.stdout.write(f"[NLG] 🖥️  GPU: {gpu_count}台, GPU0メモリ: {total_memory:.1f}GB\n")
            else:
                sys.stdout.write("[NLG] ❌ CUDA対応GPUが見つかりません\n")
                return False
            
            # 事前ダウンロード済みモデルの確認
            cache_dir = "/workspace/models"
            model_name_27b = "google/gemma-3-27b-it"
            model_name_12b = "google/gemma-3-12b-it"
            
            # キャッシュディレクトリ内のモデルを確認
            model_27b_path = os.path.join(cache_dir, "models--google--gemma-3-27b-it")
            model_12b_path = os.path.join(cache_dir, "models--google--gemma-3-12b-it")
            
            # モデル存在確認（簡単なパスチェックのみ）
            model_27b_exists = os.path.exists(model_27b_path) and os.path.exists(os.path.join(model_27b_path, "snapshots"))
            model_12b_exists = os.path.exists(model_12b_path) and os.path.exists(os.path.join(model_12b_path, "snapshots"))
            
            sys.stdout.write(f"[NLG] 📁 事前ダウンロード済みモデル確認:\n")
            sys.stdout.write(f"[NLG]   - Gemma 3 27B: {'✅ 存在' if model_27b_exists else '❌ 未ダウンロード'}\n")
            sys.stdout.write(f"[NLG]   - Gemma 3 12B: {'✅ 存在' if model_12b_exists else '❌ 未ダウンロード'}\n")
            
            # 使用可能なGPUメモリに基づいてモデルを選択
            available_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
            
            # モデル選択ロジック（GPUメモリに基づく自動選択）
            if available_memory >= 24 and model_27b_exists:
                model_name = model_name_27b
                sys.stdout.write(f"[NLG] 🎯 Gemma 3 27Bモデルを使用: {model_name} (GPU: {available_memory:.1f}GB)\n")
            elif model_12b_exists:
                model_name = model_name_12b
                sys.stdout.write(f"[NLG] 🎯 Gemma 3 12Bモデルを使用: {model_name} (GPU: {available_memory:.1f}GB)\n")
            else:
                sys.stdout.write("[NLG] ❌ 利用可能なGemma 3モデルが見つかりません\n")
                return False
            
            # GPU能力に応じたデータ型選択
            if torch.cuda.is_available() and torch.cuda.is_bf16_supported():
                compute_dtype = torch.bfloat16
                sys.stdout.write("[NLG] 🔧 bfloat16を使用（数値安定性最適化）\n")
            else:
                compute_dtype = torch.float16
                sys.stdout.write("[NLG] 🔧 float16を使用\n")
            
            # 4bit量子化設定（速度最適化）
            quantization_config = BitsAndBytesConfig(
                load_in_4bit=True,
                bnb_4bit_quant_type="nf4",
                bnb_4bit_use_double_quant=False,  # 速度重視のため無効化
                bnb_4bit_compute_dtype=compute_dtype,
                llm_int8_threshold=6.0,
                llm_int8_has_fp16_weight=False,
                bnb_4bit_quant_storage=compute_dtype  # ストレージ最適化
            )
            
            # トークナイザーを読み込み（ローカルキャッシュのみ使用）
            sys.stdout.write("[NLG] 📝 事前ダウンロード済みトークナイザーを読み込み中...\n")
            try:
                self.tokenizer = AutoTokenizer.from_pretrained(
                    model_name,
                    cache_dir=cache_dir,
                    local_files_only=True  # ローカルファイルのみ使用
                )
            except Exception as e:
                error_str = str(e)
                if "argument of type 'NoneType' is not iterable" in error_str:
                    sys.stdout.write("[NLG] ⚠️  トークナイザーでNoneTypeエラー。local_files_onlyを無効にして再試行中...\n")
                    self.tokenizer = AutoTokenizer.from_pretrained(
                        model_name,
                        cache_dir=cache_dir,
                        local_files_only=False  # オンライン接続を許可
                    )
                    sys.stdout.write("[NLG] ✅ 再試行でトークナイザー読み込み成功\n")
                else:
                    raise e
            
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token
            
            # メインモデルを読み込み（ローカルキャッシュのみ使用）
            sys.stdout.write("[NLG] 🤖 事前ダウンロード済みGemma 3 4bit量子化モデルを読み込み中...\n")
            try:
                # Flash Attention 2での読み込みを試行
                try:
                    sys.stdout.write("[NLG] 🚀 Flash Attention 2で高速化モードを試行中...\n")
                    self.local_model = AutoModelForCausalLM.from_pretrained(
                        model_name,
                        quantization_config=quantization_config,
                        torch_dtype=compute_dtype,
                        device_map="auto",
                        cache_dir=cache_dir,
                        local_files_only=True,
                        trust_remote_code=True,
                        low_cpu_mem_usage=True,
                        use_flash_attention_2=True,  # Flash Attention 2で高速化
                        attn_implementation="flash_attention_2"
                    )
                    sys.stdout.write("[NLG] ✅ Flash Attention 2による高速化を適用\n")
                except Exception as flash_error:
                    sys.stdout.write(f"[NLG] ⚠️  Flash Attention 2が利用できません: {str(flash_error)}\n")
                    sys.stdout.write("[NLG] 🔄 標準モードで読み込み中...\n")
                    self.local_model = AutoModelForCausalLM.from_pretrained(
                        model_name,
                        quantization_config=quantization_config,
                        torch_dtype=compute_dtype,
                        device_map="auto",
                        cache_dir=cache_dir,
                        local_files_only=True,
                        trust_remote_code=True,
                        low_cpu_mem_usage=True
                    )
            except Exception as e:
                error_str = str(e)
                sys.stdout.write(f"[NLG] ❌ モデル読み込みエラー詳細: {error_str}\n")
                
                if "argument of type 'NoneType' is not iterable" in error_str:
                    sys.stdout.write("[NLG] ⚠️  NoneTypeエラーを検出。複数の回避策を試行中...\n")
                    
                    # 回避策1: local_files_onlyを無効にして再試行
                    try:
                        sys.stdout.write("[NLG] 回避策1: オンライン接続を許可して再試行...\n")
                        self.local_model = AutoModelForCausalLM.from_pretrained(
                            model_name,
                            quantization_config=quantization_config,
                            torch_dtype=compute_dtype,
                            device_map="auto",
                            cache_dir=cache_dir,
                            local_files_only=False,
                            trust_remote_code=True,
                            low_cpu_mem_usage=True
                        )
                        sys.stdout.write("[NLG] ✅ 回避策1成功: モデル読み込み完了\n")
                    except Exception as e2:
                        sys.stdout.write(f"[NLG] ❌ 回避策1失敗: {str(e2)}\n")
                        
                        # 回避策2: quantization_configを無効にして再試行
                        try:
                            sys.stdout.write("[NLG] 回避策2: 量子化なしで再試行...\n")
                            self.local_model = AutoModelForCausalLM.from_pretrained(
                                model_name,
                                torch_dtype=compute_dtype,
                                device_map="auto",
                                cache_dir=cache_dir,
                                local_files_only=False,
                                trust_remote_code=True,
                                low_cpu_mem_usage=True
                            )
                            sys.stdout.write("[NLG] ✅ 回避策2成功: 量子化なしでモデル読み込み完了\n")
                        except Exception as e3:
                            sys.stdout.write(f"[NLG] ❌ 回避策2失敗: {str(e3)}\n")
                            
                            # 回避策3: device_mapを無効にして再試行
                            try:
                                sys.stdout.write("[NLG] 回避策3: CPU読み込みで再試行...\n")
                                self.local_model = AutoModelForCausalLM.from_pretrained(
                                    model_name,
                                    torch_dtype=compute_dtype,
                                    cache_dir=cache_dir,
                                    local_files_only=False,
                                    trust_remote_code=True,
                                    low_cpu_mem_usage=True
                                )
                                sys.stdout.write("[NLG] ✅ 回避策3成功: CPUでモデル読み込み完了\n")
                                # CPUからGPUに移動
                                if torch.cuda.is_available():
                                    sys.stdout.write("[NLG] GPUに移動中...\n")
                                    self.local_model = self.local_model.to("cuda:0")
                                    sys.stdout.write("[NLG] ✅ GPUに移動完了\n")
                            except Exception as e4:
                                sys.stdout.write(f"[NLG] ❌ 全ての回避策が失敗: {str(e4)}\n")
                                raise e4
                else:
                    raise e
            
            # 並列実行用の設定
            self.model_lock_1 = threading.Lock()
            self.model_lock_2 = threading.Lock()
            
            # GPU使用量を確認
            if torch.cuda.is_available():
                torch.cuda.empty_cache()  # キャッシュクリア
                allocated = torch.cuda.memory_allocated(0) / 1024**3
                cached = torch.cuda.memory_reserved(0) / 1024**3
                sys.stdout.write(f"[NLG] 🖥️  Gemma 3 GPU使用量: {allocated:.1f}GB (キャッシュ: {cached:.1f}GB)\n")
            
            sys.stdout.write("[NLG] ✅ 事前ダウンロード済みGemma 3モデルの読み込み完了\n")
            return True
            
        except Exception as e:
            import traceback
            error_str = str(e)
            sys.stdout.write(f"[NLG] ❌ Gemma 3モデル読み込み失敗: {error_str}\n")
            sys.stdout.write(f"[NLG] 🔍 詳細なトレースバック:\n")
            traceback_str = traceback.format_exc()
            for line in traceback_str.split('\n'):
                if line.strip():
                    sys.stdout.write(f"[NLG] {line}\n")
            
            if "argument of type 'NoneType' is not iterable" in error_str:
                sys.stdout.write("[NLG] 💡 NoneTypeエラーはtransformersライブラリの既知の問題です\n")
                sys.stdout.write("[NLG] 🔧 transformersライブラリの更新を推奨します: pip install --upgrade transformers\n")
            else:
                sys.stdout.write("[NLG] 💡 事前ダウンロードスクリプトを実行してください:\n")
                sys.stdout.write("[NLG]    ./scripts/setup/predownload_all_models.sh\n")
            return False

    def update(self, query):
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
        
        # 並列推論システムに追加
        now = datetime.now()
        inference_request = {
            'query': query,
            'asr_results': self.asr_results,
            'timestamp': now,
            'request_id': f"{now.strftime('%H%M%S%f')}"
        }
        
        with self.inference_lock:
            # 推論中かどうかチェック
            if self.is_inferencing and self.current_inference_start_time:
                time_diff = (now - self.current_inference_start_time).total_seconds() * 1000
                if time_diff >= 200:  # 200ms以上経過していれば新しい推論を開始
                    sys.stdout.write(f"[{now.strftime('%H:%M:%S.%f')[:-3]}][NLG] 推論中に新規リクエスト受信 (経過時間: {time_diff:.1f}ms) - 並列推論開始\n")
                    self.inference_queue.put(inference_request)
                else:
                    sys.stdout.write(f"[{now.strftime('%H:%M:%S.%f')[:-3]}][NLG] 推論中に新規リクエスト受信 (経過時間: {time_diff:.1f}ms) - 200ms未満のため待機\n")
            else:
                # 推論中でなければ即座に追加
                self.inference_queue.put(inference_request)
        
        self.update_flag = True

    def _inference_worker(self, worker_id):
        """推論ワーカー関数 - 2つのワーカーが並列で動作"""
        while True:
            try:
                # キューから推論リクエストを取得（ブロッキング）
                request = self.inference_queue.get(timeout=1.0)
                
                with self.inference_lock:
                    if not self.is_inferencing:
                        # 推論開始
                        self.is_inferencing = True
                        self.current_inference_start_time = datetime.now()
                        start_timestamp = self.current_inference_start_time.strftime('%H:%M:%S.%f')[:-3]
                        sys.stdout.write(f"[{start_timestamp}][NLG] Worker{worker_id} 推論開始 (ID: {request['request_id']})\n")
                        # GPU使用状況を確認
                        self._log_gpu_memory_usage(f"Worker{worker_id} 推論開始時")
                    else:
                        # 既に他のワーカーが推論中の場合
                        time_diff = (datetime.now() - self.current_inference_start_time).total_seconds() * 1000
                        if time_diff < 200:
                            # 200ms未満なら待機
                            sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG] Worker{worker_id} 200ms未満のため待機\n")
                            self.inference_queue.put(request)  # リクエストを戻す
                            continue
                        else:
                            # 200ms以上経過していれば並列推論開始
                            sys.stdout.write(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}][NLG] Worker{worker_id} 並列推論開始 (ID: {request['request_id']})\n")
                            # GPU使用状況を確認
                            self._log_gpu_memory_usage(f"Worker{worker_id} 並列推論開始時")
                
                # Gemma 3で推論実行
                model_lock = self.model_lock_1 if worker_id == 1 else self.model_lock_2
                result = self._perform_gemma3_inference(request, model_lock, worker_id)
                sys.stdout.write(f"[NLG] Worker{worker_id} Gemma 3推論結果: '{result}'\n")
                
                if result:
                    with self.result_lock:
                        # より新しいリクエストの結果なら更新
                        if (self.latest_result is None or 
                            request['timestamp'] >= self.latest_result['timestamp']):
                            self.latest_result = {
                                'reply': result,
                                'source_words': request['asr_results'] if request['asr_results'] else [str(request['query'])],
                                'timestamp': request['timestamp'],
                                'worker_id': worker_id,
                                'request_id': request['request_id']
                            }
                            end_time = datetime.now()
                            end_timestamp = end_time.strftime('%H:%M:%S.%f')[:-3]
                            duration = (end_time - self.current_inference_start_time).total_seconds() * 1000
                            sys.stdout.write(f"[{end_timestamp}][NLG] Worker{worker_id} 推論完了 (ID: {request['request_id']}, {duration:.1f}ms)\n")
                            # 推論完了時のGPU状況確認
                            self._log_gpu_memory_usage(f"Worker{worker_id} 推論完了時")
                
                with self.inference_lock:
                    self.is_inferencing = False
                    self.current_inference_start_time = None
                    
            except Empty:
                # タイムアウト - 正常な動作
                continue
            except Exception as e:
                sys.stdout.write(f"[NLG ERROR] Worker{worker_id}: {e}\n")
                with self.inference_lock:
                    self.is_inferencing = False
                    self.current_inference_start_time = None

    def _perform_gemma3_inference(self, request, model_lock, worker_id):
        """Gemma 3モデルで推論を実行"""
        try:
            query = request['query']
            asr_results = request['asr_results']
            
            import torch
            
            # プロンプト構築
            if asr_results and isinstance(asr_results, list) and len(asr_results) >= 1:
                if all((not x or x.strip() == "") for x in asr_results):
                    return ""
                
                # 音声認識結果をすべて列挙
                asr_lines = []
                for idx, asr in enumerate(asr_results):
                    asr_lines.append(f"認識結果{idx+1}: {asr}")
                asr_block = "\n".join(asr_lines)
                
                prompt = f"""以下の音声認識結果から、自然で親しみやすい応答を40文字以内で生成してください。

{asr_block}

応答:"""
            else:
                if not query or (isinstance(query, list) and all((not x or x.strip() == "") for x in query)):
                    return ""
                prompt = f"親しみやすく15文字以内で応答してください。\n\nユーザ: {query}\n応答:"
            
            # モデルロックを取得して推論実行（複数回試行）
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    with model_lock:
                        inputs = self.tokenizer(
                            prompt, 
                            return_tensors="pt", 
                            truncation=True, 
                            max_length=512,  # 短縮して高速化
                            padding=False    # パディングを無効化
                        )
                        inputs = {k: v.to(self.local_model.device) for k, v in inputs.items()}
                        
                        # NaN/Infチェック
                        if torch.isnan(inputs['input_ids']).any() or torch.isinf(inputs['input_ids']).any():
                            sys.stdout.write(f"[NLG] ⚠️  入力にNaN/Infを検出 (試行 {attempt+1}/{max_retries})\n")
                            continue
                        
                        with torch.no_grad():
                            # 試行回数に応じて設定を調整
                            if attempt == 0:
                                # 最初は高速設定
                                generation_config = {
                                    "max_new_tokens": 50,  # トークン数を減らして高速化
                                    "temperature": 0.8,
                                    "top_p": 0.9,
                                    "top_k": 20,  # 候補を絞って高速化
                                    "do_sample": True,
                                    "pad_token_id": self.tokenizer.eos_token_id,
                                    "repetition_penalty": 1.05,  # 軽量化
                                    "use_cache": True,
                                    "num_beams": 1,  # ビーム探索を無効化
                                }
                            elif attempt == 1:
                                # 2回目はより保守的設定
                                generation_config = {
                                    "max_new_tokens": 50,
                                    "temperature": 0.5,
                                    "top_p": 0.9,
                                    "top_k": 20,
                                    "do_sample": True,
                                    "pad_token_id": self.tokenizer.eos_token_id,
                                    "repetition_penalty": 1.2,
                                    "use_cache": False
                                }
                            else:
                                # 最後はgreedy decoding
                                generation_config = {
                                    "max_new_tokens": 30,
                                    "do_sample": False,
                                    "pad_token_id": self.tokenizer.eos_token_id,
                                    "use_cache": False
                                }
                            
                            outputs = self.local_model.generate(**inputs, **generation_config)
                        
                        # 生成されたテキストをデコード
                        generated_text = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
                        
                        # プロンプト部分を除去して応答のみ抽出
                        if "応答:" in generated_text:
                            response = generated_text.split("応答:")[-1].strip()
                        elif "アンドロイド:" in generated_text:
                            response = generated_text.split("アンドロイド:")[-1].strip()
                        else:
                            response = generated_text[len(prompt):].strip()
                        
                        result = response.replace('\n', '').replace('\r', '')
                        
                        # メモリクリーンアップ
                        del outputs, generated_text
                        if torch.cuda.is_available():
                            torch.cuda.empty_cache()
                        
                        # 結果の妥当性チェック
                        if result and len(result.strip()) > 0:
                            if attempt > 0:
                                sys.stdout.write(f"[NLG] ✅ 推論成功 (試行 {attempt+1}/{max_retries})\n")
                            return result
                        else:
                            sys.stdout.write(f"[NLG] ⚠️  空の結果 (試行 {attempt+1}/{max_retries})\n")
                            
                except RuntimeError as e:
                    error_str = str(e)
                    if "probability tensor contains either `inf`, `nan` or element < 0" in error_str:
                        sys.stdout.write(f"[NLG] ⚠️  確率テンソルエラー (試行 {attempt+1}/{max_retries}): {error_str}\n")
                        if attempt < max_retries - 1:
                            # GPU状態をリセット
                            if torch.cuda.is_available():
                                torch.cuda.empty_cache()
                            continue
                        else:
                            # フォールバック: 簡単な応答を返す
                            return "申し訳ございません、応答生成中にエラーが発生しました。"
                    else:
                        raise e
                except Exception as e:
                    sys.stdout.write(f"[NLG] ❌ 予期しないエラー (試行 {attempt+1}/{max_retries}): {str(e)}\n")
                    if attempt == max_retries - 1:
                        raise e
                    continue
            
            return "応答の生成に失敗しました。"
            
        except Exception as e:
            sys.stdout.write(f"[NLG ERROR] Worker{worker_id} Gemma 3 inference failed: {e}\n")
            return ""
    
    def run(self):
        """メインループ - 並列推論システムの結果を監視"""
        sys.stdout.write("[NLG] Gemma 3並列推論システム開始\n")
        sys.stdout.flush()
        
        while True:
            # 最新の推論結果をチェック
            with self.result_lock:
                if self.latest_result and (
                    self.latest_result['reply'] != self.last_reply or
                    self.latest_result['source_words'] != self.last_source_words
                ):
                    # 新しい結果が利用可能
                    self.last_reply = self.latest_result['reply']
                    self.last_source_words = self.latest_result['source_words']
                    
                    now = datetime.now()
                    timestamp = now.strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[{timestamp}][NLG] 新しい対話結果を適用 (Worker{self.latest_result['worker_id']}, ID: {self.latest_result['request_id']})\n")
                    sys.stdout.write(f"[{timestamp}][NLG] 最終生成結果: {self.last_reply}\n")
                    sys.stdout.write(f"[{timestamp}][NLG] 使用した音声認識結果: {self.last_source_words}\n")
                    sys.stdout.flush()
                    
                    # 結果をクリア
                    self.latest_result = None
            
            time.sleep(0.01)