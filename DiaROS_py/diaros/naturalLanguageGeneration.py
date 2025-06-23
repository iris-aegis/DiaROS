# 一旦履歴諦め

import requests
import json
import sys
import os
import time
from datetime import datetime
import openai
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain_ollama import ChatOllama

class NaturalLanguageGeneration:
    def __init__(self):
        self.rc = { "word": "" }
        
        self.query = ""
        self.update_flag = False
        self.dialogue_history = []
        self.user_speak_is_final = False
        self.last_reply = ""  # 生成した対話文をここに格納

        sys.stdout.write('NaturalLanguageGeneration  start up.\n')
        sys.stdout.write('=====================================================\n')
        # OpenAI APIキーを環境変数から設定
        openai.api_key = os.environ.get("OPENAI_API_KEY")

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
        self.update_flag = True
        # sys.stdout.write(f"[NLG] update() called with query: {self.query}\n")
        # sys.stdout.flush()


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
    
    def run(self):
        DEBUG = True
        response_cnt = 0
        # LangChainセットアップ
        ollama_model = ChatOllama(
            model="gemma3:27b",
            max_tokens=4096,
            temperature=0.2,
            top_p=0.9
        )
        while True:
            if self.update_flag:
                query = self.query
                try:
                    res = ""  # resを必ず初期化
                    if self.asr_results and isinstance(self.asr_results, list) and len(self.asr_results) >= 1:
                        if all((not x or x.strip() == "") for x in self.asr_results):
                            self.last_reply = ""
                            self.update_flag = False
                            continue
                        # 音声認識結果をすべて列挙
                        asr_lines = []
                        for idx, asr in enumerate(self.asr_results):
                            asr_lines.append(f"認識結果{idx+1}: {asr}")
                        asr_block = "\n".join(asr_lines)
                        prompt = (
"""
あなたは、ユーザーの不完全な音声入力を正確に理解し、その内容に対して親しみやすく応答する対話型AIです。あなたはユーザー（男性）の友達である、優しく明るい性格の女性アンドロイドとして振る舞い、雑談をしている状況を想定して応答します。

まず、"human"から与えられる複数の音声認識結果（認識結果1, 認識結果2, ...）をもとに、以下のルールに従ってユーザーの本来の発話を正確に推定してください。

- 各認識結果はCERが20%の音声認識器によって得られたものなので、音声認識誤りを訂正してください。
- 認識結果に含まれる `<unk>` は、いわゆるアンノウンタグであり、認識できなかった部分を示します。文脈からその部分を適切に補完するか、あるいは不要であれば無視するように判断してください。
- 認識結果に含まれる `[雑音]` はその区間に雑音があったことを示し、`[無音]` は無音区間であったことをそれぞれ示します。これらの記号自体は意味のある発話内容ではないため、最終的な予測発話に含めないでください。これらの記号は、発話が途切れたり不明瞭だったりする箇所を示唆する可能性がありますので、前後の文脈を踏まえて自然な発話となるよう適切に処理してください。
- 各認識結果の情報を最大限に活用し、内容を正確に反映させてください。
- 認識結果が重複している箇所は、不自然にならないように適切に統合してください。
- 認識結果の間に欠落していると思われる箇所は、前後の文脈に沿って自然に補完してください。
- 元の発話の意図を損なわないように、流暢で一貫性のある日本語の文章としてください。
- 単なる結合ではなく、最も確からしい元の発話を予測してください。

推定した発話内容をもとに、以下の条件でアンドロイドとして応答してください。

- ペルソナ: あなたはユーザー（男性）の友達である、優しく明るい性格の女性アンドロイドです。
- シチュエーション: ユーザーと雑談をしています。
- 応答形式: 応答は必ず一言かつ一文で、20文字以内にしてください。
- 口調: 親しみを込めた、明るく優しい、自然な会話口調（例：友達に話すようなタメ口、またはそれに近いくだけた話し方）でお願いします。
"""
                        )
                        # LangChainでリクエスト
                        messages = [("system", prompt)]
                        for line in asr_lines:
                            messages.append(("human", line))
                        query = ChatPromptTemplate.from_messages(messages)
                        chain = query | ollama_model | StrOutputParser()
                        res = chain.invoke({})
                        # 音声認識結果リストも一緒に表示
                        sys.stdout.write(f"[NLG ASRリスト] {self.asr_results}\n")
                        sys.stdout.flush()
                    else:
                        if not query or (isinstance(query, list) and all((not x or x.strip() == "") for x in query)):
                            self.last_reply = ""
                            self.update_flag = False
                            continue
                        if query == "dummy":
                            res = "はい"
                        else:
                            if query in ("user_speak_is_final"):
                                sys.stdout.write('対話履歴作成\n')
                                sys.stdout.flush()
                                self.user_speak_is_final = True
                                query = query.replace("user_speak_is_final", "", 1)
                            if ":" in query:
                                response_cnt = int(query.split(":", 1)[0])
                                query = query.split(":", 1)[1]
                            sys.stdout.write(f"[NLG] query: {query}\n")

                            text_input = query
                            sys.stdout.write(f"[NLG] input {text_input}\n")
                            sys.stdout.flush()
                            start_time = datetime.now()
                            role = "優しい性格のアンドロイドとして、ユーザの発話に対して相手を労るような返答のみを２０文字以内でしてください。"

                            messages = [
                                ("system", role),
                                ("human", text_input)
                            ]
                            query_prompt = ChatPromptTemplate.from_messages(messages)
                            chain = query_prompt | ollama_model | StrOutputParser()
                            res = chain.invoke({})
                            sys.stdout.write("[NLG生成文] " + res + "\n")
                            sys.stdout.flush()
                            if ":" in res:
                                res = res.split(":", 1)[1]
                            if self.user_speak_is_final:
                                self.dialogue_history.append("usr:" + query)
                                self.dialogue_history.append("sys:" + res)
                                self.user_speak_is_final = False
                                if len(self.dialogue_history) > 5:
                                    self.dialogue_history = self.dialogue_history[-4:]
                                sys.stdout.write('対話履歴完了\n')
                                sys.stdout.flush()
                    # 改行を除去して1行にする
                    self.last_reply = res.replace('\n', '').replace('\r', '')
                    now = datetime.now()
                    timestamp = now.strftime('%H:%M:%S.%f')[:-3]
                    sys.stdout.write(f"[{timestamp}][NLG] 対話生成完了時刻: {timestamp}\n")
                    print(f"[NLG生成文] {self.last_reply}")
                    sys.stdout.flush()
                except Exception as e:
                    self.last_reply = ""
                    sys.stdout.write(f"[NLG ERROR] {e}\n")
                    sys.stdout.flush()
                self.update_flag = False
            else:
                self.last_reply = ""
                self.update_flag = False
            time.sleep(0.01)