#!/usr/bin/env python3
"""
シンプルなOllamaテストで基本性能を測定
"""

import time
from datetime import datetime
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

def test_simple_ollama():
    print("🔧 シンプルなOllamaテスト開始")
    print("="*50)
    
    # シンプルなモデル設定
    ollama_model = ChatOllama(
        model="gemma3:12b",
        max_tokens=50,  # 短いレスポンス
        temperature=0.2,
        top_p=0.9
    )
    
    # シンプルなプロンプト
    simple_prompt = ChatPromptTemplate.from_messages([
        ("system", "20文字以内で応答してください。"),
        ("human", "こんにちは")
    ])
    
    chain = simple_prompt | ollama_model | StrOutputParser()
    
    print("📊 5回のシンプルテスト:")
    durations = []
    
    for i in range(5):
        start_time = datetime.now()
        response = chain.invoke({})
        end_time = datetime.now()
        
        duration = (end_time - start_time).total_seconds() * 1000
        durations.append(duration)
        
        print(f"  {i+1}: {duration:.1f}ms - '{response}'")
        time.sleep(0.5)  # 500ms間隔
    
    avg_duration = sum(durations) / len(durations)
    print(f"\n📈 平均時間: {avg_duration:.1f}ms")
    
    # 複雑なプロンプトテスト
    print("\n🔄 複雑なプロンプトテスト:")
    complex_prompt = ChatPromptTemplate.from_messages([
        ("system", """
あなたは、ユーザーの不完全な音声入力を正確に理解し、その内容に対して親しみやすく応答する対話型AIです。あなたはユーザー（男性）の友達である、優しく明るい性格の女性アンドロイドとして振る舞い、雑談をしている状況を想定して応答します。

まず、"human"から与えられる複数の音声認識結果（認識結果1, 認識結果2, ...）をもとに、以下のルールに従ってユーザーの本来の発話を正確に推定してください。

- 各認識結果はCERが20%の音声認識器によって得られたものなので、音声認識誤りを訂正してください。
- 認識結果に含まれる `<unk>` は、いわゆるアンノウンタスクであり、認識できなかった部分を示します。文脈からその部分を適切に補完するか、あるいは不要であれば無視するように判断してください。
- 認識結果に含まれる `[雑音]` はその区間に雑音があったことを示し、`[無音]` は無音区間であったことをそれぞれ示します。これらの記号自体は意味のある発話内容ではないため、最終的な予測発話に含めないでください。これらの記号は、発話が途切れたり不明瞭だったりする箇所を示唆する可能性がありますので、前後の文脈を踏まえて自然な発話となるよう適切に処理してください。
- 各認識結果の情報を最大限に活用し、内容を正確に反映させてください。
- 認識結果が重複している箇所は、不自然にならないように適切に統合してください。
- 認識結果の間に欠落していると思われる箇所は、前後の文脈に沿って自然に補完してください。
- 元の発話の意図を損なわないように、流暢で一貫性のある日本語の文章としてください。
- 単なる結合ではなく、最も確からしい元の発話を予測してください。

以上の手順でユーザーの発話を復元した上で、以下の条件でアンドロイドとして応答してください。

- ペルソナ: あなたはユーザー（男性）の友達である、優しく明るい性格の女性アンドロイドです。
- シチュエーション: ユーザーと雑談をしています。 
- 応答形式: 応答は必ず60文字程度にしてください。
- 口調: 親しみを込めた、明るく優しい、友達に話すようなタメ口でお願いします。

アンドロイドの応答: 
"""),
        ("human", "認識結果1: 今日はいい天気ですね\n認識結果2: [雑音]気持ちいいです")
    ])
    
    complex_chain = complex_prompt | ollama_model | StrOutputParser()
    
    complex_durations = []
    for i in range(3):
        start_time = datetime.now()
        response = complex_chain.invoke({})
        end_time = datetime.now()
        
        duration = (end_time - start_time).total_seconds() * 1000
        complex_durations.append(duration)
        
        print(f"  {i+1}: {duration:.1f}ms - '{response[:50]}...'")
        time.sleep(0.5)
    
    complex_avg = sum(complex_durations) / len(complex_durations)
    print(f"\n📈 複雑プロンプト平均: {complex_avg:.1f}ms")
    
    print(f"\n🔍 プロンプトの影響:")
    print(f"  シンプル: {avg_duration:.1f}ms")
    print(f"  複雑: {complex_avg:.1f}ms")
    print(f"  差: {complex_avg - avg_duration:.1f}ms ({(complex_avg / avg_duration):.1f}倍)")

if __name__ == "__main__":
    test_simple_ollama()