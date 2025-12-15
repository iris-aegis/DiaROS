#!/usr/bin/env python3
"""
Gemma3-12B GPU使用テスト
"""

import time
from datetime import datetime
from langchain_ollama import ChatOllama
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

def test_gemma12b_gpu():
    print("🔧 Gemma3-12B GPU使用テスト")
    print("="*50)
    
    # シンプルなモデル設定
    ollama_model = ChatOllama(
        model="gemma3:12b",
        max_tokens=100,
        temperature=0.3,
        top_p=0.9,
        num_predict=80
    )
    
    # 複雑なプロンプト（実際のASR用）
    prompt_file_path = "/workspace/DiaROS_py/diaros/prompts/asr_dialogue_prompt.txt"
    try:
        with open(prompt_file_path, 'r', encoding='utf-8') as f:
            complex_prompt_text = f.read()
    except:
        complex_prompt_text = "親しみやすく60文字程度で応答してください。"
    
    # テスト用ASR結果
    asr_lines = [
        "認識結果1: 大都だから頑張ないといけないんだよね[無音][無音]",
        "認識結果2: よね[無音]",
        "認識結果3: 今日",
        "認識結果4: 今日会社で新しいプロジェクトの話",
        "認識結果5: で新しいプロジェクトの話があって最初は",
        "認識結果6: 最初はすごく面白そうでやってみたい",
        "認識結果7: 面白そうでやってみたいって思んだけど締め切りがかなり",
        "認識結果8: んだけど締め切りがかなりタイ途だから頑張らないといけないんだよ"
    ]
    
    print("📊 複雑プロンプト + 実ASRデータテスト:")
    
    durations = []
    for i in range(3):
        messages = [("system", complex_prompt_text)]
        for line in asr_lines:
            messages.append(("human", line))
        
        complex_prompt = ChatPromptTemplate.from_messages(messages)
        chain = complex_prompt | ollama_model | StrOutputParser()
        
        start_time = datetime.now()
        response = chain.invoke({})
        end_time = datetime.now()
        
        duration = (end_time - start_time).total_seconds() * 1000
        durations.append(duration)
        
        print(f"  テスト{i+1}: {duration:.1f}ms")
        print(f"    応答: {response[:60]}...")
        print()
        
        time.sleep(1)  # 1秒間隔
    
    avg_duration = sum(durations) / len(durations)
    print(f"📈 平均推論時間: {avg_duration:.1f}ms")
    
    if avg_duration < 500:
        print("✅ 高速: 期待通りの性能です")
    elif avg_duration < 1000:
        print("🟡 普通: 許容範囲内です")
    elif avg_duration < 3000:
        print("⚠️  遅い: 最適化が必要です")
    else:
        print("❌ 非常に遅い: 問題があります")
    
    # シンプルテストとの比較
    print("\n🔄 シンプルプロンプトテスト:")
    simple_prompt = ChatPromptTemplate.from_messages([
        ("system", "20文字以内で親しみやすく応答してください。"),
        ("human", "こんにちは")
    ])
    simple_chain = simple_prompt | ollama_model | StrOutputParser()
    
    simple_durations = []
    for i in range(3):
        start_time = datetime.now()
        response = simple_chain.invoke({})
        end_time = datetime.now()
        
        duration = (end_time - start_time).total_seconds() * 1000
        simple_durations.append(duration)
        
        print(f"  シンプル{i+1}: {duration:.1f}ms - '{response}'")
        time.sleep(0.5)
    
    simple_avg = sum(simple_durations) / len(simple_durations)
    print(f"\n📈 シンプル平均: {simple_avg:.1f}ms")
    print(f"🔍 プロンプト影響: {avg_duration / simple_avg:.1f}倍遅い")

if __name__ == "__main__":
    test_gemma12b_gpu()