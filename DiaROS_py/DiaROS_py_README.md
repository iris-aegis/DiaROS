# DiaROS

## インストール

````
pip install . --user
````

## モジュール構成
* speechInput.py                    音声入力mod: PyAudio
* acousticAnalysis.py               音響分析mod: aubio
* automaticSpeechRecognition.py     音声認識mod: VadLessASR
* naturalLanguageUnderstanding.py   意図理解mod: 現状は受信したデータを送信している
* dialogManagement.py               対話管理mod: リアルタイムに応答・相槌の制御を取り仕切る
* naturalLanguageGeneration.py      応答生成mod: chatGPT-API
* speechSynthesis.py                音声合成mod: VOICEVOX

## ROS2コマンド(一部)
* Topic一覧表示
```
ros2 topic list
```
* topicのレコード(コマンドを終了するまでTopicをレコードし、終えるとレコードファイルを出力)
```
ros2 bag record topic1 topic2 ... topicN
```
* レコードの再生
```
ros2 bag play [レコードファイル名]
```