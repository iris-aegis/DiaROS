# DiaROS
**[English ver is HERE](README.md)**

## Author
西村良太 徳島大学  
nishimura@is.tokushima-u.ac.jp

## Developer
- 西村 良太 (Ryota Nishimura)
- 森 貴大 (Takahiro Mori) https://bitbucket.org/takahiro_mori_win/

## Demo video
[<img width="300" alt="youtube" src="https://user-images.githubusercontent.com/16011609/199163853-a00c3d9b-b4ea-483f-8d22-d1affb59dcd9.png">
](https://www.youtube.com/watch?v=2EkJCJpSpS4)

## 概要
リアルタイム音声対話システムをROS2対応にさせたものです．システム自体の構成は単純で，中身も単純ですが，その分，音声対話システムのROS対応の方法が理解できると思います．基本的には，python実装された音声対話システムの各モジュール間の通信をROSでラップした構成になっています．この構成にすることで，通信内容をROSで監視することが可能となり，通信内容の確認，記録，再生が可能となるため，システム開発・デバッグの効率が格段に上がります．

## 注意事項
- まだバグが含まれています．バグ取り段階です．
- ダッシュボード（nodejs, vueによる実装）は動きません
  - 無しでも音声対話システムは動きます


# システムインストール方法
以下にシステムのインストール方法を記載します．

## 0. システム環境（開発時）
- OS: Ubuntu 20.04.3


## 1. ROS2 Foxy をインストールする
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

- 順番に実行していく
- 問題なく終了し，テストも動くはずです．


### （おまけ）Turtlesim でのテストもやりたい！
https://docs.ros.org/en/foxy/Tutorials.html

```shell
$ sudo apt update
$ sudo apt install ros-foxy-turtlesim
```

```shell
[shell 1]
$ ros2 run turtlesim turtlesim_node

[shell 2]
$ ros2 run turtlesim turtle_teleop_key
```


## 2. Python環境インストール
### 2.1 pyenv インストール
まっさらなUbuntu20.04環境へのpyenvインストールメモ
https://qiita.com/sho1_24/items/96c3c9e71629de3801fb

- 各種ライブラリインストール
```shell
$ sudo apt install git gcc make zlib1g-dev libffi-dev libbz2-dev libssl-dev libreadline-dev libsqlite3-dev python3-tk tk-dev
```

- pyenvのインストール
```shell
$ git clone https://github.com/pyenv/pyenv.git ~/.pyenv
$ git clone https://github.com/pyenv/pyenv-update.git ~/.pyenv/plugins/pyenv-update
```

- 環境変数の設定
`PYTHONPATH`の設定は，pythonのデフォルトのモジュールインストール先と．`--user`でのモジュールインストール先を追記しておく．  
pythonのバージョンなどに気をつける
```bash
# .bashrc

# for pyenv(python)
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init --path)"
PPTMP=`python -c "import site; print (site.getsitepackages()[0])"`
export PYTHONPATH="$PPTMP:$PYTHONPATH"
PPTMP=`python -m site --user-site`
export PYTHONPATH="$PPTMP:$PYTHONPATH"
```


OS標準pythonコマンドへのシンボリックリンク作成
```shell
$ sudo apt install python-is-python3
```

### 2.2 Pythonインストール
```
インストール可能なバージョンの確認
$ pyenv install -l | less

インストール
$ pyenv install 3.8.13

バージョン設定
$ pyenv global 3.8.13
```


## 3. 音声対話システムインストール

### 3.1 A3RTとGoogle APIのKey取得と配置 
A3RT talkAPIは，雑談応答生成用のチャットボットAPIです．  
Google APIは，音声認識用のGoogle Speech-to-Text APIです．

- A3RT talkAPIのAPIキーを取得: https://a3rt.recruit.co.jp/product/talkAPI/
- テキストファイルにAPIキーをコピペし保存（APIキーのみ入ったテキストファイル）
- テキストファイルを以下に設置（HOMEディレクトリは各自読み替えてね）
```
/home/nishimura/secret/nishimura_A3RT_APIKEY.data
```

- Google APIキーを作成し以下に設置（キーJSONファイル作成は，以下のページに従ってください）  
https://cloud.google.com/speech-to-text/docs/before-you-begin?hl=ja
```
/home/nishimura/secret/nishimura_SDS.json
```

- A3RT APIキー, Google APIキーのパスを環境変数に追加  
`~/.bashrc` に以下を追加
```shell
export GOOGLE_APPLICATION_CREDENTIALS="/home/nishimura/secret/nishimura_SDS.json"
export A3RT_APIKEY="/home/nishimura/secret/nishimura_A3RT_APIKEY.data"
```


### 3.2 音声対話システムモジュールインストール
`DiaROS/DiaROS_py`内で以下を実行

```bash
$ python -m pip install . --user
```


### 3.3 音声対話システム用pythonモジュールのインストール

**■google cloud speech api のインストール**
```shell:
$ python -m pip install google gcloud google-auth google-api-core google-cloud-speech grpc-google-cloud-speech-v1beta1 grpcio grpcio-tools
```

```shell:
$ python -m pip install -U numpy scipy requests pyworld matplotlib==2.*
```

**■pyaudio のインストール**

```shell:
$ sudo apt-get install portaudio19-dev
$ pip install pyaudio
```


**■aubioのインストール**

そのままpipで入らない．
githubから最新版をダウンロードしてインストール．

（pipコマンドを通じてダウンロードする！）

```shell:
$ python -m pip install git+https://github.com/aubio/aubio/
```

**■その他インストール**
```shell:
$ python -m pip install gtts playsound
```

### 3.4 ROSパッケージ(MSG)のビルド
まず，colconのインストールから
```bash
$ sudo apt install python3-colcon-common-extensions
```

`/home/nishimura/program/DiaROS/DiaROS_ros` にて

```bash
# ディレクトリの移動と，環境設定
$ cd /home/nishimura/program/DiaROS/DiaROS_ros
$ . /home/nishimura/ros2_foxy/ros2-linux/local_setup.bash

# msgフォルダ（interfaces）のビルド
$ colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
$ . ./install/local_setup.bash

# ros2モジュール(diaros_package)のビルド
$ colcon build --packages-select diaros_package
$ . ./install/local_setup.bash
``` 






## 4. （まだバグってて動きません！）ダッシュボードインストール
注意：以下，まだバグっており実現できません．nodejs,vue のモジュール周りのバージョンがうまく合わず，開発時の環境が再現できないため，動作しません．動かせた人がいたら教えてください(´；ω；｀)

### 4.1 npm インストール
```
$ sudo apt update
$ sudo apt install curl
$ curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
$ sudo apt-get install -y nodejs
# $ sudo apt-get install gcc g++ make
```

確認
```
$ node -v
$ npm -v
```

npmのバージョンを上げる
```
$ sudo npm install -g npm
```

### 4.2 ダッシュボードインストール
ダッシュボード：サーバ，クライアントのインストール
```bash
$ cd /home/nishimura/program/DiaROS/dialogue-dashboard

# モジュールをインストール
$ cd 
$ sudo npm install -g typescript
$ sudo npm install -g require
$ sudo npm i @types/node
$ sudo npm audit fix --force


# サーバインストール
$ cd server
$ sudo npm ci
$ sudo npm run build

# クライアントインストール
$ cd ..
$ sudo npm ci
$ sudo npm run build
```

指示通り以下を実行
```shell:
$ sudo npm audit fix --force
```

ERESOLVE could not resolve がでたら
https://github.com/vuejs/vue-cli/issues/6270
```
$ npm i --legacy-peer-deps
```

Error: Rule can only have one resource source が出たら
https://stackoverflow.com/questions/64373393/error-rule-can-only-have-one-resource-source-provided-resource-and-test-incl
```
$ sudo npm uninstall webpack
$ sudo npm install webpack@^4.45.0
```

Cannot find module 'webpack/lib/RuleSet'
https://github.com/vuejs/vue-loader/issues/1586
```
$ sudo npm i vue-loader

$ sudo npm install --save-dev webpack webpack-cli html-webpack-plugin webpack-dev-server webpack-dev-middleware
```



## 5. 実行手順

### 5.1 音声対話システム起動
```bash
$ cd /home/nishimura/program/DiaROS/DiaROS_ros
$ . /home/nishimura/ros2_foxy/ros2-linux/local_setup.bash
$ . /home/nishimura/program/DiaROS/DiaROS_ros/install/local_setup.bash

# 音声対話システム実行
$ ros2 launch diaros_package sdsmod.launch.py
``` 


### 5.2 （まだ動作しません）ダッシュボード起動
```bash
$ cd C:\sayonari\DiaROS\dialogue-dashboard
$ npm run start # サーバ起動
$ npm run serve # クライアント起動
```

### 5.3 （まだ動作しません）ダッシュボード用と管理ツール用のrosモジュール起動
```bash
$ cd C:\sayonari\DiaROS\spoken-dialogue-system
$ call C:\dev\ros2_foxy\local_setup.bat
$ call install\local_setup.bat
ros2 run diaros_package dr # ダッシュボード連携
ros2 run diaros_package mm # 管理ツール用
```



## 6. ROS2の機能活用
### 6.0 環境構築
```bash
$ python -m pip install PyQt5==5.12 PySide2 pydot
$ pip3 uninstall PyQt5 # バージョンが混在して動かなくなるので，これでpip3版を消す！
```
https://ar-ray.hatenablog.com/entry/2021/03/10/203358


### 6.1 rqt
http://docs.ros.org/en/foxy/Concepts/About-RQt.html

RQtは，QtベースのROS向けGUIフレームワークです．ros2 foxy には，標準で以下のrqtツールが含まれています．  
（TAB補完による候補の表示）
```
$ ros2 run rqt[TAB]
rqt                 rqt_gui             rqt_plot            rqt_reconfigure     rqt_top
rqt_action          rqt_gui_cpp         rqt_publisher       rqt_service_caller  rqt_topic
rqt_console         rqt_gui_py          rqt_py_common       rqt_shell           
rqt_graph           rqt_msg             rqt_py_console      rqt_srv    
```


### 6.2 rqt_graph
```
$ ros2 run rqt_graph rqt_graph
```

### 6.3 rqt_plot
```
$ ros2 run rqt_plot rqt_plot
```

エラーが出て動かなかったら，apt-get upgrade でrosパッケージがアップグレードしましょう．
```
$ sudo apt-get update
$ sudo apt-get upgrade

$ python -m pip install --upgrade pydot pyqt5
```



### 6.x rqt_bag
https://zenn.dev/techkind/articles/2106100906_ros2_rqt_bag

rqt_bag コマンドにより，GUI表示によりトピックの流れを可視化することができます．
ros2 foxy には標準では含まれていないツールなので，上記URLを参考にツールをインストールしましょう．




## 以下，実行時のエラーに対する対処
### 9.1 サウンドデバイス（USBヘッドセット）が認識されなかったら！
https://kazuhira-r.hatenablog.com/entry/2020/02/28/000625

```
# バージョンチェック
$  sudo apt search ^linux-headers-

# 5.13.0-41が新しそうだったので，それを入れる！
$ sudo apt install linux-image-5.13.0-41-generic linux-headers-5.13.0-41-generic linux-modules-extra-5.13.0-41-generic
```


### 9.2 voice.py でwindows用音声合成を使わない！
python でgttsを読み込んでみたときに`■No module named gi`と言われる．
```
$ python
>>> import gtts
(error) No module named gi
```

そのときには，以下を実行．  
https://stackoverflow.com/questions/71369726/no-module-named-gi
```
$ sudo apt install libcairo2-dev
$ sudo apt install libxt-dev
$ sudo apt install libgirepository1.0-dev
$ pip install pycairo
$ pip install PyGObject
```


### 9.3 default のサウンドデバイスを固定
https://wolfgang-ziegler.com/blog/prevent-changing-of-default-ubuntu-sound-device

- インプットデバイスリスト確認
```
$ pactl list short sources
```

- デフォルトに設定
```
$ pactl set-default-source alsa_input.usb-Sennheiser_Communications_Sennheiser_USB_headset-00.mono-fallback
```

- '~/.bashrc'に上記設定を追記


### 9.4 毎回 Unknown PCM cards が表示される
https://stackoverflow.com/questions/31603555/unknown-pcm-cards-pcm-rear-pyaudio
```
ALSA lib pcm_dsnoop.c:641:(snd_pcm_dsnoop_open) unable to open slave
ALSA lib pcm_dmix.c:1089:(snd_pcm_dmix_open) unable to open slave
ALSA lib pcm.c:2642:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.rear
ALSA lib pcm.c:2642:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.center_lfe
ALSA lib pcm.c:2642:(snd_pcm_open_noupdate) Unknown PCM cards.pcm.side
ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_dmix.c:1089:(snd_pcm_dmix_open) unable to open slave
```

Unknown PCM cards are removed by commenting out relevant lines in `/usr/share/alsa/alsa.conf`.  
（使ってない，知らんカード用の設定はコメントアウトしろ）


