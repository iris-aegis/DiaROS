# DiaROS
**[日本語バージョンはこちら](README_jp.md)**

## Author
Ryota Nishimura (Tokushima University)  
nishimura@is.tokushima-u.ac.jp

## Developer
- Ryota Nishimura
- Takahiro Mori https://bitbucket.org/takahiro_mori_win/

## Demo video
[<img width="300" alt="youtube" src="https://user-images.githubusercontent.com/16011609/199163853-a00c3d9b-b4ea-483f-8d22-d1affb59dcd9.png">
](https://www.youtube.com/watch?v=2EkJCJpSpS4)

## Overview
This is a ROS2-compatible version of a real-time spoken dialog system. The system is simple and its contents are also simple, so you can understand how to make a spoken dialog system ROS-compatible. Basically, the system consists of ROS wrapped communication between modules of a python implementation of a spoken dialog system. This configuration makes it possible to monitor the communication contents in ROS, and to check, record, and replay the communication contents, thus dramatically increasing the efficiency of system development and debugging.

## Cautions
- This configuration still contains some bugs. It is still in the bug-fixing stage.
- Dashboard (implemented by Node.js, Vue.js) does not work.
  - The spoken dialog system works without it.


# How to install the system
The system installation procedure is described below.

## 0. System environment (during development)
- OS: Ubuntu 20.04.3


## 1. Install ROS2 Foxy
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html

- Execute in order
- It should finish without any problems and the test should work.


### (extra) Turtlesim test!
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


## 2. Python environment installation
### 2.1 pyenv installation
Pyenv installation notes for a fresh Ubuntu 20.04 environment
https://qiita.com/sho1_24/items/96c3c9e71629de3801fb

- Installing libraries
```shell
$ sudo apt install git gcc make zlib1g-dev libffi-dev libbz2-dev libssl-dev libreadline-dev libsqlite3-dev python3-tk tk-dev
```

- Installing pyenv
```shell
$ git clone https://github.com/pyenv/pyenv.git ~/.pyenv
$ git clone https://github.com/pyenv/pyenv-update.git ~/.pyenv/plugins/pyenv-update
```

- Set environment variables
The `PYTHONPATH` setting is the default module installation location for python, and the module installation location with `--user` should be appended.  
Pay attention to the python version, etc.
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


Create symbolic link to standard OS python command
```shell
$ sudo apt install python-is-python3
```

### 2.2 Python installation
```
Check for installable version
$ pyenv install -l | less

Install Python
$ pyenv install 3.8.13

Set version
$ pyenv global 3.8.13
```


## 3. installation of spoken dialog system

### 3.1 Key acquisition and deployment of A3RT and Google API 
- Get API key for A3RT: https://a3rt.recruit.co.jp/product/talkAPI/
- Copy and paste the API key into a text file and save it (text file containing only the API key)
- Place the text file in the following location (replace the HOME directory with your own)
```
/home/nishimura/secret/nishimura_A3RT_APIKEY.data
```

- Create a Google API key and put it in the following directory.
```
/home/nishimura/secret/nishimura_SDS.json
```

- Add the paths to the A3RT API key and Google API key to the environment variables.  
Add the following to `~/.bashrc`.
```shell
export GOOGLE_APPLICATION_CREDENTIALS="/home/nishimura/secret/nishimura_SDS.json"
export A3RT_APIKEY="/home/nishimura/secret/nishimura_A3RT_APIKEY.data"
```


### 3.2 Spoken dialog System Module Installation
Execute the following in ``DiaROS/DiaROS_py``.

```bash
$ python -m pip install . --user
```


### 3.3 Installation of python modules for spoken dialog system

**# Installing google cloud speech api**
```shell
$ python -m pip install google gcloud google-auth google-api-core google-cloud-speech grpc-google-cloud-speech-v1beta1 grpcio grpcio-tools
```

```shell
$ python -m pip install -U numpy scipy requests pyworld matplotlib==2.*
```

**# Installing pyaudio**

```shell
$ sudo apt-get install portaudio19-dev
$ pip install pyaudio
```


**# Installing aubio**.

You can't just pip it in.
Download and install the latest version from github.

(Download via pip command!).

```shell:
$ python -m pip install git+https://github.com/aubio/aubio/
```

**# Other installations**.
```shell:
$ python -m pip install gtts playsound
```

### 3.4 Building the ROS package (MSG)
First, install colcon.
```bash
$ sudo apt install python3-colcon-common-extensions
```

In ``/home/nishimura/program/DiaROS/DiaROS_ros``

```bash
# Move directories and configure environment
$ cd /home/nishimura/program/DiaROS/DiaROS_ros
$ . /home/nishimura/ros2_foxy/ros2-linux/local_setup.bash

# Build msg folder (interfaces)
$ colcon build --cmake-args -DCMAKE_C_FLAGS=-fPIC --packages-select interfaces
$ . . /install/local_setup.bash

# Build ros2 module (diaros_package)
$ colcon build --packages-select diaros_package
$ . . /install/local_setup.bash
``` 





## 4. dashboard installation (This section is bug fix in progress)
Note: The following is still buggy and cannot be realized, because the versions of the nodejs and vue modules do not match and the development environment cannot be reproduced. Please let me know if anyone has been able to get it to work.

### 4.1 npm install
```shell
$ sudo apt update
$ sudo apt install curl
$ curl -sL https://deb.nodesource.com/setup_16.x | sudo -E bash -
$ sudo apt-get install -y nodejs
# $ sudo apt-get install gcc g++ make
```

confirm
```
$ node -v
$ npm -v
```

Upgrade npm version.
```
$ sudo npm install -g npm
```

### 4.2 Dashboard Installation
Dashboard: server and client installation
```bash
$ cd /home/nishimura/program/DiaROS/dialog-dashboard

# Install modules
$ cd 
$ sudo npm install -g typescript
$ sudo npm install -g require
$ sudo npm i @types/node
$ sudo npm audit fix --force


# Install server
$ cd server
$ sudo npm ci
$ sudo npm run build

# Install client
$ cd ...
$ sudo npm ci
$ sudo npm run build
```

Run the following as instructed
```shell:
$ sudo npm audit fix --force
```

If ERESOLVE could not resolve
https://github.com/vuejs/vue-cli/issues/6270
```
$ npm i --legacy-peer-deps
```

If you get Error: Rule can only have one resource source
https://stackoverflow.com/questions/64373393/error-rule-can-only-have-one-resource-source-provided-resource-and-test-incl
```
$ sudo npm uninstall webpack
$ sudo npm install webpack@^4.45.0
```

Cannot find module 'webpack/lib/RuleSet'.
https://github.com/vuejs/vue-loader/issues/1586
```
$ sudo npm i vue-loader

$ sudo npm install --save-dev webpack webpack-cli html-webpack-plugin webpack-dev-server webpack-dev-middleware
````



## 5. run steps

### 5.1 Start the spoken dialog system
```bash
$ cd /home/nishimura/program/DiaROS/DiaROS_ros
$ . /home/nishimura/ros2_foxy/ros2-linux/local_setup.bash
$ . /home/nishimura/program/DiaROS/DiaROS_ros/install/local_setup.bash

# Run spoken dialog system
$ ros2 launch diaros_package sdsmod.launch.py
``` 


### 5.2 (not yet working) dashboard launch
```bash
$ cd C:\sayonari\DiaROS\dialog-dashboard
$ npm run start # start server
$ npm run serve # start client
```

### 5.3 (not working yet) run ros module for dashboard and admin tools
```bash
$ cd C:\sayonari\DiaROS\spoken-dialog-system
$ call C:\dev\ros2_foxy\local_setup.bat
$ call install\local_setup.bat
ros2 run diaros_package dr # Dashboard integration
ros2 run diaros_package mm # for admin tools
```




## 6. Utilizing the functions of ROS2
### 6.0 environment setup
```bash
$ python -m pip install PyQt5==5.12 PySide2 pydot
$ pip3 uninstall PyQt5 # This will remove the pip3 version, as it will not work with mixed versions!
```
https://ar-ray.hatenablog.com/entry/2021/03/10/203358


### 6.1 rqt
http://docs.ros.org/en/foxy/Concepts/About-RQt.html

RQt is a Qt-based GUI framework for ROS. ros2 foxy includes the following rqt tools by default.  
(displaying candidates with TAB completion)

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

If you get an error and it doesn't work, upgrade the ros package with apt-get upgrade.
```
$ sudo apt-get update
$ sudo apt-get upgrade

$ python -m pip install --upgrade pydot pyqt5
```



### 6.x rqt_bag
https://zenn.dev/techkind/articles/2106100906_ros2_rqt_bag

The rqt_bag command allows you to visualize the flow of topics in a GUI display.
This tool is not included in ros2 foxy, so please refer to the above URL to install the tool.




## 9. The following are workarounds for runtime errors.
### 9.1 If your sound device (USB headset) is not recognized!
https://kazuhira-r.hatenablog.com/entry/2020/02/28/000625

```
## Check the version
$ sudo apt search ^linux-headers-

# 5.13.0-41 looked new, put it in!
$ sudo apt install linux-image-5.13.0-41-generic linux-headers-5.13.0-41-generic linux-modules-extra-5.13.0-41-generic
```


### 9.2 voice.py without voice synthesis for windows!
When I try to load gtts with python, it says ``# No module named gi``.
```
$ python
>>> import gtts
(error) No module named gi
```

In that case, execute the following.  
https://stackoverflow.com/questions/71369726/no-module-named-gi
```
$ sudo apt install libcairo2-dev
$ sudo apt install libxt-dev
$ sudo apt install libgirepository1.0-dev
$ pip install pycairo
$ pip install PyGObject
```


### 9.3 Fix default sound device
https://wolfgang-ziegler.com/blog/prevent-changing-of-default-ubuntu-sound-device

- Check input device list.
```
$ pactl list short sources
```

- Set to default.
```
$ pactl set-default-source alsa_input.usb-Sennheiser_Communications_Sennheiser_USB_headset-00.mono-fallback
```

- Add the above settings to '~/.bashrc'.


### 9.4 Unknown PCM cards appear every time
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
(Comment out settings for cards you don't use or don't know.)


