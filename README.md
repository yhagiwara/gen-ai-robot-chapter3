# 第３章　音声認識・合成
## 概要
第３章のサンプルプログラムと補足情報などを掲載しています．

## インストール
- オーディオ関連を扱うためのライブラリを以下のコマンドでインストールします．
```
sudo apt install portaudio19-dev
sudo apt install pulseaudio
```
- Pythonのモジュールとして呼びたすために，以下のコマンドを実行します．
```
pip3 install pyaudio
```
- 音声認識ライブラリを以下のコマンドでインストールします．
```
pip3 install SpeechRecognition
```
- 音声認識器のWhisperを扱うためのライブラリを以下のコマンドでインストールします.
```
pip3 install SpeechRecognition[whisper-local] soundfile
```
- 音声合成で用いるライブラリをインストールします。
```
pip3 install kokoro
sudo apt-get install espeak-ng
pip3 install soundfile
pip3 install simpleaudio
```
- 応答文生成で用いるライブラリをインストールします。
```
sudo apt install curl
curl -fsSL https://ollama.com/install.sh | sh
```
- LLMのモデルをダウンロードします．
```
ollama serve

# 別の端末で
ollama pull llama3
```
- サンプルプログラムを以下のコマンドでGitHubからクローンします．
```
cd ~/airobot_ws/src
git clone https://github.com/AI-Robot-Book-Humble/chapter3
```
- 以下のコマンドでパッケージをビルドします．
```
cd ~/airobot_ws
colcon build
source install/setup.bash
```

## ディレクトリ構成
- **[speech_action](speech_action):** アクション通信による音声認識と音声合成のサンプルプログラム

   
## 補足情報
 - ３章のサンプルプログラムをDockerコンテナで実行する場合，**Ubuntuをホストにする場合のみ動作を確認しています**．Windowsで開発されている方は，VMWareなどのバーチャルマシンにUbuntuをインストールしてサンプルプログラムを実行する事ができます．
 - 実行するUbuntuの環境でマイクからの音声入力とスピーカーからの音の出力が出来ていることを事前に確認してください．
 - バーチャルマシン上のUbuntuを使用する場合は、遅れが生じて音声が出力されない事があります．長めの発話文を入力するなどして対応して下さい．
 - 可能であればベッドセットを使用してください．実行中にスピーカーの音がマイクに回って繰り返す場合があります．