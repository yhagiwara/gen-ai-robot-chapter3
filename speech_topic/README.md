# speech_topic
## 概要
３章のサンプルプログラム  
音声認識と音声合成を実行するプログラム


## 実行
- 音声認識の実行手順
  - 端末を開いて /speech トピックへパブリッシュしたデータを見られるようにします．
    ```
    ros2 topic echo /speech
    ```
  - 新しい端末を開いて音声認識を起動します．
    ```
    ros2 run speech_topic recognition
    ```
  - マイクに向かって発話します．
  
- 音声合成の実行手順
  - 端末を開いて音声合成を起動します．
    ```
    ros2 run speech_topic synthesis
    ```
  - 新しい端末を開いて発話させたいメッセージを /speech に送ります．
    ```
    ros2 topic pub -1 /speech std_msgs/msg/String "{data: 'I will go to the kitchen and grab a bottle.'}"
    ```
  - スピーカから音声が出力されます．

- トピック通信によるオウム返しの実行手順
  - 端末を開いて音声合成を起動します．
    ```
    ros2 run speech_topic synthesis
    ```
  - 新しい端末を開いて音声認識を起動します．
    ```
    ros2 run speech_topic recognition
    ```
  - マイクに向かって発話すると同じ発話がスピーカから返ってきます．
  

## ヘルプ
- このサンプルプログラムをDockerコンテナで実行する場合，**Ubuntuをホストにする場合のみ動作を確認しています**．Windowsで開発されている方は，VMWareなどのバーチャルマシンにUbuntuをインストールしてサンプルプログラムを実行する事ができます．
- 音声認識の実行手順において，音声がスピーカーから出力されない場合は，synthesis_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください．
- トピックによる音声認識の実行手順において，Whisperのモデルサイズや認識する言語を変更する場合は，recognition.pyの中にあるrecognize_whisperの引数を変更してください．modelは[https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)，languageは[https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)を参照し，使用可能なモデルと言語を記載してください．


## 著者
萩原　良信

## 履歴
- 2024-01-26: 改訂版
- 2022-08-28: 初期版

## ライセンス
Copyright (c) 2022-2025, Yoshinobu Hagiwara, Okuma Yuki, Valentin Cardenas Keith, Masaki Ito and Shoichi Hasegawa
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
