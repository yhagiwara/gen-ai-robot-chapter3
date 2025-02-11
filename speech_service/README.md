# speech_service
## 概要
３章のサンプルプログラム  
音声認識と音声合成を実行するプログラム


## 実行
  
- サービスによるオウム返しの実行手順
  - 音声サービスを起動します．
    ```
    ros2 run speech_service speech_service
    ```
  - 音声サービスに開始命令を送ります．
    ```
    ros2 service call /speech_service/wake_up airobot_interfaces/srv/StringCommand "{command: 'start'}"
    ```

## ヘルプ
- このサンプルプログラムをDockerコンテナで実行する場合，**Ubuntuをホストにする場合のみ動作を確認しています**．Windowsで開発されている方は，VMWareなどのバーチャルマシンにUbuntuをインストールしてサンプルプログラムを実行する事ができます．
- 音声認識の実行手順において，Whisperのモデルサイズや認識する言語を変更する場合は，speech_service.pyの中にあるrecognize_whisperの引数を変更してください．modelは[https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)，languageは[https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)を参照し，使用可能なモデルと言語を記載してください．
- サービスによるオウム返しの実行手順において，音声がスピーカーから出力されない場合は，speech_service_mpg123.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください． 

## 著者
萩原　良信

## 履歴
- 2022-08-28: 初期版
- 2024-01-26: 改訂版

## ライセンス
Copyright (c) 2022-2025, Yoshinobu Hagiwara, Masaki Ito and Shoichi Hasegawa
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
