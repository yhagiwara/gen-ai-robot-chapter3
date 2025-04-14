# speech_action
## 概要
３章のサンプルプログラム  
音声認識と音声合成を実行するプログラム


## 実行

- 音声認識の実行手順（3.1.4節）
  - 音声認識サーバを起動します。
    ```
    ros2 run speech_action speech_recognition_server
    ```
  - 新しい端末を開き、音声認識クライアントを起動します。
    ```
    ros2 run speech_action speech_recognition_client
    ```
  - クライアントが起動すると、初期プロンプトの入力を求められます。ここでは何も入力せずに Enter を押します
  - マイクに向かって「Bring me a bottle from kitchen.」などの音声指示を発話してください。
  - 途中でキャンセルしたい場合は,cを入力します


- 音声認識のゼロショット学習（3.1.5節）
  - 音声認識サーバを起動します。
    ```
    ros2 run speech_action speech_recognition_server
    ```
  - 新しい端末を開き、音声認識クライアントを起動します。
    ```
    ros2 run speech_action speech_recognition_client
    ```
  - クライアントが起動すると、初期プロンプトの入力を求められます。ここで「Hikonyan」と入力し、Enter を押します。
  - マイクに向かって「Bring me the Hikonyan.」と発話してください。
  - 途中でキャンセルしたい場合は,cを入力します
  
- 音声合成の実行手順（3.2.4節）
  - 端末を開いて音声合成サーバを起動します．
    ```
    ros2 run speech_action speech_synthesis_server
    ```
  - 新しい端末を開いて音声合成クライアントを起動します．
    ```
    ros2 run speech_action speech_synthesis_client
    ```
  - 同じ端末で発話させたいメッセージを入力して送ります．
    ```
    > I will go to the kitchen and grab a bottle.
    ```
  - スピーカから音声が出力されます．

- 音声認識と音声合成の統合の実行手順（3.3節）
  - 端末を開いて音声合成サーバを起動します．
    ```
    ros2 run speech_action speech_recognition_server
    ```
  - 新しい端末を開いて音声認識サーバを起動します．
    ```
    ros2 run speech_action speech_synthesis_server
    ```
  - さらに新しい端末を開いてオウム返しクライアントを起動します．
    ```
    ros2 run speech_action parrot_client
    ```
  - マイクに向かって発話すると同じ発話がスピーカから返ってきます．
  
- 生成 AI による応答文生成の実行手順（3.4.2節）
  - ollamaを起動します．
    ```
    ollama serve
    ```
  - 新しい端末を開いてLLMサーバを起動します．
    ```
    ros2 run speech_action llm_server
    ```
  - さらに新しい端末を開いてLLMクライアントを起動します．
    ```
    ros2 run speech_action llm_client
    ```
  - 同じ端末で質問を入力して送ります．
    ```
    > What is the highest mountain in Japan? (Only mountain name)
    ```
  - 質問に対する回答が返ってきます．

- 音声認識・合成と応答文生成の統合の実行手順（3.4.3節）
  - 5つの端末を開いて，それぞれの端末に以下のコマンドを入力します．
  - 音声認識サーバを起動します．
    ```
    ollama speech_recognition_server
    ```
  - 音声合成サーバを起動します．
    ```
    ros2 run speech_action speech_synthesis_server
    ```
  - ollamaを起動します．
    ```
    ollama serve
    ```
  - LLMサーバを起動します．
    ```
    ros2 run speech_action llm_server
    ```
  - 音声チャットクライアントを起動します．
    ```
    ros2 run speech_action q_and_a_client
    ```
  - マイクに向かって発話するとチャットすることができます．
  

## ヘルプ
- このサンプルプログラムをDockerコンテナで実行する場合，**Ubuntuをホストにする場合のみ動作を確認しています**．Windowsで開発されている方は，VMWareなどのバーチャルマシンにUbuntuをインストールしてサンプルプログラムを実行する事ができます．
- 音声認識の実行手順において，音声がスピーカーから出力されない場合は，speech_synthesis_server_soundfile.pyからの実行を試してください．音声合成されたmp3ファイルが出力されるので，そのファイルを再生して確認してください．
- 音声認識の実行手順において，Whisperのモデルサイズや認識する言語を変更する場合は，recognition.pyの中にあるrecognize_whisperの引数を変更してください．modelは[https://github.com/openai/whisper#available-models-and-languages](https://github.com/openai/whisper#available-models-and-languages)，languageは[https://github.com/openai/whisper/blob/main/whisper/tokenizer.py](https://github.com/openai/whisper/blob/main/whisper/tokenizer.py)を参照し，使用可能なモデルと言語を記載してください．
- 音声合成の実行手順において，Kokoroの音声モデルや発話する言語を変更する場合は，speech_synthesis_server.pyの中にあるkokoroの引数を変更してください．lang_codeとkokoro_voiceは[https://huggingface.co/hexgrad/Kokoro-82M/blob/main/VOICES.md](https://huggingface.co/hexgrad/Kokoro-82M/blob/main/VOICES.md)を参照し，使用可能な言語と音声モデルを記載してください．


## 著者
萩原　良信

## 履歴
- 2022-08-28: 初期版
- 2024-01-26: 改訂版

## ライセンス
Copyright (c) 2022-2025, Yoshinobu Hagiwara, Okuma Yuki, Valentin Cardenas Keith, Masaki Ito and Shoichi Hasegawa
All rights reserved.
This project is licensed under the Apache-2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
