import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from kokoro import KPipeline
import soundfile as sf
import simpleaudio as sa
import os


class SpeechSynthesisServer(Node):
    def __init__(self):
        super().__init__('speech_synthesis_server')
        self.get_logger().info('音声合成サーバを起動します．')
        self.kokoro_pipeline = KPipeline(lang_code='a')
        # self.pipeline = KPipeline(lang_code='j')
        self.kokoro_voice = "af_heart"
        self.kokoro_speed = 1
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'speech_synthesis/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の発話を中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('実行...')
            result = StringCommand.Result()
            result.answer = 'NG'
            if goal_handle.request.command != '':
                text = goal_handle.request.command
                self.get_logger().info(f'発話： {text}')
                kokoro_tts = self.kokoro_pipeline(text, voice=self.kokoro_voice, speed=self.kokoro_speed)
                _, ps, audio = next(kokoro_tts)
                self.get_logger().info(f'音素： {ps}')

                if not goal_handle.is_active:
                    self.get_logger().info('中止')
                    return result

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('キャンセル')
                    return result
                
                os.makedirs('sound', exist_ok=True)
                filename = 'sound/speech.wav'
                sf.write(filename, audio, 24000)

                wave_obj = sa.WaveObject.from_wave_file(filename)
                play_obj = wave_obj.play()
                play_obj.wait_done()

                goal_handle.succeed()
                result.answer = 'OK'
            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = SpeechSynthesisServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
