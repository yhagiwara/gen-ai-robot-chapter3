import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.action import StringCommand
from ollama import chat
from ollama import ChatResponse


class LLMActionServer(Node):
    def __init__(self):
        super().__init__('llm_action_server')
        self.get_logger().info('LLMアクションサーバを起動します．')
        self.llm_model = 'llama3'
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.action_server = ActionServer(
            self,
            StringCommand,
            'llm/command',
            self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info('前の要求を中止')
                self.goal_handle.abort()
            self.goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        with self.execute_lock:
            self.get_logger().info('実行...')
            result = StringCommand.Result()
            result.answer = 'NG'

            prompt = goal_handle.request.command.strip()
            if prompt == "":
                self.get_logger().warn("プロンプトが空")
                if goal_handle.is_active:
                    goal_handle.abort()
                return result

            try:
                self.get_logger().info(f'プロンプト受信: "{prompt}"')
                response: ChatResponse = chat(model=self.llm_model, messages=[
                    {
                        'role': 'system',
                        'content': 'From now on, we will have a conversation, so try to make sentences short.'
                    },
                    {
                        'role': 'user', 
                        'content': prompt
                    },
                ])
                answer = response['message']['content']
                self.get_logger().info(f'LLMの応答: {answer}')

                result.answer = answer
                goal_handle.succeed()
            except Exception as e:
                self.get_logger().error(f'LLM呼び出し失敗: {e}')
                goal_handle.abort()

            return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('キャンセル受信')
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = LLMActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
