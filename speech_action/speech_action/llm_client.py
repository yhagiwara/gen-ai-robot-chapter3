import threading
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from airobot_interfaces.action import StringCommand


class LLMClient(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.get_logger().info('LLMクライアントを起動します．')
        self.goal_handle = None
        self.action_client = ActionClient(
            self, StringCommand, 'llm/command')

    def send_prompt(self, prompt: str):
        self.get_logger().info('アクションサーバ待機...')
        self.action_client.wait_for_server()
        goal_msg = StringCommand.Goal()
        goal_msg.command = prompt
        self.get_logger().info(f'プロンプト送信...')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('ゴールは拒否されました')
            return

        self.goal_handle = goal_handle
        self.get_logger().info('ゴールは受け付けられました')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'LLMの応答: {result.answer}')
            self.goal_handle = None
        else:
            self.get_logger().info(f'失敗ステータス: {status}')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('キャンセル成功')
            self.goal_handle = None
        else:
            self.get_logger().info('キャンセル失敗')
    def cancel(self):
        if self.goal_handle is None:
            self.get_logger().info('キャンセル対象なし')
            return
        self.get_logger().info('キャンセル')
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)


def main():
    # ROSクライアントの初期化
    rclpy.init()
    node = LLMClient()

    # 別のスレッドでrclpy.spin()を実行する
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    try:
        while True:
            prompt = input('> ')
            if prompt == "":
                node.cancel()
            else:
                node.send_prompt(prompt)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()