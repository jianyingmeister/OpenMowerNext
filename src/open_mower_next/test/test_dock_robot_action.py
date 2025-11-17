#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from open_mower_next.action import DockRobot


class DockRobotClient(Node):
    def __init__(self):
        super().__init__('dock_robot_client')
        self._action_client = ActionClient(self, DockRobot, 'dock_robot')

    def send_goal(self, start_docking=True):
        # 等待动作服务器启动
        self.get_logger().info('Waiting for DockRobot action server...')
        self._action_client.wait_for_server()

        # 构造 goal
        goal_msg = DockRobot.Goal()
        goal_msg.start_docking = start_docking
        self.get_logger().info(f'Sending docking goal: start_docking={start_docking}')

        # 发送 goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: distance_to_dock={feedback.distance_to_dock:.2f}, '
            f'angle_error={feedback.angle_error:.2f}, status={feedback.status}'
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, message="{result.message}"')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DockRobotClient()
    node.send_goal(True)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
