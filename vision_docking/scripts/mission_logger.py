#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import os

CSV_PATH = os.path.expanduser(
    '~/robotics_study/experiments/mission_results.csv')


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        self.sub = self.create_subscription(
            String, '/mission_status', self.status_cb, 10)

        self.start_time = None
        self.last_state = None
        self.count = 0
        self.current_result = None

        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w') as f:
                f.write('序号,时间戳,成功,总耗时_s,说明\n')

        self.get_logger().info(f'任务记录器启动，输出: {CSV_PATH}')
        self.get_logger().info('在 RViz 里点 Nav2 Goal 开始第 1 次实验')

    def status_cb(self, msg):
        state = msg.data

        if state == self.last_state:
            return

        self.get_logger().info(f'任务状态: {state}')

        if state == 'NAVIGATING' and self.start_time is None:
            # 开始计时（导航阶段开始）
            self.start_time = datetime.now()
            self.count += 1
            self.current_result = None
            self.get_logger().info(
                f'>>> 第 {self.count} 次实验开始 <<<')

        elif state == 'COMPLETED':
            if self.start_time is not None:
                elapsed = (datetime.now() - self.start_time).total_seconds()
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                with open(CSV_PATH, 'a') as f:
                    f.write(f'{self.count},{ts},是,{elapsed:.1f},导航+视觉停靠\n')
                self.get_logger().info(
                    f'<<< 第 {self.count} 次成功，耗时 {elapsed:.1f}s >>>')
                self.current_result = 'success'

        elif state == 'FAILED':
            if self.start_time is not None:
                elapsed = (datetime.now() - self.start_time).total_seconds()
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                with open(CSV_PATH, 'a') as f:
                    f.write(f'{self.count},{ts},否,{elapsed:.1f},失败\n')
                self.get_logger().warn(
                    f'<<< 第 {self.count} 次失败，耗时 {elapsed:.1f}s >>>')
                self.current_result = 'failed'

        elif state == 'IDLE':
            # 回到 IDLE，说明本次实验结束
            if self.start_time is not None and self.current_result is not None:
                self.start_time = None
                self.current_result = None
                self.get_logger().info(
                    f'准备好第 {self.count + 1} 次实验，请在 RViz 点 Nav2 Goal')

        self.last_state = state


def main():
    rclpy.init()
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
