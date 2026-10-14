#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import time


class MissionState:
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    DOCKING = 'DOCKING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # 参数
        self.declare_parameter('nav_timeout', 120.0)
        self.declare_parameter('dock_timeout', 60.0)
        self.declare_parameter('auto_dock_after_nav', True)

        self.nav_timeout = self.get_parameter('nav_timeout').value
        self.dock_timeout = self.get_parameter('dock_timeout').value
        self.auto_dock_after_nav = self.get_parameter(
            'auto_dock_after_nav').value

        # 状态
        self.state = MissionState.IDLE
        self.state_enter_time = time.time()
        self.nav_goal_handle = None
        self.reset_timer = None

        # Action 客户端
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # 订阅 RViz 发的目标
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        # 订阅 docking 状态
        self.docking_sub = self.create_subscription(
            String, '/docking_status', self.docking_status_cb, 10)

        # 发布
        self.docking_start_pub = self.create_publisher(
            String, '/docking_start', 10)
        self.mission_status_pub = self.create_publisher(
            String, '/mission_status', 10)

        # 超时检查
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

        self.get_logger().info(
            '任务管理器启动。在 RViz 里点 Nav2 Goal 开始任务')

    def goal_cb(self, msg):
        if self.state != MissionState.IDLE:
            self.get_logger().warn(
                f'当前状态 {self.state}，忽略新目标')
            return

        self.get_logger().info(
            f'收到 RViz 目标: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')
        self.start_navigation(msg)

    def start_navigation(self, target_pose):
        self.transition(MissionState.NAVIGATING)

        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        goal.pose.header.frame_id = 'map'

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 未就绪')
            self.transition(MissionState.FAILED)
            return

        self.get_logger().info('发送导航目标给 Nav2')
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_cb)
        send_future.add_done_callback(self.nav_goal_response_cb)

    def nav_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        key = int(dist * 2)
        if not hasattr(self, '_last_feedback_key') or \
                self._last_feedback_key != key:
            self._last_feedback_key = key
            self.get_logger().info(f'导航中，剩余 {dist:.2f}m')

    def nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝')
            self.transition(MissionState.FAILED)
            return

        self.nav_goal_handle = goal_handle
        self.get_logger().info('导航目标已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        result = future.result()
        status = result.status

        if status == 4:
            self.get_logger().info('导航完成')
            if self.auto_dock_after_nav:
                self.get_logger().info('自动切换到视觉停靠')
                self.start_docking()
            else:
                self.transition(MissionState.COMPLETED)
        else:
            self.get_logger().error(f'导航失败，状态码: {status}')
            self.transition(MissionState.FAILED)

    def start_docking(self):
        self.transition(MissionState.DOCKING)
        msg = String()
        msg.data = 'start'
        self.docking_start_pub.publish(msg)
        self.get_logger().info('已发送 /docking_start')

    def docking_status_cb(self, msg):
        state = msg.data
        self.get_logger().info(f'停靠状态: {state}')

        if state == 'DOCKED' and self.state == MissionState.DOCKING:
            self.get_logger().info('视觉停靠完成')
            self.transition(MissionState.COMPLETED)
        elif state == 'IDLE' and self.state == MissionState.DOCKING:
            if time.time() - self.state_enter_time > 5.0:
                self.get_logger().error('视觉停靠失败')
                self.transition(MissionState.FAILED)

    def check_timeout(self):
        if self.state == MissionState.NAVIGATING:
            if time.time() - self.state_enter_time > self.nav_timeout:
                self.get_logger().error(f'导航超时 ({self.nav_timeout}s)')
                if self.nav_goal_handle is not None:
                    self.nav_goal_handle.cancel_goal_async()
                self.transition(MissionState.FAILED)

        elif self.state == MissionState.DOCKING:
            if time.time() - self.state_enter_time > self.dock_timeout:
                self.get_logger().error(f'停靠超时 ({self.dock_timeout}s)')
                self.transition(MissionState.FAILED)

    def transition(self, new_state):
        self.get_logger().info(f'任务状态: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter_time = time.time()

        msg = String()
        msg.data = self.state
        self.mission_status_pub.publish(msg)

        # 失败或完成 5 秒后自动复位到 IDLE
        if new_state in (MissionState.FAILED, MissionState.COMPLETED):
            if self.reset_timer is not None:
                self.reset_timer.cancel()
            self.reset_timer = self.create_timer(5.0, self.reset_to_idle)

    def reset_to_idle(self):
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer.destroy()
            self.reset_timer = None

        if self.state in (MissionState.FAILED, MissionState.COMPLETED):
            self.get_logger().info('自动复位到 IDLE')
            self.state = MissionState.IDLE
            self.state_enter_time = time.time()
            msg = String()
            msg.data = self.state
            self.mission_status_pub.publish(msg)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
