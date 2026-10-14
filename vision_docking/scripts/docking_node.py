#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


class DockingState:
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    COARSE_APPROACH = 'COARSE_APPROACH'
    ALIGN = 'ALIGN'
    FINE_APPROACH = 'FINE_APPROACH'
    DOCKED = 'DOCKED'


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        # 参数
        self.marker_size = 0.08
        self.half = self.marker_size / 2.0
        self.target_distance = 0.35
        self.coarse_dist = 0.55
        self.fine_angle_tol = 0.15
        self.fine_dist_tol = 0.08
        self.fine_yaw_tol = 0.08
        self.search_speed = 0.3
        self.search_timeout = 30.0
        self.marker_px_max = 800

        # 控制增益
        self.kx = 0.5
        self.ky = 1.0
        self.ktheta = 1.5
        self.v_max = 0.15
        self.omega_max = 0.5

        # 状态
        self.state = DockingState.IDLE
        self.marker_lost_count = 0
        self.stable_count = 0
        self.state_enter_time = self.get_clock().now()

        # ArUco
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        self.object_points = np.array([
            [-self.half,  self.half, 0],
            [ self.half,  self.half, 0],
            [ self.half, -self.half, 0],
            [-self.half, -self.half, 0],
        ], dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 标记数据
        self.marker_in_base = None
        self.marker_detected = False
        self.marker_px_width = 0.0

        # 统计
        self.tf_fail_count = 0
        self.tf_success_count = 0
        self.last_tf_warn_time = 0.0

        # 订阅
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_cb, 10)
        self.start_sub = self.create_subscription(
            String, '/docking_start', self.start_cb, 10)

        # 发布
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/docking_status', 10)

        # 控制定时器
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('DockingNode 启动，等待 /docking_start 指令')
        self.publish_status()

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.get_logger().info('相机内参已获取')

    def start_cb(self, msg):
        if self.state == DockingState.IDLE:
            self.get_logger().info('收到停靠指令，开始搜索标记')
            self.tf_fail_count = 0
            self.tf_success_count = 0
            self.transition(DockingState.SEARCH)
        else:
            self.get_logger().warn(
                f'当前状态 {self.state}，忽略指令（等自动复位）')

    def transition(self, new_state):
        self.get_logger().info(f'状态切换: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self.stable_count = 0
        self.marker_lost_count = 0
        self.publish_status()

    def time_in_state(self):
        elapsed = (self.get_clock().now() - self.state_enter_time)
        return elapsed.nanoseconds / 1e9

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 错误: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters)

        if ids is None or len(ids) == 0:
            self.marker_lost_count += 1
            self.marker_detected = False
            return

        self.marker_lost_count = 0
        self.marker_detected = True

        # PnP
        image_points = corners[0][0].astype(np.float32)
        success, rvec, tvec = cv2.solvePnP(
            self.object_points, image_points,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE)

        if not success:
            return

        pts = corners[0][0]
        self.marker_px_width = float(np.linalg.norm(pts[0] - pts[1]))

        # TF 转换（用最新 TF，不用图像时间戳，避免 Extrapolation Error）
        marker_in_cam = PoseStamped()
        marker_in_cam.header.frame_id = 'camera_rgb_optical_frame'
        marker_in_cam.header.stamp = rclpy.time.Time().to_msg()
        marker_in_cam.pose.position.x = float(tvec[0][0])
        marker_in_cam.pose.position.y = float(tvec[1][0])
        marker_in_cam.pose.position.z = float(tvec[2][0])
        marker_in_cam.pose.orientation.w = 1.0

        try:
            self.marker_in_base = self.tf_buffer.transform(
                marker_in_cam, 'base_link',
                timeout=Duration(seconds=0.05))
            self.tf_success_count += 1
        except Exception as e:
            self.tf_fail_count += 1
            import time as _t
            now = _t.time()
            if now - self.last_tf_warn_time > 5.0:
                self.last_tf_warn_time = now
                self.get_logger().warn(
                    f'TF 失败 ({self.tf_fail_count} 次): {e}')
            self.marker_in_base = None
            return

    def control_loop(self):
        cmd = Twist()

        if self.state == DockingState.IDLE:
            self.cmd_pub.publish(cmd)
            return

        if self.state == DockingState.SEARCH:
            cmd.angular.z = self.search_speed
            if self.marker_detected:
                self.stable_count += 1
                if self.stable_count >= 3:
                    self.transition(DockingState.COARSE_APPROACH)
            else:
                self.stable_count = 0

            if self.time_in_state() > self.search_timeout:
                self.get_logger().warn(
                    f'搜索超时（TF 成功 {self.tf_success_count} 次，'
                    f'失败 {self.tf_fail_count} 次）')
                self.transition(DockingState.IDLE)
                return

        elif self.state == DockingState.COARSE_APPROACH:
            if self.marker_lost_count > 20:
                self.get_logger().warn('标记丢失，回到搜索')
                self.transition(DockingState.SEARCH)
                return

            if self.marker_in_base is None:
                self.cmd_pub.publish(cmd)
                return

            x = self.marker_in_base.pose.position.x
            y = self.marker_in_base.pose.position.y
            dist = math.sqrt(x*x + y*y)

            if self.marker_px_width > self.marker_px_max:
                self.get_logger().warn(
                    f'标记占图 {self.marker_px_width:.0f}px，停车保护')
                self.transition(DockingState.DOCKED)
                return

            if dist < self.coarse_dist:
                self.transition(DockingState.ALIGN)
            else:
                yaw_err = math.atan2(y, x)
                x_err = x - self.target_distance

                cmd.linear.x = max(0.0, min(self.kx * x_err, self.v_max))
                cmd.angular.z = max(-self.omega_max,
                                    min(self.ky * y + self.ktheta * yaw_err,
                                        self.omega_max))

        elif self.state == DockingState.ALIGN:
            if self.marker_lost_count > 20:
                self.transition(DockingState.SEARCH)
                return

            if self.marker_in_base is None:
                self.cmd_pub.publish(cmd)
                return

            x = self.marker_in_base.pose.position.x
            y = self.marker_in_base.pose.position.y
            yaw_err = math.atan2(y, x)

            if self.marker_px_width > self.marker_px_max:
                self.get_logger().warn(
                    f'标记占图 {self.marker_px_width:.0f}px，停车保护')
                self.transition(DockingState.DOCKED)
                return

            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.omega_max,
                                min(self.ktheta * yaw_err,
                                    self.omega_max))

            if abs(yaw_err) < self.fine_angle_tol:
                self.transition(DockingState.FINE_APPROACH)

        elif self.state == DockingState.FINE_APPROACH:
            # 短时间丢失标记：停车，不移动
            if self.marker_lost_count > 3:
                # 停住
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                # 长时间丢失：原地慢速转圈找标记
                if self.marker_lost_count > 20:
                    self.get_logger().warn('FINE 阶段丢失标记，原地慢速搜索')
                    cmd.angular.z = 0.15
                self.cmd_pub.publish(cmd)
                return

            if self.marker_in_base is None:
                self.cmd_pub.publish(cmd)
                return

            x = self.marker_in_base.pose.position.x
            y = self.marker_in_base.pose.position.y
            yaw_err = math.atan2(y, x)
            x_err = x - self.target_distance
            dist_err = math.sqrt(x_err*x_err + y*y)

            if self.marker_px_width > self.marker_px_max:
                self.get_logger().warn(
                    f'标记占图 {self.marker_px_width:.0f}px，停车保护')
                self.transition(DockingState.DOCKED)
                return

            if x < 0.12 or x > 2.0:
                self.get_logger().warn(f'x 值异常: {x:.3f}，停车')
                self.transition(DockingState.DOCKED)
                return

            if (dist_err < self.fine_dist_tol and
                    abs(yaw_err) < self.fine_yaw_tol):
                self.stable_count += 1
                if self.stable_count >= 3:
                    self.transition(DockingState.DOCKED)
            else:
                self.stable_count = 0

            cmd.linear.x = max(0.0,
                               min(self.kx * x_err, self.v_max))
            cmd.angular.z = max(-self.omega_max,
                                min(self.ky * y + self.ktheta * yaw_err,
                                    self.omega_max))

        elif self.state == DockingState.DOCKED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            if self.time_in_state() > 3.0:
                self.get_logger().info('自动复位到 IDLE')
                self.transition(DockingState.IDLE)
                return

        self.cmd_pub.publish(cmd)

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
