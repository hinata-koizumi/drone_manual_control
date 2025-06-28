#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class SimpleDroneSimulator(Node):
    """
    シンプルなドローンのシミュレーター
    物理的な制約や重力を考慮した簡単なシミュレーション
    """
    
    def __init__(self):
        """初期化"""
        super().__init__('simple_drone_simulator')
        
        # 物理パラメータ
        self.mass = 1.0  # kg
        self.gravity = 9.81  # m/s^2
        self.max_thrust = 20.0  # N
        
        # 制御パラメータ
        self.roll_p = 2.0
        self.pitch_p = 2.0
        self.yaw_p = 2.0
        
        # 状態変数
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        
        # 制御コマンド
        self.control_command = np.array([0.0, 0.0, 0.0, 0.0])  # throttle, roll, pitch, yaw
        self.horizontal_velocity_x = 0.0
        self.horizontal_velocity_y = 0.0
        
        # リセット関連
        self.reset_time = 0.0
        self.reset_cooldown = 1.0  # リセット後1秒間は制御コマンドを無視
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # 通信設定
        self._setup_communication()
        
        # タイマー設定 (100Hz)
        self.timer = self.create_timer(0.01, self._simulation_step)
        
        self.get_logger().info("Simple Drone Simulator initialized")
    
    def reset_drone(self):
        """ドローンを初期位置にリスポーン"""
        # 位置と速度をリセットして即座に(0,0,0)にリスポーン
        old_position = self.position.copy()
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])  # 速度もリセット
        
        # リセット時間を記録
        self.reset_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f"Drone respawned from [{old_position[0]:.2f}, {old_position[1]:.2f}, {old_position[2]:.2f}] to [0.00, 0.00, 0.00]")
        self.get_logger().info(f"Position array after reset: {self.position}")
        self.get_logger().info(f"Reset cooldown started at {self.reset_time:.2f}s")
        
        # 位置・速度を即時パブリッシュ
        self._publish_state()
    
    def _publish_state(self):
        """現在の位置・速度を即時パブリッシュ"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        # クォータニオン計算（簡易）
        import math
        cy = math.cos(self.orientation[2] * 0.5)
        sy = math.sin(self.orientation[2] * 0.5)
        cp = math.cos(self.orientation[1] * 0.5)
        sp = math.sin(self.orientation[1] * 0.5)
        cr = math.cos(self.orientation[0] * 0.5)
        sr = math.sin(self.orientation[0] * 0.5)
        pose_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
        self.pose_pub.publish(pose_msg)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.velocity[0]
        twist_msg.twist.linear.y = self.velocity[1]
        twist_msg.twist.linear.z = self.velocity[2]
        twist_msg.twist.angular.x = self.angular_velocity[0]
        twist_msg.twist.angular.y = self.angular_velocity[1]
        twist_msg.twist.angular.z = self.angular_velocity[2]
        self.twist_pub.publish(twist_msg)
    
    def _setup_communication(self):
        """通信設定"""
        # 制御コマンドサブスクライバー
        self.control_sub = self.create_subscription(
            TwistStamped,
            '/drone/control_command',
            self._control_callback,
            self.qos_profile
        )
        
        # 位置・姿勢パブリッシャー
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone/pose',
            self.qos_profile
        )
        
        # 速度パブリッシャー
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/drone/twist',
            self.qos_profile
        )
        
        # IMUパブリッシャー
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            self.qos_profile
        )
    
    def _control_callback(self, msg: TwistStamped):
        """制御コマンドコールバック"""
        # リセット後のクールダウン期間中は制御コマンドを無視
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.reset_time < self.reset_cooldown:
            self.get_logger().debug(f"Ignoring control command during reset cooldown ({current_time - self.reset_time:.2f}s remaining)")
            return
        
        # 制御コマンドを保存
        self.control_command[0] = msg.twist.linear.z  # throttle (Z軸)
        self.control_command[1] = msg.twist.angular.x  # roll
        self.control_command[2] = msg.twist.angular.y  # pitch
        self.control_command[3] = msg.twist.angular.z  # yaw
        
        # 水平移動コマンドを直接速度に変換
        self.horizontal_velocity_x = msg.twist.linear.x  # X軸の速度
        self.horizontal_velocity_y = msg.twist.linear.y  # Y軸の速度
        
        # リセットコマンドの検出（すべての値が0の場合）
        if (abs(msg.twist.linear.x) < 0.01 and abs(msg.twist.linear.y) < 0.01 and 
            abs(msg.twist.linear.z) < 0.01 and abs(msg.twist.angular.x) < 0.01 and 
            abs(msg.twist.angular.y) < 0.01 and abs(msg.twist.angular.z) < 0.01):
            # 即座にリセット実行
            self.get_logger().info("Reset command detected - executing immediate reset")
            self.get_logger().info(f"Current position before reset: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")
            self.reset_drone()
            # 制御コマンドもリセット
            self.control_command = np.array([0.0, 0.0, 0.0, 0.0])
            self.horizontal_velocity_x = 0.0
            self.horizontal_velocity_y = 0.0
            self.get_logger().info(f"Position after reset: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")
            return
        
        self.get_logger().info(
            f"Received control command: throttle={self.control_command[0]:.2f}, "
            f"roll={self.control_command[1]:.2f}, pitch={self.control_command[2]:.2f}, "
            f"yaw={self.control_command[3]:.2f}, vx={self.horizontal_velocity_x:.2f}, "
            f"vy={self.horizontal_velocity_y:.2f}"
        )
    
    def _simulation_step(self):
        """シミュレーションステップ"""
        # 制御コマンドの処理
        throttle = self.control_command[0]
        roll_cmd = self.control_command[1]
        pitch_cmd = self.control_command[2]
        yaw_cmd = self.control_command[3]
        
        # 推力計算 (負の値を許可してlandingコマンドに対応)
        thrust = throttle * self.max_thrust
        
        # 角度制御 (簡易的なPID制御)
        roll_error = roll_cmd - self.orientation[0]
        pitch_error = pitch_cmd - self.orientation[1]
        yaw_error = yaw_cmd - self.orientation[2]
        
        # 角速度更新
        self.angular_velocity[0] = self.roll_p * roll_error
        self.angular_velocity[1] = self.pitch_p * pitch_error
        self.angular_velocity[2] = self.yaw_p * yaw_error
        
        # 推力ベクトル計算
        thrust_vector = np.array([
            thrust * np.sin(pitch_cmd),
            -thrust * np.sin(roll_cmd),
            thrust * np.cos(roll_cmd) * np.cos(pitch_cmd)
        ])
        
        # 加速度計算 (重力 + 推力)
        acceleration = thrust_vector / self.mass - np.array([0.0, 0.0, self.gravity])
        
        # 速度更新
        self.velocity += acceleration * 0.01  # dt = 0.01s
        
        # 水平移動コマンドを直接適用
        self.velocity[0] = self.horizontal_velocity_x * self.max_thrust / self.mass  # X軸速度
        self.velocity[1] = self.horizontal_velocity_y * self.max_thrust / self.mass  # Y軸速度
        
        # 位置更新
        self.position += self.velocity * 0.01
        
        # 姿勢更新
        self.orientation += self.angular_velocity * 0.01
        
        # 地面との衝突処理
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            self.velocity[2] = 0.0
        
        # 位置・姿勢パブリッシュ
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.position[0]
        pose_msg.pose.position.y = self.position[1]
        pose_msg.pose.position.z = self.position[2]
        
        # クォータニオン計算 (簡易版)
        import math
        cy = math.cos(self.orientation[2] * 0.5)
        sy = math.sin(self.orientation[2] * 0.5)
        cp = math.cos(self.orientation[1] * 0.5)
        sp = math.sin(self.orientation[1] * 0.5)
        cr = math.cos(self.orientation[0] * 0.5)
        sr = math.sin(self.orientation[0] * 0.5)
        
        pose_msg.pose.orientation.w = cy * cp * cr + sy * sp * sr
        pose_msg.pose.orientation.x = cy * cp * sr - sy * sp * cr
        pose_msg.pose.orientation.y = sy * cp * sr + cy * sp * cr
        pose_msg.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        self.pose_pub.publish(pose_msg)
        
        # 速度パブリッシュ
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "world"
        twist_msg.twist.linear.x = self.velocity[0]
        twist_msg.twist.linear.y = self.velocity[1]
        twist_msg.twist.linear.z = self.velocity[2]
        twist_msg.twist.angular.x = self.angular_velocity[0]
        twist_msg.twist.angular.y = self.angular_velocity[1]
        twist_msg.twist.angular.z = self.angular_velocity[2]
        
        self.twist_pub.publish(twist_msg)
        
        # IMUデータパブリッシュ
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = 0.0  # 簡易版
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = -self.gravity
        imu_msg.angular_velocity.x = self.angular_velocity[0]
        imu_msg.angular_velocity.y = self.angular_velocity[1]
        imu_msg.angular_velocity.z = self.angular_velocity[2]
        
        self.imu_pub.publish(imu_msg)
        
        # 状態ログ (1秒に1回)
        if int(time.time() * 100) % 100 == 0:
            self.get_logger().info(
                f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}] "
                f"Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f}]"
            )


def main():
    rclpy.init()
    node = SimpleDroneSimulator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            executor.shutdown()
        except Exception as e:
            print(f"Warning: Error during cleanup: {e}")
        
        # rclpy.shutdown()は既に呼ばれている可能性があるため、try-exceptで囲む
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # 既にシャットダウンされている場合は無視
            pass


if __name__ == '__main__':
    main() 