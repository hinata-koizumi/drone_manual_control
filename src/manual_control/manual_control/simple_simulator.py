#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time

from drone_msgs.msg import DroneControlCommand
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu


class SimpleDroneSimulator(Node):
    """
    シンプルなドローンのシミュレーター
    物理的な制約や重力を考慮した簡単なシミュレーション
    """
    
    def __init__(self):
        super().__init__('simple_drone_simulator')
        
        # QoS設定
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # ドローンの状態
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z [m]
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz [m/s]
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw [rad]
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # wx, wy, wz [rad/s]
        
        # 物理パラメータ
        self.mass = 0.65  # kg (drone_specs.yamlから)
        self.gravity = 9.81  # m/s²
        self.thrust_constant = 1.6  # N/unit (drone_specs.yamlから)
        self.max_thrust = 15.0  # N (概算)
        
        # 制御パラメータ
        self.roll_p = 4.8
        self.pitch_p = 4.8
        self.yaw_p = 3.2
        
        # サブスクライバー・パブリッシャー設定
        self._setup_communication()
        
        # シミュレーションタイマー (100Hz)
        self.timer = self.create_timer(0.01, self._simulation_step)
        
        self.get_logger().info("Simple Drone Simulator initialized")
    
    def _setup_communication(self):
        """通信設定"""
        # 制御コマンドサブスクライバー
        self.control_sub = self.create_subscription(
            DroneControlCommand,
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
    
    def _control_callback(self, msg: DroneControlCommand):
        """制御コマンドコールバック"""
        # 推力と角度コマンドを処理
        throttle = (msg.throttle1 + msg.throttle2) / 2.0
        roll_cmd = msg.angle1
        pitch_cmd = msg.angle2
        
        # 推力計算 (0.0-1.0 -> 0.0-max_thrust)
        thrust = throttle * self.max_thrust
        
        # 角度制御 (簡易的なPID制御)
        roll_error = roll_cmd - self.orientation[0]
        pitch_error = pitch_cmd - self.orientation[1]
        
        # 角速度更新
        self.angular_velocity[0] = self.roll_p * roll_error
        self.angular_velocity[1] = self.pitch_p * pitch_error
        
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
        
        # 位置更新
        self.position += self.velocity * 0.01
        
        # 姿勢更新
        self.orientation += self.angular_velocity * 0.01
        
        # 地面との衝突処理
        if self.position[2] < 0.0:
            self.position[2] = 0.0
            self.velocity[2] = 0.0
    
    def _simulation_step(self):
        """シミュレーションステップ"""
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 