#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import DroneState
from geometry_msgs.msg import Vector3
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import yaml


def main():
    rclpy.init()
    node = StateBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

class StateBridgeNode(Node):
    def __init__(self):
        super().__init__('state_bridge')
        
        # デフォルトパラメータを設定
        self.declare_parameter('input_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('output_topic', '/drone_state')
        self.declare_parameter('log_level', 'info')
        self.declare_parameter('wind_max', 3.0)
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.log_level = self.get_parameter('log_level').get_parameter_value().string_value
        self.wind_max = self.get_parameter('wind_max').get_parameter_value().double_value
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub = self.create_subscription(
            VehicleOdometry,
            input_topic,
            self._cb,
            self.qos_profile,
        )
        self.pub = self.create_publisher(
            DroneState,
            output_topic,
            self.qos_profile,
        )
        
        if self.log_level == 'debug':
            self.get_logger().debug(
                f"Subscribed to: {input_topic}, Publishing to: {output_topic}, "
                f"QoS: {self.qos_profile}"
            )
        else:
            self.get_logger().info(
                f"State bridge initialized - Input: {input_topic}, Output: {output_topic}"
            )
            
    def _cb(self, msg: VehicleOdometry):
        # PX4 VehicleOdometry → DroneState 変換
        out = DroneState()
        # クォータニオン→オイラー角変換
        q = msg.q
        # PX4: [w, x, y, z] or [x, y, z, w]? → ROS標準は[x, y, z, w]
        # PX4は[x, y, z, w]（公式ドキュメントより）
        x, y, z, w = q
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        out.roll = float(roll)
        out.pitch = float(pitch)
        out.yaw = float(yaw)
        out.position = Vector3(x=msg.position[0], y=msg.position[1], z=msg.position[2])
        out.velocity = Vector3(x=msg.velocity[0], y=msg.velocity[1], z=msg.velocity[2])
        out.angular_velocity = Vector3(x=msg.angular_velocity[0], y=msg.angular_velocity[1], z=msg.angular_velocity[2])
        # wind: 毎回ランダムな風ベクトルを生成
        wind = np.random.uniform(-self.wind_max, self.wind_max, size=3)
        out.wind = Vector3(x=float(wind[0]), y=float(wind[1]), z=float(wind[2]))
        self.pub.publish(out)

if __name__ == "__main__":
    main() 