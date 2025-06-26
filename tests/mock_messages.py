#!/usr/bin/env python3

"""
ROS 2メッセージのモッククラス
テスト環境でROS 2がなくてもテストできるようにする
"""

import time
from dataclasses import dataclass
from typing import List


class Header:
    """メッセージヘッダーのモック"""
    def __init__(self):
        self.stamp = self._get_time()
        self.frame_id = "world"
    
    def _get_time(self):
        """現在時刻を取得"""
        return time.time()


class Point:
    """3D点のモック"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Quaternion:
    """クォータニオンのモック"""
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Vector3:
    """3Dベクトルのモック"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Pose:
    """姿勢のモック"""
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class Twist:
    """速度のモック"""
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class PoseStamped:
    """タイムスタンプ付き姿勢のモック"""
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class TwistStamped:
    """タイムスタンプ付き速度のモック"""
    def __init__(self):
        self.header = Header()
        self.twist = Twist()


class Imu:
    """IMUデータのモック"""
    def __init__(self):
        self.header = Header()
        self.orientation = Quaternion()
        self.orientation_covariance = [0.0] * 9
        self.angular_velocity = Vector3()
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration = Vector3()
        self.linear_acceleration_covariance = [0.0] * 9


class DroneControlCommand:
    """ドローン制御コマンドのモック"""
    def __init__(self):
        self.throttle1 = 0.0
        self.throttle2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0


class ActuatorMotors:
    """アクチュエータモーターのモック"""
    def __init__(self):
        self.timestamp = time.time()
        self.reversible = [False] * 32
        self.control = [0.0] * 32


# テスト用の便利関数
def create_test_pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    """テスト用の姿勢を作成"""
    import math
    
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # オイラー角からクォータニオンに変換（簡易版）
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
    pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
    pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
    pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
    
    return pose


def create_test_twist(linear_x=0.0, linear_y=0.0, linear_z=0.0, 
                     angular_x=0.0, angular_y=0.0, angular_z=0.0):
    """テスト用の速度を作成"""
    twist = TwistStamped()
    twist.twist.linear.x = linear_x
    twist.twist.linear.y = linear_y
    twist.twist.linear.z = linear_z
    twist.twist.angular.x = angular_x
    twist.twist.angular.y = angular_y
    twist.twist.angular.z = angular_z
    return twist


def create_test_control_command(throttle1=0.5, throttle2=0.5, angle1=0.0, angle2=0.0):
    """テスト用の制御コマンドを作成"""
    command = DroneControlCommand()
    command.throttle1 = throttle1
    command.throttle2 = throttle2
    command.angle1 = angle1
    command.angle2 = angle2
    return command 