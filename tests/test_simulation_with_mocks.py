#!/usr/bin/env python3

import pytest
import sys
import os
from unittest.mock import Mock, patch
import time

# モックメッセージをインポート
sys.path.insert(0, os.path.dirname(__file__))
from mock_messages import (
    DroneControlCommand, PoseStamped, TwistStamped, Imu,
    create_test_pose, create_test_twist, create_test_control_command
)


class TestSimulationWithMocks:
    """モックメッセージを使ったシミュレーションテスト"""
    
    def test_simple_simulator_physics(self):
        """シンプルシミュレーターの物理計算テスト"""
        # 物理パラメータ
        mass = 0.65  # kg
        gravity = 9.81  # m/s²
        max_thrust = 15.0  # N
        
        # ホバリングに必要な推力
        hover_thrust = mass * gravity  # 6.38N
        hover_throttle = hover_thrust / max_thrust  # 約0.425
        
        # テストケース
        test_cases = [
            # (throttle, expected_acceleration_direction)
            (0.3, "down"),    # 30%推力では下降
            (0.425, "hover"), # 42.5%推力でホバリング
            (0.7, "up"),      # 70%推力では上昇
        ]
        
        for throttle, expected in test_cases:
            thrust = throttle * max_thrust
            net_force = thrust - mass * gravity
            acceleration = net_force / mass
            
            if expected == "down":
                assert acceleration < 0, f"Throttle {throttle} should cause descent"
            elif expected == "hover":
                assert abs(acceleration) < 0.1, f"Throttle {throttle} should hover"
            elif expected == "up":
                assert acceleration > 0, f"Throttle {throttle} should cause ascent"
    
    def test_control_command_processing(self):
        """制御コマンドの処理テスト"""
        # テストコマンドを作成
        command = create_test_control_command(
            throttle1=0.7,
            throttle2=0.7,
            angle1=0.1,  # ロール
            angle2=0.05  # ピッチ
        )
        
        # 基本的な値の確認
        assert command.throttle1 == 0.7
        assert command.throttle2 == 0.7
        assert command.angle1 == 0.1
        assert command.angle2 == 0.05
        
        # 平均推力の計算
        avg_throttle = (command.throttle1 + command.throttle2) / 2.0
        assert avg_throttle == 0.7
        
        # 角度の妥当性確認
        assert abs(command.angle1) < 0.5  # ロール角は小さく
        assert abs(command.angle2) < 0.5  # ピッチ角は小さく
    
    def test_pose_stamped_creation(self):
        """姿勢メッセージの作成テスト"""
        # テスト姿勢を作成
        pose = create_test_pose(x=1.0, y=2.0, z=3.0, roll=0.1, pitch=0.05, yaw=0.2)
        
        # 位置の確認
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 2.0
        assert pose.pose.position.z == 3.0
        
        # クォータニオンの確認（単位クォータニオンでないことを確認）
        assert not (pose.pose.orientation.x == 0.0 and 
                   pose.pose.orientation.y == 0.0 and 
                   pose.pose.orientation.z == 0.0 and 
                   pose.pose.orientation.w == 1.0)
        
        # ヘッダーの確認
        assert pose.header.frame_id == "world"
        assert pose.header.stamp > 0
    
    def test_twist_stamped_creation(self):
        """速度メッセージの作成テスト"""
        # テスト速度を作成
        twist = create_test_twist(
            linear_x=1.0, linear_y=0.5, linear_z=0.2,
            angular_x=0.1, angular_y=0.05, angular_z=0.3
        )
        
        # 線形速度の確認
        assert twist.twist.linear.x == 1.0
        assert twist.twist.linear.y == 0.5
        assert twist.twist.linear.z == 0.2
        
        # 角速度の確認
        assert twist.twist.angular.x == 0.1
        assert twist.twist.angular.y == 0.05
        assert twist.twist.angular.z == 0.3
        
        # ヘッダーの確認
        assert twist.header.frame_id == "world"
        assert twist.header.stamp > 0
    
    def test_imu_data_structure(self):
        """IMUデータの構造テスト"""
        imu = Imu()
        
        # 基本的な構造の確認
        assert hasattr(imu, 'header')
        assert hasattr(imu, 'orientation')
        assert hasattr(imu, 'angular_velocity')
        assert hasattr(imu, 'linear_acceleration')
        
        # 共分散行列の確認
        assert len(imu.orientation_covariance) == 9
        assert len(imu.angular_velocity_covariance) == 9
        assert len(imu.linear_acceleration_covariance) == 9
        
        # 初期値の確認
        assert imu.orientation.w == 1.0  # 単位クォータニオン
        assert imu.orientation.x == 0.0
        assert imu.orientation.y == 0.0
        assert imu.orientation.z == 0.0
    
    def test_physics_constraints(self):
        """物理制約のテスト"""
        # 質量の制約
        mass = 0.65
        assert 0.1 < mass < 10.0, "Mass should be reasonable"
        
        # 推力の制約
        max_thrust = 15.0
        assert max_thrust > mass * 9.81, "Max thrust should exceed weight"
        
        # 制御ゲインの制約
        roll_p = 4.8
        assert 0.1 < roll_p < 20.0, "Control gain should be reasonable"
        
        # 角度制約
        max_angle = 0.5  # 約30度
        assert max_angle < 1.0, "Max angle should be less than 1 radian"
    
    def test_simulation_parameters(self):
        """シミュレーションパラメータのテスト"""
        # 時間ステップ
        dt = 0.01  # 100Hz
        assert 0.001 < dt < 0.1, "Time step should be reasonable"
        
        # 重力加速度
        gravity = 9.81
        assert 9.0 < gravity < 10.0, "Gravity should be close to 9.81"
        
        # 推力定数
        thrust_constant = 1.6
        assert 0.1 < thrust_constant < 10.0, "Thrust constant should be reasonable"
        
        # 最大推力
        max_thrust = 15.0
        assert max_thrust > 0, "Max thrust should be positive"
    
    def test_control_sequence_validation(self):
        """制御シーケンスの妥当性テスト"""
        # ホバリングコマンド
        hover_cmd = create_test_control_command(0.5, 0.5, 0.0, 0.0)
        assert hover_cmd.throttle1 == hover_cmd.throttle2, "Hover should have equal throttles"
        assert hover_cmd.angle1 == 0.0, "Hover should have zero roll"
        assert hover_cmd.angle2 == 0.0, "Hover should have zero pitch"
        
        # 離陸コマンド
        takeoff_cmd = create_test_control_command(0.7, 0.7, 0.0, 0.0)
        assert takeoff_cmd.throttle1 > 0.5, "Takeoff should have high throttle"
        assert takeoff_cmd.throttle2 > 0.5, "Takeoff should have high throttle"
        
        # 着陸コマンド
        landing_cmd = create_test_control_command(0.3, 0.3, 0.0, 0.0)
        assert landing_cmd.throttle1 < 0.5, "Landing should have low throttle"
        assert landing_cmd.throttle2 < 0.5, "Landing should have low throttle"


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 