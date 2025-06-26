#!/usr/bin/env python3

import pytest
import yaml
import os
import sys
from pathlib import Path

# テスト用のモッククラス
class MockDroneControlCommand:
    def __init__(self):
        self.throttle1 = 0.0
        self.throttle2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0

class MockPoseStamped:
    def __init__(self):
        self.pose = MockPose()
    
class MockPose:
    def __init__(self):
        self.position = MockPoint()
        self.orientation = MockQuaternion()

class MockPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class MockQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0

class TestSimpleSimulation:
    """簡単なシミュレーションのテスト"""
    
    def test_drone_specs_loading(self):
        """drone_specs.yamlの読み込みテスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'drone_specs.yaml'
        
        assert config_path.exists(), f"drone_specs.yaml not found: {config_path}"
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert 'drone_specifications' in data
        specs = data['drone_specifications']
        
        # 物理パラメータの確認
        assert 'physical' in specs
        assert 'weight' in specs['physical']
        assert specs['physical']['weight'] > 0
        
        # モーターパラメータの確認
        assert 'motors' in specs
        assert 'main_rotors' in specs['motors']
        assert 'thrust_constant' in specs['motors']['main_rotors']
        assert specs['motors']['main_rotors']['thrust_constant'] > 0
        
        # 制御パラメータの確認
        assert 'control' in specs
        assert 'attitude' in specs['control']
        assert 'roll_p' in specs['control']['attitude']
        assert specs['control']['attitude']['roll_p'] > 0
    
    def test_action_sequences_loading(self):
        """action_sequences.yamlの読み込みテスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'action_sequences.yaml'
        
        assert config_path.exists(), f"action_sequences.yaml not found: {config_path}"
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert 'action_sequences' in data
        assert isinstance(data['action_sequences'], list)
        assert len(data['action_sequences']) > 0
        
        # 各アクションの確認
        for action in data['action_sequences']:
            assert 'name' in action
            assert 'action_type' in action
            assert 'duration' in action
            assert 'parameters' in action
            
            # アクションタイプの妥当性確認
            valid_types = ['hover', 'takeoff', 'landing', 'waypoint', 'circle', 'square', 'manual']
            assert action['action_type'] in valid_types
    
    def test_simple_physics_calculation(self):
        """簡単な物理計算のテスト"""
        # 重力加速度
        gravity = 9.81
        
        # ドローン質量
        mass = 0.65
        
        # 推力計算
        thrust_constant = 1.6
        throttle = 0.5  # 50%推力
        max_thrust = 15.0
        thrust = throttle * max_thrust
        
        # 上向き加速度計算
        upward_acceleration = (thrust - mass * gravity) / mass
        
        # 基本的な物理法則の確認
        assert upward_acceleration > 0  # 50%推力では上昇できる（7.5N > 6.38N）
        assert thrust > 0  # 推力は正の値
        assert mass > 0  # 質量は正の値
        
        # ホバリングに必要な推力の確認
        hover_thrust = mass * gravity  # 6.38N
        hover_throttle = hover_thrust / max_thrust  # 約0.425
        
        assert hover_throttle < 0.5  # 50%推力はホバリングに十分
        assert hover_throttle > 0.4  # 40%以上は必要
    
    def test_control_command_structure(self):
        """制御コマンドの構造テスト"""
        command = MockDroneControlCommand()
        
        # 基本的な属性の確認
        assert hasattr(command, 'throttle1')
        assert hasattr(command, 'throttle2')
        assert hasattr(command, 'angle1')
        assert hasattr(command, 'angle2')
        
        # 値の設定テスト
        command.throttle1 = 0.7
        command.throttle2 = 0.7
        command.angle1 = 0.1
        command.angle2 = 0.0
        
        assert command.throttle1 == 0.7
        assert command.throttle2 == 0.7
        assert command.angle1 == 0.1
        assert command.angle2 == 0.0
    
    def test_pose_structure(self):
        """姿勢情報の構造テスト"""
        pose = MockPoseStamped()
        
        # 基本的な属性の確認
        assert hasattr(pose, 'pose')
        assert hasattr(pose.pose, 'position')
        assert hasattr(pose.pose, 'orientation')
        assert hasattr(pose.pose.position, 'x')
        assert hasattr(pose.pose.position, 'y')
        assert hasattr(pose.pose.position, 'z')
        
        # 値の設定テスト
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 2.0
        assert pose.pose.position.z == 3.0
    
    def test_quaternion_to_euler_conversion(self):
        """クォータニオンからオイラー角への変換テスト"""
        import math
        
        # 簡単なクォータニオン（単位クォータニオン）
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
        
        # オイラー角計算
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 単位クォータニオンの場合、オイラー角は全て0
        assert abs(roll) < 1e-6
        assert abs(pitch) < 1e-6
        assert abs(yaw) < 1e-6


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 