#!/usr/bin/env python3

import os
import sys
import yaml
import pytest
import subprocess
from pathlib import Path
from typing import Dict, Any

# テスト用の設定
TEST_TIMEOUT = 30  # 秒
NODE_STARTUP_TIMEOUT = 10  # 秒

class TestManualControlIntegration:
    """統合テストクラス"""
    
    @classmethod
    def setup_class(cls):
        """テストクラス初期化時にROS 2パッケージをビルド"""
        cls.package_dir = Path(__file__).parent.parent / 'src' / 'manual_control'
        cls.workspace_dir = Path(__file__).parent.parent
        
        # ROS 2環境が利用可能かチェック
        if os.path.exists('/opt/ros/humble/setup.sh'):
            print("ROS 2 Humble detected, building package...")
            try:
                # ROS 2パッケージをビルド
                result = subprocess.run([
                    'bash', '-c', 
                    f'source /opt/ros/humble/setup.sh && cd {cls.package_dir} && colcon build --packages-select manual_control'
                ], capture_output=True, text=True, timeout=300)
                
                if result.returncode == 0:
                    print("✅ Package built successfully")
                    # ビルドされたパッケージのパスを追加
                    install_dir = cls.package_dir / 'install' / 'manual_control' / 'lib' / 'python3.10' / 'site-packages'
                    if install_dir.exists():
                        sys.path.insert(0, str(install_dir))
                        print(f"Added {install_dir} to Python path")
                    else:
                        # Python 3.11の場合も試す
                        install_dir = cls.package_dir / 'install' / 'manual_control' / 'lib' / 'python3.11' / 'site-packages'
                        if install_dir.exists():
                            sys.path.insert(0, str(install_dir))
                            print(f"Added {install_dir} to Python path")
                        else:
                            print("⚠️ Install directory not found, using direct import")
                            # 直接パスを追加
                            package_path = Path(__file__).parent.parent / 'src'
                            sys.path.insert(0, str(package_path))
                else:
                    print(f"⚠️ Package build failed: {result.stderr}")
                    # ビルドが失敗した場合も直接パスを追加
                    package_path = Path(__file__).parent.parent / 'src'
                    sys.path.insert(0, str(package_path))
            except subprocess.TimeoutExpired:
                print("⚠️ Package build timed out")
                package_path = Path(__file__).parent.parent / 'src'
                sys.path.insert(0, str(package_path))
            except Exception as e:
                print(f"⚠️ Package build error: {e}")
                package_path = Path(__file__).parent.parent / 'src'
                sys.path.insert(0, str(package_path))
        else:
            print("⚠️ ROS 2 Humble not found, using direct import")
            # ROS 2がない場合は直接パスを追加
            package_path = Path(__file__).parent.parent / 'src'
            sys.path.insert(0, str(package_path))
    
    def test_config_files_exist(self):
        """設定ファイルの存在確認"""
        config_dir = Path(__file__).parent.parent / 'config'
        
        # 必須設定ファイルの確認
        required_files = [
            'action_sequences.yaml',
            'drone_specs.yaml'
        ]
        
        for file_name in required_files:
            file_path = config_dir / file_name
            assert file_path.exists(), f"Required config file not found: {file_path}"
    
    def test_action_sequences_yaml_validity(self):
        """action_sequences.yamlの妥当性テスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'action_sequences.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert isinstance(data, dict)
        assert 'action_sequences' in data
        assert isinstance(data['action_sequences'], list)
        
        # 各アクションシーケンスの確認
        for seq in data['action_sequences']:
            assert isinstance(seq, dict)
            assert 'name' in seq
            assert 'action_type' in seq
            assert 'duration' in seq
            assert isinstance(seq['name'], str)
            assert isinstance(seq['action_type'], str)
            assert isinstance(seq['duration'], (int, float))
            assert seq['duration'] > 0
    
    def test_drone_specs_yaml_validity(self):
        """drone_specs.yamlの妥当性テスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'drone_specs.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        # 基本構造の確認
        assert isinstance(data, dict)
        
        # ドローン仕様の確認
        if 'drone_specifications' in data:
            specs = data['drone_specifications']
            assert isinstance(specs, dict)
            
            # 必須フィールドの確認
            if 'physical' in specs:
                physical = specs['physical']
                if 'weight' in physical:
                    assert isinstance(physical['weight'], (int, float))
                    assert physical['weight'] > 0
            if 'motors' in specs:
                motors = specs['motors']
                if 'main_rotors' in motors:
                    rotors = motors['main_rotors']
                    if 'thrust_constant' in rotors:
                        assert isinstance(rotors['thrust_constant'], (int, float))
                        assert rotors['thrust_constant'] > 0
    
    def test_package_structure(self):
        """パッケージ構造の確認"""
        package_dir = Path(__file__).parent.parent / 'src' / 'manual_control'
        # 必須ファイルの確認
        required_files = [
            'package.xml',
            'setup.py',
            'manual_control/__init__.py',
            'manual_control/action_executor.py',
            'resource/manual_control'
        ]
        for file_name in required_files:
            file_path = package_dir / file_name
            assert file_path.exists(), f"Required package file not found: {file_path}"
    
    def test_package_xml_validity(self):
        """package.xmlの妥当性テスト"""
        package_xml_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'package.xml'
        with open(package_xml_path, 'r') as f:
            content = f.read()
        # 基本的なXML構造の確認
        assert '<?xml' in content
        assert '<package' in content
        assert '<name>manual_control</name>' in content
        assert '<version>0.1.0</version>' in content
        assert '<description>' in content
        assert '<maintainer' in content  # 部分一致に緩和
        assert '<license>MIT</license>' in content
        # 依存関係の確認
        required_deps = [
            'rclpy',
            'std_msgs',
            'geometry_msgs',
            'sensor_msgs'
        ]
        for dep in required_deps:
            assert f'<depend>{dep}</depend>' in content
    
    def test_setup_py_validity(self):
        """setup.pyの妥当性テスト"""
        setup_py_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'setup.py'
        with open(setup_py_path, 'r') as f:
            content = f.read()
        # 基本的なPythonコードの確認
        assert 'from setuptools import setup' in content
        assert ("name='manual_control'" in content or 'name=package_name' in content)
        assert "version='0.1.0'" in content
        assert 'action_executor = manual_control.action_executor:main' in content
    
    def test_action_executor_import(self):
        """action_executorモジュールのインポートテスト"""
        try:
            # モジュールのインポートテスト
            from manual_control.action_executor import ActionExecutorNode, ActionType, ActionSequence
            
            # クラスの存在確認
            assert ActionExecutorNode is not None
            assert ActionType is not None
            assert ActionSequence is not None
            
            # ActionTypeの値確認
            assert ActionType.HOVER.value == "hover"
            assert ActionType.TAKEOFF.value == "takeoff"
            assert ActionType.LANDING.value == "landing"
            assert ActionType.WAYPOINT.value == "waypoint"
            assert ActionType.CIRCLE.value == "circle"
            assert ActionType.SQUARE.value == "square"
            assert ActionType.MANUAL.value == "manual"
            
        except ImportError as e:
            # インポートエラーが発生した場合は詳細をログに出力
            print(f"Import error details: {e}")
            print(f"Python path: {sys.path}")
            print(f"Current working directory: {os.getcwd()}")
            
            # ファイルの存在確認
            action_executor_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'manual_control' / 'action_executor.py'
            print(f"Action executor file exists: {action_executor_path.exists()}")
            
            # ROS 2環境の確認
            ros_setup = '/opt/ros/humble/setup.sh'
            print(f"ROS 2 setup exists: {os.path.exists(ros_setup)}")
            
            # ビルドディレクトリの確認
            build_dir = Path(__file__).parent.parent / 'src' / 'manual_control' / 'build'
            install_dir = Path(__file__).parent.parent / 'src' / 'manual_control' / 'install'
            print(f"Build directory exists: {build_dir.exists()}")
            print(f"Install directory exists: {install_dir.exists()}")
            
            pytest.fail(f"Failed to import action_executor module: {e}")
    
    def test_action_sequence_creation(self):
        """ActionSequenceの作成テスト"""
        try:
            from manual_control.action_executor import ActionSequence, ActionType
            
            # 基本的なActionSequenceの作成
            seq = ActionSequence(
                name="test_hover",
                action_type=ActionType.HOVER,
                duration=10.0,
                parameters={"target_altitude": 2.0},
                next_action="landing"
            )
            
            assert seq.name == "test_hover"
            assert seq.action_type == ActionType.HOVER
            assert seq.duration == 10.0
            assert seq.parameters["target_altitude"] == 2.0
            assert seq.next_action == "landing"
        except ImportError as e:
            pytest.skip(f"Module not available: {e}")
    
    def test_config_file_loading_simulation(self):
        """設定ファイル読み込みのシミュレーションテスト"""
        config_path = Path(__file__).parent.parent / 'config' / 'action_sequences.yaml'
        
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        
        try:
            from manual_control.action_executor import ActionSequence, ActionType
            
            # 設定ファイルからActionSequenceを作成
            sequences = {}
            for seq_data in data.get('action_sequences', []):
                seq = ActionSequence(
                    name=seq_data['name'],
                    action_type=ActionType(seq_data['action_type']),
                    duration=seq_data['duration'],
                    parameters=seq_data.get('parameters', {}),
                    next_action=seq_data.get('next_action')
                )
                sequences[seq.name] = seq
            
            # 作成されたシーケンスの確認
            assert len(sequences) > 0
            
            # 各シーケンスの妥当性確認
            for name, seq in sequences.items():
                assert isinstance(seq.name, str)
                assert isinstance(seq.action_type, ActionType)
                assert isinstance(seq.duration, (int, float))
                assert seq.duration > 0
                assert isinstance(seq.parameters, dict)
        except ImportError as e:
            pytest.skip(f"Module not available: {e}")


class TestManualControlBuild:
    """ビルド関連のテスト"""
    
    def test_colcon_build_simulation(self):
        """colcon buildのシミュレーションテスト"""
        # パッケージディレクトリの確認
        package_dir = Path(__file__).parent.parent / 'src' / 'manual_control'
        assert package_dir.exists()
        
        # 必須ファイルの確認
        required_files = [
            'package.xml',
            'setup.py',
            'manual_control/action_executor.py'
        ]
        
        for file_name in required_files:
            file_path = package_dir / file_name
            assert file_path.exists(), f"Build requires file not found: {file_path}"
    
    def test_entry_point_generation(self):
        """エントリーポイント生成のテスト"""
        setup_py_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'setup.py'
        
        with open(setup_py_path, 'r') as f:
            content = f.read()
        
        # エントリーポイントの確認
        assert 'action_executor = manual_control.action_executor:main' in content
        assert 'simple_simulator = manual_control.simple_simulator:main' in content
        assert 'state_monitor = manual_control.state_monitor:main' in content
        
        # 各ノードファイルのmain関数とif __name__の存在確認
        node_files = ['action_executor.py', 'simple_simulator.py', 'state_monitor.py']
        
        for node_file in node_files:
            node_path = Path(__file__).parent.parent / 'src' / 'manual_control' / 'manual_control' / node_file
            
            with open(node_path, 'r') as f:
                content = f.read()
            
            assert 'def main()' in content, f"main function not found in {node_file}"
            # 両方のクォート形式に対応
            assert ('if __name__ == "__main__":' in content or "if __name__ == '__main__':" in content), f"if __name__ == '__main__' not found in {node_file}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"]) 