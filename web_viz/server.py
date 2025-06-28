#!/usr/bin/env python3

import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Float32
import threading
import time
from flask import Flask, send_from_directory
import os
import logging
import multiprocessing
import queue
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

logging.basicConfig(level=logging.INFO)

# Flaskアプリケーション
app = Flask(__name__)

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def serve_file(path):
    return send_from_directory('.', path)

class ROS2ControlNode(Node):
    """ROS 2制御ノード（独立プロセス）"""
    def __init__(self, command_queue):
        super().__init__('ros2_control_node')
        self.command_queue = command_queue
        
        # QoS設定（シミュレーターと一致）
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # ROS 2トピックの購読
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.position_callback,
            self.qos_profile
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/drone/twist',
            self.velocity_callback,
            self.qos_profile
        )
        
        self.thrust_sub = self.create_subscription(
            Float32,
            '/drone/thrust',
            self.thrust_callback,
            self.qos_profile
        )
        
        # 制御コマンドのパブリッシャー
        self.control_pub = self.create_publisher(
            TwistStamped,
            '/drone/control_command',
            self.qos_profile
        )
        
        # 位置と速度データ
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.thrust = 0.0  # 推力データを追加
        self.orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}  # 姿勢データを追加
        
        # コマンド処理タイマー
        self.create_timer(0.1, self.process_commands)
        
        self.get_logger().info('ROS 2 Control Node started')
        self.get_logger().info('Subscribing to /drone/pose and /drone/twist')
        self.get_logger().info('Publishing to /drone/control_command')
    
    def position_callback(self, msg):
        self.position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        
        # クォータニオンからオイラー角に変換
        self.orientation = self.quaternion_to_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        
        self.get_logger().info(f'Received position: x={self.position["x"]:.2f}, y={self.position["y"]:.2f}, z={self.position["z"]:.2f}')
        self.get_logger().info(f'Received orientation: roll={self.orientation["roll"]:.2f}, pitch={self.orientation["pitch"]:.2f}, yaw={self.orientation["yaw"]:.2f}')
    
    def velocity_callback(self, msg):
        self.velocity = {
            'x': msg.twist.linear.x,
            'y': msg.twist.linear.y,
            'z': msg.twist.linear.z
        }
        self.get_logger().info(f'Received velocity: x={self.velocity["x"]:.2f}, y={self.velocity["y"]:.2f}, z={self.velocity["z"]:.2f}')
    
    def thrust_callback(self, msg):
        self.thrust = msg.data
        self.get_logger().info(f'Received thrust: {self.thrust}')
    
    def process_commands(self):
        """コマンドキューからコマンドを処理"""
        try:
            while not self.command_queue.empty():
                command = self.command_queue.get_nowait()
                self.handle_command(command)
        except queue.Empty:
            pass
    
    def handle_command(self, command):
        """WebコマンドをTwistStampedメッセージに変換して送信"""
        try:
            self.get_logger().info(f'Received command: {command}')
            
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            
            # コマンドに基づいて制御値を設定
            if command == "land":
                # 着陸: 下向きの推力
                msg.twist.linear.z = -0.3  # 30%の下向き推力
                self.get_logger().info('Setting land command: linear.z = -0.3')
            elif command == "hover":
                # ホバリング: シミュレーター側のPID制御に任せる
                msg.twist.linear.z = 0.0  # 推力0（PID制御が自動調整）
                self.get_logger().info('Setting hover command: linear.z = 0.0 (PID control)')
            elif command == "stop":
                # 停止: すべての動きを停止
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0
                self.get_logger().info('Setting stop command: all velocities = 0')
            elif command == "down":
                msg.twist.linear.z = -0.2
                self.get_logger().info('Setting down command: linear.z = -0.2')
            elif command == "forward":
                msg.twist.linear.x = 0.8
                self.get_logger().info('Setting forward command: linear.x = 0.8')
            elif command == "backward":
                msg.twist.linear.x = -0.8
                self.get_logger().info('Setting backward command: linear.x = -0.8')
            elif command == "left":
                msg.twist.linear.y = 0.8
                self.get_logger().info('Setting left command: linear.y = 0.8')
            elif command == "right":
                msg.twist.linear.y = -0.8
                self.get_logger().info('Setting right command: linear.y = -0.8')
            elif command == "reset":
                # リセット: 特別な値でリセットコマンドを送信
                msg.twist.linear.x = 999.0  # 特別な値でリセットを識別
                msg.twist.linear.y = 999.0
                msg.twist.linear.z = 999.0
                msg.twist.angular.x = 999.0
                msg.twist.angular.y = 999.0
                msg.twist.angular.z = 999.0
                self.get_logger().info('Setting reset command: special values for reset detection')
            elif command == "up":
                msg.twist.linear.z = 0.7
                self.get_logger().info('Setting up command: linear.z = 0.7')
            elif command == "turn_left":
                msg.twist.angular.z = 0.5
                self.get_logger().info('Setting turn left command: angular.z = 0.5')
            elif command == "turn_right":
                msg.twist.angular.z = -0.5
                self.get_logger().info('Setting turn right command: angular.z = -0.5')
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                return
            
            # メッセージを送信
            self.control_pub.publish(msg)
            self.get_logger().info(f'Successfully sent control command: {command}')
            
        except Exception as e:
            self.get_logger().error(f'Error in handle_command: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def get_drone_data(self):
        """ドローンの位置と速度データを取得"""
        return {
            'position': self.position,
            'velocity': self.velocity,
            'thrust': self.thrust,
            'orientation': self.orientation
        }
    
    def quaternion_to_euler(self, x, y, z, w):
        """クォータニオンをオイラー角（ロール、ピッチ、ヨー）に変換"""
        import math
        
        # クォータニオンからオイラー角への変換
        # ロール (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # ピッチ (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # 90度の場合
        else:
            pitch = math.asin(sinp)
        
        # ヨー (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return {
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        }

def ros2_node_process(command_queue, data_queue):
    """ROS 2ノードを独立プロセスで実行"""
    rclpy.init()
    node = ROS2ControlNode(command_queue)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)  # 10ms間隔に短縮
            # データをキューに送信（より頻繁に）
            try:
                drone_data = node.get_drone_data()
                data_queue.put_nowait(drone_data)
            except queue.Full:
                # キューが満杯の場合は古いデータを削除
                try:
                    data_queue.get_nowait()
                    data_queue.put_nowait(drone_data)
                except queue.Empty:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

class WebVizServer:
    """Web可視化サーバー（WebSocket + Flask）"""
    def __init__(self):
        self.clients = set()
        self.command_queue = multiprocessing.Queue()
        self.data_queue = multiprocessing.Queue()
        self.latest_data = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}, 'thrust': 0.0, 'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}}
        
        # ROS 2ノードを独立プロセスで開始
        self.ros2_process = multiprocessing.Process(
            target=ros2_node_process,
            args=(self.command_queue, self.data_queue)
        )
        self.ros2_process.start()
        
        logging.info("WebVizServer initialized")
    
    def handle_command(self, command):
        """コマンドをROS 2ノードに送信"""
        try:
            self.command_queue.put(command)
            logging.info(f"Command sent to ROS 2 node: {command}")
        except Exception as e:
            logging.error(f"Error sending command: {e}")
    
    def get_drone_data(self):
        """ドローン位置データを取得（最新データを優先）"""
        try:
            # キューから最新データを取得
            while not self.data_queue.empty():
                self.latest_data = self.data_queue.get_nowait()
            return self.latest_data
        except queue.Empty:
            return self.latest_data

# グローバル変数
web_viz_server = None

async def websocket_handler(websocket, *args, **kwargs):
    logging.info("websocket_handler called")
    global web_viz_server
    if web_viz_server:
        logging.info(f"WebSocket client connected from {getattr(websocket, 'remote_address', 'unknown')}")
        web_viz_server.clients.add(websocket)
        logging.info(f"Total clients: {len(web_viz_server.clients)}")
        try:
            async for message in websocket:
                try:
                    logging.info(f"Received message: {message}")
                    data = json.loads(message)
                    if 'command' in data:
                        logging.info(f"Processing command: {data['command']}")
                        web_viz_server.handle_command(data['command'])
                        # コマンド送信後に確認メッセージを送信
                        await websocket.send(json.dumps({"status": "command_sent", "command": data['command']}))
                except json.JSONDecodeError as e:
                    logging.error(f"JSON decode error: {e}")
                except Exception as e:
                    logging.error(f"Error processing message: {e}")
                    import traceback
                    logging.error(traceback.format_exc())
        except websockets.exceptions.ConnectionClosed:
            logging.info("WebSocket connection closed")
        except Exception as e:
            logging.error(f"WebSocket error: {e}")
            import traceback
            logging.error(traceback.format_exc())
        finally:
            web_viz_server.clients.discard(websocket)
            logging.info(f"Client disconnected. Total clients: {len(web_viz_server.clients)}")

async def broadcast_data():
    """ドローン位置データをクライアントにブロードキャスト"""
    global web_viz_server
    while True:
        if web_viz_server and web_viz_server.clients:
            try:
                data = web_viz_server.get_drone_data()
                message = json.dumps(data)
                logging.info(f"Broadcasting data to {len(web_viz_server.clients)} clients: {data}")
                
                # 切断されたクライアントを削除
                disconnected_clients = set()
                for client in web_viz_server.clients:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected_clients.add(client)
                    except Exception as e:
                        logging.error(f"Error sending to client: {e}")
                        disconnected_clients.add(client)
                
                web_viz_server.clients -= disconnected_clients
                
            except Exception as e:
                logging.error(f"Error in broadcast_data: {e}")
        
        await asyncio.sleep(0.1)  # 100ms間隔で更新

async def main():
    global web_viz_server
    
    # Web可視化サーバーを初期化
    web_viz_server = WebVizServer()
    
    # WebSocketサーバーを開始
    websocket_server = await websockets.serve(
        websocket_handler,
        "0.0.0.0",
        8765
    )
    
    logging.info("WebSocket server started on ws://0.0.0.0:8765")
    
    # データブロードキャストタスクを開始
    broadcast_task = asyncio.create_task(broadcast_data())
    
    try:
        await websocket_server.wait_closed()
    except KeyboardInterrupt:
        logging.info("Shutting down...")
    finally:
        broadcast_task.cancel()
        if web_viz_server.ros2_process.is_alive():
            web_viz_server.ros2_process.terminate()
            web_viz_server.ros2_process.join()

if __name__ == "__main__":
    # Flaskサーバーを別スレッドで開始
    def run_flask():
        app.run(host='0.0.0.0', port=8080, debug=False)
    
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    # WebSocketサーバーを開始
    asyncio.run(main()) 