#!/usr/bin/env python3

import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String
import threading
import time
from flask import Flask, send_from_directory
import os
import logging

logging.basicConfig(level=logging.INFO)

# Flaskアプリケーション
app = Flask(__name__)

@app.route('/')
def index():
    return send_from_directory('.', 'index.html')

@app.route('/<path:path>')
def serve_file(path):
    return send_from_directory('.', path)

class WebVizServer(Node):
    def __init__(self):
        super().__init__('web_viz_server')
        
        # ROS 2トピックの購読
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.position_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/drone/twist',
            self.velocity_callback,
            10
        )
        
        # コマンドパブリッシャー
        self.command_pub = self.create_publisher(
            String,
            '/drone/command',
            10
        )
        
        # 最新のデータ
        self.latest_position = None
        self.latest_velocity = None
        
        # WebSocketクライアント（グローバル変数として管理）
        self._clients = set()
        
        self.get_logger().info('Web visualization server started')
    
    @property
    def clients(self):
        return self._clients
    
    @clients.setter
    def clients(self, value):
        self._clients = value
    
    def position_callback(self, msg):
        self.latest_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        self.get_logger().info(f'Position: {self.latest_position}')
    
    def velocity_callback(self, msg):
        self.latest_velocity = {
            'x': msg.twist.linear.x,
            'y': msg.twist.linear.y,
            'z': msg.twist.linear.z
        }
        self.get_logger().info(f'Velocity: {self.latest_velocity}')
    
    def get_latest_data(self):
        """最新のデータを取得"""
        data = {}
        if self.latest_position:
            data['position'] = self.latest_position
        if self.latest_velocity:
            data['velocity'] = self.latest_velocity
        return data
    
    def handle_command(self, command):
        self.get_logger().info(f'Received command: {command}')
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

# グローバル変数としてWebVizServerインスタンスを保持
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
                    data = json.loads(message)
                    if 'command' in data:
                        web_viz_server.handle_command(data['command'])
                except json.JSONDecodeError:
                    logging.warning(f"Invalid JSON: {message}")
        except Exception as e:
            logging.info(f"WebSocket client disconnected: {e}")
        finally:
            web_viz_server.clients.remove(websocket)
            logging.info(f"Total clients: {len(web_viz_server.clients)}")

def run_ros_node():
    global web_viz_server
    rclpy.init()
    web_viz_server = WebVizServer()
    
    try:
        rclpy.spin(web_viz_server)
    except KeyboardInterrupt:
        pass
    finally:
        web_viz_server.destroy_node()
        rclpy.shutdown()

def run_flask():
    app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False)

async def broadcast_data():
    """定期的にデータをブロードキャスト"""
    global web_viz_server
    while True:
        if web_viz_server and web_viz_server.clients:
            data = web_viz_server.get_latest_data()
            if data:
                message = json.dumps(data)
                print(f"Broadcasting data to {len(web_viz_server.clients)} clients: {message}")
                await asyncio.gather(
                    *[client.send(message) for client in web_viz_server.clients],
                    return_exceptions=True
                )
        await asyncio.sleep(0.1)  # 100ms間隔

async def main():
    global web_viz_server
    
    print("Starting WebSocket and Flask servers...")
    
    # ROS 2ノードを別スレッドで実行
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()
    print("ROS 2 thread started")
    
    # Flaskサーバーを別スレッドで実行
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    print("Flask thread started")
    
    print("Starting WebSocket server on port 8081...")
    
    try:
        # WebSocketサーバーを起動
        print("Creating WebSocket server...")
        server = await websockets.serve(websocket_handler, "0.0.0.0", 8081)
        print("WebSocket server created successfully")
        print("WebSocket server started on ws://localhost:8081")
        print("Web server started on http://localhost:8080")
        
        # データブロードキャストタスクを開始
        print("Starting data broadcast task...")
        broadcast_task = asyncio.create_task(broadcast_data())
        print("Data broadcast task created")
        
        print("Waiting for tasks...")
        await asyncio.gather(broadcast_task)
    except Exception as e:
        print(f"Error starting WebSocket server: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main()) 