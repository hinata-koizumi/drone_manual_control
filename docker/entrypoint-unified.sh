#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash

# PX4環境変数
export PX4_HOME_LAT=35.6586
export PX4_HOME_LON=139.7454
export PX4_HOME_ALT=10.0
export PX4_SIM_MODEL=4001_custom_drone

# Gazebo Gardenモデルパスを追加
export GZ_SIM_RESOURCE_PATH=/root/.gz/models:/usr/share/gz/garden/models:$GZ_SIM_RESOURCE_PATH

# PYTHONPATH追加（drone_msgs, px4_msgs両対応）
export PYTHONPATH=/sim_ws/install/lib/python3.10/site-packages:/sim_ws/install/drone_msgs/local/lib/python3.10/dist-packages:$PYTHONPATH

# PX4 SITL起動（バックグラウンド）
echo "[entrypoint] PX4 airframes in build dir:" && ls -l /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
if [ -x /usr/local/bin/px4 ]; then
  # px4-alias.shがなければtouchして空ファイルを作る
  if [ ! -f /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh ]; then
    touch /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh
    chmod +x /PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/px4-alias.sh
  fi
  cd /PX4-Autopilot/build/px4_sitl_default
  /usr/local/bin/px4 etc -s etc/init.d-posix/rcS &
  PX4_PID=$!
fi

# ROS 2ワークスペースのsource
if [ -f /sim_ws/install/setup.bash ]; then
  source /sim_ws/install/setup.bash
fi

# Gazebo Garden起動（必要なら）
# gz sim ... &

# Gazeboモデルパスとモデル存在確認のデバッグ出力

echo "[entrypoint] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
ls -l /root/.gz/models/ground_plane || echo "ground_plane not found in /root/.gz/models"
ls -l /usr/share/gz/garden/models/ground_plane || echo "ground_plane not found in /usr/share/gz/garden/models"
ls -l /root/.gz/models/sun || echo "sun not found in /root/.gz/models"
ls -l /usr/share/gz/garden/models/sun || echo "sun not found in /usr/share/gz/garden/models"

# シミュレーションlaunch
ros2 launch sim_launch sim_all.launch.py headless:=true

# PX4プロセスの監視
if [ -n "$PX4_PID" ]; then
  wait $PX4_PID
fi 