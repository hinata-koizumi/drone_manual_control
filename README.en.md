# Drone Manual Control Environment

> **日本語**: [README.md](README.md)

This environment provides a ROS 2 Humble-based drone manual control system. You can control drones in real-time from a web browser and monitor drone status with 3D visualization.

## 🎯 Environment Purpose

**Reinforcement Learning Efficiency Enhancement - Drone Command Responsiveness Verification Environment**

This environment is designed for verifying drone command responsiveness in reinforcement learning algorithm development and validation (SAC, PPO, DQN, etc.). It allows manual verification of drone responsiveness, control accuracy, and physics simulation validity before implementing reinforcement learning, significantly improving learning efficiency and success rates.

### Verifiable Items
- **Command Responsiveness**: Immediate response of the airframe to control commands
- **Control Accuracy**: Accuracy in reaching target positions and velocities
- **Physical Properties**: Validity of gravity, thrust, and inertia
- **Stability**: Stability during long-duration flight
- **Safety**: Confirmation of operation within limits

## 🚀 Quick Start

### Fully Automated Deployment (Recommended)
```bash
# One-click system startup (browser opens automatically)
./scripts/quick_start.sh
```

### Step-by-Step Deployment
```bash
# 1. Build all packages
./scripts/build_all.sh

# 2. Start the system
./scripts/start_system.sh
```

### Complete Automation (with options)
```bash
# Complete automated deployment (build + startup + health check)
./scripts/auto_deploy.sh

# Deployment with options
./scripts/auto_deploy.sh --clean        # Start from clean state
./scripts/auto_deploy.sh --build-only   # Build only
./scripts/auto_deploy.sh --start-only   # Start only with existing build
./scripts/auto_deploy.sh --no-web-viz   # Start without web visualization
```

## 🎮 Verified Working Features

### Drone Control Commands
✅ **Take Off**: Drone ascends (Z-axis positive direction)  
✅ **Land**: Drone descends (Z-axis negative direction)  
✅ **Forward**: Drone moves forward (X-axis positive direction)  
✅ **Backward**: Drone moves backward (X-axis negative direction)  
✅ **Left**: Drone moves left (Y-axis positive direction)  
✅ **Right**: Drone moves right (Y-axis negative direction)  
✅ **Up**: Drone ascends (Z-axis positive direction, moderate)  
✅ **Down**: Drone descends (Z-axis negative direction, moderate)  
✅ **Stop**: Stop all movement  
✅ **Hover**: Maintain hovering state  
✅ **Reset**: Reset drone to initial position (0,0,0)  

### Real-time Display
✅ **Position Information**: Real-time X, Y, Z coordinate updates  
✅ **Velocity Information**: Real-time X, Y, Z axis velocity updates  
✅ **WebSocket Communication**: Stable real-time communication  
✅ **3D Visualization**: Browser-based drone position display  

### 3D Visualization Features
✅ **Real-time 3D Display**: Beautiful 3D visualization based on Three.js  
✅ **Camera Controls**: Mouse drag for rotation, wheel for zoom, right-click for pan  
✅ **Camera Follow Mode**: Toggleable drone tracking functionality  
✅ **Browser Zoom Prevention**: Intuitive operation with zoom only in 3D view  
✅ **Beautiful Drone 3D Model**: High-visibility white model with glossy effects  
✅ **Grid Display**: Easy-to-understand grid for position reference  
✅ **Operation Guide**: Intuitive operation instructions

## Dedicated Drone Specifications

### Aether-SL

A high-performance drone airframe specifically designed for the manual control environment.

#### **Basic Specifications**
- **Airframe Type**: Streamlined Quadcopter
- **Weight**: 0.65kg
- **Dimensions**: 32.5cm × 32.5cm × 7cm
- **Max Speed**: 23.6 m/s (85 km/h)
- **Max Altitude**: 50m
- **Flight Time**: ~25 minutes (simulation)
- **Max Range**: 20km

#### **Propulsion System**
- **Main Rotors**: 4 units (6-inch propellers, max 11000rpm)
- **Control Channels**: 4 channels (4 rotors)
- **Thrust Constant**: 1.6 (high-efficiency design)

#### **Sensor System**
- **IMU**: 9-axis (500Hz update rate, high precision)
- **GPS**: Dual-band GNSS (10Hz update rate)
- **Barometer**: Altitude measurement (60Hz update rate)
- **Magnetometer**: 3-axis (50Hz update rate)

#### **Safety Features**
- **Geofence**: 20km horizontal, 50m vertical limits
- **Failsafe**: Automatic return on RC signal loss
- **Speed Limits**: 23.6m/s horizontal, 8m/s vertical
- **Acceleration Limits**: Max 12m/s²

#### **Control Parameters**
- **Attitude Control**: PID control (Roll/Pitch: P=4.8, I=0.18, D=0.09)
- **Position Control**: PID control (XY: P=1.0, I=0.1, D=0.05)
- **Manual Control**: 80% tilt angle limit, 80% yaw angle limit

#### **Flight Modes**
- **STABILIZE**: Attitude stabilization mode
- **ALT_HOLD**: Altitude hold mode
- **LOITER**: Position hold mode
- **AUTO**: Autonomous flight mode
- **RTL**: Return to launch mode
- **MANUAL**: Manual control mode

See `config/drone_specs.yaml` for detailed specifications.

## System Architecture

### Container Structure
- **drone_msgs**: Custom message definitions
- **bridge**: ROS 2 communication bridge
- **manual_control**: Drone simulator and control system
- **web_viz**: Web visualization and control interface

### Technology Stack
- **ROS 2 Humble**: Base communication system
- **Docker Compose**: Multi-container environment
- **WebSocket**: Real-time Web UI communication
- **Python**: Simulation and control logic
- **JavaScript/Three.js**: Web UI frontend and 3D visualization

## Features

- **Real-time Control**: Instantly control drones from web browser
- **Physics Simulation**: Realistic behavior considering gravity, thrust, and control
- **ROS 2 Humble Support**: Uses the latest ROS 2 framework
- **Docker Integration**: Reproducible development environment
- **Modular Design**: Reuses existing bridge components
- **Web Visualization**: Browser-based 3D visualization and manual control
- **Complete Automation**: One-click system setup to startup
- **Multi-process Support**: Stable WebSocket communication and ROS 2 integration
- **Intuitive 3D Operation**: Free camera control with mouse operations
- **Camera Follow Function**: Automatic drone tracking camera mode
- **Complete Reset Function**: Reliable drone reset to initial position
- **Reinforcement Learning Support**: Optimized for SAC, PPO, DQN algorithm development and validation
- **Command Responsiveness Verification**: Pre-verification of airframe control accuracy and responsiveness
- **Learning Efficiency Enhancement**: Improved reinforcement learning success rates through manual verification

## Migrated Components

### Core Components
- `common/` - BridgeBase class and utilities
- `drone_msgs/` - Custom message definitions
- `px4_msgs/` - PX4 message definitions

### Simulation Environment
- `sim_launch/` - Gazebo Sim launch configurations
- `models/` - Drone models
- `custom_airframes/` - Airframe configurations

### Bridge Nodes
- `command_bridge/` - Control command conversion
- `state_bridge/` - State information conversion
- `angle_bridge/` - Angle control
- `outer_motor_bridge/` - External motor control

## New Components

### Manual Control System
- `manual_control/` - Drone simulator and control system
  - `simple_simulator.py`: Physics simulation
  - `optimized_simulator.py`: Optimized simulation
  - `action_executor.py`: Action execution
  - `state_monitor.py`: State monitoring

### Web Visualization System
- `web_viz/` - Browser-based 3D visualization and control interface
  - `server.py`: WebSocket server and ROS 2 integration
  - `index.html`: Web UI frontend

## Setup Instructions

### 1. Environment Initialization
```bash
# Copy components from existing environment
./scripts/setup_environment.sh
```

### 2. Environment Build
```bash
# Build Docker containers
docker-compose build
```

### 3. Demo Execution
```bash
# Run automatic demo
./scripts/run_demo.sh
```

### 4. Manual Execution (Optional)
```bash
# Start bridge nodes
docker-compose up -d bridge

# Start manual control nodes
docker-compose up -d manual_control
```

## Usage

### Basic Usage
1. **Environment Setup**
```bash
cd drone_manual_control
./scripts/setup_environment.sh
docker-compose build
```

2. **System Startup**
```bash
docker-compose up -d
```

3. **Web UI Access**
```bash
# Access in browser
http://localhost:8080
```

### Web Visualization Usage
1. **Access in browser after system startup**
```
http://localhost:8080
```

2. **Available Features**
- Real-time 3D drone visualization
- Manual control buttons (ascend, descend, forward, backward, left, right)
- Real-time drone status display (position, velocity)
- WebSocket connection status display
- Camera follow mode toggle
- Complete reset functionality

3. **Control Buttons**
- **Take Off**: Make drone ascend
- **Land**: Make drone descend
- **Forward/Backward**: Forward/backward movement
- **Left/Right**: Left/right movement
- **Up/Down**: Ascend/descend (moderate)
- **Stop**: Stop all movement
- **Hover**: Maintain hovering state
- **Reset**: Reset drone to initial position (0,0,0)

4. **3D Visualization Controls**
- **Mouse Drag**: Camera rotation
- **Mouse Wheel**: Zoom in/out
- **Right-click Drag**: Camera pan
- **Camera Follow Button**: ON/OFF toggle
  - **ON**: Follow drone (manual angle adjustment possible)
  - **OFF**: Fixed viewpoint (complete manual control)

5. **3D Visualization Features**
- Prevents browser-wide zoom, zoom only works in 3D view
- Beautiful white drone 3D model (with glossy effects)
- Grid display for easy position reference
- Intuitive operation guide display
- Manual angle adjustment possible even in camera follow mode

### Usage for Reinforcement Learning Developers
1. **Pre-verification of Airframe Responsiveness**
   - Confirm airframe responsiveness through manual control
   - Measure delay between control commands and actual movement
   - Verify validity of physical parameters

2. **Reward Function Design Support**
   - Evaluate difficulty of goal achievement through manual control
   - Measure execution time and accuracy of each action
   - Collect reference data for reward weighting

3. **Learning Environment Adjustment**
   - Set ranges for state space and action space
   - Optimize episode length and termination conditions
   - Design initial state distribution

### Log Monitoring
```bash
# Manual control node logs
docker-compose logs -f manual_control

# Web visualization logs
docker-compose logs -f web_viz

# All node logs
docker-compose logs -f
```

### Environment Shutdown
```bash
docker-compose down
```

## Technical Details

### Fixed Issues
1. **Thrust Calculation Fix**: Removed `abs()` function to properly handle negative throttle values for landing commands
2. **QoS Setting Unification**: Aligned communication settings between web_viz and simulator
3. **Data Sharing Improvement**: Optimized data transfer between multi-processes
4. **Horizontal Movement Support**: Properly handle `linear.x` and `linear.y` commands
5. **Real-time Updates**: UI position and velocity data updates in real-time
6. **Reset Function Improvement**: Complete reset functionality and cooldown period implementation
7. **3D Visualization Enhancement**: Browser zoom prevention, camera follow function, intuitive operation
8. **Camera Follow Mode Stabilization**: Scope issue resolution and combination with manual operation

### New Features and Improvements
1. **Complete Reset Function**: Reliable drone reset to initial position (0,0,0)
2. **Reset Cooldown**: 1-second control command ignore period after reset
3. **3D Visualization**: Beautiful 3D display based on Three.js
4. **Camera Follow Function**: Toggleable drone tracking
5. **Intuitive Operation**: Browser zoom prevention, improved mouse operation
6. **Beautiful UI**: White drone 3D model, glossy effects, grid display

### Reinforcement Learning Support Features
1. **Gym API Compatibility**: OpenAI Gym-style interface
2. **Dynamic Actuator Mapping**: Support for SAC and other algorithms
3. **State Space Standardization**: Normalization of position, velocity, and attitude
4. **Continuous Action Space**: Support for continuous thrust control
5. **Reward Function Verification**: Validation of reward design through manual control
6. **Episode Management**: Automatic reset and episode termination conditions

### Communication Flow
1. **Web UI** → **WebSocket** → **ROS 2 Control Node** → **TwistStamped** → **Simulator**
2. **Simulator** → **PoseStamped/TwistStamped** → **ROS 2 Control Node** → **WebSocket** → **Web UI**

### Physics Simulation
- **Gravity**: 9.81 m/s²
- **Thrust**: Variable thrust (based on control commands)
- **Mass**: 0.65 kg
- **Control**: PID control for attitude control
- **Collision Detection**: Ground collision handling

## Automation Scripts

### `scripts/auto_deploy.sh` - Complete Automated Deployment
- Complete automation from build to startup and health check
- Flexible operation with options

### `scripts/quick_start.sh` - Quick Start
- One-click system startup
- Browser opens automatically

### `scripts/build_all.sh` - Step-by-step Build
- Sequential build of each package
- Detailed log output

### `scripts/start_system.sh` - System Startup
- System startup using existing build

## Troubleshooting

### Common Issues
1. **WebSocket Connection Error**: Stop other processes if port 8080 is in use
2. **ROS 2 Communication Error**: Check network settings between containers
3. **UI Not Updating**: Clear browser cache
4. **3D View Not Displaying**: Check browser WebGL support
5. **Camera Follow Not Working**: Check error logs in browser console
6. **Reset Button Not Working**: Wait for cooldown period (1 second) after reset

### Debug Methods
```bash
# Real-time log monitoring
docker-compose logs -f

# Specific container log monitoring
docker-compose logs -f manual_control
docker-compose logs -f web_viz

# ROS 2 topic verification in container
docker-compose exec manual_control bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Browser developer tools debugging
# Press F12 and check console tab for logs
# Check camera follow mode toggle logs and error messages
```

## License

This project is released under the MIT License.