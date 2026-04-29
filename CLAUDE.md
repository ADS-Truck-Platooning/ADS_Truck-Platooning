# ADS Truck Platooning

Autonomous truck platooning system using CARLA simulator + ROS 2. Controls multiple semi-trailer trucks in coordinated formation with lane following, gap maintenance, and V2V communication.

## Architecture

```
platoon_ws/src/
├── v_platoon/           # CARLA-ROS2 bridge & truck spawning (Python)
├── lateral_control/     # Pure pursuit lane-following (C++)
├── longitudinal_control/# Speed & gap control (C++)
├── lane_detection/      # Camera-based lane detection (Python)
├── truck_detection/     # LiDAR-based truck detection (C++)
├── obstacle_detector/   # Obstacle tracking, ROS2 Humble port (C++)
├── platoon_launch/      # Unified launch configs
└── platoon_plot/        # Visualization utilities (Python)
```

## Build

```bash
cd ~/ADS_Truck-Platooning/platoon_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source ./install/setup.bash
```

## Run

1. Start CARLA simulator:
   ```bash
   cd ~/carla && make launch
   ```

2. Launch platoon:
   ```bash
   # 3 trucks on k-track
   ros2 launch carla-virtual-platoon carla-virtual-platoon.launch.py NumTrucks:=3 Map:=k-track

   # 3 trucks on Town04 with optional emergency truck
   ros2 launch carla-virtual-platoon carla-virtual-platoon.launch.py NumTrucks:=3 Map:=Town04_Opt SpawnEmergencyTruck:=1
   ```

## Key ROS 2 Topics

| Direction | Topic |
|-----------|-------|
| Published | `/truck{n}/front_camera`, `/truck{n}/front_lidar`, `/truck{n}/velocity` |
| Subscribed | `/truck{n}/steer_control`, `/truck{n}/throttle_control` |

## Docker

```bash
./run_docker.sh  # Mounts CARLA + project dirs, sets up GPU support
```

## Stack

- **ROS 2 Galactic** (Humble for obstacle_detector)
- **CARLA 0.9.15** + Unreal Engine 4.26
- **C++** (control + perception), **Python 3.8+** (bridge + utils)
- OpenCV, Armadillo C++

## System Requirements

- Ubuntu 20.04, 130 GB disk, 6+ GB GPU (8 GB recommended), 16+ GB RAM
- TCP ports 2000–2001 open

## License

CC BY-NC-SA 4.0
