# KISS-SLAM to HDMapping Converter

This toolset integrates the [KISS-SLAM](https://github.com/PRBonn/kiss-slam) algorithm with [HDMapping](https://github.com/MapsHD/HDMapping).

## Overview

This repository contains a ROS2 workspace that:
- Submodule to tested revision of KISS-SLAM
- Livox CustomMsg to PointCloud2 converter
- KISS-SLAM ROS2 wrapper node
- A converter that processes KISS-SLAM output and saves data in HDMapping format

## Dependencies

### System Dependencies

```bash
sudo apt install -y nlohmann-json3-dev libeigen3-dev libpcl-dev
```

### Python Dependencies

```bash
pip3 install kiss-slam rosbags numpy open3d scipy
```

## Building

### Clone the repository

```bash
mkdir -p /ws/src
cd /ws/src
git clone --recursive https://github.com/kpmrozowski/KISS-SLAM-to-HDMapping.git
cd ..
```

### Build with colcon

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Using Docker

```bash
cd /ws/src/KISS-SLAM-to-HDMapping
docker build -t kiss-slam-hdmapping .
docker run -it --rm \
    -v /path/to/your/data:/data \
    kiss-slam-hdmapping \
    /scripts/run_pipeline.sh /data/input_bag /data/output
```

## Usage

### Method 1: Using the Pipeline-Docker Script (Recommended)

The easiest way to run the full pipeline:

```bash
./scripts/run_kiss_slam_docker.sh /path/to/livox_ros2bag /path/to/output /livox/lidar
```

### Method 2: Using the Pipeline-Native Script

The native way (no docker) to run the full pipeline:

```bash
./scripts/run_kiss_slam_native.sh /path/to/livox_ros2bag /path/to/output /livox/lidar
```

This script:
1. Launches the Livox to PointCloud2 converter
2. Runs KISS-SLAM processing
3. Records SLAM output
4. Converts to HDMapping format

### Method 3: Manual Step-by-Step

#### Step 1: Launch KISS-SLAM nodes

In terminal 1:
```bash
source install/setup.bash
ros2 launch kiss_slam_ros slam.launch.py
```

#### Step 2: Record SLAM output

In terminal 2:
```bash
ros2 bag record /kiss_slam/local_map /kiss_slam/odometry -o /tmp/slam_output
```

#### Step 3: Play the input bag

In terminal 3:
```bash
ros2 bag play /path/to/your/livox_bag
```

#### Step 4: Convert to HDMapping format

After the bag finishes playing:
```bash
ros2 run kiss-slam-to-hdmapping listener /tmp/slam_output /path/to/output
```

## Output Format

The converter produces HDMapping-compatible output:

```
output/
├── session.json           # HDMapping session file
├── scan_lio_0.laz        # Point cloud chunk 0 (LAZ compressed)
├── scan_lio_1.laz        # Point cloud chunk 1 (if > 2M points)
├── trajectory_lio_0.csv  # Trajectory for chunk 0
├── trajectory_lio_1.csv  # Trajectory for chunk 1
├── poses.reg             # Pose registration file
└── lio_initial_poses.reg # Initial poses file
```

### session.json Structure

```json
{
  "Session Settings": {
    "offset_x": 0.0,
    "offset_y": 0.0,
    "offset_z": 0.0,
    "folder_name": "/path/to/output",
    "poses_file_name": "/path/to/output/poses.reg",
    "initial_poses_file_name": "/path/to/output/lio_initial_poses.reg",
    "lidar_odometry_version": "KISS-SLAM"
  },
  "laz_file_names": [
    {"file_name": "/path/to/output/scan_lio_0.laz"},
    {"file_name": "/path/to/output/scan_lio_1.laz"}
  ]
}
```

## Configuration

KISS-SLAM configuration is in `config/kiss_slam.yaml`:

```yaml
odometry:
  preprocessing:
    max_range: 50.0  # Maximum LiDAR range (meters)

local_mapper:
  voxel_size: 0.5          # Voxel size for local map
  splitting_distance: 20.0  # Distance to create new local map node

loop_closer:
  overlap_threshold: 0.4   # Loop closure overlap threshold
```

## Topics

### Input (from Livox bag)
- `/livox/lidar` - `livox_ros_driver2/msg/CustomMsg`
- `/livox/imu` - `sensor_msgs/msg/Imu` (optional)

### Intermediate
- `/livox/points` - `sensor_msgs/msg/PointCloud2` (converted from Livox)

### Output (KISS-SLAM)
- `/kiss_slam/odometry` - `nav_msgs/msg/Odometry`
- `/kiss_slam/local_map` - `sensor_msgs/msg/PointCloud2`

## Example

Using the provided dataset:

```bash
# Build the workspace
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Run the pipeline
./src/KISS-SLAM-to-HDMapping/scripts/run_pipeline.sh \
    /home/kmro/dev/slams/datasets/charles/warm-up_large-room_bag \
    /tmp/hdmapping_output

# View the output
ls -la /tmp/hdmapping_output/
```

## License

MIT License

## References

- [KISS-SLAM](https://github.com/PRBonn/kiss-slam) - Simple, Robust, Accurate 3D LiDAR SLAM
- [KISS-ICP](https://github.com/PRBonn/kiss-icp) - Odometry component
- [HDMapping](https://github.com/MapsHD/HDMapping) - HD Mapping toolkit
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) - Livox ROS2 driver
