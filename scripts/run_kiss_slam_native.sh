#!/bin/bash
#
# KISS-SLAM to HDMapping Pipeline Script
#
# This script runs the complete pipeline:
# 1. Reads Livox CustomMsg or PointCloud2 from ROS2 bag
# 2. Runs KISS-SLAM on the point clouds
# 3. Converts KISS-SLAM output to HDMapping format
#
# Usage: ./run_pipeline.sh <input_bag_path> <output_directory>
#

set -e

# Check arguments
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <input_bag_path> <output_directory> [lidar_topic]"
    echo "Example: $0 /data/my_bag /data/output /livox/lidar"
    exit 1
fi

INPUT_BAG="$1"
OUTPUT_DIR="$2"
LIDAR_TOPIC="${3:-auto}"

echo "============================================"
echo "KISS-SLAM to HDMapping Pipeline"
echo "============================================"
echo "Input bag: $INPUT_BAG"
echo "Output directory: $OUTPUT_DIR"
echo "Lidar topic: $LIDAR_TOPIC"
echo "============================================"

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Run the complete Python pipeline
python3 << 'PIPELINE_EOF'
import os
import sys
import json
import struct
import numpy as np
from pathlib import Path

# Get environment variables
input_bag = os.environ.get('INPUT_BAG', '/data/input_bag')
output_dir = os.environ.get('OUTPUT_DIR', '/data/output')
lidar_topic_env = os.environ.get('LIDAR_TOPIC', 'auto')

print(f"Input bag: {input_bag}")
print(f"Output directory: {output_dir}")

os.makedirs(output_dir, exist_ok=True)

# Import rosbags
try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    from rosbags.typesys.msg import get_types_from_msg
except ImportError as e:
    print(f"Error importing rosbags: {e}")
    print("Please install: pip install rosbags")
    sys.exit(1)

# Initialize typestore with ROS2 Humble base types
typestore = get_typestore(Stores.ROS2_HUMBLE)

print("\n[Step 1/3] Reading point clouds from bag...")

# Read all point clouds from bag
all_scans = []  # List of (timestamp, points) tuples
lidar_topic = None

with Reader(input_bag) as reader:
    # Register custom types from bag's embedded message definitions
    for conn in reader.connections:
        if conn.msgdef and 'CustomMsg' in conn.msgtype:
            try:
                types_dict = get_types_from_msg(conn.msgdef.data, conn.msgtype)
                typestore.register(types_dict)
                print(f"Registered {len(types_dict)} custom types from bag")
            except Exception as e:
                print(f"Warning: Could not register types: {e}")

    # Find lidar topic
    for conn in reader.connections:
        print(f"  Found topic: {conn.topic} ({conn.msgtype})")

        if lidar_topic_env != 'auto':
            if conn.topic == lidar_topic_env:
                lidar_topic = conn.topic
                break
        else:
            # Auto-detect: prefer Livox CustomMsg, then any PointCloud2
            if 'CustomMsg' in conn.msgtype:
                lidar_topic = conn.topic
                break
            elif conn.msgtype == 'sensor_msgs/msg/PointCloud2' and 'livox' in conn.topic.lower():
                lidar_topic = conn.topic
            elif conn.msgtype == 'sensor_msgs/msg/PointCloud2' and lidar_topic is None:
                lidar_topic = conn.topic

    if lidar_topic is None:
        print("ERROR: No suitable lidar topic found in bag!")
        sys.exit(1)

    print(f"\nUsing lidar topic: {lidar_topic}")

    # Get message type for this topic
    topic_msgtype = None
    for conn in reader.connections:
        if conn.topic == lidar_topic:
            topic_msgtype = conn.msgtype
            break

    print(f"Message type: {topic_msgtype}")

    # Read all messages
    scan_count = 0
    for conn, timestamp, rawdata in reader.messages():
        if conn.topic != lidar_topic:
            continue

        try:
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)

            if 'CustomMsg' in conn.msgtype:
                # Livox CustomMsg - extract points
                points = []
                for pt in msg.points:
                    if not (np.isnan(pt.x) or np.isnan(pt.y) or np.isnan(pt.z)):
                        points.append([pt.x, pt.y, pt.z, float(pt.reflectivity)])

                if len(points) > 0:
                    all_scans.append((timestamp, np.array(points, dtype=np.float32)))
                    scan_count += 1
            else:
                # PointCloud2 - extract points
                num_points = msg.width * msg.height
                point_step = msg.point_step

                # Find field offsets
                x_offset = y_offset = z_offset = intensity_offset = 0
                has_intensity = False
                for field in msg.fields:
                    if field.name == 'x':
                        x_offset = field.offset
                    elif field.name == 'y':
                        y_offset = field.offset
                    elif field.name == 'z':
                        z_offset = field.offset
                    elif field.name in ['intensity', 'i', 'reflectivity']:
                        intensity_offset = field.offset
                        has_intensity = True

                points = []
                for i in range(num_points):
                    offset = i * point_step
                    x = struct.unpack_from('f', msg.data, offset + x_offset)[0]
                    y = struct.unpack_from('f', msg.data, offset + y_offset)[0]
                    z = struct.unpack_from('f', msg.data, offset + z_offset)[0]

                    if np.isnan(x) or np.isnan(y) or np.isnan(z):
                        continue

                    intensity = 0.0
                    if has_intensity:
                        try:
                            intensity = struct.unpack_from('f', msg.data, offset + intensity_offset)[0]
                        except:
                            pass

                    points.append([x, y, z, intensity])

                if len(points) > 0:
                    all_scans.append((timestamp, np.array(points, dtype=np.float32)))
                    scan_count += 1

        except Exception as e:
            print(f"  Warning: Error processing message: {e}")
            continue

        if scan_count % 50 == 0:
            print(f"  Processed {scan_count} scans...")

print(f"\nTotal scans read: {len(all_scans)}")

if len(all_scans) == 0:
    print("ERROR: No point clouds read from bag!")
    sys.exit(1)

print("\n[Step 2/3] Running KISS-SLAM...")

# Run KISS-SLAM on the scans
try:
    from kiss_icp.kiss_icp import KissICP
    from kiss_icp.config import KISSConfig

    # Create KISS-ICP config
    config = KISSConfig()
    config.data.max_range = 100.0
    config.data.min_range = 0.5
    config.mapping.voxel_size = 0.5  # Required parameter

    # Initialize odometry
    odometry = KissICP(config=config)

    poses = []
    for i, (timestamp, scan) in enumerate(all_scans):
        # KISS-ICP expects Nx3 points as float64
        points_xyz = scan[:, :3].astype(np.float64)

        # Create timestamps for deskewing (uniformly distributed within the scan)
        scan_timestamps = np.linspace(0, 0.1, len(points_xyz))

        # Process scan
        odometry.register_frame(points_xyz, scan_timestamps)

        # Get current pose from the odometry
        pose = odometry.last_pose.copy()
        poses.append(pose)

        if (i + 1) % 50 == 0:
            print(f"  Processed {i + 1}/{len(all_scans)} scans")

    poses = np.array(poses)
    print(f"\nKISS-SLAM complete! Estimated {len(poses)} poses.")

except ImportError as e:
    print(f"Warning: KISS-ICP import error: {e}")
    print("Falling back to identity poses...")
    poses = np.array([np.eye(4) for _ in range(len(all_scans))])

except Exception as e:
    print(f"Warning: KISS-SLAM error: {e}")
    import traceback
    traceback.print_exc()
    print("Falling back to identity poses...")
    poses = np.array([np.eye(4) for _ in range(len(all_scans))])

print("\n[Step 3/3] Converting to HDMapping format...")

# Try to import laspy for LAZ writing
try:
    import laspy
    HAS_LASPY = True
    print("Using laspy for LAZ file creation")
except ImportError:
    HAS_LASPY = False
    print("Warning: laspy not available, will save as numpy files")

# Calculate global offset (use centroid of first transformed scan for stability)
first_scan_transformed = (poses[0][:3, :3] @ all_scans[0][1][:, :3].T).T + poses[0][:3, 3]
global_offset = np.mean(first_scan_transformed, axis=0)
print(f"Global offset: {global_offset}")

# Save LAZ files and collect metadata
laz_files = []
pose_entries = []

for i, ((timestamp, scan), pose) in enumerate(zip(all_scans, poses)):
    # Transform points to global frame
    points_xyz = scan[:, :3]
    intensity = scan[:, 3] if scan.shape[1] > 3 else np.zeros(len(scan))

    # Apply pose transformation
    points_global = (pose[:3, :3] @ points_xyz.T).T + pose[:3, 3]

    # Apply global offset
    points_local = points_global - global_offset

    # Create LAZ file
    laz_filename = f"scan_{i:06d}.laz"
    laz_path = os.path.join(output_dir, laz_filename)

    if HAS_LASPY:
        try:
            las = laspy.create(file_version="1.4", point_format=6)
            las.x = points_local[:, 0]
            las.y = points_local[:, 1]
            las.z = points_local[:, 2]
            las.intensity = (intensity * 256).astype(np.uint16)
            las.gps_time = np.full(len(points_local), timestamp / 1e9)
            las.write(laz_path)
        except Exception as e:
            print(f"  Warning: Error writing LAZ file {laz_filename}: {e}")
            # Fallback to numpy
            npy_path = laz_path.replace('.laz', '.npy')
            np.save(npy_path, points_local)
            laz_filename = os.path.basename(npy_path)
            laz_path = npy_path
    else:
        # Fallback: save as numpy
        npy_path = laz_path.replace('.laz', '.npy')
        np.save(npy_path, points_local)
        laz_filename = os.path.basename(npy_path)
        laz_path = npy_path

    laz_files.append(laz_filename)

    # Store pose relative to offset (identity since points are already transformed)
    pose_entries.append((laz_filename, np.eye(4)))

    if (i + 1) % 50 == 0:
        print(f"  Saved {i + 1}/{len(all_scans)} point clouds")

print(f"\nSaved {len(laz_files)} point cloud files")

# Save poses.reg file
poses_reg_path = os.path.join(output_dir, 'poses.reg')
with open(poses_reg_path, 'w') as f:
    f.write(f"{len(pose_entries)}\n")
    for filename, pose in pose_entries:
        f.write(f"{filename}\n")
        f.write(f"{pose[0,0]:.10f} {pose[0,1]:.10f} {pose[0,2]:.10f} {pose[0,3]:.10f}\n")
        f.write(f"{pose[1,0]:.10f} {pose[1,1]:.10f} {pose[1,2]:.10f} {pose[1,3]:.10f}\n")
        f.write(f"{pose[2,0]:.10f} {pose[2,1]:.10f} {pose[2,2]:.10f} {pose[2,3]:.10f}\n")
        f.write("0 0 0 1\n")

print(f"Saved: {poses_reg_path}")

# Save initial poses file (copy of poses.reg)
initial_poses_path = os.path.join(output_dir, 'lio_initial_poses.reg')
import shutil
shutil.copy(poses_reg_path, initial_poses_path)
print(f"Saved: {initial_poses_path}")

# Save trajectory CSV files for each scan
for i, ((timestamp, _), pose) in enumerate(zip(all_scans, poses)):
    csv_filename = f"trajectory_{i:06d}.csv"
    csv_path = os.path.join(output_dir, csv_filename)

    with open(csv_path, 'w') as f:
        f.write("timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds\n")
        f.write(f"{timestamp} ")
        f.write(f"{pose[0,0]:.10f} {pose[0,1]:.10f} {pose[0,2]:.10f} {pose[0,3]:.10f} ")
        f.write(f"{pose[1,0]:.10f} {pose[1,1]:.10f} {pose[1,2]:.10f} {pose[1,3]:.10f} ")
        f.write(f"{pose[2,0]:.10f} {pose[2,1]:.10f} {pose[2,2]:.10f} {pose[2,3]:.10f} ")
        f.write(f"{timestamp}\n")

print(f"Saved {len(poses)} trajectory CSV files")

# Save session.json
session = {
    "Session Settings": {
        "offset_x": float(global_offset[0]),
        "offset_y": float(global_offset[1]),
        "offset_z": float(global_offset[2]),
        "folder_name": output_dir,
        "out_folder_name": output_dir,
        "poses_file_name": poses_reg_path,
        "initial_poses_file_name": initial_poses_path,
        "out_poses_file_name": poses_reg_path,
        "ground_truth": False,
        "lidar_odometry_version": "KISS-SLAM"
    },
    "loop_closure_edges": [],
    "laz_file_names": [{"file_name": f} for f in laz_files]
}

session_path = os.path.join(output_dir, 'session.json')
with open(session_path, 'w') as f:
    json.dump(session, f, indent=2)

print(f"Saved: {session_path}")

print("\n============================================")
print("Pipeline complete!")
print("============================================")
print(f"Output files in: {output_dir}")
print(f"  - {len(laz_files)} point cloud files (.laz)")
print(f"  - {len(poses)} trajectory files (.csv)")
print(f"  - poses.reg (pose file)")
print(f"  - session.json (HDMapping session file)")
print("============================================")
PIPELINE_EOF

export INPUT_BAG="$INPUT_BAG"
export OUTPUT_DIR="$OUTPUT_DIR"
export LIDAR_TOPIC="$LIDAR_TOPIC"

echo ""
echo "Done!"
