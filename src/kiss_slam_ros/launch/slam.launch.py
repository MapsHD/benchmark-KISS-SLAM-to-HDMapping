#!/usr/bin/env python3
"""
Launch file for KISS-SLAM ROS2 pipeline.

Launches:
1. livox_to_pc2 - Converts Livox CustomMsg to PointCloud2
2. kiss_slam_node - KISS-SLAM processing node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'livox_topic',
        default_value='/livox/lidar',
        description='Input Livox CustomMsg topic'
    )

    pc2_topic_arg = DeclareLaunchArgument(
        'pc2_topic',
        default_value='/livox/points',
        description='Intermediate PointCloud2 topic'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/kiss_slam/odometry',
        description='Output odometry topic'
    )

    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/kiss_slam/local_map',
        description='Output local map topic'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='50.0',
        description='Maximum LiDAR range in meters'
    )

    # Livox to PointCloud2 converter node
    livox_to_pc2_node = Node(
        package='livox_to_pc2',
        executable='livox_to_pc2_node',
        name='livox_to_pc2',
        parameters=[{
            'input_topic': LaunchConfiguration('livox_topic'),
            'output_topic': LaunchConfiguration('pc2_topic'),
            'intensity_scale': 1.0
        }],
        output='screen'
    )

    # KISS-SLAM node
    kiss_slam_node = Node(
        package='kiss_slam_ros',
        executable='slam_node.py',
        name='kiss_slam',
        parameters=[{
            'input_topic': LaunchConfiguration('pc2_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'max_range': LaunchConfiguration('max_range'),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }],
        output='screen'
    )

    return LaunchDescription([
        input_topic_arg,
        pc2_topic_arg,
        odom_topic_arg,
        map_topic_arg,
        max_range_arg,
        livox_to_pc2_node,
        kiss_slam_node,
    ])
