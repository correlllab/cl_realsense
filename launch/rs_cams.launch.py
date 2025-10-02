#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import json



def make_camera(name: str, serial: str, width: int, height: int, fps: int, source_frame) -> Node:
    """
    Factory function to create a RealSense camera node.
    All parameters are shown grouped by modality; most are commented out by default.
    """
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=name,
        namespace='realsense',
        output='screen',
        emulate_tty=True,
        parameters=[{
            # === Identification & Logging ===
            'use_sim_time': False,
            'camera_name': name,
            'serial_no': serial,
            'initial_reset': True,
            


            # === Modalities ===
            'enable_color':True,
            'enable_depth':True,
            'enable_sync':False,
            'align_depth.enable':True,
            'pointcloud.enable':True,
            'enable_accel':False,            
            'enable_gyro':False,
            'enable_infra1':False,
            'enable_infra2':False,
            'enable_rgbd':False,

            # === Plugins ===
            f'{name}.color.image_raw.enable_pub_plugins':      ['image_transport/compressed'],
            f'{name}.depth.image_rect_raw.enable_pub_plugins': ['image_transport/compressedDepth'],
            f'{name}.aligned_depth_to_color.image_raw.enable_pub_plugins': ['image_transport/compressedDepth'],


            # === Qos ===
            # 'accel_info_qos': 'SENSOR_DATA',
            # 'accel_qos': 'SENSOR_DATA',
            'color_info_qos': 'SENSOR_DATA',
            'color_qos': 'SENSOR_DATA',
            'depth_info_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
            # 'gyro_info_qos': 'SENSOR_DATA',
            # 'gyro_qos': 'SENSOR_DATA',
            # 'infra1_info_qos': 'SENSOR_DATA',
            # 'infra1_qos': 'SENSOR_DATA',
            # 'infra2_info_qos': 'SENSOR_DATA',
            # 'infra2_qos': 'SENSOR_DATA',
            'pointcloud.pointcloud_qos': 'SENSOR_DATA',

            # === Profiles ===
            'depth_module.depth_profile': f"{width}x{height}x{fps}",
            # 'depth_module.infra_profile': f"{width}x{height}x{fps}",
            'rgb_camera.color_profile': f"{width}x{height}x{fps}",

            # === Filters ===
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 3,
            'pointcloud.stream_filter':2,
            'pointcloud.stream_index_filter':0,
            'pointcloud.filter_magnitude':2,
            'pointcloud.frames_queue_size':4,
            # 'spatial_filter.enable': False,
            # 'temporal_filter.enable': False,
            # 'hole_filling_filter.enable': False,

            # === Transform & Playback ===
            'publish_tf': True,
            # 'base_frame_id':             f'{name}_link',                 # head_link / left_hand_link
            # 'depth_frame_id':            f'{name}_depth_frame',
            # 'depth_optical_frame_id':    f'{name}_depth_optical_frame',
            # 'color_frame_id':            f'{name}_color_frame',
            # 'color_optical_frame_id':    f'{name}_color_optical_frame',
            'tf_publish_rate': 10.0,
            # 'json_file_path': '',
            # 'rosbag_filename': '',
            # 'rosbag_loop': False,
            # 'wait_for_device_timeout': -1.0,
            # 'reconnect_timeout': 6.0,
        }]
    )



def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    width = 640
    height = 480
    head_cam = make_camera("head", "_250122072330", width, height, 6, "head_camera_link")
    ld.add_action(head_cam)

    left_hand_cam = make_camera("left_hand", "_838212072778", width, height, 6, "left_hand_camera_link")
    ld.add_action(left_hand_cam)

    # pc_acc_node = Node(
    #     package='h12_realsense',
    #     executable='pc_acc',
    #     name='pc_acc',
    #     output='screen',
    #     emulate_tty=True,
    # )
    # # Create a TimerAction that will wait for a specified period (in seconds)
    # # before executing the actions listed inside it.
    # delayed_pc_acc_node = TimerAction(
    #     period=5.0,  # Wait for 5 seconds before starting the node
    #     actions=[pc_acc_node]
    # )
    
    # # Create the launch description and add the delayed action
    # ld.add_action(delayed_pc_acc_node)



    

    static_hand_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_hand_tf_hand',
        arguments = [
            '-0.01866', '-0.0333', '0.108',     # x y z
            '0.5', '-0.5', '0.5', '0.5',        # qx qy qz qw
            'left_hand_camera_link',
            'left_hand_link',
            
        ],
        output='screen'
    )
    delayed_hand_tf = TimerAction(
        period=10.0,  # Wait for 5 seconds before starting the static transform
        actions=[static_hand_tf]
    )
    ld.add_action(delayed_hand_tf)


    static_head_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_head_tf_head',
        arguments = [
            '0', '0', '0', #x, y, z translation
            '0.5', '-0.5', '0.5', '0.5', #x, y, z, w quats
            'head_camera_link',
            'head_link',
        ],
        output='screen'
    )
    delayed_head_tf = TimerAction(
        period=10.0,  # Wait for 5 seconds before starting the static transform
        actions=[static_head_tf]
    )
    ld.add_action(delayed_head_tf)
    

    return ld
