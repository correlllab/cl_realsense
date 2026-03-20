#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.actions import OpaqueFunction, DeclareLaunchArgument, TimerAction

from ament_index_python.packages import get_package_share_directory

import json



def make_camera(name: str, serial: str, width: int, height: int, fps: int) -> Node:
    """
    Factory function to create a RealSense camera node.
    All parameters are shown grouped by modality; most are commented out by default.
    """
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=name,
        namespace='realsense',
        output="log",
        emulate_tty=True,
        # ros_arguments=['--log-level', f'realsense.{name}:=ERROR'],
        parameters=[{
            # === Identification & Logging ===
            'use_sim_time': False,
            'camera_name': name,
            'serial_no': serial,
            'initial_reset': True,

            # === Modalities ===
            'enable_color':True,
            'enable_depth':True,
            'enable_sync':True,
            'align_depth.enable':True,
            # 'pointcloud.enable':True,
            'enable_accel':False,            
            'enable_gyro':False,
            'enable_infra1':False,
            'enable_infra2':False,
            'enable_rgbd':False,

            # === idk why we have to do this to get the pc ===
            'pointcloud__neon_.enable': True,
            'pointcloud__neon_.stream_filter': 2,
            'pointcloud__neon_.stream_index_filter': 0,
            'pointcloud__neon_.filter_magnitude': 1,
            'pointcloud__neon_.frames_queue_size': 8,

            # === Plugins ===
            # f'{name}.color.image_raw.enable_pub_plugins':      ['image_transport/compressed'],
            # f'{name}.depth.image_rect_raw.enable_pub_plugins': ['image_transport/compressedDepth'],
            # f'{name}.aligned_depth_to_color.image_raw.enable_pub_plugins': ['image_transport/compressedDepth'],


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
            'decimation_filter.enable': False,
            'decimation_filter.filter_magnitude': 1,
            'pointcloud.stream_filter': 0,
            'pointcloud.stream_index_filter': 0,
            'pointcloud.filter_magnitude': 1,
            'pointcloud.frames_queue_size': 8,
            # 'spatial_filter.enable': True,
            # 'spatial_filter.filter_magnitude': 2,
            # 'spatial_filter.filter_smooth_alpha': 0.5,
            # 'spatial_filter.filter_smooth_delta': 15,
            # 'spatial_filter.holes_fill': 2,
            # 'temporal_filter.enable': True,
            # 'temporal_filter.filter_smooth_alpha': 0.4,
            # 'temporal_filter.filter_smooth_delta': 20,
            # 'temporal_filter.holes_fill': 3,
            # 'hole_filling_filter.enable': True,
            # 'hole_filling_filter.holes_fill': 2,

            # === Transform & Playback ===
            # 'publish_tf': True,
            # 'base_frame_id':             f'{name}_link',                 # head_link / left_hand_link
            # 'depth_frame_id':            f'{name}_depth_frame',
            # 'depth_optical_frame_id':    f'{name}_depth_optical_frame',
            # 'color_frame_id':            f'{name}_color_frame',
            # 'color_optical_frame_id':    f'{name}_color_optical_frame',
            # 'tf_publish_rate': 10.0,
            # 'json_file_path': '',
            # 'rosbag_filename': '',
            # 'rosbag_loop': False,
            # 'wait_for_device_timeout': -1.0,
            # 'reconnect_timeout': 6.0,
        }]
    )

def launch_setup(context, *args, **kwargs):
    acc_node = Node(
        package = "cl_realsense",
        executable = "pc_acc",
        name = "pointcloud_accumulator",
        output = "log",
        emulate_tty=False, # This flag forces the output to be treated as a terminal
    )

    neg_ninety = str(-3.1415/2.)
    static_ee_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_ee_cam_tf',
        arguments = [
            '0.033838200335085646', '-0.17124934095715254', '0.04973827839776546',
            '0.7090084119721195', '0.004283455595113997', '0.7051866592991314', '0.000706616917886843',
            'tool0',
            'ee_cam_link',
        ],
        output="log"
    )
    
    width = 1280
    height = 720
    fps = 6
    ee_cam = make_camera("ee_cam", "_838212073340", width, height, fps)


    return [ee_cam, acc_node, static_ee_cam_tf]

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])



