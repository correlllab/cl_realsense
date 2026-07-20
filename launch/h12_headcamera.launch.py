#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import json



def make_camera(name: str, serial: str, width: int, height: int, fps: int, source_frame, enable_pointcloud: bool = False) -> Node:
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
            'pointcloud.enable':enable_pointcloud,
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
            'decimation_filter.filter_magnitude': 2,
            'pointcloud.stream_filter':2,          # RS2_STREAM_COLOR: texture-maps color onto the cloud
            'pointcloud.stream_index_filter':0,
            # 'pointcloud.filter_magnitude':2,
            # 'pointcloud.frames_queue_size':4,
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
    head_width = 1280
    head_height = 720
    head_fps = 6

    head_cam = make_camera("head", "_250122072330", head_width, head_height, head_fps, "head_camera_link", enable_pointcloud=True)
    ld.add_action(head_cam)

    # Calibrated 2026-07-13 by hand-to-eye (h12_cameracalibration): joint NLS
    # over 81 views fused from three historical sessions (10 mm board, wrist-
    # mounted, pelvis-base data converted through the URDF head mount), median
    # reprojection 11.2 px; replaces the hand-tuned values (0, -0.03, 0 /
    # 0.5, -0.5, 0.5, 0.5), a ~49 mm / 4.2 deg correction. Estimated accuracy
    # ~+/-13 mm / 1.5 deg — regenerate from a fresh collection with
    # h12_cameracalibration/handtoeye_refine.py when possible.
    static_head_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_head_tf_head',
        arguments = [
            '0.000985', '0.014977', '-0.018805',            # x, y, z translation
            '-0.510111', '0.523885', '-0.486398', '-0.478277',  # qx, qy, qz, qw
            'head_camera_link',
            'head_link',
        ],
        output='screen'
    )
    delayed_head_tf = TimerAction(
        period=10.0,  # Wait for 10 seconds before starting the static transform
        actions=[static_head_tf]
    )
    ld.add_action(delayed_head_tf)


    return ld
