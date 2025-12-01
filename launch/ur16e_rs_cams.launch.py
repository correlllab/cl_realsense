#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import LogInfo

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
            'pointcloud__neon_.filter_magnitude': 2,
            'pointcloud__neon_.frames_queue_size': 4,

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



def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    width = 640
    height = 480
    ee_cam = make_camera("ee_cam", "_838212073340", width, height, 6)
    # ee_cam = make_camera("ee_cam", "_836612071918", width, height, 6)

    ld.add_action(ee_cam)

    # proc_node = Node(
    #         package='depth_image_proc',
    #         executable='point_cloud_xyz_node',
    #         name='depth_image_to_point_cloud',
    #         output='screen',
    #         parameters=[{
    #             'depth_image': '/realsense/ee_cam/aligned_depth_to_color/image_raw',
    #             'rgb_image': '/realsense/ee_cam/color/image_raw',
    #             'camera_info': '/realsense/ee_cam/aligned_depth_to_color/camera_info',
    #             'pointcloud_topic': '/realsense/ee_cam/pointcloud',
    #         }],
    #         remappings=[
    #             ('/image_rect', '/realsense/ee_cam/aligned_depth_to_color/image_raw'),
    #             ('/camera/color/image', '/realsense/ee_cam/color/image_raw'),
    #             ('/camera_info', '/realsense/ee_cam/aligned_depth_to_color/camera_info'),
    #             ('/camera/pointcloud', '/realsense/ee_cam/pointcloud'),
    #         ]
    # )
    # ld.add_action(proc_node)


    acc_node = Node(
        package = "cl_realsense",
        executable = "pc_acc",
        name = "pointcloud_accumulator",
        output = "screen"
    )
    ld.add_action(acc_node)

    neg_ninety = str(-3.1415/2.)
    static_ee_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_ee_cam_tf',
        arguments = [
            '0.0199', '-0.1767', '0.0534',
            neg_ninety, neg_ninety, neg_ninety,
            'tool0',
            'ee_cam_link',
        ],
        output='screen'
    )
    # delayed_ee_cam_tf = TimerAction(
    #     period=10.0,  # Wait for 5 seconds before starting the static transform
    #     actions=[static_ee_cam_tf]
    # )
    ld.add_action(static_ee_cam_tf)
    print("here4 from", __file__)
    return ld
