# CL_realsense
Library for using the realsense cameras the correll lab has mounted on the head and left hand

start with
```
ros2 launch cl_realsense rs_cams.launch.py 
```
will start the topics
```
/realsense/accumulated_point_cloud
/realsense/head/aligned_depth_to_color/camera_info
/realsense/head/aligned_depth_to_color/image_raw/compressedDepth
/realsense/head/color/camera_info
/realsense/head/color/image_raw/compressed
/realsense/head/color/metadata
/realsense/head/depth/camera_info
/realsense/head/depth/color/points
/realsense/head/depth/image_rect_raw/compressedDepth
/realsense/head/depth/metadata
/realsense/head/extrinsics/depth_to_color
/realsense/head/extrinsics/depth_to_depth
/realsense/left_hand/aligned_depth_to_color/camera_info
/realsense/left_hand/aligned_depth_to_color/image_raw/compressedDepth
/realsense/left_hand/color/camera_info
/realsense/left_hand/color/image_raw/compressed
/realsense/left_hand/color/metadata
/realsense/left_hand/depth/camera_info
/realsense/left_hand/depth/color/points
/realsense/left_hand/depth/image_rect_raw/compressedDepth
/realsense/left_hand/depth/metadata
/realsense/left_hand/extrinsics/depth_to_color
/realsense/left_hand/extrinsics/depth_to_depth
unitree@unitree-hi-2-pc4:~/camera_ws$ 
```

In order for the transform to run you also need the robot tf to be publishing, this can be achived minimally with
```
ros2 launch h12_ros2_controller robot_tf_launch.py 
```
from the h12_ros2_controller package also from the correll lab
