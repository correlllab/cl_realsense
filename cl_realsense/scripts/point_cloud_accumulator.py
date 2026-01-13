import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from arpa_control.srv import GetPointCloud
import time
import struct
import os

# Configuration
TOPIC_NAMES = ["/realsense/ee_cam/depth/color/points"]  # topic suffixes for your cameras
BASE_FRAME = 'floor_link'

# Accumulation settings
VOXEL_SIZE = 0.01  # downsample voxel size

STATISTICAL_OUTLIER_REMOVAL = False
STATISTICAL_NB_NEIGHBORS = 20
STATISTICAL_STD_RATIO = 3.0

RADIUS_OUTLIER_REMOVAL = False
RADIUS_NB_POINTS = 5
RADIUS_RADIUS = 0.05

OUTPUT_TOPIC = '/realsense/accumulated_point_cloud'
PUBLISH_RATE_HZ = 6.0
TF_TIMEOUT_SEC = 0.5


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POINTCLOUD_SAVE_DIR = os.path.join(SCRIPT_DIR, "accumulated_pointclouds")
POINTCLOUD_PATH = os.path.join(POINTCLOUD_SAVE_DIR, "accumulated_pointcloud.ply")
AUTO_LOAD_ON_STARTUP = True 
MAX_DISTANCE_FROM_CAMERA = 0.5  # meters
MAX_QUEUE_SIZE = 10

def msg_to_pcd(msg: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D PointCloud (with color).
    Handles packed 'rgb' field from RealSense cameras.
    """
    points_list = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True))
    if not points_list:
        return o3d.geometry.PointCloud()

    filtered_points = []
    for p in points_list:
        distance = np.sqrt(p[0]**2 + p[1]**2 + p[2]**2)
        if distance <= MAX_DISTANCE_FROM_CAMERA:
            filtered_points.append(p)
    points_list = filtered_points
    
    if not points_list:
        return o3d.geometry.PointCloud()

    xyz = [[p[0], p[1], p[2]] for p in points_list]
    rgb_floats = [p[3] for p in points_list]

    endian = '>' if msg.is_bigendian else '<'
    rgb_bytes = struct.pack(endian + 'f' * len(rgb_floats), *rgb_floats)
    rgb_uint32s = struct.unpack(endian + 'I' * len(rgb_floats), rgb_bytes)

    colors = []
    for i in rgb_uint32s:
        r = (i >> 16) & 0xFF
        g = (i >> 8) & 0xFF
        b = i & 0xFF
        colors.append([r / 255.0, g / 255.0, b / 255.0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float32))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float32))
    return pcd

def transform_to_matrix(trans) -> np.ndarray:
    """
    Convert a TransformStamped into a 4x4 numpy transformation matrix.
    """
    q = trans.transform.rotation
    mat = np.identity(4)
    mat[0, 0] = 1 - 2*q.y**2 - 2*q.z**2
    mat[0, 1] = 2*q.x*q.y - 2*q.w*q.z
    mat[0, 2] = 2*q.x*q.z + 2*q.w*q.y
    mat[1, 0] = 2*q.x*q.y + 2*q.w*q.z
    mat[1, 1] = 1 - 2*q.x**2 - 2*q.z**2
    mat[1, 2] = 2*q.y*q.z - 2*q.w*q.x
    mat[2, 0] = 2*q.x*q.z - 2*q.w*q.y
    mat[2, 1] = 2*q.y*q.z + 2*q.w*q.x
    mat[2, 2] = 1 - 2*q.x**2 - 2*q.y**2
    mat[0:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    return mat

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('point_cloud_accumulator')
        os.makedirs(POINTCLOUD_SAVE_DIR, exist_ok=True)

        pc_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscribers = []
        for topic in TOPIC_NAMES:
            sub = self.create_subscription(PointCloud2, topic, self.pc_callback, pc_qos)
            self.get_logger().info(f"Subscribed to {topic}")
            self.subscribers.append(sub)

        self.publisher = self.create_publisher(PointCloud2, OUTPUT_TOPIC, pc_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pcd = o3d.geometry.PointCloud()
        self.clear_srv = self.create_service(Trigger, "pointcloud_accumulator/clear_arm_pointcloud", self.pc_clear_callback)
        self.get_srv = self.create_service(GetPointCloud, "pointcloud_accumulator/get_arm_pointcloud", self.arm_pointcloud_callback)
        self.srv_save = self.create_service(Trigger, "pointcloud_accumulator/save_arm_pointcloud", self.save_pointcloud_callback)
        self.srv_load = self.create_service(Trigger, "pointcloud_accumulator/load_arm_pointcloud", self.load_pointcloud_callback)
        self.stop_acc_srv = self.create_service(Trigger, "pointcloud_accumulator/stop_acc", self.stop_save_callback)
        self.start_acc_srv = self.create_service(Trigger, "pointcloud_accumulator/start_acc", self.start_save_callback)
        self.acc_bool = False


        self._lock = threading.Lock()
        self.msg_queue = []
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_accumulated_pc)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        
        
        threading.Thread(target=self._executor.spin, daemon=True).start()

        if AUTO_LOAD_ON_STARTUP:
            self.load_pointcloud()
    
    def stop_save_callback(self, request, response):
        self.acc_bool = False
        response.message = "stopped accumulating"
        response.success = True
        return response 
    def start_save_callback(self, request, response):
        self.acc_bool = True
        response.message = "started accumulating"
        response.success = True
        return response 


    def get_msg(self) -> PointCloud2:
        with self._lock:
            points = np.asarray(self.pcd.points)
            colors = np.asarray(self.pcd.colors)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = BASE_FRAME
        if points.size == 0:
            return point_cloud2.create_cloud_xyz32(header, np.zeros((0, 3), dtype=np.float32))
        
        colors_uint8 = (colors * 255).astype(np.uint8)
        
        rgb_uint32 = np.left_shift(colors_uint8[:, 0].astype(np.uint32), 16) | \
                     np.left_shift(colors_uint8[:, 1].astype(np.uint32), 8) | \
                     colors_uint8[:, 2].astype(np.uint32)
        rgb_float32 = rgb_uint32.view(np.float32)
        
        cloud_data = np.hstack([points.astype(np.float32), rgb_float32[:, np.newaxis]])
        
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg_out = point_cloud2.create_cloud(header, fields, cloud_data)
        return msg_out

    def pc_callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id
        t = msg.header.stamp.sec
        # self.get_logger().info(f"recived pointcloud from {source_frame}, at time {t}")

        try:
            trans = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=TF_TIMEOUT_SEC)
            )
        except TransformException as e:
            self.get_logger().warning(f"TF lookup failed from '{source_frame}' to '{BASE_FRAME}': {e}")
            return
        with self._lock:
            len_queue = len(self.msg_queue)
            print(f"{len_queue=}")
            if len_queue >= MAX_QUEUE_SIZE:
                return
            self.msg_queue.append((msg, trans))

    def main_loop(self):
        print("Starting PointCloudAccumulator main loop")
        while rclpy.ok():
            msg, trans = None, None
            with self._lock:
                if self.msg_queue:                    
                    msg, trans = self.msg_queue.pop(0)

            if msg is None:
                time.sleep(0.1)
                continue
            
            
            mat = transform_to_matrix(trans)
            cloud = msg_to_pcd(msg)
            if not cloud.has_points():
                continue
            
            cloud.transform(mat)
            with self._lock:
                print(f"Accumulating point cloud with {len(self.pcd.points)} points")
                if self.acc_bool:
                    self.pcd += cloud
                self.pcd = self.pcd.voxel_down_sample(VOXEL_SIZE)
                # print(f"{type(self.pcd)=}")
                #idk why these print progres bars
                if STATISTICAL_OUTLIER_REMOVAL:
                    self.pcd, _ = self.pcd.remove_statistical_outlier(
                        nb_neighbors=STATISTICAL_NB_NEIGHBORS,
                        std_ratio=STATISTICAL_STD_RATIO,
                        print_progress=False
                    )
                    # print("statistical outlier removal done")
                if RADIUS_OUTLIER_REMOVAL:
                    self.pcd, _ = self.pcd.remove_radius_outlier(
                        nb_points=RADIUS_NB_POINTS,
                        radius=RADIUS_RADIUS,
                        print_progress=False
                    )
                    # print("radius outlier removal done")

    def publish_accumulated_pc(self):
        msg_out = self.get_msg()
        self.publisher.publish(msg_out)
        print(f"Published accumulated point cloud with {len(self.pcd.points)} points")

    def pc_clear_callback(self, request, response):
        self.get_logger().info(f"Got request: {request}")
        with self._lock:
            self.pcd = o3d.geometry.PointCloud()
            self.msg_queue = []
        response.success = True
        response.message = "Reset Accumulated Point Cloud"
        return response

    def arm_pointcloud_callback(self, request, response):
        self.get_logger().info("PointCloud service called")

        msg_out = self.get_msg()

        self.get_logger().info("PointCloud message constructor")

        response.cloud = msg_out

        self.get_logger().info("PointCloud service returned")
        return response

    def save_pointcloud_callback(self, request, response):
        response.success = self.save_pointcloud()
        response.message = "Saved point cloud to disk" if response.success else "Failed to save point cloud"
        return response

    def load_pointcloud_callback(self, request, response):
        response.success = self.load_pointcloud()
        response.message = "Loaded point cloud from disk" if response.success else "Failed to load point cloud"
        return response

    def save_pointcloud(self) -> bool:
        """
        Save the accumulated point cloud.            
        Returns:
            bool: True if save was successful
        """
        pcd_copy = None
        with self._lock:
            if not self.pcd.has_points():
                self.get_logger().warn("No points to save")
                return False
            
            pcd_copy = o3d.geometry.PointCloud(self.pcd)
        
        try:
            self.get_logger().debug(f"Writing to: {POINTCLOUD_PATH}")
            
            # Save as compressed binary PLY
            success = o3d.io.write_point_cloud(
                POINTCLOUD_PATH, 
                pcd_copy, 
                write_ascii=False,
                compressed=True
            )
            self.get_logger().info(f"Saved point cloud: {POINTCLOUD_PATH} ({len(pcd_copy.points)} points)")
            return success
        except Exception as e:
            return False
        
    def load_pointcloud(self) -> bool:
        """
        Load accumulated point cloud from disk.
        
        Returns:
            bool: True if load was successful
        """
        if not os.path.exists(POINTCLOUD_PATH):
            self.get_logger().info(f"No saved point cloud found at {POINTCLOUD_PATH}")
            return False
        
        try:
            loaded_pcd = o3d.io.read_point_cloud(POINTCLOUD_PATH)
            
            if not loaded_pcd.has_points():
                self.get_logger().warn(f"Loaded point cloud is empty: {POINTCLOUD_PATH}")
                return False
            
            with self._lock:
                self.pcd = loaded_pcd
            
            num_points = len(loaded_pcd.points)
            self.get_logger().info(
                f"Loaded point cloud: {POINTCLOUD_PATH} ({num_points} points)"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading point cloud: {e}")
            return False
    

def main(args=None):
    # print("\n\n\nStarting PointCloudAccumulator")
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    try:
        node.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
