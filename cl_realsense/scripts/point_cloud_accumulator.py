import threading
from scipy.spatial.transform import Rotation
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
import os

# Configuration
TOPIC_NAMES = ["/realsense/ee_cam/depth/color/points"]  # topic suffixes for your cameras
BASE_FRAME = 'floor_link'

# Accumulation settings
VOXEL_SIZE = 0.005  # downsample voxel size

STATISTICAL_OUTLIER_REMOVAL = True
STATISTICAL_NB_NEIGHBORS = 100
STATISTICAL_STD_RATIO = 0.5

RADIUS_OUTLIER_REMOVAL = False
RADIUS_NB_POINTS = 10
RADIUS_RADIUS = VOXEL_SIZE * (RADIUS_NB_POINTS/2)


OUTPUT_TOPIC = '/realsense/accumulated_point_cloud'
SUBSCRIBER_RATE_HZ = 6.0
PUBLISH_RATE_HZ = 6.0
TF_TIMEOUT_SEC = 0.5


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
POINTCLOUD_SAVE_DIR = os.path.join(SCRIPT_DIR, "accumulated_pointclouds")
POINTCLOUD_PATH = os.path.join(POINTCLOUD_SAVE_DIR, "accumulated_pointcloud.ply")
AUTO_LOAD_ON_STARTUP = True

MIN_DISTANCE_FROM_CAMERA = 0.2  # meters
MAX_DISTANCE_FROM_CAMERA = 0.5  # meters
MAX_QUEUE_SIZE = 10
ACC_DOWN_ONLY_COS_THRESHOLD = -0.9  # cosine similarity threshold; -1.0 = exactly down, -0.9 ≈ within 26°


def msg_to_pcd(msg: PointCloud2) -> o3d.t.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D tensor PointCloud (with color).
    Handles packed 'rgb' field from RealSense cameras.
    """

    arr = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
    if arr is None or len(arr) == 0:
        return o3d.t.geometry.PointCloud()

    # read_points_numpy returns (N, 4) float32: columns are x, y, z, rgb
    arr = arr.astype(np.float32)
    xyz = arr[:, :3]
    dist = np.linalg.norm(xyz, axis=1)
    mask = (dist >= MIN_DISTANCE_FROM_CAMERA) & (dist <= MAX_DISTANCE_FROM_CAMERA)
    xyz = xyz[mask]

    if len(xyz) == 0:
        return o3d.t.geometry.PointCloud()

    rgb_uint32s = np.ascontiguousarray(arr[mask, 3]).view(np.uint32)
    colors = np.stack([
        (rgb_uint32s >> 16) & 0xFF,
        (rgb_uint32s >> 8) & 0xFF,
        rgb_uint32s & 0xFF,
    ], axis=1).astype(np.float32) / 255.0

    pcd = o3d.t.geometry.PointCloud()
    pcd.point['positions'] = o3d.core.Tensor(xyz, dtype=o3d.core.Dtype.Float32)
    pcd.point['colors'] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)

    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    if STATISTICAL_OUTLIER_REMOVAL and pcd.point['positions'].shape[0] > STATISTICAL_NB_NEIGHBORS:
        pcd, _ = pcd.remove_statistical_outliers(
            nb_neighbors=STATISTICAL_NB_NEIGHBORS,
            std_ratio=STATISTICAL_STD_RATIO,
        )
    if RADIUS_OUTLIER_REMOVAL and pcd.point['positions'].shape[0] > RADIUS_NB_POINTS:
        pcd, _ = pcd.remove_radius_outliers(
            nb_points=RADIUS_NB_POINTS,
            search_radius=RADIUS_RADIUS,
        )
    return pcd


def transform_to_matrix(trans) -> np.ndarray:
    """
    Convert a TransformStamped into a 4x4 numpy transformation matrix.
    """
    q = trans.transform.rotation
    mat = np.identity(4, dtype=np.float64)
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

        self.pcd = o3d.t.geometry.PointCloud()
        self.clear_srv = self.create_service(Trigger, "pointcloud_accumulator/clear_arm_pointcloud", self.pc_clear_callback)
        self.get_srv = self.create_service(GetPointCloud, "pointcloud_accumulator/get_arm_pointcloud", self.arm_pointcloud_callback)
        self.srv_save = self.create_service(Trigger, "pointcloud_accumulator/save_arm_pointcloud", self.save_pointcloud_callback)
        self.srv_load = self.create_service(Trigger, "pointcloud_accumulator/load_arm_pointcloud", self.load_pointcloud_callback)
        self.stop_acc_srv = self.create_service(Trigger, "pointcloud_accumulator/stop_acc", self.stop_save_callback)
        self.start_acc_srv = self.create_service(Trigger, "pointcloud_accumulator/start_acc", self.start_save_callback)
        self.acc_bool = True
        self.acc_down_only = True

        self._last_pc_time = 0.0
        self._pc_interval = 1.0 / SUBSCRIBER_RATE_HZ

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
            if self.pcd.is_empty():
                points = np.zeros((0, 3), dtype=np.float32)
                colors = np.zeros((0, 3), dtype=np.float32)
            else:
                points = self.pcd.point['positions'].numpy().copy()
                colors = self.pcd.point['colors'].numpy().copy() if 'colors' in self.pcd.point else np.zeros_like(points)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = BASE_FRAME
        if len(points) == 0:
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
        now = time.time()
        if now - self._last_pc_time < self._pc_interval:
            return
        self._last_pc_time = now

        source_frame = msg.header.frame_id
        t = rclpy.time.Time.from_msg(msg.header.stamp)

        try:
            trans = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                source_frame,
                t,
                timeout=Duration(seconds=TF_TIMEOUT_SEC)
            )
        except TransformException as e:
            self.get_logger().info(f"TF lookup failed from '{source_frame}' to '{BASE_FRAME}': {e}")
            return
        if self.acc_down_only:
            # Dot product of base frame z-axis [0,0,1] with source frame z-axis in base frame.
            # Negative means the camera z is pointing down (opposite to floor_link z-up).
            q = trans.transform.rotation
            rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
            source_z_in_base = rot.apply([0.0, 0.0, 1.0])
            if np.dot([0.0, 0.0, 1.0], source_z_in_base) > ACC_DOWN_ONLY_COS_THRESHOLD:
                self.get_logger().info(f"camera not pointed down, not accumulating frame")
                return

        with self._lock:
            if len(self.msg_queue) >= MAX_QUEUE_SIZE:
                return
            self.msg_queue.append((msg, trans))

    def main_loop(self):
        self.get_logger().info("Starting PointCloudAccumulator main loop")
        while rclpy.ok():
            msg, trans = None, None
            with self._lock:
                if self.msg_queue:
                    msg, trans = self.msg_queue.pop(0)

            if msg is None:
                time.sleep(0.1)
                continue

            cloud = msg_to_pcd(msg)
            if cloud.is_empty():
                continue

            mat = o3d.core.Tensor(transform_to_matrix(trans), dtype=o3d.core.Dtype.Float64)
            cloud.transform(mat)

            with self._lock:
                if self.acc_bool:
                    if self.pcd.is_empty():
                        self.pcd = cloud.clone()
                    else:
                        pos = np.vstack([
                            self.pcd.point['positions'].numpy(),
                            cloud.point['positions'].numpy(),
                        ])
                        col = np.vstack([
                            self.pcd.point['colors'].numpy(),
                            cloud.point['colors'].numpy(),
                        ])
                        self.pcd = o3d.t.geometry.PointCloud()
                        self.pcd.point['positions'] = o3d.core.Tensor(pos, dtype=o3d.core.Dtype.Float32)
                        self.pcd.point['colors'] = o3d.core.Tensor(col, dtype=o3d.core.Dtype.Float32)
                    self.pcd = self.pcd.voxel_down_sample(VOXEL_SIZE)
                    self.get_logger().debug(
                        f"Accumulated cloud now has {self.pcd.point['positions'].shape[0]} points"
                    )

    def publish_accumulated_pc(self):
        msg_out = self.get_msg()
        self.publisher.publish(msg_out)

    def pc_clear_callback(self, request, response):
        self.get_logger().info(f"Got request: {request}")
        with self._lock:
            self.pcd = o3d.t.geometry.PointCloud()
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
        with self._lock:
            if self.pcd.is_empty():
                self.get_logger().warn("No points to save")
                return False
            pcd_copy = self.pcd.clone()

        try:
            self.get_logger().debug(f"Writing to: {POINTCLOUD_PATH}")
            o3d.t.io.write_point_cloud(POINTCLOUD_PATH, pcd_copy, write_ascii=False, compressed=True)
            self.get_logger().info(
                f"Saved point cloud: {POINTCLOUD_PATH} ({pcd_copy.point['positions'].shape[0]} points)"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving point cloud: {e}")
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
            loaded_pcd = o3d.t.io.read_point_cloud(POINTCLOUD_PATH)

            if loaded_pcd.is_empty():
                self.get_logger().warn(f"Loaded point cloud is empty: {POINTCLOUD_PATH}")
                return False

            with self._lock:
                self.pcd = loaded_pcd

            num_points = loaded_pcd.point['positions'].shape[0]
            self.get_logger().info(
                f"Loaded point cloud: {POINTCLOUD_PATH} ({num_points} points)"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Error loading point cloud: {e}")
            return False


def main(args=None):
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
