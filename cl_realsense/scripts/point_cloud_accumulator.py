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
from ur_manipulation.srv import GetPointCloud
import time

# Configuration
TOPIC_NAMES = ["/points"]  # topic suffixes for your cameras
BASE_FRAME = 'base_link'

# Accumulation settings
VOXEL_SIZE = 0.01  # downsample voxel size

STATISTICAL_OUTLIER_REMOVAL = True
STATISTICAL_NB_NEIGHBORS = 20
STATISTICAL_STD_RATIO = 2.0

RADIUS_OUTLIER_REMOVAL = False
RADIUS_NB_POINTS = 16
RADIUS_RADIUS = 0.1

OUTPUT_TOPIC = '/realsense/accumulated_point_cloud'
PUBLISH_RATE_HZ = 6.0
TF_TIMEOUT_SEC = 0.5


def msg_to_pcd(msg: PointCloud2) -> o3d.geometry.PointCloud:
    """
    Convert ROS PointCloud2 message to Open3D PointCloud.
    """
    # Read points and colors
    points_list = list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
    
    if not points_list:
        return o3d.geometry.PointCloud()

    # Separate xyz and rgb
    xyz = [[p[0], p[1], p[2]] for p in points_list]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(xyz))    
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
        self.srv = self.create_service(Trigger, "clear_arm_pointcloud", self.pc_clear_callback)
        self.srv = self.create_service(GetPointCloud, "get_arm_pointcloud", self.arm_pointcloud_callback)

        self._lock = threading.Lock()
        self.msg_queue = []
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_accumulated_pc)
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        
        
        threading.Thread(target=self._executor.spin, daemon=True).start()
    

    def get_msg(self) -> PointCloud2:
        with self._lock:
            points = np.asarray(self.pcd.points, dtype=np.float32).copy()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = BASE_FRAME

        if points.size == 0:
            # Well-formed but empty cloud
            return point_cloud2.create_cloud_xyz32(header, np.zeros((0, 3), dtype=np.float32))

        # xyz-only helper
        return point_cloud2.create_cloud_xyz32(header, points)

    def pc_callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id
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
            self.msg_queue.append((msg, trans))

    def main_loop(self):
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
                self.pcd += cloud
                self.pcd = self.pcd.voxel_down_sample(VOXEL_SIZE)
                
                if STATISTICAL_OUTLIER_REMOVAL:
                    self.pcd, _ = self.pcd.remove_statistical_outlier(
                        nb_neighbors=STATISTICAL_NB_NEIGHBORS,
                        std_ratio=STATISTICAL_STD_RATIO
                    )
                if RADIUS_OUTLIER_REMOVAL:
                    self.pcd, _ = self.pcd.remove_radius_outlier(
                        nb_points=RADIUS_NB_POINTS,
                        radius=RADIUS_RADIUS
                    )


    def publish_accumulated_pc(self):
        msg_out = self.get_msg()
        self.publisher.publish(msg_out)

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

        response.cloud = msg_out

        return response

def main(args=None):
    #print("\n\n\nStarting PointCloudAccumulator with changes")
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
