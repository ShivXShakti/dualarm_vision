import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration



class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_distance_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/urs/oak/depth/color/points',
            self.listener_callback,
            qos_profile
        )
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Hardcoded pixel location (u, v)
        self.target_u = 0  # horizontal pixel (width axis)
        self.target_v = 0  # vertical pixel (height axis)

        self.get_logger().info('PointCloudDistanceNode started and listening to /urs/oak/depth/color/points')

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "oak_link"  # replace with your point cloud frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "point_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=1)
        self.marker_pub.publish(marker)

    
    def listener_callback(self, msg):
        width = msg.width
        height = msg.height
        self.target_u = width//2  # horizontal pixel (width axis)
        self.target_v = height//2

        # Make sure pixel coordinates are within bounds
        if not (0 <= self.target_u < width and 0 <= self.target_v < height):
            self.get_logger().error(f"Pixel ({self.target_u}, {self.target_v}) out of bounds.")
            return

        index = self.target_v * width + self.target_u

        try:
            for i, point in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)):
                if i == index:
                    x, y, z = point
                    self.publish_marker(float(x), float(y), float(z))

                    if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                        distance = np.linalg.norm([x, y, z])
                        self.get_logger().info(
                            f"Object at pixel ({self.target_u}, {self.target_v}): x={x:.3f}, y={y:.3f}, z={z:.3f}, distance={distance:.2f} m")
                    else:
                        self.get_logger().warn(f"Invalid point at pixel ({self.target_u}, {self.target_v}) (NaN or Inf).")
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading point: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_distance_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )


        self.subscription = self.create_subscription(
            PointCloud2,
            '/urs/oak/depth/color/points',
            self.listener_callback,
            qos_profile
        )
        self.get_logger().info('PointCloudDistanceNode started and listening to /oak/points')

    def listener_callback(self, msg):
        self.get_logger().info('Received PointCloud2 message')
        width = msg.width
        height = msg.height

        # Get the center point index
        center_u = width // 2
        center_v = height // 2
        index = center_v * width + center_u

        # Read point at the center
        pc_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
        try:
            for i, point in enumerate(pc_gen):
                if i == index:
                    x, y, z = point
                    if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                        distance = np.linalg.norm([x, y, z])
                        self.get_logger().info(f"Distance at center: {distance:.2f} meters")
                    else:
                        self.get_logger().warn("No valid point at center (NaN or Inf).")
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading point: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""


"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_distance_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )


        self.subscription = self.create_subscription(
            PointCloud2,
            '/stereo/points',
            self.listener_callback,
            qos_profile
        )

        self.get_logger().info('PointCloudDistanceNode started and listening to /oak/points')

    def listener_callback(self, msg):
        self.get_logger().info('Received PointCloud2 message')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
