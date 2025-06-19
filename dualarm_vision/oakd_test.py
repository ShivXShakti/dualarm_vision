import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('pointcloud_distance_node')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10)

        self.get_logger().info('PointCloudDistanceNode started and listening to /camera/depth/points')

    def pointcloud_callback(self, msg):
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
    main()
