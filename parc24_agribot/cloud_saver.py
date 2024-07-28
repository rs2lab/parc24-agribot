import rclpy
import open3d as o3d

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from .perceiver import SensorType
from .constants import DEFAULT_QoS_PROFILE_VALUE, AGRIBOT_CLOUD_SAVER_NODE_NAME
from . import vision


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__(AGRIBOT_CLOUD_SAVER_NODE_NAME)
        self.subscription = self.create_subscription(
            PointCloud2,
            SensorType.POINT_CLOUD.value,
            self.listener_callback,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._counter = 0

    def listener_callback(self, data):
        self._counter += 1
        xyz, rgb = vision.destruct_point_cloud_to_xyz_rgb(data)
        cloud = vision.od3_cloud_from_xyz_rgb(xyz, rgb)
        o3d.io.write_point_cloud(f"cloud_{self._counter}.pcd", cloud)


def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
