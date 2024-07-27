import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        left_topic = os.getenv('LEFT_CAMERA_TOPIC', '/left_camera/image_raw')
        right_topic = os.getenv('RIGHT_CAMERA_TOPIC', '/right_camera/image_raw')
        #front_topic = os.getenv('FRONT_CAMERA_TOPIC', '/zed2_center_camera/image_raw')

        self._cv_bridge = CvBridge()
        self._left_cam_capture_counter = 0
        self._right_cam_capture_counter = 0
        self._front_cam_capture_counter = 0

        self.subscription_left = self.create_subscription(Image, left_topic, self.image_callback_left, 10)
        self.subscription_right = self.create_subscription(Image, right_topic, self.image_callback_right, 10)
        #self.subscription_front = self.create_subscription(Image, front_topic, self.image_callback_front, 10)

    def image_callback_left(self, msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding ="bgr8")
        cv2.imwrite(f'left_image_{self._left_cam_capture_counter}.png', cv_image)
        self._left_cam_capture_counter += 1

    def image_callback_right(self, msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding ="bgr8")
        cv2.imwrite(f'right_image_{self._right_cam_capture_counter}.png', cv_image)
        self._right_cam_capture_counter += 1

    def image_callback_front(self, msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding ="bgr8")
        cv2.imwrite(f'front_image_{self._front_cam_capture_counter}.png', cv_image)
        self._front_cam_capture_counter += 1


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
