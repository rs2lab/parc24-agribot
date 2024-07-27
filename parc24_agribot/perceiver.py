from enum import Enum
from types import FunctionType
from cv_bridge import CvBridge
from rclpy.node import Node
from laser_geometry import LaserProjection

from .constants import DEFAULT_QoS_PROFILE_VALUE

import rclpy.time
import tf2_ros
import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg


class SensorType(Enum):
    POSE = "/zed/zed_node/pose"
    POINT_CLOUD = "/zed/zed_node/point_cloud/cloud_registered"
    FUSED_CLOUD = "/zed/zed_node/mapping/fused_cloud"
    FRONT_CAM = "/zed/zed_node/rgb/image_rect_color"  # XXX: check
    LEFT_CAM = "/left_camera/image_raw"  # XXX: check
    RIGHT_CAM = "/right_camera/image_raw"  # XXX: check
    ODOM = "/zed/zed_node/odom"
    LASER_SCAN = "/scan"
    JOINT_STATES = "/joint_states"
    IMU = "/zed/zed_node/imu/data"
    TF = "/tf"
    CLOUD_LASER = "/local/cloud_laser"


ZED_BASE_TF_FRAME = "zed_camera_link"


class AgribotPerceiver:
    def __init__(self, agent: Node) -> None:
        self._agent = agent
        self._cv_bridge = CvBridge()
        self._lp = LaserProjection()
        # self._tf_buffer = tf2_ros.Buffer()
        # self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._pose_state = None
        self._point_cloud_state = None
        self._fused_cloud_state = None
        self._laser_scan_state = None
        self._front_cam_state = None
        self._left_cam_state = None
        self._right_cam_state = None
        self._odom_state = None
        self._tf_state = None
        self._joint_states = None
        self._imu_state = None
        self._cloud_scan_state = None

        # register state update callbacks
        self._state_update_listeners = {}

        self._agent.create_subscription(
            geometry_msgs.msg.Pose,
            SensorType.POSE.value,
            self._pose_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.PointCloud2,
            SensorType.POINT_CLOUD.value,
            self._point_cloud_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.PointCloud2,
            SensorType.FUSED_CLOUD.value,
            self._fused_cloud_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.LaserScan,
            SensorType.LASER_SCAN.value,
            self._laser_scan_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.Image,
            SensorType.FRONT_CAM.value,
            self._front_cam_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.Image,
            SensorType.LEFT_CAM.value,
            self._left_cam_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.Image,
            SensorType.RIGHT_CAM.value,
            self._right_cam_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            tf2_msgs.msg.TFMessage,
            SensorType.TF.value,
            self._tf_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            nav_msgs.msg.Odometry,
            SensorType.ODOM.value,
            self._odom_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.JointState,
            SensorType.JOINT_STATES.value,
            self._joint_states_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.Imu,
            SensorType.IMU.value,
            self._imu_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )

    def register_state_update_listener(
        self, sensor_type: SensorType, callback: FunctionType
    ) -> None:
        """Add callbacks that will be called when the state of one the topics we listen to
        is changed."""
        if sensor_type in self._state_update_listeners:
            self._state_update_listeners[sensor_type].append(callback)
        else:
            self._state_update_listeners[sensor_type] = [callback]

    def _trigger_callbacks(self, sensor_type, data) -> None:
        """This triggers all registered callbacks for the specific sensor."""
        if sensor_type in self._state_update_listeners:
            callbacks = self._state_update_listeners[sensor_type]
            for callback in callbacks:
                callback(data)

    def _point_cloud_state_update_handler(self, data):
        self._point_cloud_state = data
        # self._cloud_scan_state = self._lp.projectLaser(data) # XXX
        self._trigger_callbacks(SensorType.POINT_CLOUD, data)
        # TODO: finish
        # try:
        #     transform = self._tf_buffer.lookup_transform(
        #         ZED_BASE_TF_FRAME,
        #         data.header.frame_id,
        #         rclpy.time.Time(),
        #     )
        #     self._cloud_scan_state = self.tr

    def _fused_cloud_state_update_handler(self, data):
        self._fused_cloud_state = data
        self._trigger_callbacks(SensorType.FUSED_CLOUD, data)

    def _pose_state_update_handler(self, data):
        self._pose_state = data
        self._trigger_callbacks(SensorType.POSE, data)

    def _laser_scan_state_update_handler(self, data):
        self._laser_scan_state = data
        self._trigger_callbacks(SensorType.LASER_SCAN, data)

    def _front_cam_state_update_handler(self, data):
        image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self._front_cam_state = image
        self._trigger_callbacks(SensorType.FRONT_CAM, image)

    def _left_cam_state_update_handler(self, data):
        image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self._left_cam_state = image
        self._trigger_callbacks(SensorType.LEFT_CAM, image)

    def _right_cam_state_update_handler(self, data):
        image = self._cv_bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self._right_cam_state = image
        self._trigger_callbacks(SensorType.RIGHT_CAM, image)

    def _tf_state_update_handler(self, data):
        self._tf_state = data
        self._trigger_callbacks(SensorType.TF, data)

    def _odom_state_update_handler(self, data):
        self._odom_state = data
        self._trigger_callbacks(SensorType.ODOM, data)

    def _joint_states_update_handler(self, data):
        self._joint_states = data
        self._trigger_callbacks(SensorType.JOINT_STATES, data)

    def _imu_state_update_handler(self, data):
        self._imu_state = data
        self._trigger_callbacks(SensorType.IMU, data)

    def snapshot(self) -> dict[SensorType]:
        """Returns a snapshopt of the perceived state of the environment in the specific instant
        to guarantee consistence of the data when processing them together."""
        return {
            SensorType.POINT_CLOUD: self._point_cloud_state,
            SensorType.FUSED_CLOUD: self._fused_cloud_state,
            SensorType.FRONT_CAM: self._front_cam_state,
            SensorType.LEFT_CAM: self._left_cam_state,
            SensorType.RIGHT_CAM: self._right_cam_state,
            SensorType.ODOM: self._odom_state,
            SensorType.LASER_SCAN: self._laser_scan_state,
            SensorType.JOINT_STATES: self._joint_states,
            SensorType.IMU: self._imu_state,
            SensorType.TF: self._tf_state,
        }
