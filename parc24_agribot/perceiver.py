from enum import Enum
from types import FunctionType
from cv_bridge import CvBridge
from rclpy.node import Node

from .constants import DEFAULT_QoS_PROFILE_VALUE

import nav_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import tf2_msgs.msg
import rclpy


class SensorType(Enum):
    POINT_CLOUD = "/zed2_leftd_camera/points"
    FRONT_CAM = "/zed2_center_camera/image_raw"
    LEFT_CAM = "/left_camera/image_raw"
    RIGHT_CAM = "/right_camera/image_raw"
    ODOM = "/robot_base_controller/odom"
    LASER_SCAN = "/scan"
    JOINT_STATES = "/joint_states"
    INITIAL_POSE = "/initialpose"
    GOAL_POSE = "/goal_pose"
    GPS = "/gps/fix"
    IMU = "/imu_plugin/out"
    TF = "/tf"


class AgribotPerceiver:
    def __init__(self, agent: Node) -> None:
        self._agent = agent
        self._cv_bridge = CvBridge()
        self._point_cloud_state = None
        self._laser_scan_state = None
        self._front_cam_state = None
        self._left_cam_state = None
        self._right_cam_state = None
        self._odom_state = None
        self._tf_state = None
        self._joint_states = None
        self._initial_pose_state = None
        self._goal_pose_state = None
        self._gps_state = None
        self._imu_state = None

        # register state update callbacks
        self._state_update_listeners = {}

        self._agent.create_subscription(
            geometry_msgs.msg.PoseWithCovarianceStamped,
            SensorType.INITIAL_POSE.value,
            self._initial_pose_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            geometry_msgs.msg.PoseStamped,
            SensorType.GOAL_POSE.value,
            self._goal_pose_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.PointCloud2,
            SensorType.POINT_CLOUD.value,
            self._point_cloud_state_update_handler,
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
            sensor_msgs.msg.NavSatFix,
            SensorType.GPS.value,
            self._gps_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )
        self._agent.create_subscription(
            sensor_msgs.msg.Imu,
            SensorType.IMU.value,
            self._imu_state_update_handler,
            DEFAULT_QoS_PROFILE_VALUE,
        )

        self._goal_lat_param = None
        self._goal_lon_param = None

        # self._goal_cartesian_x_pos = None
        # self._goal_cartesian_y_pos = None

        # self._first_cartesian_x_pos = None
        # self._first_cartesian_y_pos = None

        # try:
        #     self._agent.declare_parameter("goal_latitude", rclpy.Parameter.Type.DOUBLE)
        #     self._agent.declare_parameter("goal_longitude", rclpy.Parameter.Type.DOUBLE)

        #     # Get goal localization from world coordinates yaml file
        #     self._goal_lat_param = self._agent.get_parameter("goal_latitude")
        #     self._goal_lon_param = self._agent.get_parameter("goal_longitude")

        #     x, y = gps_to_cartesian(
        #         self._goal_lat_param.value, self._goal_lon_param.value
        #     )

        #     self._goal_cartesian_x_pos = x
        #     self._goal_cartesian_y_pos = y

        #     self._agent.get_logger().info(
        #         "Goal gps localization: LAT = %f, LON = %f"
        #         % (self.goal_lat, self.goal_lon)
        #     )

        #     self._agent.get_logget().info(
        #         "Goal cartesian localization: X = %f, Y = %f"
        #         % (self.goal_cartesian_x_pos, self.goal_cartesian_y_pos)
        #     )
        # except:
        #     self._agent.get_logger().warn(
        #         "Couldn't get the goal LATITUDE and LONGITUDE localizations from the ROS params"
        #     )

    @property
    def goal_lat(self) -> float | None:
        if self._goal_lat_param:
            return self._goal_lat_param.value
        return None

    @property
    def goal_lon(self) -> float | None:
        if self._goal_lon_param:
            return self._goal_lon_param.value
        return None

    # @property
    # def goal_cartesian_x_pos(self) -> float | None:
    #     return self._goal_cartesian_x_pos

    # @property
    # def goal_cartesian_y_pos(self) -> float | None:
    #     return self._goal_cartesian_y_pos

    # @property
    # def first_cartesian_x_pos(self) -> float | None:
    #     return self._first_cartesian_x_pos

    # @property
    # def first_cartesian_y_pos(self) -> float | None:
    #     return self._first_cartesian_y_pos

    # def current_cartesian_position(self) -> tuple[float] | None:
    #     if self._gps_state is not None:
    #         x, y = gps_to_cartesian(self._gps_state.latitude, self._gps_state.longitude)
    #         return (x, y)
    #     return None

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

    def _initial_pose_state_update_handler(self, data):
        self._initial_pose_state = data
        self._trigger_callbacks(SensorType.INITIAL_POSE, data)

    def _goal_pose_state_update_handler(self, data):
        self._goal_pose_state = data
        self._trigger_callbacks(SensorType.GOAL_POSE, data)

    def _point_cloud_state_update_handler(self, data):
        self._point_cloud_state = data
        self._trigger_callbacks(SensorType.POINT_CLOUD, data)

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
        self._trigger_callbacks(SensorType.ODOM, data)

    def _joint_states_update_handler(self, data):
        self._joint_states = data
        self._trigger_callbacks(SensorType.JOINT_STATES, data)

    def _gps_state_update_handler(self, data):
        # if self._first_cartesian_x_pos is None or self._first_cartesian_y_pos is None:
        #     x, y = gps_to_cartesian(data.latitude, data.longitude)
        #     self._first_cartesian_x_pos = x
        #     self._first_cartesian_y_pos = y
        #     self._agent.get_logger().info(
        #         "First GPS Position: x = %f, y = %f"
        #         % (self._first_cartesian_x_pos, self._first_cartesian_y_pos)
        #     )
        self._gps_state = data
        self._trigger_callbacks(SensorType.GPS, data)

    def _imu_state_update_handler(self, data):
        self._imu_state = data
        self._trigger_callbacks(SensorType.IMU, data)

    def snapshot(self) -> dict[SensorType]:
        """Returns a snapshopt of the perceived state of the environment in the specific instant
        to guarantee consistence of the data when processing them together."""
        return {
            SensorType.INITIAL_POSE: self._initial_pose_state,
            SensorType.GOAL_POSE: self._goal_pose_state,
            SensorType.POINT_CLOUD: self._point_cloud_state,
            SensorType.FRONT_CAM: self._front_cam_state,
            SensorType.LEFT_CAM: self._left_cam_state,
            SensorType.RIGHT_CAM: self._right_cam_state,
            SensorType.ODOM: self._odom_state,
            SensorType.LASER_SCAN: self._laser_scan_state,
            SensorType.JOINT_STATES: self._joint_states,
            SensorType.GPS: self._gps_state,
            SensorType.IMU: self._imu_state,
            SensorType.TF: self._tf_state,
        }
