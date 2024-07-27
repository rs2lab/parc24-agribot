import os
import numpy as np

from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from .extra import BasicQueue
from .perceiver import *
from .action import *
from .goal import *
from .positioning import PathMap
from . import ruler


class AgribotNavigationPlanner:
    def __init__(self, agent: Node, perceptor: AgribotPerceiver) -> None:
        self._agent = agent
        self._perceptor = perceptor
        self._plan_queue = BasicQueue()
        pkd_share = FindPackageShare(package="parc24_agribot")
        path_map_path = os.path.join(
            pkd_share.find("parc24_agribot"),
            "path_maps",
            "path_map_01",
        )
        self._path_map = PathMap.load_path_map(path_map_path)
        self._path_goal_idx = 0
        self._first = True

    @property
    def has_enqueued(self) -> bool:
        return not self._plan_queue.is_empty()

    def resolve_next(self) -> Action | Goal | None:
        """Returns the next action or goal in the queue, removing it from the structure."""
        if self.has_enqueued:
            return self._plan_queue.dequeue()
        return None

    def add_next(self, action: Action | Goal) -> None:
        """Add a new action or goal to the queue."""
        self._plan_queue.enqueue(action)

    def plan_next(self) -> Action | None:
        if self.has_enqueued:
            return self.resolve_next()

        snapshot = self._perceptor.snapshot()
        # imu = snapshot[SensorType.IMU]
        odom = snapshot[SensorType.ODOM]

        if self._path_goal_idx < len(self._path_map.poses):
            if odom:
                goal_pose = self._path_map.poses[self._path_goal_idx]

                current_orientation_list = [
                    odom.pose.pose.orientation.x,
                    odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z,
                    odom.pose.pose.orientation.w,
                ]

                goal_orientation_list = [
                    goal_pose.orientation.x,
                    goal_pose.orientation.y,
                    goal_pose.orientation.z,
                    goal_pose.orientation.w,
                ]

                (c_roll, c_pitch, c_yaw) = ruler.euler_from_quaternion(
                    current_orientation_list
                )
                (g_roll, g_pitch, g_yaw) = ruler.euler_from_quaternion(
                    goal_orientation_list
                )

                theta = g_roll - c_roll

                (c_x, c_y, c_z) = (
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.y,
                )
                (g_x, g_y, g_z) = (
                    goal_pose.position.x,
                    goal_pose.position.y,
                    goal_pose.position.y,
                )

                dist = ((g_x - c_x)**2 + (g_y - c_y)**2) ** 0.5
                # self.add_next(TimedAction(x=dist, theta=theta, secs=1))
                scale = (dist) ** (1/dist)
                self._agent.get_logger().info(f"Dist: {dist}")
                if self._first:
                    self._first = False
                    self.add_next(TimedAction(x=0, theta=0, secs=2))
                if dist > 0:
                    self.add_next(StepAction(x=dist / 8, theta=theta / 2, steps=4))
                    self.add_next(StepAction(x=dist / 20, theta=theta / 2, steps=2))
                self._path_goal_idx += 1

        return self.resolve_next()
