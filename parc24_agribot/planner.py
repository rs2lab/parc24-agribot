import os
import numpy as np
import abc

from parc24_agribot.action import Action
from parc24_agribot.goal import Goal
from parc24_agribot.perceiver import SensorType
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from .extra import BasicQueue, ForgetfulMemory
from .perceiver import *
from .action import *
from .goal import *
from .positioning import PathMap
from . import ruler, vision
import cv2


class Planning(abc.ABC):
    @abc.abstractmethod
    def plan(self, snapshot: dict[SensorType]) -> Action | Goal:
        pass


class PathMapBasedPlanning(Planning):
    def __init__(self, planner) -> None:
        super().__init__()
        self._planner = planner
        pkd_share = FindPackageShare(package="parc24_agribot")
        path_map_path = os.path.join(
            pkd_share.find("parc24_agribot"),
            "path_maps",
            "path_map_01",
        )
        self._path_map = PathMap.load_path_map(path_map_path)
        self._path_goal_idx = 0

    def add_next(self, plan):
        self._planner.add_next(plan)

    def log(self, message):
        self._planner._agent.get_logger().info(message)

    def plan(self, snapshot: dict[SensorType]) -> Action | Goal:
        odom = snapshot[SensorType.ODOM]
        if self._path_goal_idx < len(self._path_map.poses) and odom:
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
            self.log(f"Dist: {dist}")
            if dist > 0:
                self.add_next(StepAction(x=min(0.20, dist / 12), theta=theta / 8, steps=4))
                self.add_next(StepAction(x=min(0.20, dist / 20), theta=theta / 6, steps=2))
            self._path_goal_idx += 1


class ImageObstableDetectionBasedPlanning(Planning):
    def __init__(self, planner) -> None:
        super().__init__()
        self._planner = planner
        self._last_actions_memory = ForgetfulMemory()
        self._has_turned_first_row = False

    def plan(self, snapshot: dict[SensorType]) -> list[Action | Goal]:
        return self._move_forward(snapshot)

    def _move_forward(self, snapshot) -> list[Action]:
        front_cam_state = snapshot[SensorType.FRONT_CAM]
        if front_cam_state is not None:
            # front_cam_state = front_cam_state.reshape(vision.WIDTH, vision.)
            self._planner._agent.get_logger().info(f'front mask shape = {vision.FRONT_MASK.shape}')
            self._planner._agent.get_logger().info(f'front_cam_shape = {front_cam_state.shape}')
            front_cam_state = cv2.cvtColor(front_cam_state, cv2.COLOR_RGB2HSV)
            front_theta = ruler.calculate_front_theta(front_cam_state)
            last_theta_alpha = self._last_actions_memory.last()
            last_theta = last_theta_alpha[0] if last_theta_alpha is not None else None
            theta = ruler.theta_weighted_sum(front_theta=front_theta, last_theta=last_theta)
            alpha = ruler.alpha_theta(theta, last_theta=last_theta)
            scale = 0.1 ** np.abs(front_theta / 2)
            self._last_actions_memory.add((theta, alpha))
            self._planner._agent.get_logger().info(f'theta = {theta}, alpha = {alpha}')
            return [
                StepAction(x=min(0.34, 0.65 * scale), theta=theta * 0.1, steps=10),
                SingleStepStopAction(),
                StepAction(x=min(0.34, 0.2 * scale), theta=alpha * 0.1, steps=10),
                SingleStepStopAction(),
            ]
        return []

    def _make_a_turn(self, snapshot) -> list[Action]:
        pass

    def finish(self, snapshot) -> list[Action]:
        pass



class AgribotNavigationPlanner:
    def __init__(self, agent: Node, perceptor: AgribotPerceiver) -> None:
        self._agent = agent
        self._perceptor = perceptor
        self._plan_queue = BasicQueue()
        self._imgobsplanning = ImageObstableDetectionBasedPlanning(self)
        self._planner = PathMapBasedPlanning(self)

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
        self._planner.plan(snapshot)

        # for action in self._imgobsplanning.plan(snapshot):
        #     self.add_next(action)

        return self.resolve_next()
