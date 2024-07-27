import os
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from .extra import BasicQueue
from .perceiver import *
from .action import *
from .goal import *
from .positioning import PathMap


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

        for pose in self._path_map.poses:
            self.add_next(BasicGoal(pose))

        return self.resolve_next()
