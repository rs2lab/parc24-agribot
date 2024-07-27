from rclpy.node import Node

from .extra import BasicQueue
from .perceiver import *
from .action import *
from . import ruler

class AgribotNavigationPlanner:
    def __init__(self, agent: Node, perceptor: AgribotPerceiver) -> None:
        self._agent = agent
        self._perceptor = perceptor
        self._action_queue = BasicQueue()

    @property
    def has_enqueued_actions(self) -> bool:
        return not self._action_queue.is_empty()

    def resolve_next_action(self) -> Action | None:
        """Returns the next action in the queue, removing it from the structure."""
        if self.has_enqueued_actions:
            return self._action_queue.dequeue()
        return None

    def add_next_action(self, action: Action) -> None:
        """Add a new action to the action queue."""
        self._action_queue.enqueue(action)

    def plan_next_action(self) -> Action | None:
        if self.has_enqueued_actions:
            return self.resolve_next_action()

        snapshot = self._perceptor.snapshot()
        pose = snapshot[SensorType.POSE]
        cloud = snapshot[SensorType.POINT_CLOUD]
        scan = snapshot[SensorType.LASER_SCAN]
        imu = snapshot[SensorType.IMU]

        if imu :
            orientation_list = [
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            ]

            (roll, pitch, yaw) = ruler.euler_from_quaternion(orientation_list)


        return self.resolve_next_action()
