import os

from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist

from .action import Action, SingleStepStopAction
from .constants import DEFAULT_QoS_PROFILE_VALUE


# XXX: update to the correct topic name
ROBOT_NAV_CONTROL_TOPIC = os.getenv(
    "ROBOT_NAV_CONTROL_TOPIC", "/robot_base_controller/cmd_vel_unstamped"
)


class AgribotController:
    _publisher: Publisher

    def __init__(self, agent: Node) -> None:
        self._agent = agent
        self._logger = self._agent.get_logger()
        self._publisher = self._agent.create_publisher(
            Twist,
            ROBOT_NAV_CONTROL_TOPIC,
            DEFAULT_QoS_PROFILE_VALUE,
        )

    @property
    def publisher(self) -> Publisher:
        return self._publisher

    def publish(self, twist: Twist) -> None:
        self._logger.debug(
            "Sending Twist ==> "
            f"Linear(x={twist.linear.x}, y={twist.linear.y}, z={twist.linear.z}) "
            f"Angular(x={twist.angular.x}, y={twist.angular.y}, z={twist.angular.z})"
        )
        self._publisher.publish(twist)

    def execute_action(self, action: Action) -> None:
        """Executes a given action."""
        action.init()
        while action.has_next_step():
            self._logger.info(f"Executing {action}")
            action.consume_step(lambda twist: self.publish(twist))
        action.finish()

    def stop(self, *, stop_agent: bool = False) -> None:
        shutdown = (lambda: self._agent.context.shutdown()) if stop_agent else None
        self.execute_action(SingleStepStopAction(on_finished_cb=shutdown))
