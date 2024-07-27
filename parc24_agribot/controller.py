import os

from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import Twist

from .action import Action, SingleStepStopAction
from .goal import Goal, BasicGoal
from .constants import DEFAULT_QoS_PROFILE_VALUE, AGRIBOT_BASE_NAVIGATOR_NAME
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

NAV_PUB_TOPIC = "/cmd_vel"


class AgribotController:
    _publisher: Publisher
    _base_nav: BasicNavigator

    def __init__(self, agent: Node) -> None:
        self._agent = agent
        self._logger = self._agent.get_logger()
        self._publisher = self._agent.create_publisher(
            Twist, NAV_PUB_TOPIC, DEFAULT_QoS_PROFILE_VALUE
        )
        self._base_nav = BasicNavigator(AGRIBOT_BASE_NAVIGATOR_NAME)

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

    def pursue_goal(self, goal: Goal) -> None:
        goal.init()
        while goal.has_next_step():
            pose = goal.next_goal_pose()
            self._base_nav.goToPose(pose.to_pose_stamped())
            self._logger.info(f"Pursuing Goal {goal}")
            while not self._base_nav.isTaskComplete():
                feedback = self._base_nav.getFeedback()
                if feedback.navigation_duration > 600:
                    self._base_nav.cancelTask()
        goal.finish()
        result = self._base_nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal {goal} succeeded!")
        elif result == TaskResult.CANCELED:
            print("Goal {goal} was canceled!")
        elif result == TaskResult.FAILED:
            print("Goal {goal} failed!")

    def execute_action(self, action: Action) -> None:
        """Executes a given action."""
        action.init()
        while action.has_next_step():
            self._logger.info(f"Executing Action {action}")
            action.consume_step(lambda twist: self.publish(twist))
        action.finish()

    def stop(self, *, stop_agent: bool = False) -> None:
        shutdown = (lambda: self._agent.context.shutdown()) if stop_agent else None
        self.execute_action(SingleStepStopAction(on_finished_cb=shutdown))
