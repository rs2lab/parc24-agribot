import rclpy

from rclpy.node import Node
from .constants import AGRIBOT_AGENT_NODE_NAME
from .controller import AgribotController
from .planner import AgribotNavigationPlanner
from .perceiver import AgribotPerceiver
from .goal import Goal
from .action import Action


class AgribotAgent(Node):
    def __init__(self, exec_period_secs: float = 0.1) -> None:
        super().__init__(AGRIBOT_AGENT_NODE_NAME)
        self._exec_period_secs = exec_period_secs
        self.perceptor = AgribotPerceiver(self)
        self.controller = AgribotController(self)
        self.planner = AgribotNavigationPlanner(self, self.perceptor)
        self.create_timer(
            self._exec_period_secs,
            self.execute,
        )

    def execute(self) -> None:
        if plan := self.planner.plan_next():
            if isinstance(plan, Action):
                self.controller.execute_action(plan)
            elif isinstance(plan, Goal):
                self.controller.pursue_goal(plan)


def main(args=None):
    rclpy.init(args=args)
    agent = AgribotAgent(exec_period_secs=0.2)
    print(
        "\n======> Agribot Autonomous Navigator Agent"
        f" [node = {AGRIBOT_AGENT_NODE_NAME}] Started..."
    )
    try:
        rclpy.spin(agent)
        rclpy.shutdown()
    except KeyboardInterrupt as e:
        pass
    except Exception as e:
        print("======> Ended with exception:", e)
    finally:
        agent.destroy_node()
    print(
        "======> Agribot Autonomous Navigator Agent"
        f" [node = {AGRIBOT_AGENT_NODE_NAME}] Stopped."
    )


if __name__ == "__main__":
    main()
