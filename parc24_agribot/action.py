from time import time
from abc import ABC, abstractmethod
from geometry_msgs.msg import Twist


class Action(ABC):
    """Generic action class. This is an abstract class, hence, does not represent the
    concrete representation of the action to be taken by the robot."""

    def __init__(self, x: float, theta: float, on_finished_cb=None) -> None:
        self._on_finished_cb = on_finished_cb
        self._x = x
        self._theta = theta

    def __str__(self) -> str:
        return f"Action(x={self.x}, theta={self.theta})"

    @property
    def x(self):
        return self._x

    @property
    def theta(self):
        return self._theta

    def to_twist(self) -> Twist:
        twist = Twist()
        twist.linear.x = float(self.x)
        twist.angular.z = float(self.theta)
        return twist

    def init(self) -> None:
        """Initializes all variables needed to execute the action."""
        pass

    def has_next_step(self) -> bool:
        """Returns True if this action has a next step to be executed."""
        return False

    def consume_step(self, fn) -> None:
        """Consumes one step of the action. Receives a function `fn`
        that receives a twist command as a param."""
        fn(self.to_twist())

    def finish(self) -> None:
        """Finishes the execution of the action."""
        if self._on_finished_cb is not None:
            self._on_finished_cb()


class TimedAction(Action):
    """Use this for actions that should be taken in a certain amount of seconds."""

    def __init__(
        self, *, x: float, theta: float, secs: float = 0, on_finished_cb=None
    ) -> None:
        super().__init__(x, theta, on_finished_cb=on_finished_cb)
        self._secs = secs
        self._start_time = None
        self._secs_spent = None

    def __str__(self) -> str:
        return f"TimedAction(x={self.x}, theta={self.theta}, elapsed={self.secs_spent}, secs={self.secs})"

    @property
    def secs(self) -> float:
        return self._secs

    @property
    def time_left(self) -> float:
        if self._secs and self._secs_spent:
            return self._secs - self._secs_spent
        return None

    @property
    def secs_spent(self) -> float:
        return self._secs_spent

    @classmethod
    def from_twist(cls, twist, secs=0) -> Action:
        return TimedAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            secs=secs,
        )

    def init(self) -> None:
        self._start_time = time()
        self._secs_spent = 0

    def has_next_step(self) -> bool:
        self._secs_spent = time() - self._start_time
        return self._secs_spent < self._secs


class StepAction(Action):
    """Use this for actions that should be repeated a certain amount of times."""

    def __init__(
        self, *, x: float, theta: float, steps: int = 1, on_finished_cb=None
    ) -> None:
        super().__init__(x, theta, on_finished_cb=on_finished_cb)
        self._steps = steps
        self._current_step = 0
        self.init()

    def __str__(self) -> str:
        return f"StepAction(x={self.x}, theta={self.theta}, step={self.current_step}, steps={self.steps})"

    @property
    def steps(self) -> int:
        return self._steps

    @property
    def current_step(self) -> int:
        return self._current_step

    @classmethod
    def from_twist(cls, twist, steps=1) -> Action:
        return StepAction(
            theta=twist.angular.z,
            x=twist.linear.x,
            steps=steps,
        )

    def init(self) -> None:
        self._current_step = 0

    def has_next_step(self) -> bool:
        return self.current_step < self.steps

    def consume_step(self, fn) -> None:
        self._current_step += 1
        super().consume_step(fn)


class EternalStopAction(Action):
    """This will make the robot stop forever while the program
    is running."""

    def __init__(self) -> None:
        super().__init__(0, 0, on_finished_cb=None)

    def has_next_step(self) -> bool:
        return True


class SingleStepStopAction(StepAction):
    """This will send only one stop message to the robot."""

    def __init__(self, *, on_finished_cb=None) -> None:
        super().__init__(x=0, theta=0, steps=1, on_finished_cb=on_finished_cb)
