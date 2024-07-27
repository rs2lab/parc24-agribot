from abc import ABC, abstractmethod
from .positioning import PoseNode


class Goal(ABC):
    def __init__(self, on_finished_cb=None) -> None:
        super().__init__()
        self._on_finished_cb = on_finished_cb

    def init(self) -> None:
        """Initializes all variables needed to execute the action."""
        pass

    @abstractmethod
    def next_goal_pose(self) -> PoseNode:
        pass

    def has_next_step(self) -> bool:
        return False

    def finish(self) -> None:
        """Finishes the execution of the action."""
        if self._on_finished_cb is not None:
            self._on_finished_cb()


class BasicGoal(Goal):
    def __init__(self, goal_pose: PoseNode) -> None:
        super().__init__()
        self._goal_pose = goal_pose
        self._h = False

    def next_goal_pose(self) -> PoseNode:
        self._h = True
        return self._goal_pose

    def has_next_step(self) -> bool:
        return not self._h
