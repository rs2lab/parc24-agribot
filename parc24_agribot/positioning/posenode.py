from geometry_msgs.msg import PoseStamped


class Position:
    def __init__(self, x: float, y: float, z: float, timestamp: float = None):
        self._x = x
        self._y = y
        self._z = z
        self._timestamp = timestamp

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def timestamp(self):
        return self._timestamp


class Orientation:
    def __init__(self, x: float, y: float, z: float, w: float, timestamp: float = None):
        self._x = x
        self._y = y
        self._z = z
        self._w = w
        self._timestamp = timestamp

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @property
    def w(self):
        return self._w

    @property
    def timestamp(self):
        return self._timestamp


class PoseNode:
    _counter = 0

    def __init__(self, position: Position, orientation: Orientation, timestamp: float = None):
        self._counter += 1
        self._id = self._counter
        self._position = position
        self._orientation = orientation

    def __str__(self) -> str:
        return f"PoseNode{self._id}(position={str(self.position)}, orientation={str(self.orientation)})"

    @property
    def id(self) -> int:
        return self._id

    @property
    def position(self) -> float:
        return self._position

    @property
    def orientation(self) -> float:
        return self._orientation

    def to_pose_stamped(self) -> PoseStamped:
        pose = PoseStamped()
        pose.pose.position.x = self.position.x
        pose.pose.position.y = self.position.y
        pose.pose.position.z = self.position.z
        pose.pose.orientation.x = self.orientation.x
        pose.pose.orientation.y = self.orientation.y
        pose.pose.orientation.z = self.orientation.z
        pose.pose.orientation.w = self.orientation.w
        return pose
