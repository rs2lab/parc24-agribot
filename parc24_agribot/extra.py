from collections import deque
from geometry_msgs.msg import Twist, Vector3


class ForgetfulMemory:
    def __init__(self, limit_size=None):
        self._memory_size = limit_size
        self._memory = deque([], limit_size)

    def is_empty(self):
        return len(self._memory) == 0

    def add(self, value):
        self._memory.append(value)

    def last(self, n=1):
        if not self.is_empty():
            return self._memory[-n]
        return None

    def all(self):
        return list(self._memory)

    def clear(self):
        self._memory.clear()


class BasicQueue(deque):
    def __init__(self):
        super().__init__([], maxlen=None)

    def is_empty(self):
        return len(self) == 0

    def enqueue(self, value):
        self.append(value)

    def dequeue(self):
        if not self.is_empty():
            return self.popleft()
        return None

    def peek(self):
        if not self.is_empty():
            return self[0]
        return None

