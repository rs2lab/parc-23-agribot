import enum
import numpy as np

from collections import deque
from geometry_msgs.msg import Twist


class RotationType(enum.Enum):
    ANTICLOCKWISE = -1
    CLOCKWISE = 1


class ForgetfulMemory:
    def __init__(self, limit_size = None):
        self._memory_size = limit_size
        self._memory = deque([], limit_size)

    def empty(self):
        return len(self._memory) == 0

    def add(self, value):
        self._memory.append(value)

    def last(self, n=1):
        if not self.empty():
            return self._memory[-n]
        return None

    def all(self):
        return list(self._memory)

    def clear(self):
        self._memory.clear()


class BasicQueue(deque):
    def __init__(self):
        super().__init__([], maxlen = None)

    def empty(self):
        return len(self) == 0

    def enqueue(self, value):
        self.append(value)

    def dequeue(self):
        if not self.empty():
            return self.popleft()
        return None

    def peek(self):
        if not self.empty():
            return self[0]
        return None


class BasicGeoPos:
    def __init__(self, latitude, longitude):
        self._latitude = latitude
        self._longitude = longitude

    @property
    def latitude(self):
        return self._latitude

    @property
    def longitude(self):
        return self._longitude

    @property
    def lat(self):
        """Alias for latitude."""
        return self.latitude

    @property
    def lon(self):
        """Alias for longitude."""
        return self.longitude

    def to_array(self):
        return np.array((self.lat, self.lon))

    def sub(self, pos):
        """Return the subtraction of this position by the given `pos`"""
        if pos is not None:
            return BasicGeoPos(
                latitude=self.lat - pos.lat,
                longitude=self.lon - pos.lon,
            )
        return self
    
class ZeroTwist(Twist):
    def __init__(self, *args, **kwds):
        super().__init__(*args, **kwds)
        self.linear.x = 0.0
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = 0.0
