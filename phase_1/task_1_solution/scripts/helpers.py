from collections import deque


class ForgetfulMemory:
    def __init__(self, memory_size = 5):
        self._memory_size = memory_size
        self._memory = deque([], memory_size)

    def empty(self):
        return len(self._memory) == 0

    def add(self, value):
        self._memory.append(value)

    def last(self):
        if not seld.empty():
            return self._memory[-1]
        return None

    def forget_all(self):
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
