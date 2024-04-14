class LRStruct:
    __slots__ = "left", "right"
    def __init__(self, left, right):
        self.left = left
        self.right = right
class DebugLogger:
    def __init__(self, default_interval):
        self._default_interval = default_interval
        self._tick = 0
        self._printed_tags = {}
    def tick(self):
        self._tick += 1
    def lazy_print(self, func, interval=None):
        if interval == None:
            interval = self._default_interval
        if (self._tick % interval) == 0:
            print(func())
    def print(self, msg, interval=None):
        self.lazy_print(lambda: msg, interval)
    def lazy_print_once(self, tag, func):
        if not (tag in self._printed_tags):
            self._printed_tags[tag] = True
            print(func())
    def print_once(self, tag, msg):
        self.lazy_print_once(tag, lambda: msg)
    def reset_print_tag(self, tag):
        if tag in self._printed_tags:
            del self._printed_tags[tag]
class FlagEdgeDetector:
    def __init__(self, func, initial_state = False):
        self._func = func
        self._state = initial_state
    def test(self):
        result = False
        flag = self._func()
        if not self._state and flag:
            result = True
        self._state = flag
        return result
def inches_to_meters(inches):
    return inches / 39.3700787
