class Vector2: # defining these because I'm not sure what PIE's verdict is on named tuples
    __slots__ = "x", "y"
    def __init__(self, left, right):
        self.left = left
        self.right = right
class LRStruct:
    __slots__ = "left", "right"
    def __init__(self, left, right):
        self.left = left
        self.right = right
class DebugLogger:
    __slots__ = "_default_interval", "_tick", "_printed_tags"
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
def inches_to_meters(inches):
    return inches / 39.3700787
