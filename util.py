import time

class DebugLogger:
    """A debugging utility that can periodically print messages. Since it's passed around
    everywhere, it's also a good place to put telemetry."""
    _sample_duration = 5 # seconds
    def __init__(self, default_interval):
        self._default_interval = default_interval
        self._tick = 0
        self._printed_tags = {}
        self._time_samples = []
        self._init_time = time.time()
    def tick(self):
        """Advances the internal counter."""
        self._tick += 1
        timestamp = time.time()
        self._time_samples.append(timestamp)
        del_idx = 0
        for sample in self._time_samples:
            if sample >= (timestamp - self._sample_duration):
                break
            del_idx += 1
        if del_idx:
            del self._time_samples[:del_idx - 1]
    def get_ticks_per_second(self):
        """Gets the measured number of ticks per second, or 0 if insufficiently many ticks have been
        counted."""
        return (len(self._time_samples) if self._init_time + self._sample_duration < time.time()
            else 0) / self._sample_duration
    def lazy_print(self, func, interval=None):
        """Prints the result of calling the given function every {interval} of the internal
        counter."""
        if interval == None:
            interval = self._default_interval
        if (self._tick % interval) == 0:
            print(func())
    def print(self, msg, interval=None):
        """Prints a message every {interval} of the internal counter."""
        self.lazy_print(lambda: msg, interval)
    def lazy_print_once(self, tag, func):
        """Prints the result of calling the given function if the given tag has not been printed to
        yet."""
        if not (tag in self._printed_tags):
            self._printed_tags[tag] = True
            print(func())
    def print_once(self, tag, msg):
        """Prints a message if the given tag has not been printed to yet."""
        self.lazy_print_once(tag, lambda: msg)
    def reset_print_tag(self, tag):
        """Allows a tag to be reprinted."""
        if tag in self._printed_tags:
            del self._printed_tags[tag]
class FlagEdgeDetector:
    def __init__(self, func, initial_state=False):
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
    """Converts inches to meters to 8 significant figures."""
    return inches / 39.3700787
