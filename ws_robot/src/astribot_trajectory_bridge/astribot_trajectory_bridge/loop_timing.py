"""Bounded wall-time diagnostics, independent of the ROS control clock."""
from collections import deque
from contextlib import contextmanager
import math
import threading
from time import perf_counter


class LoopTiming:
    def __init__(self, capacity=8192):
        self.capacity = capacity
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        with self._lock:
            self._samples = {}
            self._counts = {}
            self._overruns = 0
            self._overrun_peak = 0.0

    def add(self, name, seconds):
        with self._lock:
            if name not in self._samples:
                self._samples[name] = deque(maxlen=self.capacity)
                self._counts[name] = 0
            self._samples[name].append(seconds)
            self._counts[name] += 1

    @contextmanager
    def measure(self, name):
        start = perf_counter()
        try:
            yield
        finally:
            self.add(name, perf_counter() - start)

    def overrun(self, seconds):
        with self._lock:
            self._overruns += 1
            self._overrun_peak = max(self._overrun_peak, seconds)

    def consume_overruns(self):
        with self._lock:
            result = self._overruns, self._overrun_peak
            self._overruns = 0
            self._overrun_peak = 0.0
            return result

    def consume(self):
        with self._lock:
            samples, counts = self._samples, self._counts
            self._samples, self._counts = {}, {}
        result = {}
        for name, values in samples.items():
            ordered = sorted(v * 1000.0 for v in values)
            result[name] = {
                'samples': len(ordered), 'dropped': counts[name] - len(ordered),
                'mean_ms': sum(ordered) / len(ordered),
                **{f'p{p}_ms': ordered[math.ceil(p / 100 * len(ordered)) - 1]
                   for p in (50, 95, 99)},
                'max_ms': ordered[-1],
            }
        return result
