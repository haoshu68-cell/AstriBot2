"""Bounded ROS payload storage; source-time selection is owned by C++."""
from collections import deque
from astribot_s1_robot_geometry._geometry_native import select_source_sample


class SourceInbox:
    def __init__(self, maximum_age_s):
        self.maximum_age_ns=round(maximum_age_s*1e9)
        self.messages=deque(maxlen=64)

    def receive(self, message):
        self.messages.append(message)

    def select(self, now_ns):
        messages=tuple(self.messages)
        stamps=[m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec for m in messages]
        index=select_source_sample(stamps,now_ns,self.maximum_age_ns)
        return messages[index] if index>=0 else None


class EnvelopeInbox:
    """Bounded V2 payload binding; strict renewal selection lives in C++."""
    def __init__(self):
        import threading
        from astribot_s1_robot_geometry._geometry_native import LeaseSelector
        self.lock = threading.Lock()
        self.selector = LeaseSelector()
        self.messages = deque(maxlen=64)
        self.samples = deque(maxlen=64)

    def receive(self, message):
        import copy
        import json
        from rosidl_runtime_py.convert import message_to_ordereddict
        from astribot_s1_robot_geometry._geometry_native import LeaseSample
        ns = lambda t: t.sec * 1_000_000_000 + t.nanosec
        # All permission/geometry fields participate in the context. Only
        # heartbeat time and source sequence may advance without a barrier.
        context = copy.deepcopy(message)
        context.header.stamp.sec = context.header.stamp.nanosec = 0
        context.limits.stamp.sec = context.limits.stamp.nanosec = 0
        context.valid_until.sec = context.valid_until.nanosec = 0
        context.source_state_sequence = 0
        sample = LeaseSample(ns(message.header.stamp), ns(message.limits.stamp),
            ns(message.valid_until), message.limits.lease_s,
            message.navigation_allowed and message.limits.transport_ready,
            json.dumps(message_to_ordereddict(context), sort_keys=True, separators=(',', ':')))
        with self.lock:
            self.samples.append(sample)
            self.messages.append(message)

    def select(self, now_ns):
        with self.lock:
            index = self.selector.select(list(self.samples), now_ns)
            return self.messages[index] if index >= 0 else None
