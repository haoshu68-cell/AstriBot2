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
