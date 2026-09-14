"""World identity and transport conversion, independent of ROS executors."""
import hashlib
import math
from dataclasses import replace
from .contracts import Version


class ExecutionContext:
    def __init__(self):
        self.version = Version('idle',0,0,0)
        self.map_key = None
        self.transform = None
        self.sequence = -1

    def task(self, identifier, state, sequence):
        if sequence <= self.sequence:return
        self.sequence = sequence
        if state == 'EXECUTING':
            if identifier != self.version.goal_id:
                self.version = replace(self.version,goal_id=identifier,path_revision=0)
        elif state in ('SUCCEEDED','FAILED','CANCELED','PREEMPTED') and identifier == self.version.goal_id:
            self.version = replace(self.version,goal_id='idle')

    def map(self, metadata, cells):
        key = (metadata,hashlib.blake2b(cells,digest_size=16).digest())
        if key != self.map_key:
            self.map_key = key
            self.version = replace(self.version,map_epoch=self.version.map_epoch+1)
            return True
        return False

    def localization(self, pose, position_limit, angle_limit):
        previous = self.transform
        self.transform = pose
        if previous is not None and (math.dist(pose[:2],previous[:2]) > position_limit or
                abs(math.remainder(pose[2]-previous[2],2*math.pi)) > angle_limit):
            self.version = replace(self.version,localization_epoch=self.version.localization_epoch+1)
            return True
        return False

    def path(self):
        self.version = replace(self.version,path_revision=self.version.path_revision+1)


def to_wire(version, message):
    message.task_id = version.goal_id
    for field in ('path_revision','map_epoch','localization_epoch','envelope_epoch','clock_epoch'):
        setattr(message,field,getattr(version,field))
    return message
