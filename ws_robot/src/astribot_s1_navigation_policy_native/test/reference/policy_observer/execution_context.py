"""World identity and transport conversion, independent of ROS executors."""
import hashlib
import math
import os
from dataclasses import replace
from .contracts import Version


_native = None
if os.environ.get('ASTRIBOT_NAV_NATIVE_KERNELS', '').lower() in ('1', 'true', 'yes'):
    try:
        from astribot_s1_navigation_policy_native import _navigation_math_native as _native
    except ImportError:  # pragma: no cover - Python-only overlays remain supported
        _native = None


class ExecutionContext:
    def __init__(self):
        self._native_context = _native.ExecutionContext() if _native is not None else None
        self._version = Version('idle',0,0,0)
        self.version = self._version
        self.map_key = None
        self.transform = None
        self.sequence = -1

    @property
    def version(self):
        return self._version

    @version.setter
    def version(self, value):
        self._version = value
        if getattr(self, '_native_context', None) is not None:
            self._native_context.set_version(
                value.goal_id, value.path_revision, value.map_epoch,
                value.envelope_epoch, value.localization_epoch,
                value.clock_epoch)

    def _sync_native_version(self):
        goal, path_revision, map_epoch, envelope_epoch, localization_epoch, clock_epoch = \
            self._native_context.version()
        self._version = Version(goal, int(path_revision), int(map_epoch),
                                int(envelope_epoch), int(localization_epoch),
                                int(clock_epoch))

    def task(self, identifier, state, sequence):
        if self._native_context is not None:
            self._native_context.task(str(identifier), str(state), int(sequence))
            if sequence > self.sequence:
                self.sequence = sequence
            self._sync_native_version()
            return
        if sequence <= self.sequence:return
        self.sequence = sequence
        if state == 'EXECUTING':
            if identifier != self.version.goal_id:
                self.version = replace(self.version,goal_id=identifier,path_revision=0)
        elif state in ('SUCCEEDED','FAILED','CANCELED','PREEMPTED') and identifier == self.version.goal_id:
            self.version = replace(self.version,goal_id='idle')

    def map(self, metadata, cells):
        key = (metadata,hashlib.blake2b(cells,digest_size=16).digest())
        if self._native_context is not None:
            if self._native_context.map(repr(key)):
                self.map_key = key
                self._sync_native_version()
                return True
            return False
        if key != self.map_key:
            self.map_key = key
            self.version = replace(self.version,map_epoch=self.version.map_epoch+1)
            return True
        return False

    def localization(self, pose, position_limit, angle_limit):
        if (self._native_context is not None and len(pose) == 3 and
                all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in pose)):
            changed = self._native_context.localization(
                [float(value) for value in pose], float(position_limit),
                float(angle_limit))
            self.transform = pose
            if changed:
                self._sync_native_version()
            return bool(changed)
        previous = self.transform
        self.transform = pose
        if previous is not None and (math.dist(pose[:2],previous[:2]) > position_limit or
                abs(math.remainder(pose[2]-previous[2],2*math.pi)) > angle_limit):
            self.version = replace(self.version,localization_epoch=self.version.localization_epoch+1)
            return True
        return False

    def path(self):
        if self._native_context is not None:
            self._native_context.path()
            self._sync_native_version()
            return
        self.version = replace(self.version,path_revision=self.version.path_revision+1)


def to_wire(version, message):
    message.task_id = version.goal_id
    for field in ('path_revision','map_epoch','localization_epoch','envelope_epoch','clock_epoch'):
        setattr(message,field,getattr(version,field))
    return message
