"""Transport transaction, independent of ROS; all evidence comes from a backend.

No automatic release on error. Recovery requires inspection of the retained ledger.
Resource ownership is cooperative: legacy direct controller clients must be absent.
"""
from dataclasses import dataclass, asdict
import fcntl
import json
import math
import os
import stat
from pathlib import Path
import time


class TaskFailure(RuntimeError):
    pass


class Canceled(TaskFailure):
    pass


def canonical_resource_domain(raw):
    if not raw.isascii() or not raw.isdecimal() or not 0 <= int(raw) <= 232 or str(int(raw)) != raw:
        raise TaskFailure('NONCANONICAL_ROS_DOMAIN')
    return raw


def check_native_resource_release(state_path):
    """Compatibility reader, called while holding the shared domain lock.

    A torn legacy marker must not bypass the native durable transaction.
    This does not provide execution ownership for the old fixed_v2 publisher.
    """
    path = Path(state_path)
    if not path.exists():
        return
    if path.is_symlink() or not path.is_file() or path.stat().st_size > 4 * 1024 * 1024:
        raise TaskFailure('NATIVE_RESOURCE_RECOVERY_REQUIRED')
    try:
        contents = path.read_text()
        if not contents or not contents.endswith('\n'):
            raise ValueError('incomplete journal')
        records = [json.loads(line) for line in contents.splitlines()]
        latest = records[-1]
        if not isinstance(latest, dict) or latest.get('schema') != 'astribot.resource/1' or type(latest.get('phase')) is not int or latest['phase'] != 0:
            raise ValueError('unreleased native resource')
    except (ValueError, TypeError, KeyError, OSError) as error:
        raise TaskFailure('NATIVE_RESOURCE_RECOVERY_REQUIRED') from error


class ResourceLease:
    """One host/domain lease for base, arms, head, torso and grippers."""
    def __init__(self, path):
        self.path = Path(path)
        self.file = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1:
            os.close(descriptor)
            raise TaskFailure('RESOURCE_FILE_INVALID')
        self.file = os.fdopen(descriptor, 'r+')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.close()
            raise TaskFailure('RESOURCES_BUSY')
        return self

    def __exit__(self, *args):
        fcntl.flock(self.file, fcntl.LOCK_UN)
        self.file.close()

    def checkpoint(self, record):
        # Keep the old protective marker until a complete replacement has been
        # synced. Never rename this file: flock is bound to its inode.
        content = json.dumps(record)
        self.file.seek(0)
        self.file.write(content)
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.truncate()
        self.file.flush()
        os.fsync(self.file.fileno())


@dataclass
class ObjectRecord:
    object_id: str
    version: int = 0
    state: str = 'WORLD'
    attachment: str = ''
    observation_source: str = 'configured_simulation_ground_truth'


class Ledger:
    def __init__(self, directory, object_id):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.object = ObjectRecord(object_id)
        self.sequence = 0
        self.stage = 'IDLE'

    @classmethod
    def resume_placed(cls, directory, object_id):
        directory = Path(directory)
        record = json.loads((directory / 'state.json').read_text())
        last = json.loads((directory / 'events.jsonl').read_text().splitlines()[-1])
        if (record != last or record['object']['object_id'] != object_id or
                record['object']['state'] != 'PLACED' or record['object']['attachment'] or
                record['stage'] not in ('FAULT', 'CANCELED')):
            raise TaskFailure('PLACED_RECOVERY_LEDGER_MISMATCH')
        ledger = cls(directory, object_id)
        ledger.sequence, ledger.stage = record['sequence'], record['stage']
        ledger.object = ObjectRecord(**record['object'])
        return ledger

    def emit(self, stage, **evidence):
        self.stage = stage
        self.sequence += 1
        record = dict(sequence=self.sequence, wall_time=time.time(), stage=stage,
                      object=asdict(self.object), **evidence)
        with (self.directory / 'events.jsonl').open('a') as stream:
            stream.write(json.dumps(record, allow_nan=False) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary = self.directory / 'state.json.tmp'
        temporary.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.directory / 'state.json')

    def transition(self, expected, new, attachment=''):
        if self.object.state != expected:
            raise TaskFailure('OBJECT_STATE_CONFLICT')
        self.object.state = new
        self.object.attachment = attachment
        self.object.version += 1
        self.emit(self.stage, transition=new)


def validate_scenario(config):
    if config.get('environment') != 'simulation':
        raise TaskFailure('SIMULATION_ONLY')
    if not config.get('object_id', '').replace('_', '').isalnum():
        raise TaskFailure('INVALID_OBJECT_ID')
    for key in ('pick_xyz', 'place_xyz', 'size_xyz', 'nav_goal'):
        values = config[key]
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            raise TaskFailure('INVALID_' + key.upper())
    for waypoint in config.get('nav_waypoints', []):
        if len(waypoint) != 3 or not all(math.isfinite(v) for v in waypoint):
            raise TaskFailure('INVALID_NAV_WAYPOINT')
    if config.get('nav_corridor'):
        from .corridor_route import CorridorRoute
        try:
            route=CorridorRoute(**config['nav_corridor'])
            final=route.via_poses()[-1]
            if (route.frame_id!=config['map_frame'] or config['base_frame']!='astribot_torso_base' or
                math.dist(final[:2],config['nav_goal'][:2])>1e-6 or
                abs(math.remainder(final[2]-config['nav_goal'][2],2*math.pi))>1e-6):
                raise ValueError('corridor endpoint/frame differs from scenario')
        except (TypeError,ValueError,KeyError) as error:
            raise TaskFailure('INVALID_NAV_CORRIDOR:'+str(error)) from error
    if any(v <= 0 for v in config['size_xyz']) or not math.isfinite(config['mass_kg']) or config['mass_kg'] <= 0:
        raise TaskFailure('INVALID_PAYLOAD')
    if len(config['orientation_xyzw']) != 4 or not all(math.isfinite(v) for v in config['orientation_xyzw']):
        raise TaskFailure('INVALID_ORIENTATION')
    if abs(sum(v*v for v in config['orientation_xyzw']) - 1.) > .001:
        raise TaskFailure('INVALID_ORIENTATION')
    if 'place_orientation_xyzw' in config:
        q = config['place_orientation_xyzw']
        if len(q) != 4 or not all(math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1.) > .001:
            raise TaskFailure('INVALID_PLACE_ORIENTATION')
    for key in ('placement_tolerance_m', 'observation_max_age_s', 'operation_timeout_s', 'navigation_timeout_s'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise TaskFailure('INVALID_' + key.upper())
    if not 0 < config['approach_m'] <= .3:
        raise TaskFailure('INVALID_APPROACH')
    if not 0 < config.get('retreat_inward_m', .08) <= .2:
        raise TaskFailure('INVALID_RETREAT')
    head = config.get('head_pick_joints', [0., 0.])
    if len(head) != 2 or any(not math.isfinite(v) or abs(v) > limit for v,limit in zip(head, (1.57, 1.22))):
        raise TaskFailure('INVALID_HEAD_TARGET')


class TransportTask:
    def __init__(self, backend, config, ledger):
        validate_scenario(config)
        self.io, self.config, self.ledger = backend, config, ledger

    def step(self, name, function, *args):
        self.io.check()
        self.ledger.emit(name)
        feedback = getattr(self.io, 'stage_feedback', lambda *args: None)
        try:
            result = function(*args)
            self.io.check()
            feedback(name, 'completed')
            return result
        except Exception as error:
            feedback(name, 'failed', str(error))
            raise

    def run(self):
        c, io, log = self.config, self.io, self.ledger
        try:
            self.step('ADMISSION', io.admit)
            self.step('MANIPULATION_HOLD', io.hold)
            self.step('SCENE_SYNC', io.create_scene)
            self.step('READY_RIGHT', io.skill, 'named', 'arm_right', 'transport_compact')
            self.step('READY_LEFT', io.skill, 'named', 'arm_left', 'ready')
            self.step('OPEN', io.skill, 'open')
            self.step('LOOK_PICK', io.look_pick)
            self.step('PERCEPTION_LOCATE', io.locate_object)
            self.step('PICK_PLAN', io.prepare_manipulation, 'PICK')
            self.step('PREGRASP', io.arm_pose, 'pick', c['approach_m'])
            self.step('GRASP_APPROACH', io.arm_pose, 'pick', 0.)
            self.step('GRASP_CONFIRM', io.skill, 'close')
            # A crash or canceled acknowledgement during attach is conservatively
            # journaled as unknown. No retry may spawn a duplicate payload.
            log.transition('WORLD', 'ATTACH_PENDING')
            self.step('ATTACH_CONFIRM', io.attach)
            log.transition('ATTACH_PENDING', 'ATTACHED', c['tcp'])
            self.step('LIFT', io.arm_pose, 'pick', c['approach_m'])
            self.step('TRANSPORT_POSTURE', io.skill, 'named', 'arm_left', 'transport_compact')
            self.step('ENVELOPE_CONFIRM', io.transport_envelope)
            self.step('TRANSPORT', io.navigate)
            self.step('PLACE_HOLD', io.hold)
            self.step('PLACE_PLAN', io.prepare_manipulation, 'PLACE')
            self.step('PREPLACE', io.arm_pose, 'place', c['approach_m'])
            self.step('PLACE_APPROACH', io.arm_pose, 'place', 0.)
            self.step('PLACE_CONFIRM', io.confirm_place)
            # Release is irreversible; record uncertainty BEFORE the command.
            log.transition('ATTACHED', 'RELEASE_PENDING', c['tcp'])
            self.step('RELEASE', io.skill, 'open')
            self.step('DETACH_CONFIRM', io.detach)
            log.transition('RELEASE_PENDING', 'PLACED')
            self.step('RETREAT', io.arm_pose, 'place', c['approach_m'], c.get('retreat_inward_m', .08))
            self.step('STOW', io.skill, 'named', 'arm_left', 'transport_compact')
            self.step('EMPTY_PAYLOAD_HOLD', io.hold)
            self.step('PLACEMENT_VERIFY', io.verify_place)
            # Leave the base held at the station. Next task must obtain a fresh
            # envelope handshake, including the now empty payload.
            log.emit('SUCCEEDED', evidence_level=io.evidence_level)
            return True
        except Exception as error:
            cleanup_error = ''
            try:
                io.stop_and_hold()
            except Exception as stop_error:
                cleanup_error = str(stop_error)
            log.emit('CANCELED' if isinstance(error, Canceled) else 'FAULT',
                     reason=str(error), stop_error=cleanup_error,
                     recovery_required=log.object.state != 'WORLD')
            return False

    def resume_placed(self):
        """Resume only the reversible tail after independent live reconciliation."""
        io, log, c = self.io, self.ledger, self.config
        try:
            if log.object.state != 'PLACED' or log.object.attachment:
                raise TaskFailure('PLACED_RECOVERY_REQUIRED')
            self.step('RECOVERY_ADMISSION', io.admit_recovery)
            self.step('RECOVERY_HOLD', io.hold)
            self.step('RETREAT', io.arm_pose, 'place', c['approach_m'], c.get('retreat_inward_m', .08))
            self.step('STOW', io.skill, 'named', 'arm_left', 'transport_compact')
            self.step('EMPTY_PAYLOAD_HOLD', io.hold)
            self.step('PLACEMENT_VERIFY', io.verify_place)
            log.emit('SUCCEEDED', evidence_level=io.evidence_level, recovered_placed_tail=True)
            return True
        except Exception as error:
            stop_error = ''
            try:io.stop_and_hold()
            except Exception as stopping:stop_error = str(stopping)
            log.emit('CANCELED' if isinstance(error, Canceled) else 'FAULT', reason=str(error),
                     stop_error=stop_error, recovery_required=True)
            return False
