from types import SimpleNamespace
from unittest.mock import Mock
from rclpy.time import Time
from astribot_s1_chassis_effort_drive.omni_effort_drive_node import OmniEffortDriveNode, JOINT_NAMES


def node():
    n = SimpleNamespace(_target_vx=0., _target_vy=0., _target_wz=0.,
        _last_cmd_vel_time=Time(seconds=1), _last_joint_state_time=Time(seconds=1),
        _wheel_positions_valid=True, _idle_reference=None,
        _wheel_position={j:1. for j in JOINT_NAMES}, _wheel_velocity={j:0. for j in JOINT_NAMES},
        _loops={j:Mock(update=Mock(return_value=(0.,0.))) for j in JOINT_NAMES},
        _setpoint_pubs={j:Mock() for j in JOINT_NAMES}, _effort_debug_pubs={j:Mock() for j in JOINT_NAMES},
        _effort_pub=Mock(), _check_tracking_error=Mock(), get_logger=lambda:Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=1)),
        _safe_param=lambda k,d:True if k=='idle_position_hold' else d,
        _inverse_kinematics=lambda *args:{j:0. for j in JOINT_NAMES})
    return n


def test_stationary_reference_resists_displacement_and_saturates():
    n=node();OmniEffortDriveNode._control_step(n)
    assert n._idle_reference==n._wheel_position
    n._wheel_position={j:1.2 for j in JOINT_NAMES}
    OmniEffortDriveNode._control_step(n)
    assert all(abs(x+.6)<1e-9 for x in n._effort_pub.publish.call_args.args[0].data)
    n._wheel_position={j:100. for j in JOINT_NAMES}
    OmniEffortDriveNode._control_step(n)
    assert list(n._effort_pub.publish.call_args.args[0].data)==[-15.]*4


def test_motion_and_stale_feedback_release_reference():
    n=node();OmniEffortDriveNode._control_step(n)
    n._target_vx=.1;OmniEffortDriveNode._control_step(n);assert n._idle_reference is None
    n._target_vx=0.;OmniEffortDriveNode._control_step(n);assert n._idle_reference is not None
    n._last_joint_state_time=None;OmniEffortDriveNode._control_step(n)
    assert n._idle_reference is None
    assert list(n._effort_pub.publish.call_args.args[0].data)==[0.]*4


def test_no_latch_while_moving_or_position_missing():
    n=node();n._wheel_velocity[JOINT_NAMES[0]]=.1;OmniEffortDriveNode._control_step(n)
    assert n._idle_reference is None
    n._wheel_velocity[JOINT_NAMES[0]]=0.;n._wheel_positions_valid=False
    OmniEffortDriveNode._control_step(n);assert n._idle_reference is None


def test_default_disabled_and_new_stop_does_not_reuse_old_reference():
    n=node();n._safe_param=lambda k,d:d
    OmniEffortDriveNode._control_step(n);assert n._idle_reference is None
    n._safe_param=lambda k,d:True if k=='idle_position_hold' else d
    OmniEffortDriveNode._control_step(n)
    n._target_vx=.1;OmniEffortDriveNode._control_step(n)
    n._wheel_position={j:2. for j in JOINT_NAMES};n._target_vx=0.
    OmniEffortDriveNode._control_step(n)
    assert n._idle_reference==n._wheel_position
    assert list(n._effort_pub.publish.call_args.args[0].data)==[0.]*4


def test_incomplete_velocity_packet_cannot_enable_hold():
    n=node()
    message=SimpleNamespace(name=list(JOINT_NAMES),position=[1.]*4,velocity=[])
    OmniEffortDriveNode._joint_state_callback(n,message)
    assert not n._wheel_positions_valid
    OmniEffortDriveNode._control_step(n)
    assert n._idle_reference is None
