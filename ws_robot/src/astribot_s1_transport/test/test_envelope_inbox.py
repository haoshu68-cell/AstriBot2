import copy
from astribot_navigation_msgs.msg import NavigationEnvelopeV2
from astribot_s1_transport.source_inbox import EnvelopeInbox


def message(stamp, until=1_300_000_000, epoch=1, allowed=True):
    m = NavigationEnvelopeV2()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
    m.valid_until.sec, m.valid_until.nanosec = divmod(until, 10**9)
    m.limits.stamp = copy.deepcopy(m.header.stamp)
    m.limits.lease_s = .3
    m.limits.transport_ready = m.navigation_allowed = allowed
    m.epoch = m.limits.epoch = epoch
    m.coordinator_session_id = 'coordinator'
    m.installed_geometry_hash = 'geometry'
    return m


def test_early_renewal_keeps_old_deadline_then_promotes():
    inbox = EnvelopeInbox()
    old = message(1_000_000_000)
    future = message(1_100_000_000, 1_400_000_000)
    inbox.receive(old); inbox.receive(future)
    assert inbox.select(1_099_000_000) is old
    assert inbox.select(1_100_000_000) is future
    assert future.header.stamp.nanosec == 100_000_000


def test_future_stream_never_extends_old_deadline():
    inbox = EnvelopeInbox()
    old = message(1_000_000_000)
    inbox.receive(old); inbox.receive(message(9_000_000_000, 9_300_000_000))
    assert inbox.select(1_299_999_999) is old
    assert inbox.select(1_300_000_000) is None


def test_revoke_and_changed_context_immediately_prevent_fallback():
    for mutate in (lambda m: setattr(m, 'navigation_allowed', False),
                   lambda m: setattr(m.limits, 'transport_ready', False),
                   lambda m: setattr(m, 'epoch', 2),
                   lambda m: setattr(m, 'installed_geometry_hash', 'changed'),
                   lambda m: setattr(m.limits, 'max_speed_m_s', .1)):
        inbox = EnvelopeInbox(); inbox.receive(message(1_000_000_000))
        future = message(1_100_000_000, 1_400_000_000); mutate(future)
        inbox.receive(future)
        immediate = not future.navigation_allowed or not future.limits.transport_ready
        assert inbox.select(1_099_000_000) is (future if immediate else None)
        assert inbox.select(1_100_000_000) is future


def test_shorter_deadline_no_fallback_and_no_clock_epoch_reuse():
    inbox = EnvelopeInbox(); inbox.receive(message(1_000_000_000))
    short = message(1_100_000_000, 1_150_000_000); inbox.receive(short)
    assert inbox.select(1_099_000_000) is None
    assert inbox.select(1_149_999_999) is short
    assert inbox.select(1_150_000_000) is None
    assert inbox.select(1_100_000_000) is None  # rollback clears acquired evidence


def test_bounded_queue_and_limits_timestamp_are_both_checked():
    inbox = EnvelopeInbox(); inbox.receive(message(1_000_000_000))
    for i in range(70): inbox.receive(message(9_000_000_000+i, 9_300_000_000+i))
    assert len(inbox.messages) == 64
    assert inbox.select(1_100_000_000) is None
    inbox = EnvelopeInbox(); m = message(1_000_000_000)
    m.limits.stamp.nanosec = 1
    inbox.receive(m)
    assert inbox.select(1_000_000_000) is None
    assert inbox.select(1_000_000_001) is m


def test_real_transport_guard_uses_coherent_selected_envelope():
    import time
    from types import SimpleNamespace as NS
    from rclpy.time import Time
    from astribot_s1_transport.ros_backend import RosBackend
    from astribot_s1_transport.core import TaskFailure
    import pytest
    inbox = EnvelopeInbox()
    old = message(1_000_000_000)
    future = message(1_100_000_000, 1_400_000_000)
    inbox.receive(old); inbox.receive(future)
    now = 1_099_000_000
    node = NS(cancel_requested=False, last_clock=0., clock_changed=time.monotonic(),
              get_clock=lambda: NS(now=lambda: Time(nanoseconds=now)),
              corridor_witness=None, sync_error='', execution_guard=None, attached=False,
              admitted=True, source_inboxes={}, odom=object(), scan=object(),
              joints=NS(name=[],position=[]), fresh=lambda m: True, c={},
              ledger=NS(stage='TRANSPORT'), fixed_v2=True, carry_joints={},
              envelope=future.limits, navigation_envelope=future, envelope_inbox=inbox)
    RosBackend.check(node)  # old valid permission, not the 1 ms early heartbeat
    future = message(1_200_000_000, 1_500_000_000, allowed=False)
    future.reason = 'ARM_HOLD_REVOKED'
    inbox.receive(future)
    with pytest.raises(TaskFailure, match='FIXED_V2_REVOKED:ARM_HOLD_REVOKED'):
        RosBackend.check(node)
