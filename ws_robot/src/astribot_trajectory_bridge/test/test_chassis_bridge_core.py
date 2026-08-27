#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘桥接状态机的分支测试。全部离线（无 SDK、无 rclpy、无仿真）。

这组测试的目标是把**每一条状态转移与每一条异常分支**都走到。在线验证需要真机、
且 leash/打滑这类分支在真机上不可控地复现，离线分支覆盖是唯一能穷尽它们的手段。
"""

import math

import pytest

from astribot_trajectory_bridge.chassis_integrator import ChassisConfigError
from astribot_trajectory_bridge.chassis_bridge_core import (
    ChassisBridgeConfig,
    ChassisBridgeCore,
    S_CMD_VEL_TIMEOUT,
    S_CORRECTION_DEGENERATE,
    S_LEASH_TRIPPED,
    S_ODOM_DRIFT_HIGH,
    S_SDK_CALL_FAILED,
    S_SLAM_LOST_STOPPED,
    S_SLAM_RELOCALIZED,
    S_SLAM_STALE,
    S_SLAM_UNAVAILABLE_OPEN_LOOP,
    ST_DISABLED,
    ST_ENABLED,
    ST_LEASH_TRIPPED,
    ST_STOPPED_NO_POSE,
)
from astribot_trajectory_bridge.ports import FakeClock, FakePose, FakeSession

PART = 'astribot_chassis'


def build(follow_ratio=1.0, **overrides):
    """搭一套 (core, session, pose, clock)。默认：位姿可用、闭环开、slam 源、
    实际位置理想跟随指令（follow_ratio=1.0，无打滑）。

    follow_ratio=0.0 模拟完全打滑（用来测 leash），0<r<1 模拟部分打滑。
    """
    clock = FakeClock(100.0)
    session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                          current={PART: [0.0, 0.0, 0.0]},
                          follow_ratio=follow_ratio)
    pose = FakePose(clock, pose=[0.0, 0.0, 0.0])
    cfg = ChassisBridgeConfig(**overrides)
    return (ChassisBridgeCore(cfg, session, pose, clock), session, pose, clock)


def codes(core):
    return [e.code for e in core.drain_events()]


class TestConfigValidation:

    def test_defaults_accepted(self):
        ChassisBridgeConfig()

    def test_zero_watchdog_rejected(self):
        with pytest.raises(ChassisConfigError) as e:
            ChassisBridgeConfig(cmd_vel_timeout_sec=0.0)
        assert '唯一止损' in str(e.value)

    def test_zero_leash_rejected(self):
        with pytest.raises(ChassisConfigError):
            ChassisBridgeConfig(leash_xy_m=0.0)

    def test_kp_over_limit_rejected(self):
        with pytest.raises(ChassisConfigError):
            ChassisBridgeConfig(kp_xy=0.9)

    def test_zero_corr_vel_rejected(self):
        with pytest.raises(ChassisConfigError):
            ChassisBridgeConfig(max_corr_vel_xy=0.0)

    def test_bad_input_frame_rejected(self):
        with pytest.raises(ChassisConfigError):
            ChassisBridgeConfig(input_frame='global')

    def test_bad_pose_source_rejected(self):
        with pytest.raises(ChassisConfigError):
            ChassisBridgeConfig(pose_source='amcl')

    def test_ground_truth_disables_jump_and_drift(self):
        cfg = ChassisBridgeConfig(pose_source='ground_truth')
        assert cfg.slam_jump_threshold_m == float('inf')
        assert cfg.odom_drift_warn_m == float('inf')


class TestEnableDisable:

    def test_starts_disabled(self):
        core, _, _, _ = build()
        assert core.state == ST_DISABLED

    def test_inner_tick_noop_when_disabled(self):
        core, session, _, _ = build()
        assert core.inner_tick() is False
        assert session.set_position_calls == []

    def test_enable_seeds_from_desired(self):
        core, session, _, _ = build()
        session.set_desired(PART, [1.5, -2.5, 0.3])
        ok, _ = core.enable()
        assert ok and core.state == ST_ENABLED
        assert core.pos_cmd == pytest.approx([1.5, -2.5, 0.3])

    def test_enable_reseeds_every_time(self):
        # 复用旧 pos_cmd 会在使能瞬间产生阶跃位置指令 -> 底盘猛冲
        core, session, _, _ = build()
        core.enable()
        core.submit_twist(1.0, 0, 0)
        for _ in range(50):
            core.inner_tick()
        moved = core.pos_cmd[0]
        assert moved > 0.0
        # 期间机器人被推走了
        session.set_desired(PART, [9.0, 9.0, 0.0])
        core.enable()
        assert core.pos_cmd == pytest.approx([9.0, 9.0, 0.0])

    def test_enable_fails_on_sdk_exception(self):
        core, session, _, _ = build()
        session.fail_after('get_desired_joints_position', 0)
        ok, detail = core.enable()
        assert not ok and core.state == ST_DISABLED
        assert S_SDK_CALL_FAILED in codes(core)

    def test_enable_rejects_wrong_dof(self):
        # ROBOT_TYPE 未设为 S1 时 chassis_dof=2（astribot_base.py:36-38）
        core, session, _, _ = build()
        session.set_desired(PART, [0.0, 0.0])
        ok, _ = core.enable()
        assert not ok
        evs = core.drain_events()
        assert any('ROBOT_TYPE' in e.detail for e in evs)

    def test_disable_stops_dispatch(self):
        core, session, _, _ = build()
        core.enable()
        core.disable()
        assert core.state == ST_DISABLED
        n = len(session.set_position_calls)
        core.inner_tick()
        assert len(session.set_position_calls) == n


class TestPoseAvailabilityAtEnable:

    def test_no_pose_open_loop_allowed_by_default(self):
        core, _, pose, _ = build(require_slam_to_enable=False)
        pose.set_pose(None)
        ok, _ = core.enable()
        assert ok and core.state == ST_ENABLED
        assert core.corr_frozen is True
        assert S_SLAM_UNAVAILABLE_OPEN_LOOP in codes(core)

    def test_no_pose_rejected_when_required(self):
        core, _, pose, _ = build(require_slam_to_enable=True)
        pose.set_pose(None)
        ok, _ = core.enable()
        assert not ok and core.state == ST_DISABLED

    def test_stale_pose_treated_as_unavailable(self):
        core, _, pose, _ = build(slam_max_age_sec=0.5)
        pose.stamp_offset = 2.0      # 位姿戳落后 2s
        ok, _ = core.enable()
        assert ok
        assert S_SLAM_STALE in codes(core)
        assert core.corr_frozen is True

    def test_ground_truth_reports_degenerate_at_enable(self):
        core, _, _, _ = build(pose_source='ground_truth')
        core.enable()
        assert S_CORRECTION_DEGENERATE in codes(core)


class TestWatchdog:

    def test_zeroes_velocity_after_timeout(self):
        core, _, _, clock = build(cmd_vel_timeout_sec=0.3,
                                  enable_slam_correction=False)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(10):
            core.inner_tick()
        before = core.pos_cmd[0]
        clock.advance(1.0)           # 超时
        for _ in range(10):
            core.inner_tick()
        # 超时后不应再前进（斜率限制会让它减速到 0，允许极小残余）
        assert core.pos_cmd[0] - before < 0.01
        assert S_CMD_VEL_TIMEOUT in codes(core)

    def test_stays_enabled_on_timeout(self):
        # cmd_vel 短暂中断是正常工况（Nav2 到点后就不发了），不该进故障态
        core, _, _, clock = build(enable_slam_correction=False)
        core.enable()
        core.submit_twist(0.5, 0, 0)
        core.inner_tick()
        clock.advance(5.0)
        core.inner_tick()
        assert core.state == ST_ENABLED

    def test_no_twist_ever_means_zero(self):
        core, session, _, _ = build(enable_slam_correction=False)
        core.enable()
        core.inner_tick()
        assert core.pos_cmd == pytest.approx([0.0, 0.0, 0.0])


class TestIntegrationAndDispatch:

    def test_one_meter_in_250_ticks(self):
        core, session, _, clock = build(enable_slam_correction=False,
                                        max_accel_xy=1e6)
        core.enable()
        for _ in range(250):
            core.submit_twist(1.0, 0.0, 0.0)
            core.inner_tick()
        assert core.pos_cmd[0] == pytest.approx(1.0, abs=1e-6)

    def test_dispatch_called_each_tick(self):
        core, session, _, _ = build(enable_slam_correction=False)
        core.enable()
        core.submit_twist(0.1, 0, 0)
        for _ in range(5):
            core.inner_tick()
        assert len(session.set_position_calls) == 5

    def test_dispatch_uses_part_name_and_three_dof(self):
        core, session, _, _ = build(enable_slam_correction=False)
        core.enable()
        core.submit_twist(0.1, 0, 0)
        core.inner_tick()
        names, pos, _, _, _ = session.set_position_calls[-1]
        assert names == [PART]
        assert len(pos[0]) == 3

    def test_sdk_failure_does_not_crash_or_disable(self):
        # 禁止吞异常：转成状态位；但也不能因单次失败退出（会让 cmd_vel 彻底断流）
        core, session, _, _ = build(enable_slam_correction=False)
        core.enable()
        core.submit_twist(0.1, 0, 0)
        session.fail_after('set_joints_position', 0)
        assert core.inner_tick() is False
        assert S_SDK_CALL_FAILED in codes(core)
        assert core.state == ST_ENABLED

    def test_read_actual_failure_reported(self):
        core, session, _, _ = build(enable_slam_correction=False)
        core.enable()
        session.fail_after('get_current_joints_position', 0)
        assert core.inner_tick() is False
        assert S_SDK_CALL_FAILED in codes(core)


class TestLeash:

    def test_trips_when_actual_lags(self):
        core, session, _, _ = build(follow_ratio=0.0, leash_xy_m=0.10,
                                    enable_slam_correction=False, max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        # follow_ratio=0.0：实际位置完全不动（完全打滑），指令一直加
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        assert core.state == ST_LEASH_TRIPPED
        assert S_LEASH_TRIPPED in codes(core)

    def test_freezes_both_integration_and_correction(self):
        # 只冻结积分而让外环继续推，等于 leash 没起作用
        core, session, _, _ = build(follow_ratio=0.0, leash_xy_m=0.05,
                                    max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        assert core.corr_frozen is True
        assert core.corr_per_tick == pytest.approx((0.0, 0.0, 0.0))

    def test_command_pulled_back_to_actual(self):
        core, session, _, _ = build(follow_ratio=0.0, leash_xy_m=0.05,
                                    enable_slam_correction=False, max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        session.set_current(PART, [0.3, 0.2, 0.0])
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        assert core.pos_cmd == pytest.approx([0.3, 0.2, 0.0])

    def test_no_dispatch_while_tripped(self):
        core, session, _, _ = build(follow_ratio=0.0, leash_xy_m=0.05,
                                    enable_slam_correction=False, max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        n = len(session.set_position_calls)
        for _ in range(10):
            core.inner_tick()
        assert len(session.set_position_calls) == n

    def test_reset_leash_requires_tripped_state(self):
        core, _, _, _ = build()
        core.enable()
        ok, detail = core.reset_leash()
        assert not ok and 'LEASH_TRIPPED' in detail

    def test_reset_leash_reseeds_and_reenables(self):
        core, session, _, _ = build(follow_ratio=0.0, leash_xy_m=0.05,
                                    enable_slam_correction=False, max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        session.set_desired(PART, [4.0, 5.0, 0.0])
        ok, _ = core.reset_leash()
        assert ok and core.state == ST_ENABLED
        assert core.pos_cmd == pytest.approx([4.0, 5.0, 0.0])

    def test_theta_only_trip(self):
        core, session, _, _ = build(follow_ratio=0.0, leash_theta_rad=0.1,
                                    enable_slam_correction=False,
                                    max_accel_xy=1e6, max_accel_theta=1e6)
        core.enable()
        core.submit_twist(0.0, 0.0, 1.0)
        for _ in range(200):
            if core.state == ST_LEASH_TRIPPED:
                break
            core.inner_tick()
        assert core.state == ST_LEASH_TRIPPED


class TestOuterLoop:

    def test_noop_when_correction_disabled(self):
        core, _, pose, _ = build(enable_slam_correction=False)
        core.enable()
        pose.set_pose([5.0, 0.0, 0.0])       # 巨大误差
        core.outer_tick()
        assert core.corr_per_tick == pytest.approx((0.0, 0.0, 0.0))

    def test_zero_error_zero_correction(self):
        core, _, _, _ = build()
        core.enable()
        core.outer_tick()
        assert core.corr_per_tick == pytest.approx((0.0, 0.0, 0.0))

    def test_drift_produces_correction(self):
        core, session, pose, clock = build(max_accel_xy=1e6)
        core.enable()
        # 指令走了 0.4m，但位姿源说只走了 0.1m -> 应产生正向校正
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(100):
            core.inner_tick()
        pose.set_pose([0.1, 0.0, 0.0])
        core.outer_tick()
        assert core.corr_per_tick[0] > 0.0

    def test_correction_is_sliced(self):
        core, _, pose, _ = build(freq=250.0, outer_rate=10.0, max_accel_xy=1e6)
        core.enable()
        core.submit_twist(1.0, 0.0, 0.0)
        for _ in range(100):
            core.inner_tick()
        pose.set_pose([0.0, 0.0, 0.0])
        core.outer_tick()
        # 单拍增量必须远小于一个外环周期的总校正量
        assert abs(core.corr_per_tick[0]) <= 0.10 / 10.0 / 25.0 + 1e-12

    def test_pose_lost_freezes_correction(self):
        core, _, pose, _ = build()
        core.enable()
        pose.set_pose(None)
        core.outer_tick()
        assert core.corr_frozen is True
        assert core.corr_per_tick == pytest.approx((0.0, 0.0, 0.0))

    def test_relocalization_reseeds_instead_of_correcting(self):
        core, _, pose, _ = build(slam_jump_threshold_m=0.30)
        core.enable()
        pose.set_pose([0.05, 0.0, 0.0])
        core.outer_tick()
        core.drain_events()
        pose.set_pose([1.00, 0.0, 0.0])      # 跳变 0.95m
        core.outer_tick()
        assert S_SLAM_RELOCALIZED in codes(core)
        assert core.corr_per_tick == pytest.approx((0.0, 0.0, 0.0))
        assert core._p_des_map == pytest.approx([1.00, 0.0, 0.0])

    def test_ground_truth_never_relocalizes(self):
        core, _, pose, _ = build(pose_source='ground_truth')
        core.enable()
        core.drain_events()
        pose.set_pose([0.0, 0.0, 0.0])
        core.outer_tick()
        pose.set_pose([50.0, 0.0, 0.0])
        core.outer_tick()
        assert S_SLAM_RELOCALIZED not in codes(core)

    def test_correction_resumes_after_pose_returns(self):
        core, _, pose, _ = build()
        core.enable()
        pose.set_pose(None)
        core.outer_tick()
        assert core.corr_frozen is True
        pose.set_pose([0.0, 0.0, 0.0])
        core.outer_tick()
        assert core.corr_frozen is False


class TestRequireSlamStopBehaviour:
    """require_slam_to_enable=true 的商业化行为（现在就固定，不留到将来设计）。"""

    def test_stops_after_grace_period(self):
        core, _, pose, clock = build(require_slam_to_enable=True,
                                     slam_loss_grace_sec=2.0)
        core.enable()
        pose.set_pose(None)
        core.outer_tick()
        assert core.state == ST_ENABLED      # 宽限期内不停车
        clock.advance(3.0)
        core.outer_tick()
        assert core.state == ST_STOPPED_NO_POSE
        assert S_SLAM_LOST_STOPPED in codes(core)

    def test_stopped_state_blocks_dispatch(self):
        core, session, pose, clock = build(require_slam_to_enable=True,
                                           slam_loss_grace_sec=0.5)
        core.enable()
        pose.set_pose(None)
        core.outer_tick()
        clock.advance(1.0)
        core.outer_tick()
        n = len(session.set_position_calls)
        core.submit_twist(1.0, 0, 0)
        core.inner_tick()
        assert len(session.set_position_calls) == n

    def test_open_loop_config_never_stops(self):
        core, _, pose, clock = build(require_slam_to_enable=False)
        core.enable()
        pose.set_pose(None)
        for _ in range(5):
            clock.advance(2.0)
            core.outer_tick()
        assert core.state == ST_ENABLED


class TestDriftDiagnostic:

    def test_reports_when_sdk_moves_more_than_pose(self):
        core, session, pose, clock = build(follow_ratio=0.0, odom_drift_window_sec=1.0,
                                          odom_drift_warn_m=0.10,
                                          max_accel_xy=1e6)
        core.enable()
        core.drain_events()
        # SDK 实际走了 1.0m，位姿源只走了 0.5m -> 打滑 0.5m
        for i in range(4):
            session.set_current(PART, [0.25 * (i + 1), 0.0, 0.0])
            pose.set_pose([0.125 * (i + 1), 0.0, 0.0])
            core.outer_tick()
            clock.advance(0.3)
        assert S_ODOM_DRIFT_HIGH in codes(core)

    def test_no_report_when_consistent(self):
        core, session, pose, clock = build(follow_ratio=0.0, odom_drift_window_sec=1.0,
                                          odom_drift_warn_m=0.10,
                                          max_accel_xy=1e6)
        core.enable()
        core.drain_events()
        for i in range(4):
            session.set_current(PART, [0.1 * (i + 1), 0.0, 0.0])
            pose.set_pose([0.1 * (i + 1), 0.0, 0.0])
            core.outer_tick()
            clock.advance(0.3)
        assert S_ODOM_DRIFT_HIGH not in codes(core)

    def test_ground_truth_never_warns(self):
        core, session, pose, clock = build(follow_ratio=0.0, pose_source='ground_truth',
                                           odom_drift_window_sec=1.0,
                                           odom_drift_warn_m=0.10)
        core.enable()
        core.drain_events()
        for i in range(4):
            session.set_current(PART, [1.0 * (i + 1), 0.0, 0.0])
            pose.set_pose([0.0, 0.0, 0.0])
            core.outer_tick()
            clock.advance(0.3)
        assert S_ODOM_DRIFT_HIGH not in codes(core)


class TestPosePortContractViolation:
    """位姿源**抛异常**时的归因。

    ``PosePort.lookup()`` 的契约是"查不到返回 ``(None, None)``、不抛"。
    真抛了说明位姿源实现有缺陷（签名不匹配、TF 缓冲未初始化、依赖未就绪……）。

    这组测试锁两件事：
    1. 报的是 ``POSE_PORT_FAILED`` 而**不是** ``SDK_CALL_FAILED`` ——
       位姿查询不是 SDK 调用，报错指向错误子系统会把诊断带偏（本项目已多次
       为此付代价，见 memory: claim-root-cause-only-with-arithmetic）；
    2. 也**不**直接折进 ``SLAM_UNAVAILABLE_OPEN_LOOP`` —— 那会把一个真 bug
       伪装成正常的降级工况，属于静默失败。

    这个缺陷是 Gate 1 实测时由一个写错签名的探针位姿源暴露出来的：
    当时报的是 SDK_CALL_FAILED，而 SDK 根本没参与。
    """

    class RaisingPose:
        """契约违约的位姿源：lookup() 抛异常而不是返回 (None, None)。"""

        def __init__(self, exc=None):
            self.exc = exc or RuntimeError('TF buffer 未初始化')
            self.calls = 0

        def lookup(self):
            self.calls += 1
            raise self.exc

    def _build_with_raising_pose(self, **overrides):
        clock = FakeClock(100.0)
        session = FakeSession(desired={PART: [0.0, 0.0, 0.0]},
                             current={PART: [0.0, 0.0, 0.0]},
                             follow_ratio=1.0)
        pose = self.RaisingPose()
        cfg = ChassisBridgeConfig(**overrides)
        return (ChassisBridgeCore(cfg, session, pose, clock), session, pose, clock)

    def test_reports_pose_port_failed_not_sdk_call_failed(self):
        core, _, pose, _ = self._build_with_raising_pose()
        core.enable()
        core.drain_events()
        core.outer_tick()
        got = codes(core)
        assert 'POSE_PORT_FAILED' in got, got
        assert S_SDK_CALL_FAILED not in got, (
            '位姿源异常被报成了 SDK 故障，会把诊断引向机器人/SDK：%r' % (got,))
        assert pose.calls > 0

    def test_detail_carries_exception_text(self):
        """detail 必须带上异常文本，否则无法定位是哪种契约违约。"""
        core, _, _, _ = self._build_with_raising_pose()
        core.enable()
        core.drain_events()
        core.outer_tick()
        ev = [e for e in core.drain_events() if e.code == 'POSE_PORT_FAILED']
        assert ev and 'TF buffer 未初始化' in ev[0].detail

    def test_does_not_crash_and_keeps_running(self):
        """契约违约不许让节点挂掉 —— 报出来，降级继续跑。"""
        core, _, _, _ = self._build_with_raising_pose()
        core.enable()
        for _ in range(5):
            core.outer_tick()
            core.submit_twist(0.05, 0.0, 0.0)
            core.inner_tick()
        assert core.state == ST_ENABLED

    def test_new_code_is_mapped_in_msg(self):
        """新枚举必须真的在 BridgeStatus.msg 里，否则上报时会抛。"""
        pytest.importorskip(
            'astribot_bridge_msgs.msg',
            reason='需要先 colcon build astribot_bridge_msgs')
        from astribot_bridge_msgs.msg import BridgeStatus
        assert hasattr(BridgeStatus, 'POSE_PORT_FAILED')
