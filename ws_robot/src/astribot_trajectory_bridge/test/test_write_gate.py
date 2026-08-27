#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WriteGate 与环境变量守卫的单元测试。

WriteGate 是 D-1 统一 domain 下**唯一**阻止"仿真指令打到真机"的机制，
所以它的每条拒绝路径都必须有测试。
"""

import pytest

from astribot_trajectory_bridge.write_gate import (
    ASTRIBOT_LOG_ENV,
    ROBOT_TYPE_ENV,
    ST_MULTIPLE_BACKENDS,
    ST_POSE_SOURCE_INVALID,
    ST_REAL_WRITE_NOT_AUTHORIZED,
    ST_ROBOT_MODE_UNEXPECTED,
    ST_SDK_NOT_ALIVE,
    ST_TARGET_MISMATCH,
    TARGET_REAL,
    TARGET_SIM,
    check_env_before_sdk_import,
    evaluate_write_gate,
    is_astribot_log_enabled,
    is_simulation_mode,
    validate_pose_source_target_combo,
)


class TestWriteGateBackendUniqueness:

    def test_single_matching_sim_backend_allowed(self):
        # 模式用实测值 'simulation'：仿真后端不会报 'safe'（那是真机模式），
        # 用 'safe' 配 sim 会被反向一致性正确地挡下。
        d = evaluate_write_gate(['sim'], TARGET_SIM, False, 'simulation')
        assert d.allowed, d.reason

    def test_no_backend_denied(self):
        d = evaluate_write_gate([], TARGET_SIM, False, 'safe')
        assert not d.allowed and d.status_code == ST_SDK_NOT_ALIVE

    def test_none_backend_denied(self):
        d = evaluate_write_gate(None, TARGET_SIM, False, 'safe')
        assert not d.allowed and d.status_code == ST_SDK_NOT_ALIVE

    def test_two_backends_denied(self):
        # D-1 下这会让指令同时打到两个后端 —— 这是选 D-1 后最危险的情形
        d = evaluate_write_gate(['sim', 'real'], TARGET_SIM, True, 'safe')
        assert not d.allowed and d.status_code == ST_MULTIPLE_BACKENDS

    def test_unknown_backend_denied(self):
        d = evaluate_write_gate(['mujoco'], TARGET_SIM, False, 'safe')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH


class TestWriteGateTargetMatch:

    def test_declared_sim_actual_real_denied(self):
        # 声明仿真、实际连到真机 —— 必须拒绝
        d = evaluate_write_gate(['real'], TARGET_SIM, True, 'safe')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH

    def test_declared_real_actual_sim_denied(self):
        d = evaluate_write_gate(['sim'], TARGET_REAL, True, 'safe')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH

    def test_invalid_declared_target_denied(self):
        d = evaluate_write_gate(['sim'], 'hardware', False, 'safe')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH


class TestWriteGateRealAuthorization:

    def test_real_without_authorization_denied(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, False, 'safe')
        assert not d.allowed
        assert d.status_code == ST_REAL_WRITE_NOT_AUTHORIZED

    def test_real_with_authorization_allowed(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, True, 'safe')
        assert d.allowed

    def test_sim_does_not_need_authorization(self):
        d = evaluate_write_gate(['sim'], TARGET_SIM, False, 'simulation')
        assert d.allowed, d.reason


class TestWriteGateRobotMode:
    """模式判定必须把"仿真"和"真机非安全模式"分成两件事。

    Gate 0 实测（2026-08-26，MuJoCo 后端）：仿真下 get_robot_mode() 返回
    ``'simulation'``；astribot_client.py:54-62 的判据是"不在
    safe/professional/extremity 三者中即为仿真"。

    若闸门只认 'safe'，仿真下会被判成"非安全模式"而拒绝写，使用者只能去开
    allow_unsafe_mode —— 而那个开关会连带把**真机的 professional/extremity
    一起放开**。所以下面两组断言必须同时成立。
    """

    # -- 真机侧：非 safe 必须挡住 --

    def test_real_professional_denied_by_default(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, True, 'professional')
        assert not d.allowed and d.status_code == ST_ROBOT_MODE_UNEXPECTED

    def test_real_extremity_denied_by_default(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, True, 'extremity')
        assert not d.allowed and d.status_code == ST_ROBOT_MODE_UNEXPECTED

    def test_real_non_safe_allowed_when_opted_in(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, True, 'professional',
                                allow_unsafe_mode=True)
        assert d.allowed

    def test_real_safe_allowed(self):
        d = evaluate_write_gate(['real'], TARGET_REAL, True, 'safe')
        assert d.allowed

    # -- 仿真侧：'simulation' 不是"非安全模式"，不许被这条挡下 --

    def test_simulation_mode_allowed_without_unsafe_flag(self):
        """!!! 核心断言 !!! 仿真下不许要求 allow_unsafe_mode。"""
        d = evaluate_write_gate(['sim'], TARGET_SIM, False, 'simulation')
        assert d.allowed, d.reason

    def test_simulation_unknown_string_also_allowed(self):
        """厂商判据是"不在三个真机模式里"，所以别的字符串同样算仿真。

        照抄判据而不是照抄当前返回值：将来仿真换个字符串也不会被误判成真机。
        """
        d = evaluate_write_gate(['sim'], TARGET_SIM, False, 'mujoco_v2')
        assert d.allowed, d.reason

    # -- 反向一致性：声明 sim 却连上真机 --

    def test_declared_sim_but_real_mode_denied(self):
        """比闸门②更强：②比"发现的后端"，这条比 SDK 自己报的模式。"""
        d = evaluate_write_gate(['sim'], TARGET_SIM, False, 'safe')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH

    def test_declared_sim_but_real_mode_denied_even_with_unsafe_flag(self):
        """allow_unsafe_mode 不许绕过"连错了机器"这件事。"""
        d = evaluate_write_gate(['sim'], TARGET_SIM, True, 'professional',
                                allow_unsafe_mode=True)
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH

    def test_declared_real_but_simulation_mode_denied(self):
        """声明 real 却连上仿真：应在闸门②被"实际后端"挡下。"""
        d = evaluate_write_gate(['sim'], TARGET_REAL, True, 'simulation')
        assert not d.allowed and d.status_code == ST_TARGET_MISMATCH


class TestIsSimulationMode:

    def test_three_real_modes_are_not_simulation(self):
        for m in ('safe', 'professional', 'extremity'):
            assert not is_simulation_mode(m), m

    def test_measured_simulation_value(self):
        assert is_simulation_mode('simulation')

    def test_unknown_counts_as_simulation_per_vendor_logic(self):
        assert is_simulation_mode('anything_else')

    def test_none_counts_as_simulation(self):
        """None 也落进 else —— 与厂商 if/elif/else 的行为一致。"""
        assert is_simulation_mode(None)


class TestPoseSourceTargetCombo:

    def test_ground_truth_on_real_denied(self):
        # ground_truth 依赖仿真侧静态真值 TF，真机上不存在
        d = validate_pose_source_target_combo('ground_truth', TARGET_REAL)
        assert not d.allowed and d.status_code == ST_POSE_SOURCE_INVALID

    def test_ground_truth_on_sim_allowed(self):
        assert validate_pose_source_target_combo('ground_truth', TARGET_SIM).allowed

    def test_slam_on_real_allowed(self):
        assert validate_pose_source_target_combo('slam', TARGET_REAL).allowed

    def test_slam_on_sim_allowed(self):
        assert validate_pose_source_target_combo('slam', TARGET_SIM).allowed


class TestAstribotLogEnv:
    """判据必须与 SDK 源码一致：astribot_interface.py:34 是**白名单**。"""

    @pytest.mark.parametrize('value', ['1', 'true', 'on', 'TRUE', 'On'])
    def test_truthy_values(self, value):
        assert is_astribot_log_enabled({ASTRIBOT_LOG_ENV: value})

    @pytest.mark.parametrize('value', ['', '0', 'yes', '2', 'enabled', 'true '])
    def test_falsy_values(self, value):
        # 'yes'/'2'/'true ' 都会被 SDK 判为 quiet —— 不能"非空即认为开了"
        assert not is_astribot_log_enabled({ASTRIBOT_LOG_ENV: value})

    def test_missing_is_falsy(self):
        assert not is_astribot_log_enabled({})


class TestEnvBeforeImport:

    GOOD = {ASTRIBOT_LOG_ENV: '1', ROBOT_TYPE_ENV: 'S1'}

    def test_good_env_passes(self):
        ok, problems = check_env_before_sdk_import(self.GOOD, loaded_modules=set())
        assert ok and problems == []

    def test_missing_astribot_log_reported(self):
        ok, problems = check_env_before_sdk_import(
            {ROBOT_TYPE_ENV: 'S1'}, loaded_modules=set())
        assert not ok
        assert any(ASTRIBOT_LOG_ENV in p for p in problems)

    def test_missing_robot_type_reported(self):
        ok, problems = check_env_before_sdk_import(
            {ASTRIBOT_LOG_ENV: '1'}, loaded_modules=set())
        assert not ok
        # 必须点明后果：chassis_dof 会是 2 而不是 3
        assert any('chassis_dof' in p for p in problems)

    def test_invalid_robot_type_reported(self):
        ok, problems = check_env_before_sdk_import(
            {ASTRIBOT_LOG_ENV: '1', ROBOT_TYPE_ENV: 'S2'}, loaded_modules=set())
        assert not ok

    def test_already_imported_reported(self):
        # 已 import 后再设环境变量已经来不及（fd 重定向在 import 时发生）
        ok, problems = check_env_before_sdk_import(
            self.GOOD, loaded_modules={'astribot_sdk'})
        assert not ok
        assert any('已经被 import' in p for p in problems)

    def test_middleware_import_also_detected(self):
        ok, problems = check_env_before_sdk_import(
            self.GOOD, loaded_modules={'astribot_ros_middleware'})
        assert not ok

    def test_all_problems_reported_together(self):
        # 不是报第一个就返回 —— 一次把所有问题列全，省一轮往返
        ok, problems = check_env_before_sdk_import({}, loaded_modules={'astribot_sdk'})
        assert not ok and len(problems) == 3
