#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F2 · 静止净偏航力矩 & F3 · 力矩饱和计数。

两项合一个脚本：都是「不发任何 cmd_vel，只看四个轮的力矩」，共用同一段采样。
分开跑两次会让第二次落在不同的物理状态上，反而不可比。

为什么这两项是一票否决
====================
* **F2**：曾经净偏航力矩 60 N·m，让车体自转 3.3 rad/s ——**完全没有 cmd_vel**。
  修复后实测 −0.098。一个静止时就在自转的底盘，A/B 组的每个数字都是错的。
* **F3**：曾经力矩饱和告警 1548 次，是轮 PID 发散成 ±15 N·m bang-bang 的直接指纹。
  修复后实测 0。

判据：|净偏航力矩| <= 0.5 N·m（修复后实测值 0.098 的 5 倍余量）；饱和计数 == 0。

净偏航力矩怎么算：用 wz 列做投影 —— wz 系数本身就是每个轮对偏航的力臂。
**轮序必须与节点一致**（RF, LF, RR, LR），错了会算成别的量且不报错，
所以轮序在 bench/kinematics.py 里有单测钉住。

用法::

    # 只起 warehouse_sim.launch.py，不要起 nav2；机器人静止放置
    python3 f_health/f2_static_yaw_torque.py --env sim [--duration 8]
"""

import os
import sys
import time

import rclpy
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import kinematics as kin                      # noqa: E402
from bench import stats as st                            # noqa: E402
from bench.session import (Session, base_arg_parser, results_dir,   # noqa: E402
                           run_main)

TEST_ID = 'f2_static_yaw_torque'

NET_YAW_THRESHOLD_NM = 0.5          # 修复后实测 −0.098；故障时 60
SATURATION_RATIO = 0.9              # 与节点的 warn_effort_ratio 一致
SATURATION_COUNT_THRESHOLD = 0      # 修复后实测 0；故障时 1548
# 静止时单轮力矩也应该很小。摩擦补偿的库仑项是 0.1 N·m，给 5 倍余量。
PER_WHEEL_THRESHOLD_NM = 0.5


def main(argv=None):
    p = base_arg_parser(TEST_ID, 'F2 静止净偏航力矩 + F3 力矩饱和计数（一票否决项）')
    p.add_argument('--duration', type=float, default=8.0, help='采样时长（墙钟秒）')
    args = p.parse_args(argv)

    if args.env != 'sim':
        print('F2/F3 只在 sim 下可测：真机的轮力矩不经过我们的节点，没有这两个话题。')
        return 2

    with Session(TEST_ID, args.env, force=args.force,
                 need_open_loop=True,       # 必须确认没人在发 cmd_vel
                 extra_context={
                     'wheel_order': list(kin.WHEEL_ORDER),
                     'effort_topics': list(kin.EFFORT_DEBUG_TOPICS),
                     'effort_limit_nm': kin.WHEEL_EFFORT_LIMIT_NM,
                     'saturation_ratio': SATURATION_RATIO,
                     'sample_duration_sec': args.duration,
                 }) as s:

        # 每个轮一个话题，各自缓存最新值；按"四个都更新过"组成一帧
        latest = {w: None for w in kin.WHEEL_ORDER}
        frames = []          # [(t, [RF, LF, RR, LR]), ...]

        def _make_cb(wheel):
            def _cb(msg):
                latest[wheel] = float(msg.data)
                if all(v is not None for v in latest.values()):
                    frames.append((time.monotonic(),
                                   [latest[w] for w in kin.WHEEL_ORDER]))
            return _cb

        subs = [s.node.create_subscription(Float64, topic, _make_cb(w), 100)
                for w, topic in zip(kin.WHEEL_ORDER, kin.EFFORT_DEBUG_TOPICS)]

        print('确认机器人静止、无人发 /cmd_vel。开始采样 '
              f'{args.duration:g}s ...')
        s.spin_for(1.0)          # 丢掉启动瞬态
        frames.clear()
        s.spin_for(args.duration)
        for sub in subs:
            s.node.destroy_subscription(sub)

        result = st.Result(TEST_ID, args.env,
                           description='静止状态下四轮力矩：净偏航 + 饱和计数',
                           context=s.context)
        result.note('净偏航力矩用 wz 列投影计算，wz 系数本身就是各轮的偏航力臂。'
                    '轮序 (RF, LF, RR, LR) 与节点一致，有单测钉住。')

        yaw_m = result.metric('净偏航力矩绝对值', 'N·m',
                              threshold=NET_YAW_THRESHOLD_NM, min_n=10,
                              threshold_basis='修复后实测 −0.098 N·m，取 5 倍余量；'
                                              '故障时是 60 N·m（车体自转 3.3 rad/s）')
        sat_m = result.metric('力矩饱和帧数', '帧',
                              threshold=SATURATION_COUNT_THRESHOLD, min_n=1,
                              threshold_basis='修复后实测 0；故障时 1548。'
                                              f'饱和定义 |τ| >= {SATURATION_RATIO:g} × '
                                              f'{kin.WHEEL_EFFORT_LIMIT_NM:g} N·m')
        wheel_m = result.metric('单轮力矩绝对值最大', 'N·m',
                                threshold=PER_WHEEL_THRESHOLD_NM, min_n=10,
                                threshold_basis='静止时应接近 0；库仑摩擦补偿项 0.1 N·m，'
                                                '取 5 倍余量')

        if len(frames) < 10:
            missing = [w for w, v in latest.items() if v is None]
            result.note(f'⚠️ 只组成 {len(frames)} 帧完整数据。'
                        + (f'这些轮的话题始终没有数据：{missing}。' if missing else '')
                        + '先 `ros2 topic list | grep wheel_effort` 确认话题存在，'
                          '再确认 omni_effort_drive_node 在跑。')
            print(result.to_markdown())
            paths = result.save(results_dir(args.results_dir))
            print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
            return 1

        sat_limit = SATURATION_RATIO * kin.WHEEL_EFFORT_LIMIT_NM
        sat_frames = 0
        for t, efforts in frames:
            yaw_m.add(abs(kin.net_yaw_moment(efforts)), t=t,
                      efforts=dict(zip(kin.WHEEL_ORDER, efforts)))
            wheel_m.add(max(abs(e) for e in efforts), t=t)
            if any(abs(e) >= sat_limit for e in efforts):
                sat_frames += 1
        sat_m.add(sat_frames, total_frames=len(frames))

        # 带符号的净偏航均值：符号能指出是哪个方向的偏置，修的时候有用
        signed = [kin.net_yaw_moment(e) for _, e in frames]
        signed_mean = sum(signed) / len(signed)
        result.note(f'净偏航力矩带符号均值 {signed_mean:+.4f} N·m'
                    f'（{len(frames)} 帧）。符号指出偏置方向。')
        if abs(signed_mean) > NET_YAW_THRESHOLD_NM:
            result.note('⚠️ 静止就有净偏航力矩。按历史经验先查两件事：'
                        '(1) F1 的控制环频率是否正常（掉频会让 PID 发散）；'
                        '(2) pid_kp 实际生效值是否 < 1.0、friction_viscous_nm_s '
                        '是否为 1.0 而不是 0.02 —— 参数静默回落是同一个根因。')
        if sat_frames > 0:
            result.note(f'⚠️ {sat_frames}/{len(frames)} 帧出现力矩饱和。'
                        '静止时饱和 = 轮 PID 发散成 bang-bang 的指纹，'
                        '不要继续往下测。')

        print(result.to_markdown())
        paths = result.save(results_dir(args.results_dir))
        print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
        return 0 if result.verdict == st.PASS else 1


if __name__ == '__main__':
    sys.exit(run_main(main))
