#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F1 · 控制环真实频率。

**这是整套测试的第一项，不过就停下来修。**

为什么它排第一：历史上 ``control_period_sec`` 被参数泄漏设成 0.5，把 100 Hz 变成
1.85 Hz，连带 PID/摩擦参数静默回落到已知发散的默认值，制造出「底盘爬行 2 cm/s +
MPPI 求解失败 + 控制器掉频」三个看似无关的症状。修复后实测 95 Hz。
带着一个掉频的控制环去解释 A~E 组的任何数字都是浪费时间。

测法：数 ``/wheel_effort_controller/commands`` 的到达间隔。这个话题是控制环每周期
必发的，所以它的频率就是控制环频率。**不看节点自报的参数值**——参数值对不上实际
频率恰恰是那次故障的核心（yaml 写着 0.01，实际跑 0.5）。

顺带查两件同源的事：
* 节点实际加载的 params 文件路径（参数泄漏的直接证据）
* 关键参数的实际生效值（pid_kp、friction_viscous_nm_s）

用法::

    python3 f_health/f1_control_loop_rate.py --env sim [--duration 10]
"""

import os
import statistics
import sys
import time

import rclpy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bench import kinematics as kin                      # noqa: E402
from bench import stats as st                            # noqa: E402
from bench.session import (Session, base_arg_parser, results_dir,   # noqa: E402
                           run_main)

TEST_ID = 'f1_control_loop_rate'
EFFORT_TOPIC = '/wheel_effort_controller/commands'

# 标称 100 Hz（control_period_sec 0.01）。判据 95 Hz = 修复后的实测值。
NOMINAL_HZ = 1.0 / kin.CONTROL_PERIOD_SEC
RATE_THRESHOLD_HZ = 95.0
# 单周期抖动：超过 2 个标称周期算一次迟到。控制环是硬实时循环，不该有这种空档。
LATE_FACTOR = 2.0
LATE_RATIO_THRESHOLD = 0.01      # 迟到比例上限 1%


def main(argv=None):
    p = base_arg_parser(TEST_ID, 'F1 控制环真实频率（一票否决项）')
    p.add_argument('--duration', type=float, default=10.0, help='采样时长（墙钟秒）')
    args = p.parse_args(argv)

    if args.env != 'sim':
        print('F1 目前只在 sim 下实现：真机的力矩话题不由我们发布，'
              '真机等效项应改为量测桥接节点的下发间隔（见 c_interface/）。')
        return 2

    with Session(TEST_ID, args.env, force=args.force,
                 need_open_loop=False,     # 本项不发指令，不占 /cmd_vel
                 extra_context={'effort_topic': EFFORT_TOPIC,
                                'nominal_hz': NOMINAL_HZ,
                                'sample_duration_sec': args.duration}) as s:

        stamps = []

        def _cb(_msg):
            stamps.append(time.monotonic())

        sub = s.node.create_subscription(Float64MultiArray, EFFORT_TOPIC, _cb, 200)

        print(f'采样 {EFFORT_TOPIC} {args.duration:g}s ...')
        # 先丢掉最初 1s，避开启动瞬态
        s.spin_for(1.0)
        stamps.clear()
        s.spin_for(args.duration)
        s.node.destroy_subscription(sub)

        result = st.Result(TEST_ID, args.env,
                           description='由力矩指令话题的到达间隔反推控制环频率',
                           context=s.context)
        result.note(f'判据 {RATE_THRESHOLD_HZ:g} Hz 是那次参数泄漏修复后的实测值'
                    f'（1.85 -> 95 Hz）；标称 {NOMINAL_HZ:g} Hz。')
        result.note('刻意不读节点自报的 control_period_sec：参数写的值与实际频率'
                    '不一致，恰恰是那次故障的核心。')

        rate_m = result.metric('控制环频率', 'Hz', threshold=RATE_THRESHOLD_HZ,
                               direction=st.MORE_IS_BETTER, min_n=1,
                               threshold_basis='参数泄漏修复后实测 95 Hz；'
                                               'control_period_sec=0.01 标称 100 Hz')
        late_m = result.metric('迟到周期比例', '', threshold=LATE_RATIO_THRESHOLD,
                               direction=st.LESS_IS_BETTER, min_n=1,
                               threshold_basis=f'间隔 > {LATE_FACTOR:g}x 标称周期算迟到；'
                                               f'硬实时循环不该有这种空档')

        if len(stamps) < 10:
            result.note(f'⚠️ {args.duration:g}s 内只收到 {len(stamps)} 条消息。'
                        f'要么控制环没在跑，要么话题名不对。'
                        f'先 `ros2 topic hz {EFFORT_TOPIC}` 手工确认。')
            print(result.to_markdown())
            paths = result.save(results_dir(args.results_dir))
            print(f'\n结果已写入：\n  {paths[0]}\n  {paths[1]}')
            return 1

        gaps = [stamps[i + 1] - stamps[i] for i in range(len(stamps) - 1)]
        measured_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        rate_m.add(measured_hz, n_msgs=len(stamps),
                   span_sec=stamps[-1] - stamps[0])

        late = sum(1 for g in gaps if g > LATE_FACTOR * kin.CONTROL_PERIOD_SEC)
        late_m.add(late / len(gaps), late_count=late, gap_count=len(gaps))

        # 只记录、不设判据的诊断量
        jitter = result.metric('周期抖动 σ', 's')
        jitter.add(statistics.stdev(gaps) if len(gaps) >= 2 else 0.0)
        worst = result.metric('最大周期间隔', 's')
        worst.add(max(gaps))

        result.note(f'实测 {measured_hz:.2f} Hz（{len(stamps)} 条 / '
                    f'{stamps[-1] - stamps[0]:.2f} s），最大间隔 {max(gaps) * 1000:.1f} ms。')
        if measured_hz < RATE_THRESHOLD_HZ:
            result.note('⚠️ 频率不达标。**先查参数泄漏**：'
                        '`pgrep -af omni_effort_drive_node | grep -o "params-file [^ ]*"`，'
                        '确认它加载的不是协调器的 yaml。'
                        '同时检查每个 IncludeLaunchDescription 是否都显式传了 params_file'
                        '——第一个 include 会占住共享名字，后面的静默加载错的文件。')

        print(result.to_markdown())
        paths = result.save(results_dir(args.results_dir))
        print(f'结果已写入：\n  {paths[0]}\n  {paths[1]}')
        return 0 if result.verdict == st.PASS else 1


if __name__ == '__main__':
    sys.exit(run_main(main))
