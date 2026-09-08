#!/usr/bin/env python3
# Copyright 2026 Astribot.
"""从 rosbag2 里量「机头朝向 − 路径局部切向」误差。

这是路线 A（把 MPPI 的横移自由度掐掉，让机头必须转到路径方向）的**唯一**
有效验收判据。为什么不能用蟹行角 ``atan2(vy, vx)``：改完之后 vy 恒为 0，
蟹行角按定义恒等于 0，量它等于量自己的输入 —— 任何配置都会"通过"。
蟹行角只在**改之前**有诊断价值（它能证明机器人在横着走）。

位姿来源是 /tf 里 map -> odom -> <base> 两级复合，不是 /odom（那是 odom 系，
而 /plan 在 map 系，两者在 SLAM 下不重合）。

用法:
    python3 tools/measure_heading_vs_tangent.py <bag目录或db3> [选项]
"""
import argparse
import bisect
import os
import sqlite3
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, 'ws_robot', 'src', 'astribot_s1_navigation'))

DEFAULT_BASE_FRAME = 'astribot_torso_base'
# 平移判据：低于这个速度模长的样本按"没在走"处理。与 explore_metrics 的
# DEFAULT_MIN_STEP_M 无关 —— 这是速度门限不是位移门限。
DEFAULT_MOVING_MPS = 0.02
# 离路径超过这个距离时，"最近段切向"已经不代表机器人该朝的方向了。
DEFAULT_MAX_DIST_M = 0.60


def _find_db3(path):
    if os.path.isfile(path):
        return path
    hits = [os.path.join(path, f) for f in sorted(os.listdir(path))
            if f.endswith('.db3')]
    if not hits:
        raise SystemExit('在 %s 下没找到 .db3' % path)
    if len(hits) > 1:
        print('⚠ 找到 %d 个 db3，只读第一个: %s' % (len(hits), hits[0]))
    return hits[0]


def _yaw(q):
    return np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _compose(parent, child):
    """两级 2D 变换复合：返回 (x, y, yaw)。只取平面分量。"""
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c, s = np.cos(pyaw), np.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw)


def read_bag(db3, base_frame, map_frame='map', odom_frame='odom'):
    """返回 (poses, plans, cmds)。

    poses  [(t, x, y, yaw)]  map 系，按 odom->base 的每条 tf 出一个样本
    plans  [(t, Nx2 ndarray)]
    cmds   [(t, vx, vy, wz)]
    """
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from tf2_msgs.msg import TFMessage

    # ⚠必须只读且**不加锁**打开：普通 sqlite3.connect() 会去拿写锁，正在录制的
    # rosbag2 下一次提交就抛 `SqliteException: database is locked` 并**整个退出**。
    # 2026-09-03 实测踩过：我用本工具读一个还在录的 bag 做"中途快照"，把录制器
    # 杀了，末尾 61.1s（约 2 轮）永久丢失，而症状极其隐蔽——快照与"终版"读数
    # 逐字相同，看起来像"数据没变化"，不像"录制已经死了"。
    # mode=ro 只挡写，nolock=1 才是关键：不建任何锁文件、不碰 WAL。
    con = sqlite3.connect('file:%s?mode=ro&nolock=1' % os.path.abspath(db3),
                          uri=True)
    topics = {name: tid for tid, name in con.execute('select id,name from topics')}
    for need in ('/tf', '/plan'):
        if need not in topics:
            raise SystemExit('bag 里没有 %s，量不出朝向误差' % need)

    poses, plans, cmds = [], [], []
    latest_map_odom = None
    rows = con.execute(
        'select topic_id,timestamp,data from messages where topic_id in (?,?,?)'
        ' order by timestamp',
        (topics['/tf'], topics['/plan'], topics.get('/cmd_vel', -1)))
    for tid, ts, data in rows:
        t = ts / 1e9
        if tid == topics['/tf']:
            msg = deserialize_message(bytes(data), TFMessage)
            for tr in msg.transforms:
                p, c = tr.header.frame_id, tr.child_frame_id
                if p == map_frame and c == odom_frame:
                    latest_map_odom = (tr.transform.translation.x,
                                       tr.transform.translation.y,
                                       float(_yaw(tr.transform.rotation)))
                elif p == odom_frame and c == base_frame:
                    if latest_map_odom is None:
                        continue      # map->odom 还没来，这一拍算不出 map 系位姿
                    ob = (tr.transform.translation.x,
                          tr.transform.translation.y,
                          float(_yaw(tr.transform.rotation)))
                    x, y, yaw = _compose(latest_map_odom, ob)
                    poses.append((t, x, y, yaw))
        elif tid == topics['/plan']:
            msg = deserialize_message(bytes(data), Path)
            if len(msg.poses) >= 2:
                plans.append((t, np.array(
                    [(p.pose.position.x, p.pose.position.y) for p in msg.poses])))
        else:
            msg = deserialize_message(bytes(data), Twist)
            cmds.append((t, msg.linear.x, msg.linear.y, msg.angular.z))
    return poses, plans, cmds


def analyse(poses, plans, cmds, tangent_step, max_dist, moving_mps):
    from astribot_s1_navigation.explore_metrics import geometry as g

    if not plans:
        raise SystemExit('bag 里没有顶点数 >= 2 的 /plan')
    plan_t = [t for t, _ in plans]
    cmd_t = [t for t, _, _, _ in cmds]

    # 按"当时在跟的那条路径"把位姿分组 —— 逐样本重算折线抽稀会慢一个数量级。
    groups = {}
    for t, x, y, yaw in poses:
        k = bisect.bisect_right(plan_t, t) - 1
        if k < 0:
            continue                  # 第一条路径之前的位姿没有参照
        groups.setdefault(k, []).append((t, x, y, yaw))

    err_all, dist_all, spd_all = [], [], []
    for k, samples in sorted(groups.items()):
        arr = np.array(samples)
        e, d = g.heading_tangent_errors(
            arr[:, 1:3], arr[:, 3], plans[k][1], tangent_step_m=tangent_step)
        err_all.append(e)
        dist_all.append(d)
        if cmds:
            spd = []
            for t in arr[:, 0]:
                j = bisect.bisect_right(cmd_t, t) - 1
                spd.append(np.hypot(cmds[j][1], cmds[j][2]) if j >= 0 else np.nan)
            spd_all.append(np.array(spd))
        else:
            spd_all.append(np.full(len(arr), np.nan))

    err = np.concatenate(err_all)
    dist = np.concatenate(dist_all)
    spd = np.concatenate(spd_all)
    return err, dist, spd, len(groups)


def report(err, dist, spd, n_plans, max_dist, moving_mps, label):
    deg = np.degrees(np.abs(err))
    finite = np.isfinite(deg)
    near = finite & (dist <= max_dist)
    moving = near & np.isfinite(spd) & (spd >= moving_mps)

    print('=' * 68)
    print('机头 − 路径切向 误差   %s' % label)
    print('=' * 68)
    print('【数据可用性】不通过的行后面的数一律不作为结论')
    print('  位姿样本(可配对到某条 /plan)      %d' % len(deg))
    print('  参与的 /plan 条数                 %d' % n_plans)
    print('  切向可算(非 nan)                  %d (%.1f%%)'
          % (finite.sum(), 100.0 * finite.mean() if len(deg) else 0.0))
    print('  且离路径 <= %.2fm                  %d (%.1f%%)'
          % (max_dist, near.sum(), 100.0 * near.mean() if len(deg) else 0.0))
    print('  且速度 >= %.2fm/s (真在走)         %d (%.1f%%)'
          % (moving_mps, moving.sum(), 100.0 * moving.mean() if len(deg) else 0.0))
    if moving.sum() < 100:
        print('  🔴 平移中样本不足 100，分位数不可靠')

    for name, mask in (('全部近路径样本', near), ('仅平移中样本', moving)):
        sub = deg[mask]
        if sub.size == 0:
            print('\n【%s】无样本' % name)
            continue
        print('\n【%s】n=%d  单位: 度' % (name, sub.size))
        qs = [50, 75, 90, 95, 99]
        line = '  ' + '  '.join('P%-2d %6.2f' % (q, np.percentile(sub, q))
                                for q in qs)
        print(line)
        print('  mean %6.2f   max %6.2f' % (sub.mean(), sub.max()))
        print('  >57.3°(死区边界) %5.1f%%   >90°(在倒着走) %5.1f%%'
              % (100.0 * (sub > 57.3).mean(), 100.0 * (sub > 90.0).mean()))
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--base-frame', default=DEFAULT_BASE_FRAME)
    ap.add_argument('--tangent-step', type=float, default=None,
                    help='折线抽稀步长(m)，默认取 geometry 里的 0.20')
    ap.add_argument('--max-dist', type=float, default=DEFAULT_MAX_DIST_M)
    ap.add_argument('--moving', type=float, default=DEFAULT_MOVING_MPS)
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    from astribot_s1_navigation.explore_metrics import geometry as g
    step = g.DEFAULT_TANGENT_STEP_M if args.tangent_step is None else args.tangent_step

    db3 = _find_db3(args.bag)
    poses, plans, cmds = read_bag(db3, args.base_frame)
    print('读到: 位姿 %d  路径 %d  cmd_vel %d   (切向基线 %.2fm)'
          % (len(poses), len(plans), len(cmds), step))
    if not poses:
        raise SystemExit(
            '一个 map 系位姿都没算出来。检查 --base-frame（当前 %s）'
            % args.base_frame)
    err, dist, spd, n = analyse(poses, plans, cmds, step, args.max_dist, args.moving)
    report(err, dist, spd, n, args.max_dist, args.moving,
           args.label or os.path.basename(os.path.abspath(args.bag)))


if __name__ == '__main__':
    main()
