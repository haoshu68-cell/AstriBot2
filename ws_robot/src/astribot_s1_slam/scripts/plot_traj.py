#!/usr/bin/env python3
"""Lightweight evo-style trajectory plotter for TUM-format pose logs.

Built for checking highrate_tf.txt (see highrate_odom.hpp) for re-anchor
jumps, but works on any TUM file with columns "ts x y z qx qy qz qw" —
including lidar_poses.txt/chassis_traj.txt (extra trailing columns, e.g.
velocity/bias, are ignored).

No ROS/ evo dependency, just numpy+matplotlib. Typical use:

  # one file: trajectory + per-step jump/dt diagnostics
  ./plot_traj.py log/highrate_tf.txt

  # overlay against the LiDAR-rate poses to see how the 20Hz output
  # tracks/interpolates between the ~10Hz corrections
  ./plot_traj.py log/highrate_tf.txt SESSION/lidar_poses.txt \\
      --labels highrate(20Hz) lidar(10Hz)

  # headless (no display): just save the PNG
  ./plot_traj.py log/highrate_tf.txt --save out.png --no-show
"""

import argparse
import sys

import numpy as np

try:
    import matplotlib
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")


def load_tum(path):
    """Reads columns ts,x,y,z,qx,qy,qz,qw — ignores '#' comments and any
    columns past the 8th (lidar_poses.txt/save_pose() appends v/bg/ba/g/v6)."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            rows.append([float(x) for x in parts[:8]])
    if not rows:
        sys.exit(f"{path}: no valid TUM rows found")
    a = np.array(rows)
    order = np.argsort(a[:, 0])
    return a[order]


def trim_frozen_tail(path, a, verbose=True):
    """Drops a trailing run of rows whose timestamp didn't advance at all —
    the expected tail artifact when the pose source (e.g. HighRateOdom's
    20Hz timer) keeps ticking/publishing the last known state after its
    input (IMU) stops arriving, e.g. because the bag finished playing
    before the node was stopped. Not a real anomaly; left in by default
    would otherwise dominate rate/jump statistics. Use --keep-tail to
    disable."""
    t = a[:, 0]
    dt = np.diff(t)
    frozen = np.nonzero(dt < 1e-6)[0]
    if frozen.size == 0:
        return a
    cut = frozen[0] + 1
    if cut >= len(t):
        return a
    if verbose:
        print(f"{path}: trimming {len(t)-cut} frozen-timestamp tail rows "
              f"(pose source kept publishing after its input stopped) — pass --keep-tail to disable")
    return a[:cut]


def quat_to_rpy(qx, qy, qz, qw):
    """ZYX (yaw-pitch-roll) Euler angles, radians, from xyzw quaternions."""
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


def quat_angle_diff(q1, q2):
    """Angle (rad) of the relative rotation between consecutive xyzw quaternion
    rows q1[i]->q2[i], via |dot|-based geodesic distance (sign-ambiguity safe)."""
    dot = np.clip(np.abs(np.sum(q1 * q2, axis=1)), -1.0, 1.0)
    return 2.0 * np.arccos(dot)


def diagnostics(name, t, xyz, quat, expect_dt=None):
    dt = np.diff(t)
    xyz_jump = np.diff(xyz, axis=0)  # signed per-axis step, x/y/z columns
    pos_jump = np.linalg.norm(xyz_jump, axis=1)
    rot_jump = np.rad2deg(quat_angle_diff(quat[:-1], quat[1:]))

    print(f"\n== {name} ({len(t)} poses) ==")
    print(f"  duration: {t[-1]-t[0]:.2f}s")
    if dt.size:
        print(f"  dt:       mean={dt.mean()*1e3:.1f}ms  std={dt.std()*1e3:.1f}ms  "
              f"min={dt.min()*1e3:.1f}ms  max={dt.max()*1e3:.1f}ms")
        if expect_dt is not None:
            bad = np.abs(dt - expect_dt) > 0.5 * expect_dt
            print(f"  dt outside +/-50% of {expect_dt*1e3:.0f}ms: {bad.sum()}/{dt.size}")
    if pos_jump.size:
        print(f"  pos jump: mean={pos_jump.mean()*1e3:.1f}mm  median={np.median(pos_jump)*1e3:.1f}mm  "
              f"max={pos_jump.max()*1e3:.1f}mm")
        # Per-axis (signed) jump — catches an axis-specific bias (e.g. a
        # gravity/scale error that only shows up on z) that the norm above
        # averages away against the other two axes.
        for i, axis in enumerate('xyz'):
            print(f"    {axis} jump: mean={xyz_jump[:, i].mean()*1e3:+.2f}mm  std={xyz_jump[:, i].std()*1e3:.2f}mm")
        print(f"  rot jump: mean={rot_jump.mean():.3f}deg  median={np.median(rot_jump):.3f}deg  "
              f"max={rot_jump.max():.3f}deg")
        # Outlier steps = candidate anchor()-induced jumps riding on top of the
        # otherwise-smooth IMU-propagated trajectory.
        thresh = max(np.median(pos_jump) * 5.0, 1e-3)
        outliers = np.nonzero(pos_jump > thresh)[0]
        if outliers.size:
            worst = outliers[np.argsort(-pos_jump[outliers])][:10]
            print(f"  {outliers.size} position-jump outliers (>5x median), worst 10:")
            for i in worst:
                print(f"    t={t[i]:.3f}->{t[i+1]:.3f}  dt={dt[i]*1e3:.1f}ms  "
                      f"jump={pos_jump[i]*1e3:.1f}mm  rot={rot_jump[i]:.3f}deg")
    return dt, pos_jump, rot_jump


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+', help='TUM trajectory file(s)')
    ap.add_argument('--labels', nargs='+', help='legend labels, same order as files')
    ap.add_argument('--expect-dt', type=float, default=0.05,
                     help='expected tick period in seconds, for the dt sanity check (default 0.05 = 20Hz)')
    ap.add_argument('--t0', choices=['first', 'none'], default='first',
                     help='"first": rebase every file\'s time axis to its own first sample (default, easiest to compare shapes); "none": keep raw timestamps (use to see real time offset between files)')
    ap.add_argument('--save', default=None, help='PNG output path (default: <first file>.png)')
    ap.add_argument('--no-show', action='store_true', help='do not open an interactive window')
    ap.add_argument('--keep-tail', action='store_true',
                     help="don't trim a trailing frozen-timestamp run (see trim_frozen_tail's docstring)")
    args = ap.parse_args()

    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    labels = args.labels if args.labels else [f for f in args.files]
    if len(labels) != len(args.files):
        sys.exit('--labels must have the same count as files')

    datasets = []
    for path, label in zip(args.files, labels):
        a = load_tum(path)
        if not args.keep_tail:
            a = trim_frozen_tail(path, a)
        t = a[:, 0]
        if args.t0 == 'first':
            t = t - t[0]
        xyz = a[:, 1:4]
        quat = a[:, 4:8]  # xyzw
        roll, pitch, yaw = quat_to_rpy(quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3])
        # Unwrap yaw so a real continuous rotation through +/-180 deg draws as a
        # smooth ramp instead of a vertical "jump" line at the wrap point.
        yaw = np.unwrap(yaw)
        dt, pos_jump, rot_jump = diagnostics(label, t, xyz, quat, args.expect_dt)
        datasets.append(dict(label=label, t=t, xyz=xyz, rpy=np.rad2deg(np.stack([roll, pitch, yaw], axis=1)),
                              dt=dt, pos_jump=pos_jump, rot_jump=rot_jump))

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 3)

    ax_xy = fig.add_subplot(gs[:, 0])
    ax_xyz = fig.add_subplot(gs[0, 1:])
    ax_rpy = fig.add_subplot(gs[1, 1:], sharex=ax_xyz)
    ax_jump = fig.add_subplot(gs[2, 1], sharex=ax_xyz)
    ax_dt = fig.add_subplot(gs[2, 2], sharex=ax_xyz)

    for d in datasets:
        ax_xy.plot(d['xyz'][:, 0], d['xyz'][:, 1], label=d['label'], linewidth=1)
    ax_xy.set_xlabel('x [m]'); ax_xy.set_ylabel('y [m]')
    ax_xy.set_title('top-down (x-y)'); ax_xy.axis('equal'); ax_xy.legend(); ax_xy.grid(True)

    for d in datasets:
        for i, axis in enumerate('xyz'):
            ax_xyz.plot(d['t'], d['xyz'][:, i], label=f"{d['label']} {axis}", linewidth=1)
    ax_xyz.set_ylabel('position [m]'); ax_xyz.set_title('x/y/z vs time'); ax_xyz.grid(True)
    ax_xyz.legend(fontsize=7, ncol=len(datasets))

    for d in datasets:
        for i, axis in enumerate(['roll', 'pitch', 'yaw']):
            ax_rpy.plot(d['t'], d['rpy'][:, i], label=f"{d['label']} {axis}", linewidth=1)
    ax_rpy.set_ylabel('deg'); ax_rpy.set_title('roll/pitch/yaw vs time'); ax_rpy.grid(True)
    ax_rpy.legend(fontsize=7, ncol=len(datasets))

    for d in datasets:
        ax_jump.plot(d['t'][1:], d['pos_jump']*1e3, label=d['label'], linewidth=1)
    ax_jump.set_ylabel('mm'); ax_jump.set_title('per-step position jump'); ax_jump.grid(True)
    ax_jump.legend(fontsize=7)

    for d in datasets:
        ax_dt.plot(d['t'][1:], d['dt']*1e3, label=d['label'], linewidth=1)
    if args.expect_dt:
        ax_dt.axhline(args.expect_dt*1e3, color='gray', linestyle='--', linewidth=0.8)
    ax_dt.set_ylabel('ms'); ax_dt.set_title('per-step dt'); ax_dt.grid(True)
    ax_dt.legend(fontsize=7)

    fig.tight_layout()

    save_path = args.save or (args.files[0] + '.png')
    fig.savefig(save_path, dpi=150)
    print(f"\nsaved figure: {save_path}")

    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
