#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time

import yaml

MODES = ('default', 'terrain')
CSV_FIELDS = ['world', 'goal', 'mode', 'outcome', 'success', 'path_length_m',
              'straight_line_m', 'max_tilt_deg', 'time_s', 'final_error_m']
GOAL_TOLERANCE = 1.5
TIPPED_DEG = 60.0
STUCK_WINDOW = 30.0
STUCK_DIST = 0.3


def tilt_deg(q):
    cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_tilt))))


def run_trial(start, goal, timeout):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter

    rclpy.init()
    nav = BasicNavigator('benchmark_navigator')
    nav.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    track = []
    max_tilt = [0.0]

    def on_gt(msg):
        p = msg.pose.pose.position
        track.append((p.x, p.y, p.z))
        max_tilt[0] = max(max_tilt[0], tilt_deg(msg.pose.pose.orientation))

    nav.create_subscription(Odometry, 'ground_truth/odom', on_gt, 10)

    nav._waitForNodeToActivate('bt_navigator')
    t_wait = time.time()
    while not track and time.time() - t_wait < 30.0:
        rclpy.spin_once(nav, timeout_sec=0.2)
    t_settle = time.time()
    while time.time() - t_settle < 3.0:
        rclpy.spin_once(nav, timeout_sec=0.1)
    track.clear()
    max_tilt[0] = 0.0

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = goal[0] - start[0]
    pose.pose.position.y = goal[1] - start[1]
    pose.pose.orientation.w = 1.0
    nav.goToPose(pose)

    t0 = nav.get_clock().now().nanoseconds * 1e-9
    outcome = None
    history = []
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        elapsed = nav.get_clock().now().nanoseconds * 1e-9 - t0
        if track:
            history.append((elapsed, track[-1][0], track[-1][1]))
            while history and history[0][0] < elapsed - STUCK_WINDOW:
                history.pop(0)
        if max_tilt[0] > TIPPED_DEG:
            outcome = 'tipped'
        elif elapsed > timeout:
            outcome = 'timeout'
        elif (elapsed > STUCK_WINDOW and history and
              math.hypot(history[-1][1] - history[0][1],
                         history[-1][2] - history[0][2]) < STUCK_DIST):
            outcome = 'stuck'
        if outcome:
            nav.cancelTask()
            break
    elapsed = nav.get_clock().now().nanoseconds * 1e-9 - t0
    result = nav.getResult()

    final = track[-1] if track else (start[0], start[1], 0.0)
    error = math.hypot(final[0] - goal[0], final[1] - goal[1])
    if outcome is None:
        outcome = 'reached' if result == TaskResult.SUCCEEDED else 'nav2_failed'
    if outcome == 'reached' and error > GOAL_TOLERANCE:
        outcome = 'wrong_place'
    length = sum(math.dist(a, b) for a, b in zip(track, track[1:]))

    nav.destroy_node()
    rclpy.shutdown()
    return {
        'outcome': outcome,
        'success': outcome == 'reached',
        'path_length_m': round(length, 2),
        'straight_line_m': round(math.hypot(goal[0] - start[0], goal[1] - start[1]), 2),
        'max_tilt_deg': round(max_tilt[0], 1),
        'time_s': round(elapsed, 1),
        'final_error_m': round(error, 2),
        'track': [[round(v, 2) for v in p] for p in track[::5]],
    }


def load_goals():
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('terrain_nav_gazebo'), 'config', 'goals.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['goals']


def stop_group(proc):
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def run_one(g, mode, args, log_dir):
    start, goal = g['start'], g['goal']
    launch = ['ros2', 'launch', 'terrain_nav_navigation', 'bringup.launch.py',
              f'mode:={mode}', f"world:={g['world']}", 'headless:=true', 'rviz:=false',
              f'x:={start[0]}', f'y:={start[1]}', f'z:={start[2]}']
    log_path = os.path.join(log_dir, f"{g['world']}_{g['name']}_{mode}.log")
    with open(log_path, 'w') as log:
        stack = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            child = subprocess.run(
                [sys.executable, os.path.abspath(__file__), '--trial',
                 json.dumps({'start': start, 'goal': goal, 'timeout': args.timeout})],
                capture_output=True, text=True, timeout=args.timeout + 120)
            line = [ln for ln in child.stdout.splitlines() if ln.startswith('{')]
            result = json.loads(line[-1]) if line else {'outcome': 'error', 'success': False}
            if not line:
                log.write('\n--- trial stderr ---\n' + child.stderr[-3000:])
        except subprocess.TimeoutExpired:
            result = {'outcome': 'error', 'success': False}
        finally:
            stop_group(stack)
    subprocess.run(['pkill', '-9', '-f', 'ign gazebo -r -s'], check=False)
    time.sleep(2)
    return result


def run_all(args):
    goals = [g for g in load_goals() if not args.only or g['name'] in args.only]
    os.makedirs(args.out, exist_ok=True)
    log_dir = os.path.join(args.out, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    csv_path = os.path.join(args.out, 'benchmark.csv')
    tracks_path = os.path.join(args.out, 'tracks.json')

    tracks = {}
    if os.path.exists(tracks_path) and args.append:
        with open(tracks_path) as f:
            tracks = json.load(f)
    write_header = not (args.append and os.path.exists(csv_path))
    with open(csv_path, 'a' if args.append else 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction='ignore')
        if write_header:
            writer.writeheader()
        total = len(goals) * len(args.modes)
        for i, (g, mode) in enumerate(((g, m) for g in goals for m in args.modes), 1):
            print(f"[{i}/{total}] {g['world']}/{g['name']} mode={mode} ...", flush=True)
            r = run_one(g, mode, args, log_dir)
            row = {'world': g['world'], 'goal': g['name'], 'mode': mode, **r}
            writer.writerow(row)
            f.flush()
            tracks[f"{g['world']}/{g['name']}/{mode}"] = r.get('track', [])
            print(f"    -> {r['outcome']}  length={r.get('path_length_m')} m  "
                  f"max_tilt={r.get('max_tilt_deg')} deg  time={r.get('time_s')} s", flush=True)
    with open(tracks_path, 'w') as f:
        json.dump(tracks, f)


def read_rows(out):
    path = os.path.join(out, 'benchmark.csv')
    with open(path) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        if r['outcome'] in ('reached', 'wrong_place'):
            ok = float(r['final_error_m']) <= GOAL_TOLERANCE
            r['outcome'] = 'reached' if ok else 'wrong_place'
            r['success'] = str(ok)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def write_table(rows, out):
    by = {(r['world'], r['goal'], r['mode']): r for r in rows}
    keys = list(dict.fromkeys((r['world'], r['goal']) for r in rows))

    def cell(r):
        if r is None:
            return '-'
        mark = 'yes' if r['success'] == 'True' else f"no ({r['outcome']})"
        return f"{mark} / {float(r['path_length_m']):.1f} m / {float(r['max_tilt_deg']):.0f}°"

    lines = ['| World | Goal | Default Nav2 | Terrain-aware |', '|---|---|---|---|']
    for world, goal in keys:
        lines.append(f"| {world.replace('_terrain', '')} | `{goal}` | "
                     f"{cell(by.get((world, goal, 'default')))} | "
                     f"{cell(by.get((world, goal, 'terrain')))} |")
    summary = []
    for mode in MODES:
        rs = [r for r in rows if r['mode'] == mode]
        if not rs:
            summary.append('-')
            continue
        ok = [r for r in rs if r['success'] == 'True']
        mean_len = (f"{sum(float(r['path_length_m']) for r in ok) / len(ok):.1f} m"
                    if ok else '-')
        worst = max(float(r['max_tilt_deg']) for r in rs)
        mean_tilt = sum(float(r['max_tilt_deg']) for r in rs) / len(rs)
        summary.append(f'**{len(ok)}/{len(rs)} reached**, mean length {mean_len}, '
                       f'mean max tilt {mean_tilt:.0f}° (worst {worst:.0f}°)')
    lines.append(f'| **All** | | {summary[0]} | {summary[1]} |')
    lines += ['', 'Cells: reached? / path length / max tilt (ground truth).']
    path = os.path.join(out, 'results.md')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return path


def write_plot(rows, out, plot_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    keys = list(dict.fromkeys((r['world'], r['goal']) for r in rows))
    by = {(r['world'], r['goal'], r['mode']): r for r in rows}
    colours = {'default': '#9aa0a6', 'terrain': '#1a73e8'}
    labels = {'default': 'Default Nav2 (2D scan)', 'terrain': 'Terrain-aware'}
    x = np.arange(len(keys))
    w = 0.38

    fig, (ax_t, ax_l) = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
    for k, mode in enumerate(MODES):
        tilt, length, failed = [], [], []
        for world, goal in keys:
            r = by.get((world, goal, mode))
            tilt.append(float(r['max_tilt_deg']) if r else 0)
            length.append(float(r['path_length_m']) if r and r['success'] == 'True' else 0)
            failed.append(r is not None and r['success'] != 'True')
        pos = x + (k - 0.5) * w
        ax_t.bar(pos, tilt, w, color=colours[mode], label=labels[mode])
        ax_l.bar(pos, length, w, color=colours[mode], label=labels[mode])
        for px, f in zip(pos, failed):
            if f:
                ax_l.text(px, 0.5, 'FAIL', rotation=90, ha='center', va='bottom',
                          fontsize=8, color='#c5221f', fontweight='bold')
    ax_t.axhline(20, color='#c5221f', ls='--', lw=1)
    ax_t.text(len(keys) - 0.5, 21, 'robot stability limit (20°)', ha='right',
              fontsize=8, color='#c5221f')
    ax_t.set_ylabel('max tilt [deg]')
    ax_t.set_title('Max tilt per goal (lower is safer)')
    ax_t.legend(loc='upper left')
    ax_l.set_ylabel('path length [m]')
    ax_l.set_title('Path length of successful runs (FAIL = goal not reached)')
    ax_l.set_xticks(x)
    ax_l.set_xticklabels([f"{g}\n({w_.replace('_terrain', '')})" for w_, g in keys],
                         fontsize=8)
    for ax in (ax_t, ax_l):
        ax.grid(axis='y', alpha=0.3)
        ax.set_axisbelow(True)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(plot_path)), exist_ok=True)
    fig.savefig(plot_path, dpi=110)
    print('plot:', plot_path)


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--trial':
        spec = json.loads(sys.argv[2])
        print(json.dumps(run_trial(spec['start'], spec['goal'], spec['timeout'])), flush=True)
        return

    parser = argparse.ArgumentParser(description='Run the default vs terrain-aware Nav2 benchmark.')
    parser.add_argument('--out', default='benchmark_results', help='output directory')
    parser.add_argument('--plot', default='', help='plot path (default: <out>/benchmark.png)')
    parser.add_argument('--modes', nargs='+', default=list(MODES), choices=MODES)
    parser.add_argument('--only', nargs='*', default=[], help='goal names to run')
    parser.add_argument('--timeout', type=float, default=150.0, help='per-goal limit [sim s]')
    parser.add_argument('--append', action='store_true', help='append to an existing CSV')
    parser.add_argument('--plot-only', action='store_true', help='only rebuild table and plot')
    args = parser.parse_args()

    if not args.plot_only:
        run_all(args)
    rows = read_rows(args.out)
    write_table(rows, args.out)
    write_plot(rows, args.out, args.plot or os.path.join(args.out, 'benchmark.png'))


if __name__ == '__main__':
    main()
