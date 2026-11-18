#!/usr/bin/env python3

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

from terrain_nav_benchmark import run_benchmark as bench

LABELS = {'default': 'Default Nav2 (2D scan)', 'terrain': 'Terrain-aware (this project)'}


def record_frames(out_dir, label, every):
    import numpy as np
    import rclpy
    from PIL import Image as PILImage
    from PIL import ImageDraw
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node('demo_recorder')
    count = [0]

    def on_image(msg):
        count[0] += 1
        if count[0] % every:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        pil = PILImage.fromarray(img)
        draw = ImageDraw.Draw(pil)
        draw.rectangle([0, 0, msg.width, 26], fill=(0, 0, 0))
        draw.text((10, 7), label, fill=(255, 255, 255))
        pil.save(os.path.join(out_dir, f'{count[0]:06d}.png'))

    node.create_subscription(Image, 'demo_camera', on_image, qos_profile_sensor_data)
    rclpy.spin(node)


def run_mode(g, mode, frames_dir, timeout):
    start, goal = g['start'], g['goal']
    stack = subprocess.Popen(
        ['ros2', 'launch', 'terrain_nav_navigation', 'bringup.launch.py',
         f'mode:={mode}', f"world:={g['world']}", 'headless:=true', 'rviz:=false',
         'demo_camera:=true', f'x:={start[0]}', f'y:={start[1]}', f'z:={start[2]}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/demo_camera@sensor_msgs/msg/Image[ignition.msgs.Image'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    recorder = None
    try:
        trial = subprocess.Popen(
            [sys.executable, bench.__file__, '--trial',
             json.dumps({'start': start, 'goal': goal, 'timeout': timeout})],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        time.sleep(15)
        recorder = subprocess.Popen(
            [sys.executable, __file__, '--record', frames_dir, LABELS[mode]],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        out, _ = trial.communicate(timeout=timeout + 120)
        time.sleep(2)
        lines = [ln for ln in out.splitlines() if ln.startswith('{')]
        return json.loads(lines[-1]) if lines else {'outcome': 'error'}
    finally:
        for proc in (recorder, bridge):
            if proc:
                proc.terminate()
                proc.wait(timeout=10)
        bench.stop_group(stack)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo -r -s'], check=False)
        time.sleep(2)


def stitch(dirs, out_gif, speed, fps):
    n = max(len(glob.glob(os.path.join(d, '*.png'))) for d in dirs)
    inputs, filters = [], []
    for i, d in enumerate(dirs):
        frames = sorted(glob.glob(os.path.join(d, '*.png')))
        for k, f in enumerate(frames):
            os.rename(f, os.path.join(d, f'seq_{k:06d}.png'))
        inputs += ['-framerate', str(fps * speed), '-i', os.path.join(d, 'seq_%06d.png')]
        pad = (n - len(frames)) / (fps * speed)
        filters.append(f'[{i}:v]scale=480:-2,tpad=stop_mode=clone:stop_duration={pad:.2f}[v{i}]')
    graph = ';'.join(filters) + (
        f';[v0][v1]hstack=inputs=2,fps={fps},split[a][b];'
        '[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', *inputs,
                    '-filter_complex', graph, '-loop', '0', out_gif], check=True)


def main():
    if len(sys.argv) > 3 and sys.argv[1] == '--record':
        record_frames(sys.argv[2], sys.argv[3], every=2)
        return

    parser = argparse.ArgumentParser(description='Record a side-by-side demo GIF for one goal.')
    parser.add_argument('--goal', default='rock_field', help='goal name from goals.yaml')
    parser.add_argument('--out', default='demo.gif')
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--speed', type=float, default=4.0, help='playback speed-up')
    parser.add_argument('--fps', type=int, default=8, help='GIF frame rate')
    args = parser.parse_args()

    g = next(x for x in bench.load_goals() if x['name'] == args.goal)
    work = tempfile.mkdtemp(prefix='terrain_demo_')
    dirs = []
    for mode in bench.MODES:
        d = os.path.join(work, mode)
        os.makedirs(d)
        r = run_mode(g, mode, d, args.timeout)
        print(f"{mode}: {r.get('outcome')}  frames={len(os.listdir(d))}", flush=True)
        dirs.append(d)
    stitch(dirs, args.out, speed=args.speed * 7.5 / args.fps, fps=args.fps)
    shutil.rmtree(work)
    print('demo:', args.out)


if __name__ == '__main__':
    main()
