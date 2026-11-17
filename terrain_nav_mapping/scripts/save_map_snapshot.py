#!/usr/bin/env python3

import argparse

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def layer_to_array(msg: GridMap, name: str) -> np.ndarray:
    arr = msg.data[msg.layers.index(name)]
    rows = arr.layout.dim[1].size
    cols = arr.layout.dim[0].size
    m = np.array(arr.data, dtype=np.float32).reshape((cols, rows)).T
    return np.roll(m, (-msg.outer_start_index, -msg.inner_start_index), axis=(1, 0))


def plot(msg: GridMap, out: str):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    cx, cy = msg.info.pose.position.x, msg.info.pose.position.y
    lx, ly = msg.info.length_x, msg.info.length_y
    extent = [cy + ly / 2, cy - ly / 2, cx - lx / 2, cx + lx / 2]
    panels = [
        ('elevation', 'terrain', None, 'height [m]'),
        ('slope', 'viridis', (0, 30), 'slope [deg]'),
        ('step', 'magma', (0, 0.4), 'step [m]'),
        ('traversability', 'RdYlGn_r', (0, 1), 'cost (1 = lethal)'),
    ]
    fig, axes = plt.subplots(1, len(panels), figsize=(5 * len(panels), 4.6))
    for ax, (name, cmap, lim, label) in zip(axes, panels):
        data = layer_to_array(msg, name)
        if name == 'slope':
            data = np.degrees(data)
        vmin, vmax = lim if lim else (np.nanmin(data), np.nanmax(data))
        im = ax.imshow(data, cmap=cmap, vmin=vmin, vmax=vmax, extent=extent)
        ax.set_title(name)
        ax.set_xlabel('y [m]')
        ax.set_ylabel('x [m]')
        fig.colorbar(im, ax=ax, fraction=0.046, label=label)
    fig.suptitle(f'{msg.header.frame_id} frame, {msg.info.resolution:.2f} m cells')
    fig.savefig(out, dpi=90, bbox_inches='tight')


class Snapshot(Node):
    def __init__(self, args):
        super().__init__('map_snapshot')
        self.args = args
        self.done = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GridMap, 'elevation_map', self.callback, qos)

    def callback(self, msg):
        if self.done:
            return
        plot(msg, self.args.out)
        if self.args.npz:
            np.savez(self.args.npz, **{n: layer_to_array(msg, n) for n in msg.layers})
        self.get_logger().info(f'saved {self.args.out}')
        self.done = True


def main():
    parser = argparse.ArgumentParser(description='Save the next /elevation_map message as a PNG.')
    parser.add_argument('--out', default='elevation_map.png')
    parser.add_argument('--npz', default='')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Snapshot(args)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
