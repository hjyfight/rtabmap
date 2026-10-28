#!/usr/bin/env python3
"""Render the synthetic result saved by visualize_synthetic.py."""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'logs' / 'visualization'


def room_points():
    surfaces = []

    def plane(axis, value, bounds_a, bounds_b):
        other = [i for i in range(3) if i != axis]
        a, b = np.meshgrid(np.arange(*bounds_a, 0.08), np.arange(*bounds_b, 0.08))
        points = np.zeros((a.size, 3), dtype=np.float32)
        points[:, axis] = value
        points[:, other[0]] = a.ravel()
        points[:, other[1]] = b.ravel()
        surfaces.append(points)

    for x in (-4.0, 6.0):
        plane(0, x, (-3, 4), (-1, 2))
    for y in (-3.0, 4.0):
        plane(1, y, (-4, 6), (-1, 2))
    plane(2, -1.0, (-4, 6), (-3, 4))
    plane(0, 2.5, (0.8, 1.8), (-1, 0.6))
    plane(1, 0.8, (2.5, 3.5), (-1, 0.6))
    plane(2, 0.6, (2.5, 3.5), (0.8, 1.8))
    return np.concatenate(surfaces)


def subsample(xyz, limit=25000):
    if len(xyz) <= limit:
        return xyz
    idx = np.linspace(0, len(xyz) - 1, limit).astype(int)
    return xyz[idx]


def main():
    data = np.load(OUT / 'synthetic_result.npz')
    map_xyz = data['map_xyz']
    ground_xyz = data['ground_xyz']
    obstacle_xyz = data['obstacle_xyz']
    odom = data['odom']
    gt = data['gt']
    world = room_points()

    fig = plt.figure(figsize=(18, 13))
    fig.suptitle('RTAB-Map synthetic Livox SLAM', fontsize=16)

    ax = fig.add_subplot(2, 2, 1, projection='3d')
    w = subsample(world, 20000)
    ax.scatter(w[:, 0], w[:, 1], w[:, 2], c=w[:, 2], cmap='viridis', s=0.4, linewidths=0)
    ax.set_title(f'Synthetic input room ({len(world)} points)')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
    ax.view_init(elev=25, azim=-60)

    ax = fig.add_subplot(2, 2, 2, projection='3d')
    g = subsample(ground_xyz, 18000)
    o = subsample(obstacle_xyz, 18000)
    ax.scatter(g[:, 0], g[:, 1], g[:, 2], c='#2ca02c', s=0.4, alpha=0.35, linewidths=0, label='ground')
    ax.scatter(o[:, 0], o[:, 1], o[:, 2], c='#d62728', s=0.4, alpha=0.35, linewidths=0, label='obstacles')
    ax.plot(odom[:, 0], odom[:, 1], odom[:, 2], c='#1f77b4', lw=3.5, label='ICP odometry', zorder=10)
    ax.plot(gt[:, 0], gt[:, 1], gt[:, 2], 'k--', lw=2, label='ground truth', zorder=11)
    ax.scatter(odom[0, 0], odom[0, 1], odom[0, 2], c='#1f77b4', s=50, zorder=12)
    ax.scatter(odom[-1, 0], odom[-1, 1], odom[-1, 2], c='#ff7f0e', s=60, zorder=12)
    ax.set_title(f'Assembled map ({len(map_xyz)} points)')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
    ax.view_init(elev=55, azim=-65)
    ax.legend(loc='upper right', markerscale=8)

    ax = fig.add_subplot(2, 2, 3)
    m = subsample(map_xyz, 25000)
    sc = ax.scatter(m[:, 0], m[:, 1], c=m[:, 2], cmap='terrain', s=0.5, alpha=0.5, linewidths=0)
    ax.plot(odom[:, 0], odom[:, 1], c='#1f77b4', lw=3, label='ICP odometry')
    ax.plot(gt[:, 0], gt[:, 1], 'k--', lw=2, label='ground truth')
    ax.scatter(odom[0, 0], odom[0, 1], c='#1f77b4', s=60, zorder=5)
    ax.scatter(odom[-1, 0], odom[-1, 1], c='#ff7f0e', s=70, zorder=5)
    ax.set_xlim(-0.4, 1.6)
    ax.set_ylim(-0.5, 0.7)
    ax.set_title('Top-down zoom on the 1.2 m trajectory')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    plt.colorbar(sc, ax=ax, label='z [m]', shrink=0.7)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(odom[:, 0], odom[:, 1], 'o-', ms=2, lw=1.5, label='ICP odometry')
    ax.plot(gt[:, 0], gt[:, 1], 'k--', lw=2, label='ground truth')
    est = np.linalg.norm(odom[-1] - odom[0])
    true = np.linalg.norm(gt[-1] - gt[0])
    ax.set_title(f'Trajectory: est {est:.3f} m vs truth {true:.3f} m')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(OUT / 'synthetic_visualization.png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    print((OUT / 'synthetic_visualization.png').resolve(), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
