#!/usr/bin/env python3
"""Plot two recorded runs with identical map extents, colors and viewpoints."""
import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/rtabmap-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

LABELS = {'lidar': 'LiDAR only', 'depth': 'LiDAR + depth fusion',
          'rgbd': 'LiDAR + RGB-D loop closure'}
COLORS = {1: '#e63946', 2: '#ff9f1c', 3: '#9467bd'}


def load_run(path):
    manifest = json.loads((path / 'manifest.json').read_text())
    graph = json.loads((path / 'graph.json').read_text())
    xyz = np.load(path / 'cloud.npz')['xyz']
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz) or not graph['poses']:
        raise ValueError(f'{path}: empty map/graph; check sensor and SLAM topics')
    poses = {int(k): np.asarray(v) for k, v in graph['poses'].items() if int(k) > 0}
    # The graph can include reversed duplicate constraints; count each once.
    links = {(min(e['from'], e['to']), max(e['from'], e['to']), e['type'])
             for e in graph['links'] if e['type'] in COLORS
             and e['from'] in poses and e['to'] in poses}
    return dict(path=path, manifest=manifest, graph=graph, xyz=xyz, poses=poses, links=links)


def draw(ax, run, spatial, limits, height_limits, slice_range=None):
    xyz = run['xyz']
    if slice_range is not None:
        xyz = xyz[(xyz[:, 2] >= slice_range[0]) & (xyz[:, 2] <= slice_range[1])]
    # Deterministic display sampling only; saved map remains complete.
    xyz = xyz[np.linspace(0, len(xyz) - 1, min(len(xyz), 90000)).astype(int)]
    axes = [xyz[:, 0], xyz[:, 1]] + ([xyz[:, 2]] if spatial else [])
    ax.scatter(*axes, c=xyz[:, 2], cmap='viridis', vmin=height_limits[0],
               vmax=height_limits[1], s=0.5, alpha=0.6, linewidths=0)
    poses = run['poses']
    for e in run['graph']['links']:
        if e['type'] in (0, 4) and e['from'] in poses and e['to'] in poses:
            line = np.array([poses[e['from']], poses[e['to']]])
            ax.plot(*[line[:, i] for i in range(3 if spatial else 2)],
                    color='#1665ad', lw=1.1)
    for a, b, kind in sorted(run['links']):
        line = np.array([poses[a], poses[b]])
        ax.plot(*[line[:, i] for i in range(3 if spatial else 2)],
                color=COLORS[kind], lw=1.2, alpha=0.9)
    for i, setter in enumerate([ax.set_xlim, ax.set_ylim] + ([ax.set_zlim] if spatial else [])):
        setter(*limits[i])
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    if spatial:
        ax.set_zlabel('Z [m]')
        ax.view_init(elev=35, azim=-65)
        ax.set_box_aspect([b - a for a, b in limits])
        ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = False
    else:
        ax.set_aspect('equal', adjustable='box')
    ax.grid(False)
    mode = run['manifest']['mode']
    n_global = sum(t == 1 for _, _, t in run['links'])
    n_geom = len(run['links']) - n_global
    view = '3D map' if spatial else 'Top view'
    if slice_range is not None:
        view = f'Horizontal slice: Z = {slice_range[0]:.2f} to {slice_range[1]:.2f} m'
    ax.set_title(f'{LABELS[mode]} — {view}\n'
                 f'{len(poses)} nodes | global loops: {n_global} | proximity: {n_geom}')


def plot_trajectories(runs, output):
    if not all(r['graph'].get('raw_poses') for r in runs):
        return
    all_points = np.vstack([np.array(list(r['poses'].values())) for r in runs] +
                           [np.array(list(r['graph']['raw_poses'].values())) for r in runs])
    lo, hi = all_points.min(axis=0), all_points.max(axis=0)
    pad = np.maximum((hi - lo) * 0.08, 0.2)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for ax, run in zip(axes, runs):
        stamps = run['graph'].get('stamps', {})
        ids = sorted(run['poses'], key=lambda i: (stamps.get(str(i), i), i))
        raw = run['graph']['raw_poses']
        original = np.array([raw[str(i)] for i in ids if str(i) in raw])
        optimized = np.array([run['poses'][i] for i in ids])
        ax.plot(original[:, 0], original[:, 1], '--', color='#777777', lw=1.8,
                label='ICP odometry (before graph optimization)')
        ax.plot(optimized[:, 0], optimized[:, 1], color='#1665ad', lw=1.8,
                marker='.', markersize=4, label='Saved optimized trajectory')
        for a, b, kind in sorted(run['links']):
            line = np.array([run['poses'][a], run['poses'][b]])
            ax.plot(line[:, 0], line[:, 1], color=COLORS[kind], lw=1.3)
        ax.scatter(*optimized[0, :2], s=75, marker='o', c='#16834b', label='Start', zorder=5)
        ax.scatter(*optimized[-1, :2], s=75, marker='x', c='#c33e36', label='End', zorder=5)
        ax.set(xlim=(lo[0]-pad[0], hi[0]+pad[0]), ylim=(lo[1]-pad[1], hi[1]+pad[1]),
               xlabel='X [m]', ylabel='Y [m]', title=LABELS[run['manifest']['mode']])
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='best', fontsize=8)
    fig.suptitle('Trajectory correction relative to each run\'s start (not ground-truth error)')
    fig.tight_layout()
    fig.savefig(output / 'trajectories.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('fusion', type=Path)
    parser.add_argument('--output', type=Path, default=Path('logs/comparison_figures'))
    parser.add_argument('--slice-z', type=float, default=0.0)
    parser.add_argument('--slice-width', type=float, default=0.3)
    args = parser.parse_args()
    if not np.isfinite([args.slice_z, args.slice_width]).all() or args.slice_width <= 0:
        parser.error('slice height must be finite and width must be positive')
    runs = [load_run(args.baseline), load_run(args.fusion)]
    if runs[0]['manifest']['mode'] != 'lidar' or runs[1]['manifest']['mode'] == 'lidar':
        parser.error('expected a lidar baseline followed by a depth or rgbd run')
    bounds = np.array([[r['xyz'].min(axis=0), r['xyz'].max(axis=0)] for r in runs])
    lo, hi = bounds[:, 0].min(axis=0), bounds[:, 1].max(axis=0)
    pad = np.maximum((hi - lo) * 0.03, 0.1)
    limits = list(zip(lo - pad, hi + pad))
    height_limits = (lo[2], max(hi[2], lo[2] + 0.01))
    args.output.mkdir(parents=True, exist_ok=True)
    legend = [Line2D([0], [0], color=c, label=s) for c, s in
              [('#1665ad', 'Odometry graph edges'), (COLORS[1], 'Global loop'),
               (COLORS[2], 'Spatial proximity loop'), (COLORS[3], 'Temporal proximity loop')]]
    fig = plt.figure(figsize=(16, 16))
    slice_range = (args.slice_z - args.slice_width / 2, args.slice_z + args.slice_width / 2)
    for col, run in enumerate(runs):
        for row, spatial in enumerate((True, False, False)):
            ax = fig.add_subplot(3, 2, row * 2 + col + 1,
                                 projection='3d' if spatial else None)
            draw(ax, run, spatial, limits, height_limits, slice_range if row == 2 else None)
    fig.legend(handles=legend, loc='lower center', ncol=4)
    fig.suptitle('Recorded SLAM maps and accepted loop constraints (height coloring)', fontsize=15)
    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    fig.savefig(args.output / 'comparison.png', dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    for ax, run in zip(axes, runs):
        draw(ax, run, False, limits, height_limits)
    fig.legend(handles=legend, loc='lower center', ncol=4)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(args.output / 'top_view.png', dpi=180)
    plt.close(fig)
    for run, label in zip(runs, ('lidar', 'fusion')):
        fig = plt.figure(figsize=(9, 8))
        draw(fig.add_subplot(111, projection='3d'), run, True, limits, height_limits)
        fig.tight_layout()
        fig.savefig(args.output / f'{label}_map.png', dpi=180)
        plt.close(fig)
    plot_trajectories(runs, args.output)
    summary = [{'mode': r['manifest']['mode'], 'points': len(r['xyz']),
                'nodes': len(r['poses']), 'global_links': sum(t == 1 for _, _, t in r['links']),
                'proximity_links': sum(t in (2, 3) for _, _, t in r['links'])} for r in runs]
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print((args.output / 'comparison.png').resolve())


if __name__ == '__main__':
    main()
