#!/usr/bin/env python3
"""Compare saved LiDAR and RGB-D-assisted runs without modifying their databases."""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]


def read_ascii_ply(path):
    """Read named XYZ fields from the ASCII point cloud exported by RTAB-Map."""
    with path.open() as stream:
        if stream.readline().strip() != 'ply':
            raise ValueError(f'{path}: not a PLY file')
        count, fields, element, ascii_format = None, [], None, False
        for _ in range(1000):
            parts = stream.readline().split()
            if not parts:
                raise ValueError(f'{path}: incomplete PLY header')
            if parts[:2] == ['format', 'ascii']:
                ascii_format = True
            elif parts[0] == 'element':
                element = parts[1]
                if element == 'vertex':
                    count = int(parts[2])
            elif parts[0] == 'property' and element == 'vertex':
                if parts[1] == 'list':
                    raise ValueError('List properties in PLY vertices are unsupported')
                fields.append(parts[-1])
            elif parts[0] == 'end_header':
                break
        else:
            raise ValueError(f'{path}: PLY header too long')
        if not ascii_format or not count or not all(k in fields for k in ('x', 'y', 'z')):
            raise ValueError(f'{path}: expected nonempty ASCII XYZ point cloud')
        xyz = np.loadtxt(stream, max_rows=count,
                         usecols=[fields.index(k) for k in ('x', 'y', 'z')], ndmin=2)
        if len(xyz) != count:
            raise ValueError(f'{path}: incomplete point cloud')
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if not len(xyz):
            raise ValueError(f'{path}: no finite points')
        return xyz.astype(np.float32)


def read_poses(path):
    rows = np.loadtxt(path, comments='#', ndmin=2)
    if rows.shape[1] != 9 or not np.isfinite(rows).all():
        raise ValueError(f'{path}: expected timestamp XYZ quaternion ID (format 11)')
    result = {}
    for row in rows:
        node_id = int(row[8])
        if row[8] != node_id or node_id in result:
            raise ValueError(f'{path}: invalid or repeated node ID')
        if node_id <= 0:
            continue  # Landmarks are not robot trajectory samples.
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
        matrix[:3, 3] = row[1:4]
        result[node_id] = (float(row[0]), matrix)
    if not result:
        raise ValueError(f'{path}: no robot poses')
    return result


def in_start_frame(xyz, start):
    return (np.asarray(xyz) - start[:3, 3]) @ start[:3, :3]


def unique_loop_links(links, ids):
    return sorted({(min(e['from'], e['to']), max(e['from'], e['to']), e['type'])
                   for e in links if e['type'] in (1, 2, 3)
                   and e['from'] in ids and e['to'] in ids})


def export_run(source, output, mode, voxel=0.05, max_range=20.0):
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output / 'source_snapshot.db'
    # SQLite backup includes committed WAL data and never writes the source.
    with sqlite3.connect(source.as_uri() + '?mode=ro', uri=True) as origin:
        with sqlite3.connect(snapshot) as target:
            origin.backup(target)
    with sqlite3.connect(snapshot.as_uri() + '?mode=ro', uri=True) as db:
        optimized = db.execute('select length(opt_poses) from Admin').fetchone()
        if not optimized or not optimized[0]:
            raise ValueError(f'{source}: 尚无最终优化位姿，请 Ctrl+C 正常结束建图并保存数据库后再比较。')
        raw = {}
        for node_id, blob in db.execute('select id,pose from Node where id>0'):
            if blob is None or len(blob) != 48:
                raise ValueError(f'{source}: invalid pose for node {node_id}')
            matrix = np.eye(4)
            matrix[:3, :] = np.frombuffer(blob, dtype='<f4').reshape(3, 4)
            raw[node_id] = matrix
        links = [dict(zip(('from', 'to', 'type'), row)) for row in
                 db.execute('select from_id,to_id,type from Link')]
        row = db.execute('select parameters from Info order by rowid desc limit 1').fetchone()
        parameters = dict(item.split(':', 1) for item in (row[0] if row else '').split(';')
                          if ':' in item)
    cmd = [shutil.which('rtabmap-export'), '--scan', '--cloud', '--ascii', '--poses',
           '--poses_format', '11', '--opt', '2', '--voxel', str(voxel),
           '--min_range', '0.3', '--max_range', str(max_range), '--threads', '1',
           '--output', 'map', '--output_dir', str(output), str(snapshot)]
    if cmd[0] is None:
        raise ValueError('未找到 rtabmap-export，请先 source setup.bash')
    print(f'Exporting {mode}: {source.name}', flush=True)
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    with (output / 'export.log').open('w') as log:
        subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    xyz = read_ascii_ply(output / 'map_cloud.ply')
    poses = read_poses(output / 'map_poses.txt')
    first_id = min(poses, key=lambda i: (poses[i][0], i))
    start = poses[first_id][1]
    if first_id not in raw:
        raise ValueError('First optimized node is absent from raw trajectory')
    xyz = in_start_frame(xyz, start)
    positions = {i: in_start_frame(p[:3, 3], start).tolist() for i, (_, p) in poses.items()}
    raw_positions = {i: in_start_frame(raw[i][:3, 3], raw[first_id]).tolist()
                     for i in poses if i in raw}
    graph = {'frame_id': 'first_robot_pose', 'poses': positions, 'raw_poses': raw_positions,
             'stamps': {i: stamp for i, (stamp, _) in poses.items()}, 'links': links}
    (output / 'graph.json').write_text(json.dumps(graph, indent=2))
    np.savez_compressed(output / 'cloud.npz', xyz=xyz.astype(np.float32))
    manifest = {
        'mode': mode, 'source_database': str(source),
        'snapshot_sha256': hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        'created_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'optimization': 'saved optimized poses (--opt 2); no new loop detection',
        'alignment': 'each run rigidly expressed in its first robot pose; no ICP map alignment',
        'export_voxel_m': voxel, 'export_min_range_m': 0.3, 'export_max_range_m': max_range,
        'export_command': cmd, 'backend_parameters': parameters,
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    ordered = sorted(poses, key=lambda i: (poses[i][0], i))
    track = np.array([positions[i] for i in ordered])
    loops = unique_loop_links(links, set(poses))
    correction = np.array([np.linalg.norm(np.asarray(positions[i]) - raw_positions[i])
                           for i in ordered if i in raw_positions])
    metrics = {
        'mode': mode, 'source_database': str(source), 'points': len(xyz), 'nodes': len(poses),
        'global_loop_constraints': sum(t == 1 for _, _, t in loops),
        'spatial_loop_constraints': sum(t == 2 for _, _, t in loops),
        'temporal_loop_constraints': sum(t == 3 for _, _, t in loops),
        'trajectory_length_m': float(np.linalg.norm(np.diff(track, axis=0), axis=1).sum()),
        'estimated_start_end_distance_m': float(np.linalg.norm(track[-1] - track[0])),
        'pose_correction_max_m': float(correction.max()),
        'pose_correction_median_m': float(np.median(correction)),
        'retained_node_time_span_s': poses[ordered[-1]][0] - poses[ordered[0]][0],
        'cloud_bounds_m': [xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()],
        'note': 'Endpoint distance and pose correction are not ground-truth accuracy errors.',
    }
    (output / 'metrics.json').write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, ensure_ascii=False), flush=True)
    return metrics


def write_report(output, metrics):
    manifests = [json.loads((output / mode / 'manifest.json').read_text())
                 for mode in ('lidar', 'rgbd')]
    left, right = [m['backend_parameters'] for m in manifests]
    differences = {key: {'lidar': left.get(key), 'rgbd': right.get(key)}
                   for key in sorted(left.keys() | right.keys()) if left.get(key) != right.get(key)}
    (output / 'parameter_differences.json').write_text(json.dumps(differences, indent=2))
    a, b = metrics
    rows = [('地图点数（统一导出过滤后）', 'points', '.0f'),
            ('保留图节点数', 'nodes', '.0f'),
            ('全局回环约束', 'global_loop_constraints', '.0f'),
            ('空间邻近回环约束', 'spatial_loop_constraints', '.0f'),
            ('保留关键帧折线长度 / m', 'trajectory_length_m', '.3f'),
            ('估计首尾距离 / m', 'estimated_start_end_distance_m', '.3f'),
            ('对齐起点后的最大位姿修正 / m', 'pose_correction_max_m', '.4f')]
    table = '\n'.join(f'| {label} | {a[key]:{fmt}} | {b[key]:{fmt}} |' for label, key, fmt in rows)
    diff_table = '\n'.join(f'| `{key}` | `{values["lidar"]}` | `{values["rgbd"]}` |'
                           for key, values in differences.items())
    text = f'''# 保存地图对比

纯雷达来源：`{a['source_database']}`。

融合来源：`{b['source_database']}`。

| 项目 | 雷达＋IMU | 雷达＋IMU＋RGB-D 辅助回环 |
| --- | ---: | ---: |
{table}

![俯视图](figures/top_view.png)

[三维、俯视及水平切片](figures/comparison.png) · [优化前后轨迹](figures/trajectories.png)

## 读取结果时注意

- 使用 SQLite 一致性快照导出，两份原始数据库不修改；读取已保存优化位姿，未新增回环或重新优化。
- 两组使用相同的导出体素和测距范围、坐标比例、相机视角及高度配色。底部网格关闭。
- 地图和轨迹均变换到各自第一帧坐标系；未做地图 ICP 对齐，未缩放地图。
  两轮起点位置和朝向不同，仍会影响直接比较。地图导出的原始 PLY 保留原数据库坐标系。
- 蓝线为相邻关键帧图边；橙线为空间邻近回环；红线为全局回环。反向重复约束只计一次。
- 点数受关键帧数量及覆盖范围影响，不能作为精度排名；关键帧折线长度不等于实际完整路程。
- 首尾距离要求实际回到同一点才有闭合误差意义；位姿修正量也不是对真值的误差。
- 两次真实采集的输入不同。本项目纯雷达配置关闭回环，融合配置同时开启视觉候选和空间回环；
  这是两种完整建图配置的比较，不能把差异全部归因于 RGB-D 视觉。
- 若没有全局回环约束，则本次没有保存全局视觉回环成功证据；空间回环不能替代该证据。

## 数据库记录的后端参数差异

| 参数 | 雷达＋IMU | 融合 |
| --- | --- | --- |
{diff_table}

导出设置及数据快照摘要见每组 `manifest.json`；详细指标见 `metrics.json`。
'''
    (output / 'comparison_report.md').write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('lidar_db', type=Path)
    parser.add_argument('rgbd_db', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--voxel', type=float, default=0.05)
    parser.add_argument('--max-range', type=float, default=20.0)
    parser.add_argument('--slice-z', type=float, default=0.0)
    parser.add_argument('--slice-width', type=float, default=0.3)
    args = parser.parse_args()
    sources = [p.expanduser().resolve() for p in (args.lidar_db, args.rgbd_db)]
    if sources[0] == sources[1] or not all(p.is_file() for p in sources):
        parser.error('请提供两份不同且存在的数据库：纯雷达在前，RGB-D 融合在后。')
    if not np.isfinite([args.voxel, args.max_range, args.slice_z, args.slice_width]).all() or min(
            args.voxel, args.max_range, args.slice_width) <= 0:
        parser.error('体素、范围、切片宽度必须为正有限数，切片高度必须为有限数。')
    output = (args.output or ROOT / 'logs/comparison' /
              datetime.datetime.now().strftime('saved_maps_%Y%m%d_%H%M%S_%f')).resolve()
    output.mkdir(parents=True, exist_ok=False)
    metrics = [export_run(source, output / mode, mode, args.voxel, args.max_range)
               for source, mode in zip(sources, ('lidar', 'rgbd'))]
    (output / 'metrics.json').write_text(json.dumps(metrics, ensure_ascii=False, indent=2))
    subprocess.run([sys.executable, str(ROOT / 'scripts/plot_slam_comparison.py'),
                    str(output / 'lidar'), str(output / 'rgbd'), '--output', str(output / 'figures'),
                    '--slice-z', str(args.slice_z), '--slice-width', str(args.slice_width)], check=True)
    write_report(output, metrics)
    print(f'Comparison saved: {output}', flush=True)


if __name__ == '__main__':
    main()
