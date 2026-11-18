#!/usr/bin/env bash
# Run on the workstation. Preview by default; preserve Orin runtime data.
set -euo pipefail
SYNC_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SYNC_APPLY=false
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  printf '用法：bash scripts/sync_orin_source.sh [--apply] [用户@主机 [Orin项目绝对路径]]\n默认只预览；--apply 同步源码并备份被覆盖文件，不自动构建或启动节点。\n'
  exit 0
fi
if [[ ${1:-} == --apply ]]; then
  SYNC_APPLY=true
  shift
fi
if (( $# > 2 )); then
  printf '用法：bash scripts/sync_orin_source.sh [--apply] [用户@主机 [Orin项目绝对路径]]\n' >&2
  exit 2
fi
SYNC_HOST=${1:-qiyuan@192.168.1.103}
SYNC_REMOTE=${2:-/home/qiyuan/Project/rtabmap_slam}
if [[ ! $SYNC_HOST =~ ^[a-zA-Z0-9_.-]+@[a-zA-Z0-9_.-]+$ ||
      ! $SYNC_REMOTE =~ ^/[a-zA-Z0-9_./-]+$ || $SYNC_REMOTE == / ]]; then
  printf '请填写 用户@主机 和不含空格的项目绝对路径。\n' >&2
  exit 2
fi
command -v rsync >/dev/null
command -v git >/dev/null
SYNC_STAMP=$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S_%N)
SYNC_LOG="$SYNC_PROJECT/logs/orin_source_sync/$SYNC_STAMP"
SYNC_BACKUP="$SYNC_REMOTE/logs/source_sync_backups/$SYNC_STAMP"
mkdir -p "$SYNC_LOG"
python3 - "$SYNC_PROJECT" "$SYNC_LOG" <<'PY'
import hashlib
import json
from pathlib import Path
import stat
import subprocess
import sys

root, output = map(Path, sys.argv[1:])
paths = subprocess.check_output([
    'git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard', '--',
    'README.md', '.gitignore', 'setup.bash', 'config', 'docs', 'launch',
    'scripts', 'src', 'tests'], cwd=root).split(b'\0')
records = []
for raw in sorted(set(paths)):
    if not raw:
        continue
    name = raw.decode('utf-8')
    path = root / name
    if not path.is_file() and not path.is_symlink():
        continue
    mode = stat.S_IMODE(path.lstat().st_mode)
    if path.is_symlink():
        records.append(dict(path=name, mode=mode, link=str(path.readlink())))
    else:
        records.append(dict(path=name, mode=mode,
                            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            size=path.stat().st_size))
if not records:
    raise SystemExit('没有找到待同步源码。请在工作站的 Git 项目中运行。')
(output / 'files.list0').write_bytes(b''.join(
    row['path'].encode('utf-8') + b'\0' for row in records))
(output / 'manifest.json').write_text(json.dumps(records, ensure_ascii=False, indent=2) + '\n')
print(f"源码清单：{len(records)} 个文件，{sum(row.get('size', 0) for row in records):,} 字节。")
PY
SYNC_OPTIONS=(--recursive --links --times --perms --checksum --relative
  --compress --protect-args --from0 --files-from="$SYNC_LOG/files.list0"
  --backup --backup-dir="$SYNC_BACKUP" --itemize-changes --stats
  --log-file="$SYNC_LOG/rsync.log" -e 'ssh -o ConnectTimeout=8')
if ! $SYNC_APPLY; then
  SYNC_OPTIONS+=(--dry-run)
  printf '预览源码同步：%s:%s\n' "$SYNC_HOST" "$SYNC_REMOTE"
else
  printf '同步源码：%s:%s\n覆盖文件备份：%s\n' "$SYNC_HOST" "$SYNC_REMOTE" "$SYNC_BACKUP"
fi
if rsync "${SYNC_OPTIONS[@]}" "$SYNC_PROJECT/" "$SYNC_HOST:$SYNC_REMOTE/" >"$SYNC_LOG/output.txt"; then
  tail -n 23 "$SYNC_LOG/output.txt"
else
  SYNC_STATUS=$?
  tail -n 30 "$SYNC_LOG/output.txt" >&2
  exit "$SYNC_STATUS"
fi
printf '本次清单和日志：%s\n' "$SYNC_LOG"
if $SYNC_APPLY; then
  printf '源码同步完成。在 Orin 项目目录执行 bash scripts/build_semantic.sh --headless，然后运行相关测试。\n'
fi
