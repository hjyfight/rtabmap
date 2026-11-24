#!/usr/bin/env bash
# Run on the current workstation once; the deployed service needs no workstation.
set -eo pipefail
ORIN_DEPLOY_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ORIN_DEPLOY_ROOT"
ORIN_DEPLOY_HOST=${1:-qiyuan@192.168.1.103}
ORIN_DEPLOY_REMOTE=${2:-/home/qiyuan/Project/rtabmap_slam}
[[ $ORIN_DEPLOY_HOST =~ ^[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+$ && $ORIN_DEPLOY_REMOTE =~ ^/[A-Za-z0-9_./-]+$ ]] || {
  printf '需要 用户@主机 和项目绝对路径（不含空格）。\n' >&2; exit 1;
}
ORIN_NODE_VERSION=v22.16.0
ORIN_NODE_ARCHIVE="node-$ORIN_NODE_VERSION-linux-arm64.tar.xz"
mkdir -p .cache/orin-web
if [[ ! -f .cache/orin-web/$ORIN_NODE_ARCHIVE || ! -f .cache/orin-web/SHASUMS256.txt ]]; then
  curl -fL --connect-timeout 15 --max-time 180 "https://nodejs.org/dist/$ORIN_NODE_VERSION/SHASUMS256.txt" -o .cache/orin-web/SHASUMS256.txt
  curl -fL --connect-timeout 15 --max-time 180 "https://nodejs.org/dist/$ORIN_NODE_VERSION/$ORIN_NODE_ARCHIVE" -o ".cache/orin-web/$ORIN_NODE_ARCHIVE"
fi
python3 - "$ORIN_NODE_ARCHIVE" <<'PY'
import hashlib,sys
from pathlib import Path
name=sys.argv[1];root=Path('.cache/orin-web')
rows=[line.split() for line in (root/'SHASUMS256.txt').read_text().splitlines()]
expected=next(row[0] for row in rows if row[1]==name)
actual=hashlib.sha256((root/name).read_bytes()).hexdigest()
if expected!=actual:raise SystemExit('Official Node.js SHA256 mismatch')
print('Official ARM64 Node.js archive SHA256 verified')
PY
ssh -o BatchMode=yes -o ConnectTimeout=8 "$ORIN_DEPLOY_HOST" \
  "test \"\$(uname -m)\" = aarch64 && mkdir -p '$ORIN_DEPLOY_REMOTE/.cache/orin-web' '$ORIN_DEPLOY_REMOTE/.tools/miti-research/root/usr/lib/miti'"
rsync -rt --compress --bwlimit=1000 -e 'ssh -o BatchMode=yes -o ConnectTimeout=8' \
  ".cache/orin-web/$ORIN_NODE_ARCHIVE" .cache/orin-web/SHASUMS256.txt \
  "$ORIN_DEPLOY_HOST:$ORIN_DEPLOY_REMOTE/.cache/orin-web/"
rsync -rt --compress --copy-links --bwlimit=1000 -e 'ssh -o BatchMode=yes -o ConnectTimeout=8' \
  --exclude='/node' --exclude='/public/research' --exclude='/.next/cache' --exclude='/.miti' \
  .tools/miti-research/root/usr/lib/miti/ \
  "$ORIN_DEPLOY_HOST:$ORIN_DEPLOY_REMOTE/.tools/miti-research/root/usr/lib/miti/"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$ORIN_DEPLOY_HOST" \
  "cd '$ORIN_DEPLOY_REMOTE' && tar -xJf '.cache/orin-web/$ORIN_NODE_ARCHIVE' -C .tools && ln -sfn '$ORIN_DEPLOY_REMOTE/.tools/node-$ORIN_NODE_VERSION-linux-arm64' '.tools/miti-research/root/usr/lib/miti/node' && .tools/node-$ORIN_NODE_VERSION-linux-arm64/bin/node --version"
printf 'Portable Orin web assets deployed. Run bash scripts/start_orin_research_web.sh on Orin.\n'
