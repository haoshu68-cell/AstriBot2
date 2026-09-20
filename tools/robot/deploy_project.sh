#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == --help ]]; then
    echo "用法: ROBOT=astribot@10.249.22.137 bash $0 [版本名]"
    echo '同步当前源码到机器人独立版本目录并编译；不切换运行服务、不启动 SDK 或发送运动指令。'
    exit 0
fi
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT="${ROBOT:-astribot@10.249.22.137}"
VERSION="${1:-$(date +%Y%m%d_%H%M%S)}"
BASE="${REMOTE_PROJECT_BASE:-/home/astribot/astribot_projects}"
VENDOR="${REMOTE_VENDOR_SDK:-/home/astribot/Downloads/astribot_sdk_aarch64}"
[[ "$VERSION" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '版本名仅允许字母、数字、下划线、连字符' >&2; exit 2; }
for path in "$BASE" "$VENDOR"; do
    [[ "$path" =~ ^/[A-Za-z0-9_/-]+$ ]] || { echo "不支持的远端路径: $path" >&2; exit 2; }
done
REMOTE="$BASE/releases/$VERSION"
SSH=(ssh -o ConnectTimeout=10 "$ROBOT")
"${SSH[@]}" "test \"\$(uname -m)\" = aarch64 && test -f '$VENDOR/env.sh' && test ! -e '$REMOTE'"

SNAPSHOT="$(mktemp -d /tmp/astribot-deploy.XXXXXX)"
trap 'rm -rf -- "$SNAPSHOT"' EXIT
for entry in tools docs examples config astribot_config maps chassis_benchmark README.md README_zh.md LICENSE env.sh install.sh; do
    rsync -a --exclude '__pycache__' --exclude '*.pyc' --exclude .git \
        --exclude .pytest_cache --exclude results "$REPO/$entry" "$SNAPSHOT/"
done
mkdir -p "$SNAPSHOT/ws_robot" "$SNAPSHOT/deployment"
rsync -a --exclude '__pycache__' --exclude '*.pyc' --exclude .git --exclude .pytest_cache \
    "$REPO/ws_robot/src" "$REPO/ws_robot/third_party" "$REPO/ws_robot/README.md" "$SNAPSHOT/ws_robot/"
python3 - "$SNAPSHOT" "$REPO" "$VERSION" "$VENDOR" <<'PY'
import hashlib, json, shutil, subprocess, sys
from pathlib import Path
root, repo = map(Path, sys.argv[1:3])
# Keep the local dependency sources for audit without replacing ARM vendor binaries
# or mixing the vendor Python ABI with the workstation's native extensions.
source_suffixes = {'.py', '.pyi', '.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp',
                   '.hxx', '.sh', '.bash', '.cmake', '.msg', '.srv', '.action',
                   '.proto', '.xml', '.yaml', '.yml', '.json', '.md', '.txt'}
for dependency in ('astribot_sdk', 'astribot_msgs', 'third_party'):
    for source in (repo / dependency).rglob('*'):
        if not source.is_file() or source.is_symlink():
            continue
        if any(p in {'.git', '__pycache__', '.pytest_cache'} for p in source.parts):
            continue
        if source.suffix not in source_suffixes:
            continue
        target = root / 'deployment/source_reference' / source.relative_to(repo)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted(root.rglob('*')) if p.is_file()}
record = {'version': sys.argv[3], 'git_head': subprocess.check_output(
    ['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
    'working_tree_status': subprocess.check_output(
    ['git', '-C', str(repo), 'status', '--short', '--untracked-files=normal'], text=True),
    'vendor_sdk_root': sys.argv[4], 'native_dependencies': ['astribot_sdk', 'astribot_msgs', 'third_party'],
    'dependency_source_reference': 'deployment/source_reference (audit only; runtime uses ARM vendor SDK)',
    'excluded': ['.git', 'runs', 'build', 'install', 'log', 'Python caches', 'local native SDK binaries'],
    'sha256': files}
(root / 'deployment/source_manifest.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
print('源码快照:', len(files), '个文件；版本', sys.argv[3])
PY
"${SSH[@]}" "mkdir -p '$BASE/releases' && mkdir '$REMOTE'"
rsync -az --no-owner --no-group --stats "$SNAPSHOT/" "$ROBOT:$REMOTE/"
"${SSH[@]}" "ln -s '$VENDOR/astribot_sdk' '$REMOTE/astribot_sdk' && ln -s '$VENDOR/astribot_msgs' '$REMOTE/astribot_msgs' && ln -s '$VENDOR/third_party' '$REMOTE/third_party'"
"${SSH[@]}" "python3 - '$REMOTE'" <<'PY'
import hashlib, json, sys
from pathlib import Path
root=Path(sys.argv[1]); manifest=json.loads((root/'deployment/source_manifest.json').read_text())
bad=[name for name, digest in manifest['sha256'].items()
     if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest]
if bad: raise SystemExit('源码校验失败: '+str(bad))
print('机器人 SHA256 校验通过:', len(manifest['sha256']), '个文件')
PY
"${SSH[@]}" "env -i HOME=/home/astribot USER=astribot LANG=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 bash --noprofile --norc '$REMOTE/tools/robot/build_robot.sh' --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > '$REMOTE/deployment/build.log' 2>&1" || {
    "${SSH[@]}" "tail -70 '$REMOTE/deployment/build.log'"
    exit 1
}
"${SSH[@]}" "tail -25 '$REMOTE/deployment/build.log'"
echo "部署构建完成: $REMOTE"
echo '接下来执行操作手册中的加载校验和只读检查，再选择启动时间。'
