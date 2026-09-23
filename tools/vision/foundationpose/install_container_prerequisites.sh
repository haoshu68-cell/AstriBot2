#!/usr/bin/env bash
# Official signed repositories only; reviewed installation for Ubuntu 22.04 amd64.
# References and the activation boundary are documented in INSTALL_CONTAINER.md.
set -Eeuo pipefail
export LC_ALL=C
umask 077

usage() {
  cat <<'EOF'
Usage:
  install_container_prerequisites.sh --prepare --evidence-dir /absolute/new/path
  sudo install_container_prerequisites.sh --install --navigation-released \
    --evidence-dir /absolute/prepared/path

--prepare requires a non-root user. It writes only the evidence directory and
downloads signed repository metadata into private APT lists/cache directories.
--install requires root and explicit confirmation that the navigation owner has
released the maintenance window. Docker can start automatically during install.
No container is run, no user is added to the docker group, and no existing
Docker/containerd installation is modified. Existing packages are never upgraded.
EOF
}

die() { printf 'REFUSED: %s\n' "$*" >&2; exit 1; }
mode= evidence_dir= navigation_released=no
while (($#)); do
  case "$1" in
    --prepare|--install) [[ -z "$mode" ]] || die 'Select one mode.'; mode=${1#--}; shift ;;
    --evidence-dir) (($# >= 2)) || die 'Missing evidence directory.'; evidence_dir=$2; shift 2 ;;
    --navigation-released) navigation_released=yes; shift ;;
    --help|-h) usage; exit 0 ;;
    *) die "Unknown argument: $1" ;;
  esac
done
[[ -n "$mode" && "$evidence_dir" == /* ]] || { usage; exit 2; }
[[ "$evidence_dir" =~ ^/[A-Za-z0-9_./-]+$ ]] || die 'Evidence path may only contain ASCII letters/digits, slash, underscore, dot and hyphen.'
if [[ "$mode" == prepare ]]; then
  ((EUID != 0)) || die '--prepare must run as the ordinary user.'
  [[ "$navigation_released" == no ]] || die 'The release flag is only for --install.'
else
  ((EUID == 0)) || die '--install requires root; use sudo yourself. The script never reads a password.'
  [[ "$navigation_released" == yes ]] || die 'The navigation owner must release the maintenance window first.'
fi
for cmd in python3 apt-get apt-cache dpkg-query dpkg curl gpg sha256sum systemctl timeout; do
  command -v "$cmd" >/dev/null || die "Required preparation dependency missing: $cmd. Do not bootstrap during navigation."
done
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && "$(dpkg --print-architecture)" == amd64 ]] || die 'Only Ubuntu 22.04 amd64 is supported.'
[[ -r /usr/share/keyrings/ubuntu-archive-keyring.gpg ]] || die 'Ubuntu archive keyring is missing.'
[[ -d /run/systemd/system ]] || die 'A systemd host is required.'

installed() { [[ "$(dpkg-query -W -f='${db:Status-Status}' "$1" 2>/dev/null || true)" == installed ]]; }
check_fresh_host() {
  local p path
  for p in docker-ce docker-ce-cli docker-ce-rootless-extras docker.io docker-compose \
    docker-compose-v2 docker-doc docker-buildx docker-buildx-plugin docker-compose-plugin \
    podman-docker containerd containerd.io runc nvidia-container-toolkit \
    nvidia-container-toolkit-base nvidia-container-runtime nvidia-docker2 \
    libnvidia-container-tools libnvidia-container1; do
    installed "$p" && die "Existing/conflicting package: $p. No automatic removal or migration."
  done
  for p in docker dockerd containerd runc nvidia-ctk; do
    command -v "$p" >/dev/null && die "Existing executable: $p. This installer only handles a fresh host."
  done
  for path in /var/lib/docker /var/lib/containerd /run/docker.sock /var/run/docker.sock; do
    [[ ! -e "$path" && ! -L "$path" ]] || die "Existing runtime state: $path"
  done
  for p in docker.service docker.socket containerd.service; do
    [[ "$(systemctl show -p LoadState --value "$p" 2>/dev/null)" == not-found ]] || die "Existing systemd unit: $p"
  done
  python3 - <<'PY'
from pathlib import Path
import re
files = [Path('/etc/apt/sources.list')]
files += list(Path('/etc/apt/sources.list.d').glob('*.list'))
files += list(Path('/etc/apt/sources.list.d').glob('*.sources'))
for p in files:
    if p.exists() and re.search(r'download\.docker\.com|nvidia\.github\.io/libnvidia-container', p.read_text()):
        raise SystemExit(f'REFUSED: existing container repository configuration: {p}')
for p in [Path('/etc/apt/keyrings/docker.asc'), Path('/etc/apt/keyrings/nvidia-container-toolkit.asc'),
          Path('/etc/apt/sources.list.d/docker.sources'), Path('/etc/apt/sources.list.d/nvidia-container-toolkit.sources')]:
    if p.exists() or p.is_symlink():
        raise SystemExit(f'REFUSED: refusing to overwrite {p}')
PY
}
check_fresh_host

if [[ "$mode" == prepare ]]; then
  [[ ! -e "$evidence_dir" && ! -L "$evidence_dir" ]] || die 'Use a new evidence directory; preparation never overwrites an earlier run.'
  mkdir -p -- "$evidence_dir"
else
  [[ -f "$evidence_dir/PREPARED" ]] || die 'Successful --prepare evidence is required.'
fi
evidence_dir=$(realpath -- "$evidence_dir")
[[ "$evidence_dir" =~ ^/[A-Za-z0-9_./-]+$ ]] || die 'Resolved evidence path contains unsupported characters.'
exec > >(tee -a "$evidence_dir/${mode}.log") 2>&1
trap 'rc=$?; if ((rc)); then printf "FAILED: exit %s; inspect %s/%s.log. No automatic rollback.\n" "$rc" "$evidence_dir" "$mode" >&2; fi' EXIT
printf 'Mode: %s | UTC: %s | evidence: %s\n' "$mode" "$(date -u +%FT%TZ)" "$evidence_dir"
work=$evidence_dir/apt
# Read this config before APT considers /etc/apt/apt.conf.d, so host update hooks
# cannot escape the preparation directory. Root installation uses the same scope.
export APT_CONFIG=$work/apt.conf
apt_opts=(-o "Dir::Etc::sourcelist=$work/sources.list"
  -o "Dir::Etc::sourceparts=$work/sources.list.d"
  -o "Dir::State::lists=$work/lists"
  -o "Dir::Cache=$work/cache"
  -o "Dir::State=$work/state" -o "Dir::Log=$work/log"
  -o 'Dir::State::status=/var/lib/dpkg/status'
  -o "APT::Sandbox::User=$(id -un)" -o APT::Get::List-Cleanup=0
  -o APT::Update::Error-Mode=any -o Acquire::AllowInsecureRepositories=false
  -o Acquire::AllowDowngradeToInsecureRepositories=false
  -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 -o Acquire::Retries=0)
install_opts=(--no-install-recommends --no-remove --no-upgrade)
log_run() { local name=$1; shift; "$@" 2>&1 | tee "$evidence_dir/$name"; }
inventory() { dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Status}\n' | sort; }
audit_plan() {
  python3 - "$1" "$2" <<'PY'
import re, subprocess, sys
plan, out = sys.argv[1:]
allowed_nvidia = {'nvidia-container-toolkit', 'nvidia-container-toolkit-base', 'libnvidia-container-tools', 'libnvidia-container1'}
result = []
for line in open(plan):
    if re.match(r'^(Remv|Purg) ', line):
        raise SystemExit('REFUSED: removal/purge in APT plan: ' + line.strip())
    if not line.startswith('Inst '):
        continue
    m = re.match(r'^Inst (\S+) \((\S+) ', line)
    if not m:
        raise SystemExit('REFUSED: upgrade or unrecognized APT action: ' + line.strip())
    package, version = m.groups()
    base = package.split(':')[0]
    if re.match(r'^(ros-|cuda|libcuda|libcublas|libcufft|libcurand|libcusolver|libcusparse|libcudnn|nsight|nvidia-|libnvidia-|linux-)', base) and base not in allowed_nvidia:
        raise SystemExit('REFUSED: driver/CUDA/ROS/kernel package in plan: ' + package)
    status = subprocess.run(['dpkg-query', '-W', '-f=${db:Status-Status}', package], capture_output=True, text=True)
    if status.stdout == 'installed':
        raise SystemExit('REFUSED: existing package would change: ' + package)
    result.append(package + '=' + version)
if not result:
    raise SystemExit('REFUSED: empty installation plan')
with open(out, 'w') as stream:
    stream.write('\n'.join(sorted(result)) + '\n')
PY
}

if [[ "$mode" == prepare ]]; then
  mkdir -p "$work"/{sources.list.d,lists/partial,cache/archives/partial,keyrings,gnupg,apt.conf.d,state,log}
  cat > "$APT_CONFIG" <<EOF
Dir::Etc::parts "$work/apt.conf.d";
Dir::Etc::main "/dev/null";
EOF
  : > "$work/sources.list"
  log_run dependency-versions.txt dpkg-query -W -f='${binary:Package}\t${Version}\t${db:Status-Status}\n' ca-certificates curl gnupg python3 apt
  inventory > "$evidence_dir/packages.before.tsv"
  curl --fail --show-error --silent --location --proto '=https' --proto-redir '=https' \
    --connect-timeout 15 --max-time 90 https://download.docker.com/linux/ubuntu/gpg -o "$work/keyrings/docker.asc"
  curl --fail --show-error --silent --location --proto '=https' --proto-redir '=https' \
    --connect-timeout 15 --max-time 90 https://nvidia.github.io/libnvidia-container/gpgkey -o "$work/keyrings/nvidia-container-toolkit.asc"
  for key in "$work/keyrings/"*.asc; do
    gpg --batch --homedir "$work/gnupg" --show-keys --with-fingerprint "$key"
  done
  cat > "$work/sources.list.d/ubuntu.sources" <<'EOF'
Types: deb
URIs: https://archive.ubuntu.com/ubuntu
Suites: jammy jammy-updates
Components: main universe restricted multiverse
Architectures: amd64
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: https://security.ubuntu.com/ubuntu
Suites: jammy-security
Components: main universe restricted multiverse
Architectures: amd64
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF
  cat > "$work/sources.list.d/docker.sources" <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: amd64
Signed-By: $work/keyrings/docker.asc
EOF
  cat > "$work/sources.list.d/nvidia-container-toolkit.sources" <<EOF
Types: deb
URIs: https://nvidia.github.io/libnvidia-container/stable/deb/amd64
Suites: /
Architectures: amd64
Signed-By: $work/keyrings/nvidia-container-toolkit.asc
EOF
  log_run apt-update.prepare.txt timeout --signal=TERM --kill-after=10s 300s apt-get "${apt_opts[@]}" update
  targets=(docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    nvidia-container-toolkit nvidia-container-toolkit-base libnvidia-container-tools libnvidia-container1)
  : > "$evidence_dir/requested-packages.txt"
  for package in "${targets[@]}"; do
    candidate=$(apt-cache "${apt_opts[@]}" policy "$package" | awk '/Candidate:/ {print $2}')
    [[ -n "$candidate" && "$candidate" != '(none)' ]] || die "No signed repository candidate for $package"
    printf '%s=%s\n' "$package" "$candidate" >> "$evidence_dir/requested-packages.txt"
  done
  apt-cache "${apt_opts[@]}" policy "${targets[@]}" > "$evidence_dir/apt-policy.txt"
  cat "$evidence_dir/requested-packages.txt"
  mapfile -t requests < "$evidence_dir/requested-packages.txt"
  log_run apt-simulate.prepare.txt apt-get "${apt_opts[@]}" "${install_opts[@]}" --simulate install "${requests[@]}"
  audit_plan "$evidence_dir/apt-simulate.prepare.txt" "$evidence_dir/planned-packages.txt"
  # Freeze every newly installed dependency, not just the top-level packages.
  mapfile -t pinned < "$evidence_dir/planned-packages.txt"
  log_run apt-simulate.pinned.txt apt-get "${apt_opts[@]}" "${install_opts[@]}" --simulate install "${pinned[@]}"
  audit_plan "$evidence_dir/apt-simulate.pinned.txt" "$evidence_dir/pinned-check.txt"
  cmp "$evidence_dir/planned-packages.txt" "$evidence_dir/pinned-check.txt" || die 'Pinning changed the package transaction.'
  printf 'Prepared at %s; no system files/packages/services changed.\n' "$(date -u +%FT%TZ)" > "$evidence_dir/PREPARED"
  printf 'PREPARED only. Review planned-packages.txt and apt-policy.txt. Installation still requires navigation release.\n'
  exit 0
fi

# Installation only begins here, after the explicit release flag and freshness checks.
inventory > "$evidence_dir/packages.at-install.tsv"
cmp "$evidence_dir/packages.before.tsv" "$evidence_dir/packages.at-install.tsv" || die 'Host package inventory changed; prepare again in a new directory.'
mapfile -t pinned < "$evidence_dir/planned-packages.txt"
((${#pinned[@]} > 0)) || die 'Missing frozen package transaction.'
# Refresh and authenticate all repository metadata before root consumes the plan.
log_run apt-update.install.txt timeout --signal=TERM --kill-after=10s 300s apt-get "${apt_opts[@]}" update
log_run apt-simulate.install.txt apt-get "${apt_opts[@]}" "${install_opts[@]}" --simulate install "${pinned[@]}"
audit_plan "$evidence_dir/apt-simulate.install.txt" "$evidence_dir/install-check.txt"
cmp "$evidence_dir/planned-packages.txt" "$evidence_dir/install-check.txt" || die 'Installation transaction differs from reviewed preparation.'
mkdir -p "$evidence_dir/backups"
if [[ -e /etc/docker/daemon.json || -L /etc/docker/daemon.json ]]; then
  [[ -f /etc/docker/daemon.json && ! -L /etc/docker/daemon.json ]] || die 'daemon.json must be a regular file.'
  cp -a /etc/docker/daemon.json "$evidence_dir/backups/daemon.json.before"
  python3 -m json.tool /etc/docker/daemon.json >/dev/null
else
  printf '/etc/docker/daemon.json did not exist before installation.\n' > "$evidence_dir/backups/daemon.json.ABSENT"
fi
command -v ip >/dev/null && ip -j address > "$evidence_dir/ip-address.before.json"
command -v ip >/dev/null && ip -j route > "$evidence_dir/ip-route.before.json"
command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=uuid,driver_version --format=csv,noheader > "$evidence_dir/gpu-driver.before.txt"
install -d -m 0755 /etc/apt/keyrings
for key in docker nvidia-container-toolkit; do
  install -m 0644 "$work/keyrings/$key.asc" "/etc/apt/keyrings/$key.asc"
  sed "s|$work/keyrings/|/etc/apt/keyrings/|g" "$work/sources.list.d/$key.sources" > "/etc/apt/sources.list.d/$key.sources"
  chmod 0644 "/etc/apt/sources.list.d/$key.sources"
done
# Preserve existing dpkg conffiles; never upgrade/remove packages or auto-restart
# unrelated existing services through needrestart. Docker's own postinst may start it.
log_run apt-install.txt env DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=l \
  apt-get "${apt_opts[@]}" "${install_opts[@]}" -o Dpkg::Options::=--force-confold --yes install "${pinned[@]}"
log_run nvidia-runtime-configure.txt nvidia-ctk runtime configure --runtime=docker
log_run daemon-validation.txt dockerd --validate --config-file=/etc/docker/daemon.json
cp -a /etc/docker/daemon.json "$evidence_dir/daemon.json.after"
log_run docker-restart.txt systemctl restart docker
docker_client=(env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH docker --host unix:///var/run/docker.sock)
log_run docker-version.txt "${docker_client[@]}" version
log_run docker-runtimes.txt "${docker_client[@]}" info --format '{{json .Runtimes}}'
python3 - "$evidence_dir/docker-runtimes.txt" <<'PY'
import json, sys
if 'nvidia' not in json.load(open(sys.argv[1])):
    raise SystemExit('REFUSED: local Docker daemon did not register the NVIDIA runtime')
PY
log_run toolkit-version.txt nvidia-ctk --version
inventory > "$evidence_dir/packages.after.tsv"
python3 - "$evidence_dir" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1])
def installed(name):
    return {p.removesuffix(':amd64'): v for p, v, status in
            (line.rstrip('\n').split('\t') for line in (root/name).read_text().splitlines())
            if status == 'installed'}
before, after = installed('packages.before.tsv'), installed('packages.after.tsv')
planned = dict(line.split('=', 1) for line in (root/'planned-packages.txt').read_text().splitlines())
planned = {p.removesuffix(':amd64'): v for p, v in planned.items()}
if any(after.get(p) != v for p, v in before.items()):
    raise SystemExit('REFUSED: pre-existing installed package changed; inspect inventories')
if {p: v for p, v in after.items() if p not in before} != planned:
    raise SystemExit('REFUSED: installed package delta differs from the frozen transaction')
PY
command -v ip >/dev/null && ip -j address > "$evidence_dir/ip-address.after.json"
command -v ip >/dev/null && ip -j route > "$evidence_dir/ip-route.after.json"
if [[ -f "$evidence_dir/gpu-driver.before.txt" ]]; then
  nvidia-smi --query-gpu=uuid,driver_version --format=csv,noheader > "$evidence_dir/gpu-driver.after.txt"
  cmp "$evidence_dir/gpu-driver.before.txt" "$evidence_dir/gpu-driver.after.txt" || die 'GPU driver identity changed; inspect evidence.'
fi
printf 'Installed at %s. No container was launched; no docker-group membership was changed.\n' "$(date -u +%FT%TZ)" > "$evidence_dir/INSTALLED"
printf 'INSTALLED. Use sudo docker. Container/GPU workload acceptance is still pending.\n'
