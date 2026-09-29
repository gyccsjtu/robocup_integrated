#!/usr/bin/env bash
# Prepare the verified ROS Noetic / Gazebo 11 / PX4 1.13.2 baseline on an
# Ubuntu 20.04 virtual machine. Run from a Linux checkout of robocup_ws.
#
# Offline-friendly design (2026-09-07):
#   * the VM cannot reach github.com / raw.githubusercontent.com, so every
#     network fetch prefers a reachable mirror or a local file;
#   * PX4 / XTDrone sources are transferred from the host instead of cloned;
#     when the pinned commit is already checked out nothing is fetched.
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
px4_dir="${repo_root}/third_party/PX4-Autopilot"
xtdrone_dir="${repo_root}/third_party/XTDrone"
px4_commit="46a12a09bf11c8cbafc5ad905996645b4fe1a9df"
xtdrone_commit="62339a816ef815113a0366a62e8aca4be3000f80"

# mirrors reachable from inside the VM (github.com is not)
ROS_MIRROR="${ROS_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu}"
ROSDISTRO_MIRROR="${ROSDISTRO_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro}"
PYPI_INDEX="${PYPI_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
ROS_KEY_FILE="${ROS_KEY_FILE:-/tmp/ros.key}"

log() { printf '[robocup-vm] %s\n' "$*"; }
warn() { printf '[robocup-vm] WARN: %s\n' "$*" >&2; }
fail() { printf '[robocup-vm] ERROR: %s\n' "$*" >&2; exit 1; }

[[ "$(. /etc/os-release; printf '%s' "$VERSION_ID")" == "20.04" ]] || \
  fail 'This script is pinned to Ubuntu 20.04.'

log 'ensuring curl/ca-certificates are present'
sudo apt-get update -y
sudo apt-get install -y curl ca-certificates

# ---------------------------------------------------------------- ROS apt key
log 'installing the ROS archive signing key (local file first, mirrors after)'
# a previous failed fetch can leave a zero-byte keyring; treat that as missing
if [[ -f /usr/share/keyrings/ros-archive-keyring.gpg && ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  warn 'removing an empty ros-archive-keyring.gpg left by an earlier run'
  sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
fi
if [[ ! -s /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  if [[ -f "${ROS_KEY_FILE}" ]]; then
    log "using local key ${ROS_KEY_FILE}"
    gpg --dearmor < "${ROS_KEY_FILE}" | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
  else
    log 'local key missing; trying mirrors'
    for url in \
      "${ROSDISTRO_MIRROR}/ros.key" \
      "http://repo.ros2.org/repos.key" \
      "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"; do
      if curl -fsSL --max-time 25 "${url}" | sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg; then
        log "key fetched from ${url}"; break
      fi
      warn "key fetch failed from ${url}"
    done
  fi
fi
[[ -s /usr/share/keyrings/ros-archive-keyring.gpg ]] || fail 'ROS archive key unavailable.'

# -------------------------------------------------------------- ROS apt source
log "adding the ROS Noetic package source (${ROS_MIRROR})"
if [[ ! -f /etc/apt/sources.list.d/ros1-latest.list ]]; then
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} focal main" |
    sudo tee /etc/apt/sources.list.d/ros1-latest.list >/dev/null
fi
sudo apt-get update -y

# ------------------------------------------------------------------ apt packages
log 'installing Ubuntu, ROS Noetic, Gazebo 11, MAVROS, and PX4 build dependencies'
sudo apt-get install -y \
  build-essential ca-certificates cmake curl gawk gazebo11 genromfs git \
  libeigen3-dev libgazebo11-dev libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev libopencv-dev libxml2-dev libxml2-utils ninja-build \
  pkg-config protobuf-compiler python3-catkin-tools python3-dev python3-empy \
  python3-jinja2 python3-numpy python3-pip python3-rosdep python3-toml \
  python3-yaml rsync unzip wget zip

sudo apt-get install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras

if [[ -f /usr/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  log 'GeographicLib datasets already installed; skipping download'
else
  log 'installing GeographicLib datasets (best effort: needs sourceforge)'
  sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh || \
    warn 'GeographicLib datasets failed; MAVROS may warn until they are installed.'
fi

# ------------------------------------------------------------------ rosdep
# rosdep defaults to raw.githubusercontent.com, which this VM cannot reach.
log 'initialising rosdep against the tuna rosdistro mirror (best effort)'
export ROSDISTRO_INDEX_URL="${ROSDISTRO_MIRROR}/index-v4.yaml"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init >/dev/null 2>&1 || warn 'rosdep init failed (continuing).'
fi
if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo sed -i "s|https://raw.githubusercontent.com/ros/rosdistro/master|${ROSDISTRO_MIRROR}|g" \
    /etc/ros/rosdep/sources.list.d/20-default.list || true
fi
rosdep update || warn 'rosdep update failed (continuing; not required for the pinned build).'

# ------------------------------------------------------------------ pip
# pip's bundled TLS stack cannot complete a handshake inside this VM (the
# vendored urllib3 fails with SSLEOFError while the system stack works), so a
# wheelhouse prepared on the host is preferred over a live index.
log 'installing the PX4 Python requirements'
PIP_ARGS=(--user --no-cache-dir)
if [[ -n "${PIP_PROXY:-}" ]]; then PIP_ARGS+=(--proxy "${PIP_PROXY}"); fi
WHEELHOUSE="${WHEELHOUSE:-${HOME}/robocup/wheels}"
if [[ -d "${WHEELHOUSE}" ]] && compgen -G "${WHEELHOUSE}/*" > /dev/null; then
  log "installing offline from the wheelhouse ${WHEELHOUSE}"
  python3 -m pip install "${PIP_ARGS[@]}" --no-index --find-links "${WHEELHOUSE}" \
    -r "${repo_root}/deps/requirements-px4-1.13.txt"
else
  log "installing from ${PYPI_INDEX}"
  python3 -m pip install "${PIP_ARGS[@]}" -i "${PYPI_INDEX}" \
    -r "${repo_root}/deps/requirements-px4-1.13.txt"
fi

# --------------------------------------------------------- PX4 / XTDrone sources
# Sources are transferred from the host (the VM cannot clone from GitHub).
# Clone only when they are genuinely missing.
mkdir -p "${repo_root}/third_party"

if [[ ! -d "${px4_dir}/.git" ]]; then
  log 'PX4 source missing; cloning (requires GitHub access)'
  git clone https://github.com/PX4/PX4-Autopilot.git "${px4_dir}"
fi
if [[ "$(git -C "${px4_dir}" rev-parse HEAD 2>/dev/null || true)" != "${px4_commit}" ]]; then
  log 'checking out the pinned PX4 commit'
  git -C "${px4_dir}" checkout --detach "${px4_commit}"
else
  log "PX4 already at ${px4_commit:0:8}; skipping fetch/checkout (offline)"
fi
if git -C "${px4_dir}" submodule status --recursive 2>/dev/null | grep -q '^-'; then
  log 'some PX4 submodules are uninitialised; updating them locally'
  git -C "${px4_dir}" submodule update --init --recursive || warn 'submodule update incomplete.'
else
  log 'PX4 submodules already initialised; skipping (offline)'
fi

if [[ ! -d "${xtdrone_dir}/.git" ]]; then
  log 'XTDrone source missing; cloning from Gitee'
  git clone --branch 1_13_2 https://gitee.com/robin_shaun/XTDrone.git "${xtdrone_dir}"
fi
if [[ "$(git -C "${xtdrone_dir}" rev-parse HEAD 2>/dev/null || true)" != "${xtdrone_commit}" ]]; then
  git -C "${xtdrone_dir}" checkout --detach "${xtdrone_commit}" 2>/dev/null || \
    warn "XTDrone pinned commit unavailable locally; staying at $(git -C "${xtdrone_dir}" rev-parse --short HEAD)"
else
  log "XTDrone already at ${xtdrone_commit:0:8}; skipping fetch/checkout (offline)"
fi

[[ "$(git -C "${px4_dir}" rev-parse HEAD)" == "${px4_commit}" ]] || fail 'PX4 revision verification failed.'

# ------------------------------------------------------------------ build
log 'applying the repository XTDrone overlay and building PX4 SITL'
ROBOCUP_WORKSPACE="${repo_root}" PX4_SOURCE_DIR="${px4_dir}" XTD_SOURCE_DIR="${xtdrone_dir}" \
  bash "${repo_root}/scripts/container/prepare_xtdrone_px4.sh"
make -C "${px4_dir}" px4_sitl_default

log 'building the XTDrone Gazebo ROS packages'
ROBOCUP_WORKSPACE="${repo_root}" PX4_SOURCE_DIR="${px4_dir}" XTD_SOURCE_DIR="${xtdrone_dir}" \
  XTD_CATKIN_WS="${repo_root}/build/xtdrone_catkin_ws" \
  bash "${repo_root}/scripts/container/build_xtdrone_gazebo_ros.sh"

log 'baseline build complete; run scripts/vm/start_single_uav_native.sh next'
