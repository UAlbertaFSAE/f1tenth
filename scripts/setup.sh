#!/usr/bin/env bash
set -euo pipefail

# Repo-level environment bootstrapper for this workspace.
# - Installs ROS 2 Humble if missing (Ubuntu 22.04 only, otherwise prints guidance)
# - Installs prerequisites (Eigen3, build tools)
# - Installs CDT (artem-ogre/CDT) if missing
# - Runs rosdep to install ROS package deps
# - Creates/updates a Python venv and installs Python deps + f1tenth_gym
# - Installs shell aliases by sourcing scripts/formula_bashrc.sh from ~/.bashrc

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# This script lives at: <workspace>/src/scripts/setup.sh
FORMULA_HOME="$(cd -- "$SCRIPT_DIR/../.." && pwd -P)"  # workspace root
SETUP_SCRIPT="$FORMULA_HOME/src/scripts/setup.sh"

# Prefer system Python for ROS/rosdep compatibility (avoid conda-base python).
SYSTEM_PYTHON="/usr/bin/python3"
if [[ ! -x "$SYSTEM_PYTHON" ]]; then
  SYSTEM_PYTHON="python3"
fi

# ---------- logging helpers ----------
log() { echo -e "[setup] $*"; }
warn() { echo -e "[setup][WARN] $*"; }
err() { echo -e "[setup][ERROR] $*" >&2; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1
}

ensure_colcon_ignores_venv() {
  # If a venv exists under the ROS source tree (<ws>/src/venv), colcon may try to
  # treat Python packages inside site-packages as ROS packages, causing cryptic
  # build errors (Cython/cx_Freeze missing, etc.).
  local legacy_venv="$FORMULA_HOME/src/venv"
  if [[ -d "$legacy_venv" ]]; then
    touch "$legacy_venv/COLCON_IGNORE" || true
  fi
}

source_ros_humble() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    err "Missing /opt/ros/humble/setup.bash (ROS 2 Humble not installed or not sourced)"
    return 1
  fi

  # ROS setup scripts may reference unset variables; with `set -u` this errors.
  local had_nounset=0
  case $- in
    *u*) had_nounset=1 ;;
  esac

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ $had_nounset -eq 1 ]]; then
    set -u
  fi
}

require_sudo() {
  if ! need_cmd sudo; then
    err "sudo not found; cannot install system packages."
    exit 1
  fi
  sudo -v
}

is_ubuntu_jammy() {
  if [[ ! -f /etc/os-release ]]; then
    return 1
  fi
  # shellcheck disable=SC1091
  . /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_CODENAME:-}" == "jammy" ]]
}

has_ros2_humble() {
  [[ -f /opt/ros/humble/setup.bash ]] || need_cmd ros2
}

has_cuda() {
  # "CUDA available" heuristic: NVIDIA driver/tooling or CUDA toolkit present.
  if need_cmd nvidia-smi; then
    nvidia-smi >/dev/null 2>&1 && return 0
  fi
  if need_cmd nvcc; then
    nvcc --version >/dev/null 2>&1 && return 0
  fi
  [[ -d /usr/local/cuda ]] && return 0
  return 1
}

has_zed_sdk() {
  [[ -d /usr/local/zed ]] && return 0
  # libsl_zed is part of the SDK; check ldconfig cache if available.
  if need_cmd ldconfig; then
    ldconfig -p 2>/dev/null | grep -q "libsl_zed" && return 0
  fi
  return 1
}

install_ros2_humble_ubuntu() {
  log "Installing ROS 2 Humble (Ubuntu 22.04/jammy)…"
  require_sudo

  sudo apt-get update -y
  sudo apt-get install -y --no-install-recommends \
    locales \
    curl \
    gnupg \
    lsb-release

  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  sudo mkdir -p /etc/apt/keyrings
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

  sudo apt-get update -y
  sudo apt-get install -y \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions
}

install_apt_prereqs() {
  log "Installing apt prerequisites (build tools, Eigen3, rosdep helpers)…"
  require_sudo
  sudo apt-get update -y --fix-missing
  sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-catkin-pkg \
    python3-empy \
    libeigen3-dev \
    ros-humble-asio-cmake-module
}

init_rosdep_if_needed() {
  # Some systems have an old rosdep source list pointing at python3-rosdep2's
  # debian.yaml. If python3-rosdep2 is removed, `rosdep update` fails with:
  #   file:///usr/share/python3-rosdep2/debian.yaml: No such file or directory
  # Remove the stale list so rosdep can update normally.
  if [[ -f /etc/ros/rosdep/sources.list.d/10-debian.list ]] \
    && grep -q "file:///usr/share/python3-rosdep2/debian.yaml" /etc/ros/rosdep/sources.list.d/10-debian.list \
    && [[ ! -f /usr/share/python3-rosdep2/debian.yaml ]]; then
    warn "Removing stale rosdep source /etc/ros/rosdep/sources.list.d/10-debian.list (references missing python3-rosdep2/debian.yaml)…"
    require_sudo
    sudo rm -f /etc/ros/rosdep/sources.list.d/10-debian.list
  fi

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep…"
    require_sudo
    sudo rosdep init
  fi
  log "Updating rosdep…"
  # Clear user's cache index so sources list changes take effect immediately.
  rm -f "$HOME/.ros/rosdep/sources.cache/index" 2>/dev/null || true
  rosdep update
}

install_rosdeps_for_workspace() {
  # Detect source-space: prefer FORMULA_HOME/src (this repo) else FORMULA_HOME/src/src.
  local from_path
  if [[ -d "$FORMULA_HOME/src" ]]; then
    from_path="$FORMULA_HOME/src"
  else
    err "Cannot find source directory at $FORMULA_HOME/src"
    exit 1
  fi

  log "Installing ROS dependencies via rosdep (from $from_path)…"
  source_ros_humble
  # Ensure pip rosdeps install into the active venv (no sudo).
  rosdep install -i --from-paths "$from_path" --rosdistro humble -y --as-root pip:false
}

install_cdt_if_needed() {
  if [[ -f /usr/local/include/CDT.h ]]; then
    log "CDT already installed (/usr/local/include/CDT.h found)."
    return 0
  fi

  log "Installing CDT (artem-ogre/CDT) to /usr/local…"
  require_sudo

  local workdir="$FORMULA_HOME/.deps"
  mkdir -p "$workdir"

  if [[ ! -d "$workdir/CDT/.git" ]]; then
    git clone --depth 1 https://github.com/artem-ogre/CDT.git "$workdir/CDT"
  else
    (cd "$workdir/CDT" && git fetch --depth 1 origin && git reset --hard origin/master)
  fi

  rm -rf "$workdir/CDT/build"
  mkdir -p "$workdir/CDT/build"
  (cd "$workdir/CDT/build" && cmake -DCDT_USE_AS_COMPILED_LIBRARY=ON -DCDT_ENABLE_CALLBACK_HANDLER=ON -DCDT_USE_64_BIT_INDEX_TYPE=ON ..)
  (cd "$workdir/CDT/build" && cmake --build . -j"$(nproc)")
  (cd "$workdir/CDT/build" && sudo cmake --install .)
}

activate_python_venv() {
  local venv_dir="$FORMULA_HOME/venv"
  if [[ ! -d "$venv_dir" ]]; then
    "$SYSTEM_PYTHON" -m venv "$venv_dir"
  fi

  # venv is outside <ws>/src, but marking it ignored is harmless.
  touch "$venv_dir/COLCON_IGNORE" 2>/dev/null || true

  # shellcheck disable=SC1091
  source "$venv_dir/bin/activate"
}

setup_python_venv() {
  log "Setting up Python venv and installing Python dependencies…"

  activate_python_venv

  if [[ -f "$FORMULA_HOME/requirements.txt" ]]; then
    python -m pip install -r "$FORMULA_HOME/requirements.txt"
  else
    warn "No requirements.txt found at $FORMULA_HOME/requirements.txt; skipping."
  fi

  # Install f1tenth_gym (and its dependencies) in editable mode.
  local gym_dir=""
  if [[ -d "$FORMULA_HOME/src/src/simulation/f1tenth_gym" ]]; then
    gym_dir="$FORMULA_HOME/src/src/simulation/f1tenth_gym"
  elif [[ -d "$FORMULA_HOME/src/simulation/f1tenth_gym" ]]; then
    gym_dir="$FORMULA_HOME/src/simulation/f1tenth_gym"
  fi

  if [[ -n "$gym_dir" ]]; then
    # Install from the local path (non-editable). Colcon/ament tooling can be
    # sensitive to editable installs in mixed ROS/Python environments.
    python -m pip install "$gym_dir"
  else
    warn "f1tenth_gym not found under $FORMULA_HOME/src; skipping."
  fi
}

install_formula_bashrc() {
  log "Registering shell aliases from scripts/formula_bashrc.sh…"

  local bashrc="$HOME/.bashrc"
  local formula_rc="$FORMULA_HOME/src/scripts/formula_bashrc.sh"

  if [[ ! -f "$formula_rc" ]]; then
    err "Missing $formula_rc"
    exit 1
  fi

  local begin="# >>> f1tenth formula >>>"
  local end="# <<< f1tenth formula <<<"

  if [[ -f "$bashrc" ]] && grep -qF "$begin" "$bashrc"; then
    log "~/.bashrc already contains formula block; leaving as-is."
  else
    cat >> "$bashrc" <<EOF

$begin
# Added by $FORMULA_HOME/setup.sh
if [ -f "$formula_rc" ]; then
  source "$formula_rc"
fi
$end
EOF
    log "Added formula block to ~/.bashrc"
  fi

  # Source it for this script run (won't persist to parent shell).
  # shellcheck disable=SC1091
  source "$formula_rc"
}

main() {
  log "FORMULA_HOME=$FORMULA_HOME"

  if ! has_ros2_humble; then
    if is_ubuntu_jammy; then
      install_ros2_humble_ubuntu
    else
      err "ROS 2 Humble not detected, and auto-install is only implemented for Ubuntu 22.04 (jammy)."
      err "Install ROS 2 Humble manually, then re-run: $SETUP_SCRIPT"
      exit 1
    fi
  else
    log "ROS 2 Humble detected."
  fi

  source_ros_humble

  ensure_colcon_ignores_venv

  install_apt_prereqs
  init_rosdep_if_needed

  # CUDA / ZED prompt
  if has_cuda; then
    log "CUDA/NVIDIA tooling detected."
    if [[ -t 0 ]]; then
      read -r -p "CUDA detected. Do you want to enable ZED packages (zed_wrapper etc.)? [y/N] " ans
    else
      ans="N"
    fi

    if [[ "${ans}" =~ ^[Yy]$ ]]; then
      if ! has_zed_sdk; then
        warn "ZED SDK not detected. Please install it first from:"
        warn "https://www.stereolabs.com/en-ca/developers/release"
        warn "After installing the SDK, re-run this script."
      else
        log "ZED SDK detected."
        # Persist an opt-in env var used by scripts/formula_bashrc.sh to decide whether to build ZED packages.
        if [[ -t 0 ]]; then
          read -r -p "Persist enabling ZED builds by setting IS_JETSON=TRUE in ~/.bashrc? [y/N] " persist
        else
          persist="N"
        fi
        if [[ "${persist}" =~ ^[Yy]$ ]]; then
          if ! grep -qE '^export IS_JETSON=' "$HOME/.bashrc" 2>/dev/null; then
            echo 'export IS_JETSON=TRUE' >> "$HOME/.bashrc"
            log "Added 'export IS_JETSON=TRUE' to ~/.bashrc"
          else
            warn "~/.bashrc already sets IS_JETSON; not changing it."
          fi
        fi
        export IS_JETSON=TRUE
      fi
    fi
  else
    log "CUDA not detected; ZED packages will remain skipped by default."
  fi

  install_cdt_if_needed

  # Activate venv before rosdep so any `pip` rosdeps install into the venv.
  activate_python_venv
  install_rosdeps_for_workspace
  setup_python_venv
  install_formula_bashrc

  log "Success."
  echo
  echo "Next steps (new terminal recommended):"
  echo "  source ~/.bashrc"
  echo
  echo "Common commands:" 
  echo "  rc_home        # cd to workspace"
  echo "  rc_build       # colcon build"
  echo "  rc_source      # source install/setup.bash"
  echo "  rc_run_auto    # launch autonomous config"
  echo "  rc_run_sim     # launch simulator config"
}

main "$@"
