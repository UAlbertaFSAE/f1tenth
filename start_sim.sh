


#!/usr/bin/env bash
set -euo pipefail

# Start the F1TENTH Gazebo simulation from the repository root.
# Assumes this script lives at the root of the f1tenth repository.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$REPO_DIR/docker/compose.sim.yml"
PROJECT_NAME="f1tenth-sim"
MODEL_NAME="${UA26_MODEL_NAME:-ua26_car}"
WORLD_PATH="/sim/gazebo_sim/models/${MODEL_NAME}/world.sdf"
HOST_WORLD_PATH="$REPO_DIR/gazebo_sim/models/${MODEL_NAME}/world.sdf"

if ! command -v docker >/dev/null 2>&1; then
  echo "Error: docker was not found in PATH."
  exit 1
fi

if [[ ! -f "$COMPOSE_FILE" ]]; then
  echo "Error: could not find $COMPOSE_FILE"
  echo "Run this script from the f1tenth repository, or place the script at the repo root."
  exit 1
fi

if [[ ! -f "$HOST_WORLD_PATH" ]]; then
  cat <<MSG
Error: simulation world not found:
  $HOST_WORLD_PATH

You must provide the UA26 simulation assets yourself.
Expected model folder name: $MODEL_NAME
Expected world file: gazebo_sim/models/$MODEL_NAME/world.sdf
Expected model file: gazebo_sim/models/$MODEL_NAME/model.sdf

If you use a different model folder name, either:
  1. rename it to '$MODEL_NAME', or
  2. run with UA26_MODEL_NAME=<your_name> ./start_sim.sh
     and make sure world.sdf uses model://<your_name>.
MSG
  exit 1
fi

echo "[1/2] Starting simulation container..."
docker compose \
  -p "$PROJECT_NAME" \
  -f "$COMPOSE_FILE" \
  up -d sim

echo "[2/2] Starting Gazebo Sim..."
exec docker compose \
  -p "$PROJECT_NAME" \
  -f "$COMPOSE_FILE" \
  exec sim bash -lc "
    source /opt/ros/humble/setup.bash
    export GZ_SIM_RESOURCE_PATH=/sim/gazebo_sim/models:\${GZ_SIM_RESOURCE_PATH:-}
    export GZ_SIM_RENDER_ENGINE=\${GZ_SIM_RENDER_ENGINE:-ogre}
    exec gz sim -v 4 '$WORLD_PATH'
  "
