#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  if [ -d "/opt/ros/jazzy" ]; then
    export ROS_DISTRO=jazzy
  elif [ -d "/opt/ros/humble" ]; then
    export ROS_DISTRO=humble
  else
    echo "No se encontro una instalacion de ROS 2 en /opt/ros."
    return 1 2>/dev/null || exit 1
  fi
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"

mkdir -p "${REPO_DIR}/src"

# ---------------------------------------------------------------------------
# Dependencias externas (clonadas en src/, NO trackeadas en este repo).
# Cada una se fija a un commit concreto (probado en Humble) y, si existe,
# se le reaplica el parche con nuestras modificaciones locales (patches/).
#
# Formato: "destino|url|commit_fijado|fichero_de_parche"
# Deja el parche vacio si el repo no tiene modificaciones locales.
# ---------------------------------------------------------------------------
DEPS=(
  "m-explore-ros2|https://github.com/robo-friends/m-explore-ros2.git|03746a24c3dc1f91bca7a2d290cb1a03ab4ad861|"
  "icclab_summit_xl|https://github.com/icclab/icclab_summit_xl.git|f97185a66870f6fdf71169f44f3b54d4f711b6cf|patches/icclab_summit_xl.patch"
  "sum|https://github.com/RobotnikAutomation/summit_xl_common.git|77a728201069cf74c1be04730785086d3f1aa493|patches/summit_xl_common.patch"
  "robotnik_common|https://github.com/RobotnikAutomation/robotnik_common.git|0153704842c35f115da65c7404d6ce6392f9c817|"
  "robotnik_sensors|https://github.com/RobotnikAutomation/robotnik_sensors.git|faaab6e1db429a6708c65d7311b148bba592fd1a|"
)

for entry in "${DEPS[@]}"; do
  IFS='|' read -r dst url commit patch <<< "${entry}"
  dst_path="${REPO_DIR}/src/${dst}"

  if [ ! -d "${dst_path}/.git" ]; then
    echo "Cloning ${dst}..."
    git clone "${url}" "${dst_path}"
    echo "  -> checkout ${commit}"
    git -C "${dst_path}" checkout --quiet "${commit}" \
      || echo "  !! No se pudo fijar el commit ${commit} en ${dst} (revisar manualmente)."

    if [ -n "${patch}" ] && [ -f "${REPO_DIR}/${patch}" ]; then
      echo "  -> aplicando parche ${patch}"
      if git -C "${dst_path}" apply --check "${REPO_DIR}/${patch}" 2>/dev/null; then
        git -C "${dst_path}" apply "${REPO_DIR}/${patch}"
      else
        echo "  !! El parche ${patch} no aplica limpio en ${dst}."
        echo "     (Esperable al migrar a otra distro de ROS; aplicar los cambios a mano)."
      fi
    fi
  fi
done

# map_merge has known compatibility issues on newer ROS distros.
rm -rf "${REPO_DIR}/src/m-explore-ros2/map_merge"

python3 -m pip install --user --upgrade \
  jpl-rosa \
  langchain-ollama \
  langchain-core \
  langchain \
  pydantic \
  anthropic \
  langchain-anthropic

cd "${REPO_DIR}"
colcon build --symlink-install \
  --base-paths "${REPO_DIR}" "${REPO_DIR}/src"
source "${REPO_DIR}/install/setup.bash"

export GZ_SIM_RESOURCE_PATH="${REPO_DIR}/world/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_LOG_DIR="${REPO_DIR}/.ros/log"
mkdir -p "${ROS_LOG_DIR}"

echo "** ROS2 ${ROS_DISTRO} initialized (repo: ${REPO_DIR}) **"
