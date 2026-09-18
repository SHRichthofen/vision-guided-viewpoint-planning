#!/usr/bin/env bash
#
# Fetch the upstream vendor packages this workspace builds against.
#
# `vcs import` alone is not enough: agx_arm_ros carries a nested submodule
# (agx_arm_urdf) whose upstream pin is older than the revision this workspace
# was validated against, and the joint limits it defines feed the view-goal
# solver. This script pins that submodule explicitly.
#
# Usage:  ./scripts/setup_deps.sh
set -euo pipefail

# agx_arm_urdf revision validated with this workspace ("Correct joint limitation").
# Upstream agx_arm_ros still records 3080af4b579238c850b709c411abdc88fc930c82.
AGX_ARM_URDF_REV=f539fee0871a31466af9727a623fb2190a689279

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URDF_DIR="${WS_ROOT}/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf"

command -v vcs >/dev/null || {
  echo "vcstool not found. Install it with: sudo apt install python3-vcstool" >&2
  exit 1
}

echo "==> Importing vendor repositories into src/"
vcs import "${WS_ROOT}/src" < "${WS_ROOT}/dependencies.repos"

echo "==> Initialising nested submodules"
git -C "${WS_ROOT}/src/agx_arm_ros" submodule update --init --recursive

echo "==> Pinning agx_arm_urdf to ${AGX_ARM_URDF_REV}"
git -C "${URDF_DIR}" fetch --quiet origin "${AGX_ARM_URDF_REV}" 2>/dev/null || git -C "${URDF_DIR}" fetch --quiet origin
git -C "${URDF_DIR}" checkout --quiet "${AGX_ARM_URDF_REV}"

echo "==> Done. Next:"
echo "    rosdep install --from-paths ${WS_ROOT}/src --ignore-src -r -y"
echo "    colcon build --symlink-install"
