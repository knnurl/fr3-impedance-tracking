# Copyright 2026 Kaan Ural
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# shellcheck shell=bash
# Source this in EVERY terminal of the FR3 cell, before launching anything:
#
#     source <ws>/src/fr3-impedance-tracking/fr3_cell/scripts/fr3_env.sh
#
# Safe to source twice. It sources ROS 2 Humble if nothing has yet, franka_ros2
# from FRANKA_WS (default ~/franka_ros2_ws) when that is a separate workspace,
# then this workspace on top, and defines two commands:
#
#     fr3_preflight [robot_ip]     read-only checks; exit code = FAIL count
#     fr3_cell [arg:=value ...]    terminal 2: ros2 launch fr3_cell fr3_cell.launch.py
#
# Why this file exists: the FCI loop exchanges UDP packets with the robot
# every 1 ms. If DDS is not pinned away from the robot NIC, ROS traffic
# shares that link and the kernel UDP buffers, the RT thread misses its
# deadline, and the arm drops the connection mid-motion:
#
#     libfranka: Connection reset by peer   (franka::NetworkException)
#
# A CycloneDDS config WITHOUT an <Interfaces> block tunes buffers but isolates
# nothing. Edit config/cyclonedds_fr3.xml for your network interface names,
# then verify with fr3_preflight.

# <ws>/src/<repo>/fr3_cell/scripts. The path as sourced, not resolved: the
# repo may be a symlink into <ws>/src. FR3_WS overrides the workspace.
_FR3_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_FR3_WS="${FR3_WS:-$(cd "${_FR3_DIR}/../../../.." && pwd)}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(cd "${_FR3_DIR}/../config" && pwd)/cyclonedds_fr3.xml"

# The robot (Franka's default FCI address). fr3_preflight defaults to it.
export FR3_ROBOT_IP="${FR3_ROBOT_IP:-172.16.0.2}"

# Run logs (panel traces, tracking logs, REC bags, state_recorder CSVs), one
# folder per day. Set FR3_LOG_DIR before sourcing to put them elsewhere.
export FR3_LOG_DIR="${FR3_LOG_DIR:-${HOME}/.local/share/fr3_cell/runs}"

# ROS, then franka_ros2 (if it lives in its own workspace), then this
# workspace LAST so it overlays both.
if [ -z "$ROS_DISTRO" ] && [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
_FRANKA_WS="${FRANKA_WS:-${HOME}/franka_ros2_ws}"
if [ "$_FRANKA_WS" != "$_FR3_WS" ] && [ -f "${_FRANKA_WS}/install/setup.bash" ]; then
    case ":${AMENT_PREFIX_PATH}:" in
        *":${_FRANKA_WS}/install/"*) ;;
        *)
            # shellcheck disable=SC1091
            source "${_FRANKA_WS}/install/setup.bash" ;;
    esac
fi
if [ -f "${_FR3_WS}/install/local_setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${_FR3_WS}/install/local_setup.bash"
fi

# The controller manager (terminal 1) must find fr3_mating_controllers in
# THIS workspace, or spawning the impedance controller fails with "Loader for
# controller ... not found". An index lookup only - no DDS.
_FR3_PREFIX="$(ros2 pkg prefix fr3_mating_controllers 2>/dev/null || true)"

echo "FR3 env: ${_FR3_WS}"
echo "  DDS ${CYCLONEDDS_URI#file://} | robot ${FR3_ROBOT_IP} | logs ${FR3_LOG_DIR}"
case "$_FR3_PREFIX" in
    "${_FR3_WS}/install/"*)
        echo "  commands: fr3_preflight, fr3_cell" ;;
    *)
        echo "  [FAIL] fr3_mating_controllers resolves to '${_FR3_PREFIX:-nothing}', not ${_FR3_WS}/install"
        echo "  [FAIL] the impedance controller cannot be spawned - colcon build in ${_FR3_WS}" ;;
esac

eval "fr3_cell() { ros2 launch fr3_cell fr3_cell.launch.py \"\$@\"; }"
eval "fr3_preflight() { $(printf '%q' "${_FR3_DIR}/fr3_preflight.sh") \"\$@\"; }"

unset _FR3_DIR _FR3_WS _FR3_PREFIX _FRANKA_WS
