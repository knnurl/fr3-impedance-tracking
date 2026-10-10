#!/usr/bin/env bash
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

# Build and test this repo's packages: gtests, pytest (panel, grip, vision,
# estimator parity), the tracking smoke test and every linter. No robot or
# camera is touched; the tests that start ROS nodes use their own DDS domains.
#
#     ./run_tests.sh                 # from anywhere inside <ws>/src/<this repo>
#     WS=/path/to/ws ./run_tests.sh
#
# The workspace is <ws> (two levels up) unless WS is set. franka_ros2 must be
# built in it, or in FRANKA_WS (default ~/franka_ros2_ws) when that exists.
# The path as called, not resolved: the repo may be a symlink into <ws>/src.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WS:-$(cd "$REPO/../.." && pwd)}"
if [ "$(basename "$(dirname "$REPO")")" != src ]; then
    echo "run_tests.sh: $REPO is not inside <ws>/src - set WS=/path/to/ws" >&2
    exit 2
fi
FRANKA_WS="${FRANKA_WS:-$HOME/franka_ros2_ws}"
PKGS=(fr3_mating_controllers fr3_tracking object_pose_cpp roscam fr3_cell)

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "$FRANKA_WS/install/setup.bash" ] && [ "$FRANKA_WS" != "$WS" ]; then
    # shellcheck disable=SC1091
    source "$FRANKA_WS/install/setup.bash"
fi
cd "$WS" || exit 1
# Humble's ament_cppcheck skips cppcheck 2.x as slow unless told otherwise.
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

colcon build --symlink-install --packages-select "${PKGS[@]}" || exit 1
colcon test --packages-select "${PKGS[@]}" --event-handlers console_direct- >/dev/null
colcon test-result --all --verbose
