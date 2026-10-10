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

# The cell panel against the mock cell, in the container, on this X display.
#
#     ./docker/run_mock.sh                  # the panel (Ctrl-C to quit)
#     ./docker/run_mock.sh bash             # a shell in the image instead
#     ./docker/run_mock.sh /ws/src/fr3-impedance-tracking/run_tests.sh
IMAGE="${IMAGE:-fr3-impedance-tracking}"
if [ -z "${DISPLAY:-}" ]; then
    echo "run_mock.sh: DISPLAY is not set - run it from a desktop session" >&2
    exit 1
fi
# Let local containers draw on this display (undone on exit).
xhost +local:docker >/dev/null
trap 'xhost -local:docker >/dev/null' EXIT
docker run --rm -it \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --ipc=host \
    "$IMAGE" "$@"
