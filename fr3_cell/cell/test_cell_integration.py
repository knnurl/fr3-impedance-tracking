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

"""The two panel runs that need their own process: the screenshot diff against
the baselines (visual.py) and the ROS side against the mock cell
(mock_smoke.py, isolated DDS domain 88). Each is skipped, not failed, where
its dependencies are missing."""

import importlib.util
import os
import pathlib
import subprocess
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent


def _have(*mods):
    return all(importlib.util.find_spec(m) is not None for m in mods)


def _have_font(family):
    try:
        out = subprocess.run(['fc-list', f':family={family}'], capture_output=True,
                             text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return bool(out.strip())


@pytest.mark.skipif(not _have('PySide6', 'PIL'), reason='PySide6 / Pillow not installed')
@pytest.mark.skipif(not _have_font('Lato'),
                    reason='the baselines were captured with Lato (apt install fonts-lato)')
def test_visual_baselines_unchanged():
    env = dict(os.environ, QT_QPA_PLATFORM='offscreen')
    r = subprocess.run([sys.executable, str(HERE / 'visual.py')], env=env,
                       capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.skipif(not _have('rclpy', 'franka_msgs', 'topic_tools'),
                    reason='ROS 2 / franka_ros2 not sourced')
def test_ros_side_against_the_mock_cell():
    r = subprocess.run([sys.executable, str(HERE / 'mock_smoke.py')],
                       capture_output=True, text=True, timeout=400)
    tail = '\n'.join(r.stdout.splitlines()[-45:])
    assert r.returncode == 0, tail + r.stderr[-2000:]
