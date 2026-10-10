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

"""The selectable values the window offers, kept here so view.py needs
nothing but Qt. test_cell_pins.py fails if any drift from core.py."""

STEP_MM = ['0.5', '1', '2', '5', '10', '20', '30', '50']
STEP_MM_DEFAULT = '30'
ROT_DEG = ['0.25', '0.5', '1', '2', '3', '5']
ROT_DEG_DEFAULT = '3'
TARGET_MM = ['80', '100', '150', '200', '250', '300']
TARGET_MM_DEFAULT = '100'
POS_TOL_MM = ['1', '2', '3', '5']
POS_TOL_MM_DEFAULT = '2'
INPLANE = ['off', '0', '90', '-90', '180']
INPLANE_DEFAULT = '90'
FLOOR_MM_DEFAULT = '100'
SETPOINT_MM = ['5', '10', '20', '50']
SETPOINT_MM_DEFAULT = '10'
AXES = ['base Z (up)', 'base X', 'base Y', 'tool Z (stroke)']
OVER_LEAD = ['hold', 'stop', 'clamp']
OVER_LEAD_DEFAULT = 'hold'
POSE_SOURCES = ['marker', 'depth_checked']   # core.POSE_SOURCES (the object pose contract)
POSE_SOURCE_DEFAULT = 'marker'
GAIN_LIMITS = {'k_xy': (0.0, 3000.0), 'k_z': (0.0, 3000.0), 'k_rp': (0.0, 300.0),
               'k_yaw': (0.0, 300.0), 'zeta': (0.1, 2.0)}
GAIN_LABELS = {'k_xy': 'k lateral', 'k_z': 'k tool Z', 'k_rp': 'k roll/pitch',
               'k_yaw': 'k yaw', 'zeta': 'zeta'}
GAIN_UNITS = {'k_xy': 'N/m', 'k_z': 'N/m', 'k_rp': 'Nm/rad', 'k_yaw': 'Nm/rad', 'zeta': ''}
MAX_LEAD_MM = 60.0
CONTACT_N = 20.0            # core.CONTACT_WRENCH[0], sent by PRE-FLIGHT
REFLEX_N = 40.0             # core.COLLISION_WRENCH[0]
