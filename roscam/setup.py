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

from setuptools import setup

package_name = 'roscam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kaan Ural',
    maintainer_email='k.ural1@salford.ac.uk',
    description='Vision for the FR3 cell: ArUco marker pose (cam_pub), the in-process '
                'D405 node (vision_standalone), the depth/colour object estimator, '
                'hand-eye calibration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_pub = roscam.cam_pub:main',
            'handeye_calib = roscam.handeye_calib:main',
            'connector_pose = roscam.connector_pose:main',
            'vision_standalone = roscam.vision_standalone:main',
        ],
    },
)
