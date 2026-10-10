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

"""The object pose estimator of roscam/object_pose.py in C++ (Estimator).

Use it through roscam.object_pose.CppObjectPoseEstimator, which prepares
the model (mesh samples and edges) in Python and hands it over, so both
versions run on identical data; roscam/test/test_object_pose_cpp.py holds
them to the same answers."""

from ._object_pose_cpp import Estimator  # noqa: F401
