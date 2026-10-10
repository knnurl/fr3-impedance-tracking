# fr3-impedance-tracking on ROS 2 Humble: builds the cell, the FR3 bring-up
# packages and their dependencies; by default opens the panel against the
# mock cell (no robot).
#
#     docker build -t fr3-impedance-tracking -f docker/Dockerfile .
#     ./docker/run_mock.sh
#
# For the real arm prefer the native build (PREEMPT_RT kernel, host network,
# the robot NIC): see docs/installation.md.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Build tools, the C++ linters, the panel's fonts, and the X11/xcb
# libraries PySide6 needs.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-vcstool python3-colcon-common-extensions git \
        clang-format cppcheck fonts-lato fonts-dejavu-core fontconfig \
        libgl1 libegl1 libfontconfig1 libdbus-1-3 libxkbcommon-x11-0 \
        libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
        libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 libxcb-xkb1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# franka_ros2 and franka_description, pinned (cached unless the .repos changes).
COPY fr3_impedance_tracking.repos /tmp/
RUN mkdir -p src && vcs import src < /tmp/fr3_impedance_tracking.repos

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY . src/fr3-impedance-tracking

# Only what the cell and the FR3 bring-up need (franka_ros2 also carries
# Gazebo and spacemouse packages).
ARG PKGS="fr3_cell franka_bringup franka_fr3_moveit_config franka_gripper"
RUN apt-get update && rosdep update --rosdistro humble \
    && rosdep install -y --rosdistro humble --ignore-src \
        --from-paths $(colcon list --packages-up-to ${PKGS} --paths-only) \
    && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install --packages-up-to ${PKGS} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fr3_cell", "fr3_cell.launch.py", "mock:=true"]
