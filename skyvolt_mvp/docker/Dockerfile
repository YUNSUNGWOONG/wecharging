# Reproducible env: ROS 2 Humble + Gazebo Garden + Python algorithm libs.
# Built for headless CI; for desktop add nvidia-container-toolkit.
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Garden via OSRF
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    | gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-garden && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    numpy scipy matplotlib pyyaml \
    fastapi uvicorn[standard] pydantic \
    pytest pytest-cov hypothesis

WORKDIR /ws
COPY src /ws/src

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install || true

ENV PYTHONPATH=/ws/src/skyvolt_localization:/ws/src/skyvolt_fleet:/ws/src/skyvolt_eval:$PYTHONPATH

CMD ["/bin/bash"]
