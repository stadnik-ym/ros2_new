ARG ROS_DISTRO=jazzy
FROM --platform=arm64 ros:${ROS_DISTRO}-ros-base AS builder

SHELL [ "/bin/bash", "-c" ]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ./src ./src

RUN source/opt/${ROS_DISTRO}/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM --platform=arm64 ros:${ROS_DISTRO}-ros-base AS runtime

SHELL [ "/bin/bash", "-c" ]

WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ros2_ws/install ./install
