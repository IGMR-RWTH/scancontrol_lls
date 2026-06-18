
# Dev dependency stage
FROM ros:humble-ros-base AS dev_environment

# Set environment variables
ENV ROS_WS=/home/ros/ros_ws
WORKDIR $ROS_WS

# Install dependencies
RUN --mount=type=bind,source=/,target=$ROS_WS \
    cd $ROS_WS \
    && ls -a .\
    && sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    aravis-0.8 \
    libaravis-dev \
    libusb-1.0 \
    gdb \
    gdbserver \
    curl \
    ca-certificates \
    unzip \
    patch \
    python3-colcon-common-extensions \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/*

    # build stage
FROM dev_environment AS builder

# Set environment variables
ENV ROS_WS=/home/ros/ros_ws
WORKDIR $ROS_WS
COPY ./src $ROS_WS/src

# Fetch the vendored SDK (ext/ is git-ignored, so it is not in the context),
# then build the workspace.
RUN  cd $ROS_WS && \
    . /opt/ros/humble/setup.sh \
    && bash src/scancontrol_lls/get_sdk.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# runtime stage
FROM dev_environment AS runner
ENV ROS_WS=/home/ros/ros_ws
WORKDIR $ROS_WS

# Copy the built install/ directory 
COPY --from=builder $ROS_WS/install $ROS_WS/install

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source $ROS_WS/install/setup.bash && exec \"$@\"", "--"]

CMD ["ros2", "run", "scancontrol_lls_driver", "lls_driver"]