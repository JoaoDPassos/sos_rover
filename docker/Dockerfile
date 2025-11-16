FROM ros:humble-ros-core

# Install build tools and colcon
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rm -rf /var/lib/apt/lists/*

# Install serial-ros2 into /usr/local
RUN git clone https://github.com/RoverRobotics-forks/serial-ros2.git /tmp/serial-ros2 && \
    cd /tmp/serial-ros2 && \
    mkdir -p build && cd build && \
    . /opt/ros/humble/setup.sh && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig && \
    rm -rf /tmp/serial-ros2

# Default workspace directory
WORKDIR /ros2_ws

# Auto-source ROS + serial (and workspace if it exists) in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /usr/local/share/serial/local_setup.bash ]; then source /usr/local/share/serial/local_setup.bash; fi" >> /root/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc
