FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-pip \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init \
    && rosdep update

WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" >>  ~/.bashrc

CMD ["sleep", "infinity"]

