FROM osrf/ros:humble-desktop-full

ENV ROS_WS=/root/bot_ws
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

RUN apt update && apt install -y \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
COPY . .

WORKDIR $ROS_WS
RUN rosdep update --rosdistro=humble

RUN rosdep install --from-paths src -y --ignore-src --rosdistro=humble || \
    (apt update && rosdep install --from-paths src -y --ignore-src --rosdistro=humble)

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $ROS_WS/install/setup.bash" >> /root/.bashrc

WORKDIR $ROS_WS

CMD ["bash"]