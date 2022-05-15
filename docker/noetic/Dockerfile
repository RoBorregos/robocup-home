# Use the existing ROS-NOETIC image
FROM osrf/ros:noetic-desktop-full

LABEL maintainer="Jose Cisneros <A01283070@itesm.mx>"

# Install dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano \
    git \
    autoconf \
    libtool \
    python3-pip \
    mesa-utils \
    --no-install-recommends terminator \
    ros-noetic-audio-common \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-moveit-ros-planning \
    ros-noetic-moveit-ros-planning-interface \
    ros-noetic-moveit-core \
    ros-noetic-moveit-msgs \
    ros-noetic-rosserial-arduino \
    ros-noetic-moveit-ros-perception \
    ros-noetic-moveit \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rosserial && \
    rm -rf /var/lib/apt/lists/*

# Init catkin_home directoy
RUN mkdir /catkin_home
COPY catkin_home/ catkin_home/

# Change Workdir
WORKDIR /catkin_home

# catkin_make
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash;catkin_make'

# Add ROS environment variables automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "[ -f /catkin_home/devel/setup.bash ] && source /catkin_home/devel/setup.bash" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/catkin_home/src/simulation/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=/catkin_home/src/simulation:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc


ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
