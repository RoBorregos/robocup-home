# Use the existing ROS-MELODIC image
FROM osrf/ros:melodic-desktop-full

# Install dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    nano \
    git \
    autoconf \
    libtool \
    python-pip \
    ros-melodic-audio-common \
    ros-melodic-move-base && \
    rm -rf /var/lib/apt/lists/*

# Init catkin_home directoy
RUN mkdir /catkin_home
COPY catkin_home/ catkin_home/

# Change Workdir
WORKDIR /catkin_home

# catkin_make
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash;catkin_make'

# Add ROS environment variables automatically
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "[ -f /catkin_home/devel/setup.bash ] && source /catkin_home/devel/setup.bash" >> ~/.bashrc
