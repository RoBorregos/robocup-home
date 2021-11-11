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
    ros-noetic-audio-common \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-rosserial-arduino \ 
    ros-noetic-rosserial && \
    rm -rf /var/lib/apt/lists/*

# Init catkin_home directoy
RUN mkdir /catkin_home
COPY catkin_home/ catkin_home/

# Change Workdir
WORKDIR catkin_home

# catkin_make
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash;catkin_make'

# Add ROS environment variables automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "[ -f /$REPO_WS/devel/setup.bash ] && source /$REPO_WS/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
