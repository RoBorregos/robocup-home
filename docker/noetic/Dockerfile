# Use the existing ROS-NOETIC image
FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
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

RUN pip3 install -U rosdep rosinstall_generator wstool rosinstall six vcstools

# catkin_make
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash;catkin_make'

# Add ROS environment variables automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "[ -f /catkin_home/devel/setup.bash ] && source /catkin_home/devel/setup.bash" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/catkin_home/src/simulation/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
RUN echo "export GAZEBO_RESOURCE_PATH=/catkin_home/src/simulation:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc


ENTRYPOINT [ "/bin/bash", "-l", "-c" ]



# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]


# LABEL maintainer="Jose Cisneros <A01283070@itesm.mx>"

# # Install dependencies
# RUN apt-get update -qq && \
#     apt-get install -y \
#     build-essential \
#     nano \
#     git \
#     autoconf \
#     libtool \
#     python3-pip \
#     mesa-utils \
#     --no-install-recommends terminator \
#     ros-noetic-audio-common \
#     ros-noetic-map-server \
#     ros-noetic-move-base \
#     ros-noetic-rosserial-arduino \ 
#     ros-noetic-rosserial && \
#     rm -rf /var/lib/apt/lists/*

# # Init catkin_home directoy
# RUN mkdir /catkin_home
# COPY catkin_home/ catkin_home/

# # Change Workdir
# WORKDIR /catkin_home

# # catkin_make
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash;catkin_make'

# # Add ROS environment variables automatically
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# RUN echo "[ -f /catkin_home/devel/setup.bash ] && source /catkin_home/devel/setup.bash" >> ~/.bashrc
# RUN echo "export GAZEBO_MODEL_PATH=/catkin_home/src/simulation/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
# RUN echo "export GAZEBO_RESOURCE_PATH=/catkin_home/src/simulation:$GAZEBO_RESOURCE_PATH" >> ~/.bashrc


# ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
