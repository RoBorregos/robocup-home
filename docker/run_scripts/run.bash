#!/bin/bash

# Variables required for logging as a user with the same id as the user running this script
export LOCAL_USER_ID=`id -u $USER`
export LOCAL_GROUP_ID=`id -g $USER`
export LOCAL_GROUP_NAME=`id -gn $USER`
DOCKER_USER_ARGS="--env LOCAL_USER_ID --env LOCAL_GROUP_ID --env LOCAL_GROUP_NAME"

# Variables for forwarding ssh agent into docker container
SSH_AUTH_ARGS=""
if [ ! -z $SSH_AUTH_SOCK ]; then
    DOCKER_SSH_AUTH_ARGS="-v $(dirname $SSH_AUTH_SOCK):$(dirname $SSH_AUTH_SOCK) -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK"
fi

DOCKER_NETWORK_ARGS="--net host"
if [[ "$@" == *"--net "* ]]; then
    DOCKER_NETWORK_ARGS=""
fi

DOCKER_COMMAND="docker run"

ADDITIONAL_COMMANDS=""
if [ $# -eq 1 ]; then
    if [ "$1" == "IS_NAVIGATION" ]; then
        ADDITIONAL_COMMANDS="--device='/dev/ttyUSB0'"
    elif [ "$1" == "IS_OBJECT_DETECTION" ]; then
        ADDITIONAL_COMMANDS="--volume /dev/video0:/dev/video0
            --volume $PWD/object_detection:/object_detection"
    elif [ "$1" == "IS_SPEECH" ]; then
        ADDITIONAL_COMMANDS="--env PULSE_SERVER=unix:/tmp/pulseaudio.socket
        --env PULSE_COOKIE=/tmp/pulseaudio.cookie
        --device /dev/video0
        --volume ~/.config/pulse/cookie:/tmp/pulseaudio.cookie
        --volume /tmp/pulseaudio.socket:/tmp/pulseaudio.socket
        --volume /tmp/pulseaudio.client.conf:/etc/pulse/client.conf"
    fi
fi

xhost +

$DOCKER_COMMAND -it -d\
    $DOCKER_USER_ARGS \
    $DOCKER_SSH_AUTH_ARGS \
    $DOCKER_NETWORK_ARGS \
    $ADDITIONAL_COMMANDS \
    --privileged \
    -v "$PWD/catkin_home/src:/catkin_home/src" \
    -v "$PWD/lib:/lib/" \
    -v "$PWD/catkin_home/typings:/catkin_home/typings" \
    -v /var/run/docker.sock:/var/run/docker.sock \
    --name=ros-home \
    ros:home \
    bash

if [ $# -eq 1 ]; then
    if [ $1 == "IS_NAVIGATION" ]; then
        ARDUINO_PATH="${HOME}/Arduino/libraries/ros_lib" 
        rm -f -r ${ARDUINO_PATH}
        docker cp ros-home:/Arduino/libraries/ros_lib $ARDUINO_PATH
        chmod -R 777 ${ARDUINO_PATH}
    elif [ $1 == "IS_OBJECT_DETECTION" ]; then
cat << EOF	
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Create conda environment and Download Models if necessary.

Execute inside container: 

cd object_detection && ./object_detection_setup.sh && echo 'conda activate object_detection_env' >> ~/.bashrc

*Remember to activate object_detection_env environment
conda activate object_detection_env
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
EOF
    elif [ "$1" == "IS_SPEECH" ]; then
cat << EOF	
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Install python dependencies inside container.

GPU
python3 -m pip install --upgrade pip && pip3 install -r speechDependenciesGpu.txt

Without-GPU
python3 -m pip install --upgrade pip && pip3 install -r speechDependencies.txt

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
EOF
    fi
fi
