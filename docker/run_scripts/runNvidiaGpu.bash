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

# Settings required for having nvidia GPU acceleration inside the docker
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=compute,utility"

dpkg -l | grep nvidia-container-toolkit &> /dev/null
HAS_NVIDIA_TOOLKIT=$?
which nvidia-docker > /dev/null
HAS_NVIDIA_DOCKER=$?
if [ $HAS_NVIDIA_TOOLKIT -eq 0 ]; then
  docker_version=`docker version --format '{{.Client.Version}}' | cut -d. -f1`
  if [ $docker_version -ge 19 ]; then
	  DOCKER_COMMAND="docker run --gpus all"
  else
	  DOCKER_COMMAND="docker run --runtime=nvidia"
  fi
elif [ $HAS_NVIDIA_DOCKER -eq 0 ]; then
  DOCKER_COMMAND="nvidia-docker run"
else
  echo "Running without nvidia-docker, if you have an NVidia card you may need it"\
  "to have GPU acceleration"
  DOCKER_COMMAND="docker run"
fi

ADDITIONAL_COMMANDS=""
if [ $# -eq 1 ]; then
    if [ "$1" == "IS_NAVIGATION" ]; then
        ADDITIONAL_COMMANDS="--device='/dev/ttyUSB0'"
    elif [[ "$1" == "IS_OBJECT_DETECTION" || "$1" == "IS_OBJECT_DETECTION_PROD" ]]; then
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
  $DOCKER_GPU_ARGS \
  $DOCKER_SSH_AUTH_ARGS \
  $DOCKER_NETWORK_ARGS \
  $ADDITIONAL_COMMANDS \
  --privileged \
  -v "$PWD/catkin_home/src:/catkin_home/src" \
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

cd /object_detection && ./object_detection_setup.sh && echo 'conda activate object_detection_env' >> ~/.bashrc

*Remember to activate object_detection_env environment
conda activate object_detection_env
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
EOF
    elif [ "$1" == "IS_SPEECH" ]; then
cat << EOF	
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

IF GPU:
pip3 uninstall paddlepaddle
pip3 install paddlepaddle-gpu==1.2.1.post97

Deepspeech - Download Models:
cd ./catkin_home/src/action_selectors/scripts/DeepSpeech/models/lm/
./download_lm.sh
cd ./catkin_home/src/action_selectors/scripts/DeepSpeech/models/baidu_en8k/
./download_model.sh

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
EOF
    fi
fi
