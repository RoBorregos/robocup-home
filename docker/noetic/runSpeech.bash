docker run \
    -it -d \
    --gpus all \
    --env PULSE_SERVER=unix:/tmp/pulseaudio.socket \
    --env PULSE_COOKIE=/tmp/pulseaudio.cookie \
    --device /dev/video0 \
    --volume ~/.config/pulse/cookie:/tmp/pulseaudio.cookie \
    --volume /tmp/pulseaudio.socket:/tmp/pulseaudio.socket \
    --volume /tmp/pulseaudio.client.conf:/etc/pulse/client.conf \
    --user $(id -u):$(id -g) \
    -v ${PWD}/catkin_home/src:/catkin_home/src \
    -v ${PWD}/catkin_home/typings:/catkin_home/typings \
    --network host \
    --name ros-noetic-speech \
    ros:noetic-speech
