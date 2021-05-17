XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it -d\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --device="/dev/ttyUSB0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/catkin_home/src:/catkin_home/src" \
    --volume="$PWD/catkin_home/typings:/catkin_home/typings" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    --name=ros-melodic-navigation-gpu \
    ros:melodic-navigation-gpu \
    bash

ARDUINO_PATH="${HOME}/Arduino/libraries/ros_lib" 
rm -f -r ${ARDUINO_PATH}
mkdir -p $ARDUINO_PATH
docker cp ros-melodic-navigation-gpu:/Arduino/libraries/ros_lib $ARDUINO_PATH
chmod -R 777 ${ARDUINO_PATH}
