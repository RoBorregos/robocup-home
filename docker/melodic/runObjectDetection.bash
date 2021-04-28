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
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/object_detection:/object_detection" \
    --volume="$PWD/catkin_home/src:/catkin_home/src" \
    --volume="$PWD/catkin_home/typings:/catkin_home/typings" \
    --volume="/dev/video0:/dev/video0" \
    --net=host \
    --privileged \
    --name=ros-melodic-object-detection \
    ros:melodic-object-detection \
    bash

# Run
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
