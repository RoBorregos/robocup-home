#!/bin/bash
echo "Set variables for ROS-on-multiple-machines for this terminal."
echo "Usage:"
echo "$0 {ROS_MASTER_IP_OR_HOSTNAME} {ROS_TALKER_IP_OR_HOSTNAME} or"
echo "$0 {ROS_MASTER_IP_OR_HOSTNAME} or"
echo "$0"
echo "More information: https://github.com/RoBorregos/Robocup-Home/wiki/Computers#running-ros-on-multiple-machines"
echo ""
if [ $# -gt 2 ]
then
    echo "ERROR: Too many arguments."
    exit
fi

if [ $# == 2 ]
then
    #Both ips provided
    ROS_MASTER_URI="http://$1:11311"
    ROS_IP=$2 
    export ROS_MASTER_URI
    export ROS_IP
elif [ $# == 1 ] || [ $# == 0 ]
then 
    my_ip=`hostname -I |  awk '{print $1;}'` #Assumes the first ip returned is the one needed
    ROS_IP=$my_ip
    export ROS_IP

    if [ $# == 1 ]
    then
        ROS_MASTER_URI="http://$1:11311"
        export ROS_MASTER_URI
    else
        # This should be the network address (my_ip & net_mask) plus one.
        # An example: 192.15.0.1. Then, use the default gateway, as 
        # this same logic is usually used for it. 
        router_device_ip=`ip r | awk 'NR==1{print $3;}'`
        ROS_MASTER_URI="http://$router_device_ip:11311"
        export ROS_MASTER_URI
    fi
fi

echo "Done setting ROS_MASTER_URI=$ROS_MASTER_URI and ROS_IP=$ROS_IP"
