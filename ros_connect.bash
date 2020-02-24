#!/bin/bash
echo "Be sure to call this script: source ./ros_connect {ROS_MASTER_IP}"
if [ $# == 1 ] 
then 
    my_ip=`hostname -I |  awk '{print $1;}'` #Assumes the first ip returned is the one needed
    echo "Your IP: " $my_ip
    echo "ROS MASTER IP: " $1
    ROS_MASTER_URI="http://$1:11311"
    ROS_IP=$my_ip 
    export ROS_MASTER_URI
    export ROS_IP
    echo "Done setting ROS_MASTER_URI and ROS_IP"
else
    echo "No IP address of master ROS computer provided"
fi