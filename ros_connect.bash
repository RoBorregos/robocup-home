#!/bin/bash
if [ $# -lt '3' ]; then 
    echo $#
    my_ip=` hostname -I `
    echo $my_ip
else
echo "No IP address of host computer provided"
fi