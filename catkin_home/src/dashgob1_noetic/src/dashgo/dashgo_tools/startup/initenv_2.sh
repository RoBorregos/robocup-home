#!/bin/bash

echo  'KERNEL=="tty*", ATTRS{devpath}=="1.1",MODE="0666", GROUP:="dialout",  SYMLINK+="port1"' >/etc/udev/rules.d/port1.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.2",MODE="0666", GROUP:="dialout",  SYMLINK+="port2"' >/etc/udev/rules.d/port2.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.3",MODE="0666", GROUP:="dialout",  SYMLINK+="port3"' >/etc/udev/rules.d/port3.rules
echo  'KERNEL=="tty*", ATTRS{devpath}=="1.4",MODE="0666", GROUP:="dialout",  SYMLINK+="port4"' >/etc/udev/rules.d/port4.rules
#echo  'KERNEL=="tty*", KERNELS=="usb3", MODE="0666", GROUP:="dialout",  SYMLINK+="port5"' >/etc/udev/rules.d/port5.rules

service udev reload
sleep 2
service udev restart
