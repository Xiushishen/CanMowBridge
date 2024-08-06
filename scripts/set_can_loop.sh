#!/bin/bash

# set CANbus devices
sudo busybox devmem 0x0c303018 w 0xc458
sudo busybox devmem 0x0c303010 w 0xc400

# load CANbus kernel module
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# set CANbus internet port
sudo ip link set down can0
sudo ip link set can0 type can bitrate 1000000 loopback on
sudo ip link set up can0

