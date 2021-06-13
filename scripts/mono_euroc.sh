#!/bin/bash

gnome-terminal --window -- bash -c "roslaunch ORB_SLAM2 mono_euroc.launch;exec bash"

sleep 5

gnome-terminal --window -- bash -c "rosbag play data/V1_01_easy.bag --clock;exec bash"