#!/bin/bash

gnome-terminal --window -- bash -c "roslaunch ORB_SLAM2 mono_tum.launch;exec bash"

sleep 5

gnome-terminal --window -- bash -c "rosbag play data/rgbd_dataset_freiburg1_xyz.bag;exec bash"