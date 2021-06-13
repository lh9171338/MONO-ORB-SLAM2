# CERES-MONO-ORB-SLAM2-ROS

# 1. Prerequisites

## C++11 Compiler
We use the new thread and chrono functionalities of C++11.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Tested with OpenCV 3.4.3**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Tested with Eigen3 3.3.4**.

## Ceres-Solver
We use [Ceres-Solver](http://www.ceres-solver.org/index.html) to perform optimization. Dowload and install instructions can be found at: http://www.ceres-solver.org/index.html.

## DBoW2 (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition. The modified library is included in the *Thirdparty* folder.

## ROS
You need to install [ROS](http://wiki.ros.org/ROS/Installation) first. Install instruction can be found at: http://wiki.ros.org/ROS/Installation. We just tested ROS melodic version.

# 2. Building ORB-SLAM2 library and examples

create workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
```

Clone the repository:
```
cd src
git clone -b g2o-ros https://github.com/lh9171338/MONO-ORB-SLAM2.git
```

We provide a script `build.sh` to build the *Thirdparty* libraries. Please make sure you have installed all required dependencies (see section 1). Execute:
```
cd MONO-ORB-SLAM2
chmod +x build.sh
./build.sh
```

Build the package:
```
cd ../..
catkin_make
```

# 3. Examples

## TUM Dataset

1. Download a bag file from http://vision.in.tum.de/data/datasets/rgbd-dataset/download.

2. Edit the script `scripts/mono_tum.sh` and the launch file`launch/mono_tum.launch` according to the dataset, then execute the following command.
```
./scripts/mono_tum.sh 

```

## EuRoC Dataset

1. Download a bag file from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets.

2. Edit the script `scripts/mono_euroc.sh` and the launch file `launch/mono_euroc.launch` according to the dataset, then execute the following command.
```
./scripts/mono_euroc.sh 

```
