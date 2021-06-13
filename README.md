# CERES-MONO-ORB-SLAM2

# 1. Prerequisites

## C++11 Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Tested with OpenCV 3.4.3**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Tested with Eigen3 3.3.4**.

## Ceres-Solver
We use [Ceres-Solver](http://www.ceres-solver.org/index.html) to perform optimization. Dowload and install instructions can be found at: http://www.ceres-solver.org/index.html.

## DBoW2 (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition. The modified library is included in the *Thirdparty* folder.

# 2. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone -b ceres https://github.com/lh9171338/MONO-ORB-SLAM2.git
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 1). Execute:
```
cd MONO-ORB-SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, and **mono_euroc** in *examples* folder.

# 3. Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./examples/mono_tum Vocabulary/ORBvoc.txt examples/TUMX.yaml PATH_TO_SEQUENCE_FOLDER

```

You can also use the script `scripts/mono_tum.sh`. Execute:
```
chmod +x scripts/mono_tum.sh
./scripts/mono_tum.sh
```

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.
```
./examples/mono_kitti Vocabulary/ORBvoc.txt examples/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

You can also use the script `scripts/mono_kitti.sh`. Execute:
```
chmod +x scripts/mono_kitti.sh
./scripts/mono_kitti.sh
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./examples/mono_euroc Vocabulary/ORBvoc.txt examples/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data examples/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./examples/mono_euroc Vocabulary/ORBvoc.txt examples/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data examples/EuRoC_TimeStamps/SEQUENCE.txt 
```

You can also use the script `scripts/mono_euroc.sh`. Execute:
```
chmod +x scripts/mono_euroc.sh
./scripts/mono_euroc.sh
````
