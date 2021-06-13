/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"

using namespace std;

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame& frame); // 拷贝构造函数

    // Constructor for Monocular cameras.
    Frame(const cv::Mat& imGray, const double& timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat& K,
          cv::Mat& distCoef); // 单目图像帧构造函数

    // Compute Bag of Words representation.
    void ComputeBoW(); // 计算BoW特征

    // Set the camera pose.
    void SetPose(const cv::Mat& Tcw); // 设置相机位姿
    cv::Mat GetRotation(); // 获取相机旋转矩阵
    cv::Mat GetTranslation(); // 获取相机平移向量
    cv::Mat GetRotationInverse(); // 获取相机旋转矩阵的逆矩阵
    cv::Mat GetCameraCenter(); // 获取相机中心坐标

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit); // 判断地图点是否在当前帧视野中

    // Image
    bool IsInImage(const float& x, const float& y); // 判断点是否在去畸变图像中

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint& kp, int& posX, int& posY); // 计算关键点的网格坐标

    vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r, const int minLevel=-1, const int maxLevel=-1) const; // 获取某个区域的内的所有关键点索引

public:
    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId; // 帧id

    // Frame timestamp.
    double mTimeStamp; // 帧时间戳

    // Reference Keyframe.
    KeyFrame* mpReferenceKF; // 参考关键帧

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK; // 相机内参矩阵
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef; // 相机畸变系数

    // Camera pose.
    cv::Mat mTcw; // 相机位姿

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractor; // ORB特征提取器

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    vector<cv::KeyPoint> mvKeys; // 原始关键点
    vector<cv::KeyPoint> mvKeysUn; // 去畸变的关键点

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors; // 描述子

    // MapPoints associated to keypoints, NULL pointer if no association.
    vector<MapPoint*> mvpMapPoints; // 地图点数组

    // Flag to identify outlier associations.
    vector<bool> mvbOutlier; // 外点标记数组

    // Number of KeyPoints.
    int N; // 关键点数量

    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary; // ORB词袋

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec; // 词袋向量
    DBoW2::FeatureVector mFeatVec; // 特征向量

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    vector<size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]; // 关键点在网格分布的索引数组

    // Scale pyramid info.
    int mnScaleLevels; // 尺度数量
    float mfScaleFactor; // 尺度倍数
    float mfLogScaleFactor; // 对数尺度倍数
    vector<float> mvScaleFactors; // 尺度数组
    vector<float> mvInvScaleFactors; // 逆尺度数组
    vector<float> mvLevelSigma2; // 方差数组
    vector<float> mvInvLevelSigma2; // 逆方差数组

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

private:
    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat& imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid(); // 计算关键点在网格分布的索引数组

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
