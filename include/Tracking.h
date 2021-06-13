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


#ifndef TRACKING_H
#define TRACKING_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "ORBmatcher.h"
#include "Converter.h"
#include "PnPsolver.h"
#include "Optimizer.h"

using namespace std;

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class System;
class Initializer;

class Tracking
{
public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string& strSettingPath);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetViewer(Viewer* pViewer);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageMonocular(const cv::Mat& im, const double& timestamp);

    void Reset();

public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,    // 加载配置文件
        NO_IMAGES_YET=0,        // 等待图像
        NOT_INITIALIZED=1,      // 未初始化
        OK=2,                   // 正常工作
        LOST=3                  // 跟踪失败
    };
    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initial variables
    Frame mInitialFrame;
    vector<int> mvIniMatches;
    vector<cv::Point3f> mvIniP3D;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

private:
    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();

    // (1) Track reference keyframe
    bool TrackReferenceKeyFrame();

    // (2) Track with motion model
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    // (3) Relocalize
    bool Relocalization();

    // Track local map
    bool TrackLocalMap();
    void UpdateLocalMap();
    void UpdateLocalKeyFrames();
    void UpdateLocalPoints();
    void SearchLocalPoints();

    // Insert new keyframe
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Other Thread Pointers
    LocalMapping* mpLocalMapper;

    // ORB
    ORBextractor* mpORBextractor;
    ORBextractor* mpIniORBextractor;

    // BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initialization (only for monocular)
    Initializer* mpInitializer;

    // Local Map
    KeyFrame* mpReferenceKF; // 选择共视地图点最多的共视关键帧作为当前帧的参考关键帧
    vector<KeyFrame*> mvpLocalKeyFrames; // 局部关键帧，包括：1) 当前帧的所有共视关键帧; 2) 当前帧的所有共视关键帧的共视关键帧;
                                        // 3) 当前帧的所有共视关键帧的一个子关键帧; 4) 当前帧的所有共视关键帧的父关键帧
                                        // mnTrackReferenceForFrame作为已添加的标志位
    vector<MapPoint*> mvpLocalMapPoints; // 局部地图点，包括所有局部关键帧的地图点
                                        // mnTrackReferenceForFrame作为已添加的标志位
    
    // System
    System* mpSystem;
    
    // Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // Map
    Map* mpMap;

    // Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;

    // New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Current matches in frame
    int mnMatchesInliers;

    // Last Frame, KeyFrame and Relocalisation Info
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    // Motion Model
    cv::Mat mVelocity;

    // Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
