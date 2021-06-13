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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <mutex>
#include <opencv2/opencv.hpp>
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "ORBmatcher.h"

using namespace std;

namespace ORB_SLAM2
{

class Frame;
class KeyFrame;
class Map;

class MapPoint
{
public:
    MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap); // 利用关键帧构造地图点
    MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame, const int& idxF); // 利用普通帧构造地图点

    // World Position
    void SetWorldPos(const cv::Mat& Pos); // 设置地图点世界坐标
    cv::Mat GetWorldPos(); // 获取地图点世界坐标

    // Reference KeyFrame
    KeyFrame* GetReferenceKeyFrame(); // 获取参考关键帧

    // Observations
    void AddObservation(KeyFrame* pKF, size_t idx); // 添加观测
    void EraseObservation(KeyFrame* pKF); // 删除观测
    map<KeyFrame*, size_t> GetObservations(); // 获取所有观测，返回观察映射
    int Observations(); // 获取观测数量
    int GetIndexInKeyFrame(KeyFrame* pKF); // 获取地图点某关键帧中的索引
    bool IsInKeyFrame(KeyFrame* pKF); // 判断地图在某关键帧中是否被检测到

    // Bad Flag
    void SetBadFlag(); // 设置bad标记
    bool isBad(); // 判断地图点是否为bad

    // Replace
    void Replace(MapPoint* pMP); // 替换地图点
    MapPoint* GetReplaced(); // 获取用来替换的地图点

    // Tracking counters
    void IncreaseVisible(int n=1); // 增加可视计数
    void IncreaseFound(int n=1); // 增加发现计数
    float GetFoundRatio(); // 获取“发现/可视”计数比例

    // Descriptor
    void ComputeDistinctiveDescriptors(); // 计算描述子
    cv::Mat GetDescriptor(); // 获取描述子

    // Normal
    void UpdateNormalAndDepth(); // 计算平均观测方向向量，最小和最大不变性距离
    cv::Mat GetNormal(); // 获取平均观测方向向量

    // Distance Invariance
    float GetMinDistanceInvariance(); // 获取最小不变性距离
    float GetMaxDistanceInvariance(); // 获取最大不变性距离

    // Predict Scale
    int PredictScale(const float& currentDist, KeyFrame* pKF); // 预测距离对应的尺度
    int PredictScale(const float& currentDist, Frame* pF); // 预测距离对应的尺度

public:
    static long unsigned int nNextId;
    long unsigned int mnId; // 地图点id
    // This avoid that two points are created simultaneously in separate threads (id conflict)
    static mutex mMutexId; // 地图点id互斥锁

    long int mnFirstKFid; // 第一个关键帧的id

    // Variables used by the tracking
    float mTrackProjX; // 在Frame::isInFrustum中被赋值，在ORBmatcher::SearchByProjection中被使用
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;

    long unsigned int mnTrackReferenceForFrame; // 用于更新局部地图点
    long unsigned int mnLastFrameSeen; // 用于检查可视

    // Variables used by local mapping
    long unsigned int mnBALocalForKF; // 用于局部BA
    long unsigned int mnFuseCandidateForKF; // 用于融合地图点

private:
    // Position in absolute coordinates
    cv::Mat mWorldPos; // 地图点世界坐标

    // Map
    Map* mpMap; // 地图

    // Reference KeyFrame
    KeyFrame* mpRefKF; // 参考关键帧

    // Keyframes observing the point and associated index in keyframe
    map<KeyFrame*, size_t> mObservations; // 观测映射
    int nObs; // 观测数量

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad; // 地图点bad标记，LocalMapping线程删除地图点和融合地图点时被设置
    MapPoint* mpReplaced; // 用来替换的地图点

    // Tracking counters
    int mnVisible; // 可视计数
    int mnFound; // 发现（被ORB关键点检测器检测到）计数

    // Mean viewing direction
    cv::Mat mNormalVector; // 平均观测方向向量

    // Best descriptor to fast matching
    cv::Mat mDescriptor; // 描述子

    // Scale invariance distances
    float mfMinDistance; // 最小尺度不变距离
    float mfMaxDistance; // 最大尺度不变距离

    // Mutex
    mutex mMutexPos; // 位置互斥锁
    mutex mMutexFeatures; // 特征互斥锁
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
