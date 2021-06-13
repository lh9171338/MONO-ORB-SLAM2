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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <mutex>
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Converter.h"
#include "ORBmatcher.h"

using namespace std;

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat& Tcw); // 设置相机位姿
    cv::Mat GetPose(); // 获取相机位姿
    cv::Mat GetPoseInverse(); // 获取相机位姿的逆矩阵
    cv::Mat GetCameraCenter(); // 获取相机中心坐标
    cv::Mat GetRotation(); // 获取相机旋转矩阵
    cv::Mat GetTranslation(); // 获取相机平移向量

    // Bag of Words Representation
    void ComputeBoW(); // 计算BoW特征

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int& weight); // 共视图添加连接
    void EraseConnection(KeyFrame* pKF); // 共视图删除连接
    void UpdateConnections(); // 计算自身共视图
    void UpdateBestCovisibles(); // 计算其他帧共视图
    set<KeyFrame*> GetConnectedKeyFrames(); // 获取所有共视帧，返回集合
    vector<KeyFrame*> GetVectorCovisibleKeyFrames(); // 获取所有共视帧，返回数组
    vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int& N); // 获取权重最高的前N个共视帧，返回数组
    vector<KeyFrame*> GetCovisiblesByWeight(const int& w); // 获取权重大于w的共视帧，返回数组
    int GetWeight(KeyFrame* pKF); // 获取共视图某帧权重

    // Spanning tree functions
    void AddChild(KeyFrame* pKF); // 添加子关键帧
    void EraseChild(KeyFrame* pKF); // 删除子关键帧
    void ChangeParent(KeyFrame* pKF); // 改变父关键帧
    set<KeyFrame*> GetChilds(); // 获取所有子关键帧，返回集合
    KeyFrame* GetParent(); // 获取父关键帧

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t& idx); // 添加地图点
    void EraseMapPointMatch(const size_t& idx); // 删除地图点
    void EraseMapPointMatch(MapPoint* pMP); // 删除地图点
    void ReplaceMapPointMatch(const size_t& idx, MapPoint* pMP); // 替换地图点
    set<MapPoint*> GetMapPoints(); // 获取所有有效地图点，返回集合
    vector<MapPoint*> GetMapPointMatches(); // 获取所有地图点，返回数组
    MapPoint* GetMapPoint(const size_t& idx); // 获取地图点
    int TrackedMapPoints(const int& minObs); // 跟踪地图点，返回数量

    // KeyPoint functions
    vector<size_t> GetFeaturesInArea(const float& x, const float& y, const float& r) const; // 获取某个区域的内的所有关键点索引

    // Image
    bool IsInImage(const float& x, const float& y) const; // 判断点是否在去畸变图像中

    // Set/check bad flag
    void SetBadFlag(); // 设置bad标记
    bool isBad(); // 判断关键帧是否为bad

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q); // 单目计算场景深度

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId < pKF2->mnId;
    }

    // The following variables are accessed from only 1 thread or never change (no mutex needed).
public:
    static long unsigned int nNextId;
    long unsigned int mnId; // 关键帧id

    const double mTimeStamp; // 时间戳

    // Calibration parameters
    const cv::Mat mK; // 相机内参矩阵
    const float fx, fy, cx, cy, invfx, invfy;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const vector<cv::KeyPoint> mvKeys; // 原始关键点
    const vector<cv::KeyPoint> mvKeysUn; // 去畸变的关键点
    const cv::Mat mDescriptors; // 描述子
    // Number of KeyPoints
    const int N; // 关键点数量

    // BoW
    DBoW2::BowVector mBowVec; // 词袋向量
    DBoW2::FeatureVector mFeatVec; // 特征向量

    // Scale
    const int mnScaleLevels; // 尺度数量
    const float mfScaleFactor; // 尺度倍数
    const float mfLogScaleFactor; // 对数尺度倍数
    const vector<float> mvScaleFactors; // 尺度数组
    const vector<float> mvLevelSigma2; // 方差数组
    const vector<float> mvInvLevelSigma2; // 逆方差数组

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame; // 用于更新局部关键帧
    long unsigned int mnFuseTargetForKF; // 用于融合地图点

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF; // 用于局部BA
    long unsigned int mnBAFixedForKF; // 用于局部BA

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery; // 用于重定位
    int mnLoopWords; // 用于重定位
    float mLoopScore; // 用于重定位
    long unsigned int mnRelocQuery; // 用于重定位
    int mnRelocWords; // 用于重定位
    float mRelocScore; // 用于重定位

    // The following variables need to be accessed trough a mutex to be thread safe.
private:
    // SE3 Pose and camera center
    cv::Mat mTcw; // 相机位姿
    cv::Mat mTwc; // 相机位姿逆矩阵
    cv::Mat mOw; // 相机中心坐标
    mutex mMutexPose; // 相机位姿互斥锁

    // Map
    Map* mpMap; // 地图

    // MapPoints associated to keypoints
    vector<MapPoint*> mvpMapPoints; // 地图点数组

    // BoW
    KeyFrameDatabase* mpKeyFrameDB; // 关键帧数据库
    ORBVocabulary* mpORBvocabulary; // ORB词袋

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;
    vector<vector<vector<size_t>>> mGrid; // 关键点在网格分布的索引数组

    // Covisibility connection
    map<KeyFrame*, int> mConnectedKeyFrameWeights; // 共视图映射
    vector<KeyFrame*> mvpOrderedConnectedKeyFrames; // 共视图关键帧数组
    vector<int> mvOrderedWeights; // 共视图权重数组
    mutex mMutexConnections; // 共视图姿互斥锁

    // Spanning Tree
    bool mbFirstConnection; // 初始化标记
    KeyFrame* mpParent; // 父关键帧
    set<KeyFrame*> mspChildrens; // 子关键帧集合

    // Bad flags
    bool mbBad; // bad标记，LocalMapping线程删除关键帧时被设置

    // Mutex
    mutex mMutexFeatures; // 特征姿互斥锁

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
