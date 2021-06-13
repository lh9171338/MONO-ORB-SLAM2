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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <mutex>
#include "MapPoint.h"
#include "KeyFrame.h"

using namespace std;

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    // Clear
    void clear(); // 清除地图

    // KeyFrame
    void AddKeyFrame(KeyFrame* pKF); // 添加关键帧
    void EraseKeyFrame(KeyFrame* pKF); // 删除关键帧
    vector<KeyFrame*> GetAllKeyFrames(); // 获取所有关键帧，返回数组
    long unsigned int KeyFramesInMap(); // 获取关键帧数量

    // MapPoint
    void AddMapPoint(MapPoint* pMP); // 添加地图点
    void EraseMapPoint(MapPoint* pMP); // 删除地图点
    vector<MapPoint*> GetAllMapPoints(); // 获取所有地图点，返回数组
    long unsigned int MapPointsInMap(); // 获取地图点数量

    // Local MapPoints
    void SetLocalMapPoints(const vector<MapPoint*>& vpMPs); // 设置局部地图点数组
    vector<MapPoint*> GetLocalMapPoints(); // 获取局部地图点数组

    mutex mMutexMapUpdate; // 地图更新互斥锁，用于局部BA

private:
    mutex mMutexMap; // 地图互斥锁

    set<KeyFrame*> mspKeyFrames; // 关键帧集合
    set<MapPoint*> mspMapPoints; // 地图点集合
    vector<MapPoint*> mvpLocalMapPoints; // 局部地图点数组，用于可视化
};

} //namespace ORB_SLAM

#endif // MAP_H
