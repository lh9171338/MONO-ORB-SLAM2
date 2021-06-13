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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrame.h"
#include "CameraPoseVisualization.h"


using namespace std;

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string& strSettingPath);

    void SetCurrentCameraPose(const cv::Mat& Tcw, const double timeStamp);
    void DrawOdometry(ros::Publisher& pubOdom, ros::Publisher& pubPath, string frame_id, string child_frame_id);
    void DrawMapPoints(ros::Publisher& pub, string frame_id);
    void DrawCurrentCamera(ros::Publisher& pub, string frame_id);
    void DrawKeyFrames(ros::Publisher& pub, string frame_id);

private:

    Map* mpMap;
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;
    double mTimeStamp;
    nav_msgs::Path mPath;

    mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
