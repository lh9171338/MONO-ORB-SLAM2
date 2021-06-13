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

#include "MapDrawer.h"


namespace ORB_SLAM2
{

MapDrawer::MapDrawer(Map* pMap, const string& strSettingPath) : mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
}

void MapDrawer::DrawOdometry(ros::Publisher& pubOdom, ros::Publisher& pubPath, string frame_id, string child_frame_id)
{
    Eigen::Vector7d pose;
    double timeStamp;
    {
        unique_lock<mutex> lock(mMutexCamera);
        if(mCameraPose.empty())
            return;
        pose = Converter::toVector7d(mCameraPose.inv());
        timeStamp = mTimeStamp;
    }
    Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
    Eigen::Vector3d t(pose.block<3, 1>(4, 0));

    // Publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time(timeStamp);
    odometry.header.frame_id = frame_id;
    odometry.child_frame_id = child_frame_id;
    odometry.pose.pose.position.x = t.x();
    odometry.pose.pose.position.y = t.y();
    odometry.pose.pose.position.z = t.z();

    odometry.pose.pose.orientation.w = q.w();
    odometry.pose.pose.orientation.x = q.x();
    odometry.pose.pose.orientation.y = q.y();
    odometry.pose.pose.orientation.z = q.z();
    pubOdom.publish(odometry);

    // Publish path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.pose = odometry.pose.pose;
    mPath.header = odometry.header;
    mPath.poses.push_back(pose_stamped);
    pubPath.publish(mPath);

    // Publish TF
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time(timeStamp);
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.rotation = odometry.pose.pose.orientation;
    broadcaster.sendTransform(transformStamped);
}

void MapDrawer::DrawMapPoints(ros::Publisher& pub, string frame_id)
{
    double timeStamp;
    {
        unique_lock<mutex> lock(mMutexCamera);
        timeStamp = mTimeStamp;
    }

    const vector<MapPoint*>& vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*>& vpLocalMPs = mpMap->GetLocalMapPoints();

    set<MapPoint*> spLocalMPs(vpLocalMPs.begin(), vpLocalMPs.end());

    if(vpMPs.empty())
        return;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 绘制非Local的地图点，颜色为黑色
    for(size_t i = 0; i < vpMPs.size(); i++)
    {
        if(vpMPs[i]->isBad() || spLocalMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGB point(0, 0, 0);
        point.x = pos.at<float>(0, 0);
        point.y = pos.at<float>(1, 0);
        point.z = pos.at<float>(2, 0);
        pointcloud->points.push_back(point);
    }

    // 绘制Local的地图点，颜色为红色
    for(set<MapPoint*>::iterator sit = spLocalMPs.begin(); sit != spLocalMPs.end(); sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGB point(255, 0, 0);
        point.x = pos.at<float>(0, 0);
        point.y = pos.at<float>(1, 0);
        point.z = pos.at<float>(2, 0);
        pointcloud->points.push_back(point);
    }

    // Publish MapPoints
    pointcloud->width = pointcloud->points.size();
    pointcloud->height = 1;
    pointcloud->header.stamp = pcl_conversions::toPCL(ros::Time(timeStamp));
    pointcloud->header.frame_id = frame_id;
    pub.publish(pointcloud);
}

void MapDrawer::DrawCurrentCamera(ros::Publisher& pub, string frame_id)
{
    Eigen::Vector7d pose;
    double timeStamp;
    {
        unique_lock<mutex> lock(mMutexCamera);
        if(mCameraPose.empty())
            return;
        pose = Converter::toVector7d(mCameraPose.inv());
        timeStamp = mTimeStamp;
    }
    Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
    Eigen::Vector3d t(pose.block<3, 1>(4, 0));

    // Publish Camera
    std_msgs::Header header;
    header.stamp = ros::Time(timeStamp);
    header.frame_id = frame_id;
    CameraPoseVisualization cameraposevisual(0, 1, 0, 1); // 绿色
    cameraposevisual.setScale(mCameraSize);
    cameraposevisual.setLineWidth(mCameraLineWidth);
    cameraposevisual.reset();
    cameraposevisual.add_pose(t, q);
    cameraposevisual.publish_by(pub, header);
}

void MapDrawer::DrawKeyFrames(ros::Publisher& pub, string frame_id)
{
    Eigen::Vector7d pose;
    double timeStamp;
    {
        unique_lock<mutex> lock(mMutexCamera);
        timeStamp = mTimeStamp;
    }

    std_msgs::Header header;
    header.stamp = ros::Time(timeStamp);
    header.frame_id = frame_id;
    CameraPoseVisualization cameraposevisual(0, 0, 1, 1); // 蓝色
    cameraposevisual.setScale(mKeyFrameSize);
    cameraposevisual.setLineWidth(mKeyFrameLineWidth);
    cameraposevisual.reset();

    // 绘制所有关键帧
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    for(size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        cv::Mat Twc = pKF->GetPoseInverse().t();
        pose = Converter::toVector7d(pKF->GetPoseInverse());
        Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
        Eigen::Vector3d t(pose.block<3, 1>(4, 0));
        cameraposevisual.add_pose(t, q);
    }

    // 绘制共视图
    for(size_t i = 0; i < vpKFs.size(); i++)
    {
        // Covisibility Graph
        const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
        Eigen::Vector3d Ow = Converter::toVector3d(vpKFs[i]->GetCameraCenter());
        if(!vCovKFs.empty())
        {
            for(vector<KeyFrame*>::const_iterator vit = vCovKFs.begin(); vit != vCovKFs.end(); vit++)
            {
                if((*vit)->mnId < vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3d Ow2 = Converter::toVector3d((*vit)->GetCameraCenter());
                cameraposevisual.add_edge(Ow, Ow2, mGraphLineWidth);
            }
        }
        // Spanning tree
        KeyFrame* pParent = vpKFs[i]->GetParent();
        if(pParent)
        {
            Eigen::Vector3d Owp = Converter::toVector3d(pParent->GetCameraCenter());
            cameraposevisual.add_edge(Ow, Owp, mGraphLineWidth);
        }
    }

    cameraposevisual.publish_by(pub, header);
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat& Tcw, const double timeStamp)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mTimeStamp = timeStamp;
}


} //namespace ORB_SLAM
