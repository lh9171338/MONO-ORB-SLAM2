#ifndef RVIZVIEWER_H
#define RVIZVIEWER_H

#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "System.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"


using namespace std;

namespace ORB_SLAM2
{

class System;
class FrameDrawer;
class MapDrawer;

class Viewer
{
public:
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const string& strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    // Finish
    void RequestFinish();
    bool isFinished();

    // Stop
    void RequestStop();
    bool isStopped();
    void Release();

private:

    // Finish
    bool isFinishRequested();
    void SetFinish();

    // Stop
    bool Stop();

private:

    // 1/fps
    double mT;

    // ROS
    ros::NodeHandle nh_priv;
    string mStrOdomFrame;
    string mStrBodyFrame;
    ros::Publisher mPubOdom;
    ros::Publisher mPubPath;
    ros::Publisher mPubFrame;
    ros::Publisher mPubMapPoint;
    ros::Publisher mPubCamera;
    ros::Publisher mPubKeyFrame;

    // Drawer
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // Finish
    bool mbFinishRequested;
    bool mbFinished;
    mutex mMutexFinish;

    // Stop
    bool mbStopRequested;
    bool mbStopped;
    mutex mMutexStop;
};

}


#endif // RVIZVIEWER_H
	

