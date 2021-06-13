#include "Viewer.h"

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, const string& strSettingPath):
        mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mbFinishRequested(false),
        mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fps = fSettings["Camera.fps"];
    if(fps < 1)
        fps = 30;
    mT = 1.0 / fps;

    nh_priv = ros::NodeHandle("~");
    nh_priv.param<string>("mStrOdomFrame", mStrOdomFrame, "odom");
    nh_priv.param<string>("mStrBodyFrame", mStrBodyFrame, "body");

    // Advertise topics
    mPubOdom = nh_priv.advertise<nav_msgs::Odometry>("odometry", 1000);
    mPubPath = nh_priv.advertise<nav_msgs::Path>("path", 1000);
    mPubFrame = nh_priv.advertise<sensor_msgs::Image>("image", 1000);
    mPubMapPoint = nh_priv.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("mappoint", 1000);
    mPubCamera = nh_priv.advertise<visualization_msgs::MarkerArray>("camera", 1000);
    mPubKeyFrame = nh_priv.advertise<visualization_msgs::MarkerArray>("keyframe", 1000);
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    ros::Duration duration(mT);

    while(ros::ok())
    {
        // Publish topics
        mpFrameDrawer->DrawFrame(mPubFrame);
        mpMapDrawer->DrawOdometry(mPubOdom, mPubPath, mStrOdomFrame, mStrBodyFrame);
        mpMapDrawer->DrawMapPoints(mPubMapPoint, mStrOdomFrame);
        mpMapDrawer->DrawCurrentCamera(mPubCamera, mStrOdomFrame);
        mpMapDrawer->DrawKeyFrames(mPubKeyFrame, mStrOdomFrame);

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(isFinishRequested())
            break;

        // Sleep
        ros::spinOnce();
        duration.sleep();
    }
    ROS_INFO("222");

    SetFinish();
//    while(1)
//    {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
//
//        if(menuFollowCamera && bFollow)
//        {
//            s_cam.Follow(Twc);
//        }
//        else if(menuFollowCamera && !bFollow)
//        {
//            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
//            s_cam.Follow(Twc);
//            bFollow = true;
//        }
//        else if(!menuFollowCamera && bFollow)
//        {
//            bFollow = false;
//        }
//
//        d_cam.Activate(s_cam);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//        mpMapDrawer->DrawCurrentCamera(Twc);
//        if(menuShowKeyFrames || menuShowGraph)
//            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
//        if(menuShowPoints)
//            mpMapDrawer->DrawMapPoints();
//
//        pangolin::FinishFrame();
//
//        cv::Mat im = mpFrameDrawer->DrawFrame();
//        cv::imshow("ORB-SLAM2: Current Frame", im);
//        cv::waitKey(mT);
//
//        if(isFinishRequested())
//            break;
//    }
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::isFinishRequested()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;
}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
