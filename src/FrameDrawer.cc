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

#include "FrameDrawer.h"

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap): mpMap(pMap)
{
    mState = Tracking::SYSTEM_NOT_READY;
    mImage = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
}

void FrameDrawer::Update(Tracking* pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mImage);
    mTimeStamp = pTracker->mCurrentFrame.mTimeStamp;
    mvCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbMap = vector<bool>(N, false);

    if(pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED)
    {
        mvIniKeys = pTracker->mInitialFrame.mvKeys;
        mvIniMatches = pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState == Tracking::OK)
    {
        for(int i = 0; i < N; i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP && !pMP->isBad())
                mvbMap[i] = true;
        }
    }
    mState = static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::DrawFrame(ros::Publisher& pub)
{
    cv::Mat image;
    double timeStamp;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondences with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    // Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state = mState;
        timeStamp = mTimeStamp;
        if(mState == Tracking::SYSTEM_NOT_READY)
            mState = Tracking::NO_IMAGES_YET;

        mImage.copyTo(image);

        if(mState == Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState == Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbMap = mvbMap;
        }
        else if(mState == Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(image.channels() < 3) // this should be always true
        cvtColor(image, image, CV_GRAY2BGR);

    // Draw
    if(state == Tracking::NOT_INITIALIZED) // INITIALIZING
    {
        for(unsigned int i = 0; i < vMatches.size(); i++)
        {
            if(vMatches[i] >= 0)
            {
                cv::line(image, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt, cv::Scalar(0, 255, 0));
            }
        }        
    }
    else if(state == Tracking::OK) // TRACKING
    {
        mnTracked = 0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i = 0; i < n; i++)
        {
            if(vbMap[i])
            {
                cv::Point2f pt1, pt2;
                pt1.x = vCurrentKeys[i].pt.x - r;
                pt1.y = vCurrentKeys[i].pt.y - r;
                pt2.x = vCurrentKeys[i].pt.x + r;
                pt2.y = vCurrentKeys[i].pt.y + r;

                // This is a match to a MapPoint in the map
                cv::rectangle(image, pt1, pt2,cv::Scalar(0, 255, 0));
                cv::circle(image, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
                mnTracked++;
            }
        }
    }

    cv::Mat imageWithInfo;
    DrawTextInfo(image, state, imageWithInfo);

    // Publish image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageWithInfo).toImageMsg();
    msg->header.stamp = ros::Time(timeStamp);
    pub.publish(msg);
}

void FrameDrawer::DrawTextInfo(cv::Mat& image, int nState, cv::Mat& imageText)
{
    stringstream s;
    if(nState == Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState == Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState  == Tracking::OK)
    {
        s << "SLAM MODE |  ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
    }
    else if(nState == Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState == Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline = 0;
    cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    imageText = cv::Mat(image.rows + textSize.height + 10, image.cols, image.type());
    image.copyTo(imageText.rowRange(0, image.rows).colRange(0, image.cols));
    imageText.rowRange(image.rows, imageText.rows) = cv::Mat::zeros(textSize.height + 10, image.cols, image.type());
    cv::putText(imageText, s.str(), cv::Point(5, imageText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

} //namespace ORB_SLAM
