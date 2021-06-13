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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include <mutex>
#include "KeyFrame.h"
#include "Map.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

using namespace std;

namespace ORB_SLAM2
{

class Tracking;
class Map;

class LocalMapping
{
public:
    LocalMapping(Map* pMap);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    // Reset
    void RequestReset();

    // Finish
    void RequestFinish();
    bool isFinished();

    // Accept
    bool isAcceptKeyFrames();

    // Interrupt
    void InterruptBA();

private:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void KeyFrameCulling();

    void SearchInNeighbors();
    void CreateNewMapPoints();
    void MapPointCulling();

    cv::Mat ComputeF12(KeyFrame* pKF1, KeyFrame* pKF2);
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    // Reset
    void ResetIfRequested();
    bool mbResetRequested;
    mutex mMutexReset;

    // Finish flag
    bool isFinishRequested();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    mutex mMutexFinish;

    // Accept flag
    void SetAcceptKeyFrames(bool flag);
    bool mbAcceptKeyFrames;
    mutex mMutexAccept;

    // Interrupt
    bool mbAbortBA;

    Map* mpMap;
    Tracking* mpTracker;

    // KeyFrame and MapPoint
    KeyFrame* mpCurrentKeyFrame;
    list<KeyFrame*> mlNewKeyFrames;
    list<MapPoint*> mlpRecentAddedMapPoints;
    mutex mMutexNewKFs;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
