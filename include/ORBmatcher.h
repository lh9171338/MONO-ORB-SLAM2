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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

using namespace std;

namespace ORB_SLAM2
{

class Frame;
class MapPoint;
class KeyFrame;

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat& a, const cv::Mat& b);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame& F, const vector<MapPoint*>& vpMapPoints, const float th=3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame& CurrentFrame, const Frame& LastFrame, const float th);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame& CurrentFrame, KeyFrame* pKF, const set<MapPoint*>& sAlreadyFound, const float th, const int ORBdist);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    int SearchByBoW(KeyFrame* pKF, Frame& F, vector<MapPoint*>& vpMapPointMatches);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame& F1, Frame& F2, vector<int>& vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame* pKF1, KeyFrame* pKF2, cv::Mat F12, vector<pair<size_t, size_t>>& vMatchedPairs);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint*>& vpMapPoints, const float th=3.0);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

private:

    bool CheckDistEpipolarLine(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& F12, const KeyFrame* pKF);
    void ComputeThreeMaxima(vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
