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

#include "ORBmatcher.h"

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

int ORBmatcher::SearchByProjection(Frame& F, const vector<MapPoint*>& vpMapPoints, const float th)
{
    int nmatches = 0;

    for(size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int& nPredictedLevel = pMP->mnTrackScaleLevel;
        const float scaleFactor = F.mvScaleFactors[nPredictedLevel];

        // The size of the window will depend on the viewing direction
        float r = 4.0;
        if(pMP->mTrackViewCos > 0.998)
            r = 2.5;
        r *= th;

        const vector<size_t> vIndices = F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY, r * scaleFactor,
                                                            nPredictedLevel - 1, nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist = 256;
        int bestLevel = -1;
        int bestDist2 = 256;
        int bestLevel2 = -1;
        int bestIdx = -1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                continue;

            const cv::Mat& d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor, d);

            if(dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx = idx;
            }
            else if(dist < bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2 = dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist <= TH_HIGH)
        {
            if(bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
                continue;

            F.mvpMapPoints[bestIdx] = pMP;
            nmatches++;
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByBoW(KeyFrame* pKF, Frame& F, vector<MapPoint*>& vpMapPointMatches)
{
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<MapPoint*>(F.N, static_cast<MapPoint*>(NULL));

    const DBoW2::FeatureVector& vFeatVecKF = pKF->mFeatVec;

    int nmatches = 0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            for(size_t iKF = 0; iKF < vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];
                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat& dKF= pKF->mDescriptors.row(realIdxKF);

                int bestDist1 = 256;
                int bestIdxF = -1 ;
                int bestDist2 = 256;

                for(size_t iF = 0; iF < vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat& dF = F.mDescriptors.row(realIdxF);

                    const int dist =  DescriptorDistance(dKF, dF);
                    if(dist < bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdxF = realIdxF;
                    }
                    else if(dist < bestDist2)
                    {
                        bestDist2 = dist;
                    }
                }

                if(bestDist1 <= TH_LOW && bestDist1 < mfNNratio * bestDist2)
                {
                    vpMapPointMatches[bestIdxF] = pMP;

                    const cv::KeyPoint& kp = pKF->mvKeysUn[realIdxKF];
                    if(mbCheckOrientation)
                    {
                        float rot = kp.angle - F.mvKeys[bestIdxF].angle;
                        if(rot < 0.0)
                            rot += 360.0f;
                        int bin = int(round(rot * factor)) % HISTO_LENGTH;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdxF);
                    }
                    nmatches++;
                }
            }
            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for(int i = 0; i < HISTO_LENGTH; i++)
        {
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(size_t j = 0; j < rotHist[i].size(); j++)
            {
                vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame& F1, Frame& F2, vector<int>& vnMatches12, int windowSize)
{
    int nmatches = 0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(), INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

    for(size_t i1 = 0; i1 < F1.mvKeysUn.size(); i1++)
    {
        cv::KeyPoint& kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1 > 0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(kp1.pt.x, kp1.pt.y, windowSize, level1, level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
        {
            size_t i2 = *vit;
            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1, d2);
            if(vMatchedDistance[i2] <= dist)
                continue;

            if(dist < bestDist)
            {
                bestDist2 = bestDist;
                bestDist = dist;
                bestIdx2 = i2;
            }
            else if(dist < bestDist2)
            {
                bestDist2 = dist;
            }
        }

        if(bestDist <= TH_LOW && bestDist < mfNNratio * bestDist2)
        {
            if(vnMatches21[bestIdx2] >= 0)
            {
                vnMatches12[vnMatches21[bestIdx2]] = -1;
                nmatches--;
            }
            vnMatches12[i1] = bestIdx2;
            vnMatches21[bestIdx2] = i1;
            vMatchedDistance[bestIdx2] = bestDist;
            nmatches++;

            if(mbCheckOrientation)
            {
                float rot = F1.mvKeysUn[i1].angle - F2.mvKeysUn[bestIdx2].angle;
                if(rot < 0.0)
                    rot += 360.0f;
                int bin = int(round(rot * factor)) % HISTO_LENGTH;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(i1);
            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for(int i = 0; i < HISTO_LENGTH; i++)
        {
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(size_t j = 0; j < rotHist[i].size(); j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1] >= 0)
                {
                    vnMatches12[idx1] = -1;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchForTriangulation(KeyFrame* pKF1, KeyFrame* pKF2, cv::Mat F12, vector<pair<size_t, size_t>>& vMatchedPairs)
{    
    const DBoW2::FeatureVector& vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector& vFeatVec2 = pKF2->mFeatVec;

    // Compute epipoler in second image
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat C2 = R2w * Cw + t2w;
    const float invz = 1.0f / C2.at<float>(2);
    const float ex = pKF2->fx * C2.at<float>(0) * invz + pKF2->cx;
    const float ey = pKF2->fy * C2.at<float>(1) * invz + pKF2->cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node
    int nmatches = 0;
    vector<bool> vbMatched2(pKF2->N, false);
    vector<int> vMatches12(pKF1->N, -1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1 = 0; i1 < f1it->second.size(); i1++)
            {
                const size_t idx1 = f1it->second[i1];
                
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                
                // If there is already a MapPoint skip
                if(pMP1)
                    continue;
                
                const cv::KeyPoint& kp1 = pKF1->mvKeysUn[idx1];
                const cv::Mat& d1 = pKF1->mDescriptors.row(idx1);
                
                int bestDist = TH_LOW;
                int bestIdx2 = -1;
                
                for(size_t i2 = 0; i2 < f2it->second.size(); i2++)
                {
                    size_t idx2 = f2it->second[i2];
                    
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
                    
                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    const cv::Mat& d2 = pKF2->mDescriptors.row(idx2);
                    const int dist = DescriptorDistance(d1, d2);
                    if(dist > TH_LOW || dist > bestDist)
                        continue;

                    const cv::KeyPoint& kp2 = pKF2->mvKeysUn[idx2];
                    const float distex = ex - kp2.pt.x;
                    const float distey = ey - kp2.pt.y;
                    if(distex * distex + distey * distey< 100 * pKF2->mvScaleFactors[kp2.octave])
                        continue;

                    if(CheckDistEpipolarLine(kp1, kp2, F12, pKF2))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                
                if(bestIdx2 >= 0)
                {
                    const cv::KeyPoint& kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1] = bestIdx2;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = kp1.angle - kp2.angle;
                        if(rot < 0.0)
                            rot += 360.0f;
                        int bin = int(round(rot * factor)) % HISTO_LENGTH;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for(int i = 0; i < HISTO_LENGTH; i++)
        {
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(size_t j = 0; j < rotHist[i].size(); j++)
            {
                int idx1 = rotHist[i][j];
                vMatches12[idx1] = -1;
                nmatches--;
            }
        }
    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i = 0; i < vMatches12.size(); i++)
    {
        if(vMatches12[i] < 0)
            continue;
        vMatchedPairs.push_back(make_pair(i, vMatches12[i]));
    }

    return nmatches;
}

int ORBmatcher::Fuse(KeyFrame* pKF, const vector<MapPoint*>& vpMapPoints, const float th)
{
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();

    const float& fx = pKF->fx;
    const float& fy = pKF->fy;
    const float& cx = pKF->cx;
    const float& cy = pKF->cy;

    cv::Mat Ow = pKF->GetCameraCenter();

    int nFused = 0;

    const int nMPs = vpMapPoints.size();

    for(int i = 0; i < nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2) < 0.0f)
            continue;

        const float invz = 1 / p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0) * invz;
        const float y = p3Dc.at<float>(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u, v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw - Ow;
        const float dist3D = cv::norm(PO);

        // Depth must be inside the scale pyramid of the image
        if(dist3D < minDistance || dist3D > maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn) < 0.5 * dist3D)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

        // Search in a radius
        const float radius = th * pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint& kp = pKF->mvKeysUn[idx];
            const int& kpLevel= kp.octave;

            if(kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                continue;

            const float& kpx = kp.pt.x;
            const float& kpy = kp.pt.y;
            const float ex = u - kpx;
            const float ey = v - kpy;
            const float e2 = ex * ex + ey * ey;

            if(e2 * pKF->mvInvLevelSigma2[kpLevel] > 5.99)
                continue;

            const cv::Mat& dKF = pKF->mDescriptors.row(idx);
            const int dist = DescriptorDistance(dMP, dKF);

            if(dist < bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist <= TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations() > pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            else
            {
                pMP->AddObservation(pKF, bestIdx);
                pKF->AddMapPoint(pMP, bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::SearchByProjection(Frame& CurrentFrame, const Frame& LastFrame, const float th)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);
    const cv::Mat twc = -Rcw.t() * tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0, 3).col(3);
    const cv::Mat tlc = Rlw * twc + tlw;

    for(int i = 0; i < LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            // Project
            cv::Mat x3Dw = pMP->GetWorldPos();
            cv::Mat x3Dc = Rcw * x3Dw + tcw;

            const float xc = x3Dc.at<float>(0);
            const float yc = x3Dc.at<float>(1);
            const float invzc = 1.0 / x3Dc.at<float>(2);

            if(invzc < 0)
                continue;

            float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
            float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;
            if(!CurrentFrame.IsInImage(u, v))
                continue;

            int nLastOctave = LastFrame.mvKeys[i].octave;

            // Search in a window. Size depends on scale
            float radius = th * CurrentFrame.mvScaleFactors[nLastOctave];

            vector<size_t> vIndices2;
            vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nLastOctave - 1, nLastOctave + 1);

            if(vIndices2.empty())
                continue;

            const cv::Mat dMP = pMP->GetDescriptor();

            int bestDist = 256;
            int bestIdx2 = -1;

            for(vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
            {
                const size_t i2 = *vit;
                if(CurrentFrame.mvpMapPoints[i2])
                    continue;

                const cv::Mat& d = CurrentFrame.mDescriptors.row(i2);

                const int dist = DescriptorDistance(dMP, d);
                if(dist < bestDist)
                {
                    bestDist = dist;
                    bestIdx2 = i2;
                }
            }

            if(bestDist <= TH_HIGH)
            {
                CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = LastFrame.mvKeysUn[i].angle - CurrentFrame.mvKeysUn[bestIdx2].angle;
                    if(rot < 0.0)
                        rot += 360.0f;
                    int bin = int(round(rot*factor)) % HISTO_LENGTH;
                    assert(bin >= 0 && bin < HISTO_LENGTH);
                    rotHist[bin].push_back(bestIdx2);
                }
            }
        }
    }

    // Apply rotation consistency
    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for(int i = 0; i < HISTO_LENGTH; i++)
        {
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(size_t j = 0; j < rotHist[i].size(); j++)
            {
                CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByProjection(Frame& CurrentFrame, KeyFrame* pKF, const set<MapPoint*>& sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0, 3).colRange(0, 3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0, 3).col(3);
    const cv::Mat Ow = -Rcw.t() * tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i = 0; i < HISTO_LENGTH; i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f / HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i = 0; i < vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                // Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0 / x3Dc.at<float>(2);

                const float u = CurrentFrame.fx * xc * invzc + CurrentFrame.cx;
                const float v = CurrentFrame.fy * yc * invzc + CurrentFrame.cy;

                if(CurrentFrame.IsInImage(u, v))
                    continue;

                // Compute predicted scale level
                cv::Mat PO = x3Dw - Ow;
                float dist3D = cv::norm(PO);

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D < minDistance || dist3D > maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

                // Search in a window
                const float radius = th * CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel - 1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat& d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP, d);

                    if(dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdx2 = i2;
                    }
                }

                if(bestDist <= ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2] = pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot < 0.0)
                            rot += 360.0f;
                        int bin = int(round(rot*factor)) % HISTO_LENGTH;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

        for(int i = 0; i < HISTO_LENGTH; i++)
        {
            if(i == ind1 || i == ind2 || i == ind3)
                continue;
            for(size_t j = 0; j < rotHist[i].size(); j++)
            {
                CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint& kp1,const cv::KeyPoint& kp2,const cv::Mat& F12,const KeyFrame* pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x * F12.at<float>(0, 0) + kp1.pt.y * F12.at<float>(1, 0) + F12.at<float>(2, 0);
    const float b = kp1.pt.x * F12.at<float>(0, 1) + kp1.pt.y * F12.at<float>(1, 1) + F12.at<float>(2, 1);
    const float c = kp1.pt.x * F12.at<float>(0, 2) + kp1.pt.y * F12.at<float>(1, 2) + F12.at<float>(2, 2);

    const float num = a * kp2.pt.x + b * kp2.pt.y + c;
    const float den = a * a + b * b;
    if(den == 0)
        return false;

    const float dsqr = num * num / den;

    return dsqr < 3.84 * pKF2->mvLevelSigma2[kp2.octave];
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    for(int i = 0; i < L; i++)
    {
        const int s = histo[i].size();
        if(s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if(s > max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if(s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if(max2 < 0.1f * (float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if(max3 < 0.1f * (float)max1)
    {
        ind3 = -1;
    }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
