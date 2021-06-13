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


#include "Converter.h"

namespace ORB_SLAM2
{

vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat& Descriptors)
{
    vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j = 0; j < Descriptors.rows; j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

cv::Mat Converter::toCvMat(const Eigen::Matrix4d& m)
{
    cv::Mat cvMat(4, 4, CV_32F);
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d& m)
{
    cv::Mat cvMat(3, 3, CV_32F);
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            cvMat.at<float>(i, j) = m(i, j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Vector3d& m)
{
    cv::Mat cvMat(3, 1, CV_32F);
    for(int i = 0; i < 3; i++)
            cvMat.at<float>(i) = m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            cvMat.at<float>(i, j) = R(i, j);
        }
    }
    for(int i = 0; i < 3; i++)
    {
        cvMat.at<float>(i, 3) = t(i);
    }

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Vector7d& m)
{
    Eigen::Quaterniond q(m.block<4, 1>(0, 0));
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t(m.block<3, 1>(4, 0));
    cv::Mat Tcw = Converter::toCvMat(R, t);

    return Tcw.clone();
}

Eigen::Vector3d Converter::toVector3d(const cv::Mat& cvVector)
{
    Eigen::Vector3d v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

Eigen::Vector3d Converter::toVector3d(const cv::Point3f& cvPoint)
{
    Eigen::Vector3d v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix3d Converter::toMatrix3d(const cv::Mat& cvMat3)
{
    Eigen::Matrix3d M;

    M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
         cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
         cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

    return M;
}

Eigen::Vector7d Converter::toVector7d(const cv::Mat& cvMat4)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    R << cvMat4.at<float>(0, 0), cvMat4.at<float>(0, 1), cvMat4.at<float>(0, 2),
            cvMat4.at<float>(1, 0), cvMat4.at<float>(1, 1), cvMat4.at<float>(1, 2),
            cvMat4.at<float>(2, 0), cvMat4.at<float>(2, 1), cvMat4.at<float>(2, 2);
    t << cvMat4.at<float>(0, 3), cvMat4.at<float>(1, 3), cvMat4.at<float>(2, 3);

    Eigen::Quaterniond q(R);
    Eigen::Vector7d pose;
    pose.block<4, 1>(0, 0) = q.coeffs();
    pose.block<3, 1>(4, 0) = t;

    return pose;
}

vector<float> Converter::toQuaternion(const cv::Mat& M)
{
    Eigen::Matrix3d eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

Eigen::Quaterniond Converter::toQuaterniond(const cv::Mat& M)
{
    Eigen::Matrix3d eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    return q;
}

} //namespace ORB_SLAM
