#ifndef CERES_OPTIMIZER_H_
#define CERES_OPTIMIZER_H_

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <algorithm>
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

using namespace std;

namespace ORB_SLAM2 {

class KeyFrame;
class MapPoint;
class Map;

class ReprojectCost {
public:
    ReprojectCost(const Eigen::Matrix3d& K, const Eigen::Vector2d& observation,
                  const Eigen::Matrix2d& information)
      : mK(K), mObservation(observation), mInformation(information) {}

    template <typename T>
    bool operator()(
            const T* const q_ptr,
            const T* const t_ptr,
            const T* const p3d_ptr,
            T* residuals_ptr) const {
        Eigen::Map<const Eigen::Quaternion<T>> q(q_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(t_ptr);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p3d(p3d_ptr);

        // Project the 3D point to the camera frame.
        Eigen::Matrix<T, 3, 1> p2d = mK.template cast<T>() * (q * p3d + t);

        // Compute the residuals
        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
        residuals[0] = mObservation.template cast<T>()[0] - p2d[0] / p2d[2];
        residuals[1] = mObservation.template cast<T>()[1] - p2d[1] / p2d[2];

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft(mInformation.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Matrix3d& K,
                                     const Eigen::Vector2d& observation,
                                     const Eigen::Matrix2d& information) {
        return new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 3, 3>(
            new ReprojectCost(K, observation, information));
    }

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const Eigen::Matrix3d mK;
    const Eigen::Vector2d mObservation;
    const Eigen::Matrix2d mInformation;
};

class StopFlagCallback : public ceres::IterationCallback {
public:
    StopFlagCallback(bool* pbStopFlag) : mpbStopFlag(pbStopFlag) {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
        if (mpbStopFlag) {
            if (*mpbStopFlag) {
                return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
            }
            else {
                return ceres::SOLVER_CONTINUE;
            }
        }
        return ceres::SOLVER_CONTINUE;
    }

private:
    const bool* mpbStopFlag;
};

class Optimizer {
public:

    void static BundleAdjustment(const vector<KeyFrame*>& vpKF, const vector<MapPoint*>& vpMP,
                             int nIterations=5, bool* pbStopFlag=nullptr, const bool bRobust=true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool* pbStopFlag=nullptr, const bool bRobust=true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap);
    int static PoseOptimization(Frame* pFrame);

    bool static CheckOutlier(const Eigen::Matrix3d& K, const Eigen::Vector2d& observation, float inv_sigma,
                             const Eigen::Quaterniond& q, const Eigen::Vector3d& t, const Eigen::Vector3d& p3d,
                             double thresh);
};

}  // namespace ORB_SLAM2

#endif
