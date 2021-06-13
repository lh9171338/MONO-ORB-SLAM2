#include "Optimizer.h"


namespace ORB_SLAM2 {

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const bool bRobust)
{
    vector<KeyFrame*> vpKF = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKF, vpMP, nIterations, pbStopFlag, bRobust);
}

void Optimizer::BundleAdjustment(const vector<KeyFrame*>& vpKF, const vector<MapPoint*>& vpMP,
                                      int nIterations, bool* pbStopFlag, const bool bRobust)
{
    // Setup optimizer
    ceres::Problem problem;
    ceres::LossFunction* loss_function = nullptr;
    if(bRobust)
        loss_function = new ceres::HuberLoss(sqrt(5.991));
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;
    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;

    // Set KeyFrame vertices
    map<KeyFrame*, Eigen::Vector7d> mKeyFramePose;
    for(auto& pKF : vpKF)
    {
        if(pKF->isBad())
            continue;
        mKeyFramePose[pKF] = Converter::toVector7d(pKF->GetPose());
    }

    // Set MapPoint vertices
    map<MapPoint*, Eigen::Vector3d> mMapPointPos;
    for(auto& pMP : vpMP)
    {
        if(pMP->isBad())
            continue;
        mMapPointPos[pMP] = Converter::toVector3d(pMP->GetWorldPos());

        // SET EDGES
        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for(auto mit = observations.begin(); mit != observations.end(); mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad())
                continue;

            Eigen::Matrix3d K = Converter::toMatrix3d(pKF->mK);
            const cv::KeyPoint& kpUn = pKF->mvKeysUn[mit->second];
            Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
            const float invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

            ceres::CostFunction* cost_function = ReprojectCost::Create(K, obs, information);
            problem.AddResidualBlock(
                cost_function,
                loss_function,
                mKeyFramePose[pKF].block<4, 1>(0, 0).data(),
                mKeyFramePose[pKF].block<3, 1>(4, 0).data(),
                mMapPointPos[pMP].data()
            );
            problem.SetParameterization(mKeyFramePose[pKF].block<4, 1>(0, 0).data(), quaternion_parameterization);
            ordering->AddElementToGroup(mKeyFramePose[pKF].block<4, 1>(0, 0).data(), 1);
            ordering->AddElementToGroup(mKeyFramePose[pKF].block<3, 1>(4, 0).data(), 1);
            ordering->AddElementToGroup(mMapPointPos[pMP].data(), 0);
            if(pKF->mnId == 0)
            {
                problem.SetParameterBlockConstant(mKeyFramePose[pKF].block<4, 1>(0, 0).data());
                problem.SetParameterBlockConstant(mKeyFramePose[pKF].block<3, 1>(4, 0).data());
            }
        }
    }

    // Optimize
    ceres::Solver::Options options;
    if(pbStopFlag)
        options.callbacks.push_back(new StopFlagCallback(pbStopFlag));

    options.max_num_iterations = nIterations;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_ordering.reset(ordering);
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Recover optimized data
    // Keyframes
    for(auto it = mKeyFramePose.begin(); it != mKeyFramePose.end(); it++)
    {
        KeyFrame* pKF = it->first;
        Eigen::Vector7d pose = it->second;
        pKF->SetPose(Converter::toCvMat(pose));
    }

    // MapPoints
    for(auto it = mMapPointPos.begin(); it != mMapPointPos.end(); it++)
    {
        MapPoint* pMP = it->first;
        Eigen::Vector3d pos = it->second;
        pMP->SetWorldPos(Converter::toCvMat(pos));
        pMP->UpdateNormalAndDepth();
    }
}

int Optimizer::PoseOptimization(Frame* pFrame)
{
    // Setup optimizer
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(sqrt(5.991));
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;

    // Set Frame vertex
    Eigen::Vector7d pose = Converter::toVector7d(pFrame->mTcw);

    // Set MapPoint vertices
    int nInitialCorrespondences = 0;
    const int N = pFrame->N;
    vector<Eigen::Vector3d> vMapPointPos(N);
    for(int i = 0; i < N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            // Monocular observation
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            vMapPointPos[i] = Converter::toVector3d(pMP->GetWorldPos());
            Eigen::Matrix3d K = Converter::toMatrix3d(pFrame->mK);
            const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
            Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

            ceres::CostFunction* cost_function = ReprojectCost::Create(K, obs, information);
            problem.AddResidualBlock(
                    cost_function,
                    loss_function,
                    pose.block<4, 1>(0, 0).data(),
                    pose.block<3, 1>(4, 0).data(),
                    vMapPointPos[i].data()
            );
            problem.SetParameterization(pose.block<4, 1>(0, 0).data(), quaternion_parameterization);
            problem.SetParameterBlockConstant(vMapPointPos[i].data());
        }
    }
    if(nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2s[4] = {5.991, 5.991, 5.991, 5.991};
    const int its[4] = {10, 10, 10, 10};

    int nBad = 0;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;

    for(size_t it = 0; it < 4; it++)
    {
        double thresh = chi2s[it];
        int nIterations = its[it];

        // Optimize
        options.max_num_iterations = nIterations;
        ceres::Solve(options, &problem, &summary);

        nBad = 0;
        for(int i = 0; i < N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                Eigen::Matrix3d K = Converter::toMatrix3d(pFrame->mK);
                const cv::KeyPoint& kpUn = pFrame->mvKeysUn[i];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

                Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
                Eigen::Vector3d t(pose.block<3, 1>(4, 0));
                bool bOutlier = CheckOutlier(K, obs, invSigma2, q, t, vMapPointPos[i], thresh);
                if(bOutlier)
                {
                    pFrame->mvbOutlier[i] = true;
                    nBad++;
                    if(problem.HasParameterBlock(vMapPointPos[i].data()))
                    {
                        vector<ceres::ResidualBlockId> residualBlockIds;
                        problem.GetResidualBlocksForParameterBlock(vMapPointPos[i].data(), &residualBlockIds);
                        for(auto residualBlockId : residualBlockIds)
                            problem.RemoveResidualBlock(residualBlockId);
                    }
                }
                else
                {
                    pFrame->mvbOutlier[i] = false;
                    if(!problem.HasParameterBlock(vMapPointPos[i].data()))
                    {
                        ceres::CostFunction* cost_function = ReprojectCost::Create(K, obs, information);
                        problem.AddResidualBlock(
                                cost_function,
                                it >= 2 ? nullptr : loss_function,
                                pose.block<4, 1>(0, 0).data(),
                                pose.block<3, 1>(4, 0).data(),
                                vMapPointPos[i].data()
                        );
                        problem.SetParameterization(pose.block<4, 1>(0, 0).data(), quaternion_parameterization);
                        problem.SetParameterBlockConstant(vMapPointPos[i].data());
                    }
                }
            }
        }
        if(problem.NumResidualBlocks() < 10)
            break;
    }

    // Recover optimized pose and return number of inliers
    pFrame->SetPose(Converter::toCvMat(pose));

    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    map<KeyFrame*, Eigen::Vector7d> mLocalKFPose;
    mLocalKFPose[pKF] = Converter::toVector7d(pKF->GetPose());
    pKF->mnBALocalForKF = pKF->mnId;
    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(auto& pKFi : vNeighKFs)
    {
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            mLocalKFPose[pKFi] = Converter::toVector7d(pKFi->GetPose());
    }

    // Local MapPoints seen in Local KeyFrames
    map<MapPoint*, Eigen::Vector3d> mLocalMPPos;
    for(auto mit = mLocalKFPose.begin(); mit != mLocalKFPose.end(); mit++)
    {
        KeyFrame* pKFi = mit->first;
        vector<MapPoint*> vpMP = pKFi->GetMapPointMatches();
        for(auto& pMP : vpMP)
        {
            if(pMP && !pMP->isBad())
            {
                if(pMP->mnBALocalForKF != pKF->mnId)
                {
                    mLocalMPPos[pMP] = Converter::toVector3d(pMP->GetWorldPos());
                    pMP->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    map<KeyFrame*, Eigen::Vector7d> mFixedKFPose;
    for(auto mit = mLocalMPPos.begin(); mit != mLocalMPPos.end(); mit++)
    {
        MapPoint* pMP = mit->first;
        map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for(auto it = observations.begin(); it != observations.end(); it++)
        {
            KeyFrame* pKFi = it->first;
            if(pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
            {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if(!pKFi->isBad())
                    mFixedKFPose[pKFi] = Converter::toVector7d(pKFi->GetPose());
            }
        }
    }

    // Setup optimizer
    ceres::Problem problem;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(sqrt(5.991));
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;
    ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;

    // Set edges
    for(auto mit = mLocalMPPos.begin(); mit != mLocalMPPos.end(); mit++)
    {
        MapPoint* pMP = mit->first;
        map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for(auto it = observations.begin(); it != observations.end(); it++)
        {
            KeyFrame* pKFi = it->first;
            if(!pKFi->isBad())
            {
                Eigen::Matrix3d K = Converter::toMatrix3d(pKFi->mK);
                const cv::KeyPoint& kpUn = pKFi->mvKeysUn[it->second];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

                ceres::CostFunction* cost_function = ReprojectCost::Create(K, obs, information);
                if(mLocalKFPose.count(pKFi))
                {
                    problem.AddResidualBlock(
                            cost_function,
                            loss_function,
                            mLocalKFPose[pKFi].block<4, 1>(0, 0).data(),
                            mLocalKFPose[pKFi].block<3, 1>(4, 0).data(),
                            mLocalMPPos[pMP].data()
                    );
                    problem.SetParameterization(mLocalKFPose[pKFi].block<4, 1>(0, 0).data(), quaternion_parameterization);
                    ordering->AddElementToGroup(mLocalKFPose[pKFi].block<4, 1>(0, 0).data(), 1);
                    ordering->AddElementToGroup(mLocalKFPose[pKFi].block<3, 1>(4, 0).data(), 1);
                    ordering->AddElementToGroup(mLocalMPPos[pMP].data(), 0);
                    if(pKFi->mnId == 0)
                    {
                        problem.SetParameterBlockConstant(mLocalKFPose[pKFi].block<4, 1>(0, 0).data());
                        problem.SetParameterBlockConstant(mLocalKFPose[pKFi].block<3, 1>(4, 0).data());
                    }
                }
                else  // if(mFixedKFPose.count(pKFi))
                {
                    problem.AddResidualBlock(
                            cost_function,
                            loss_function,
                            mFixedKFPose[pKFi].block<4, 1>(0, 0).data(),
                            mFixedKFPose[pKFi].block<3, 1>(4, 0).data(),
                            mLocalMPPos[pMP].data()
                    );
                    problem.SetParameterization(mFixedKFPose[pKFi].block<4, 1>(0, 0).data(), quaternion_parameterization);
                    problem.SetParameterBlockConstant(mFixedKFPose[pKFi].block<4, 1>(0, 0).data());
                    problem.SetParameterBlockConstant(mFixedKFPose[pKFi].block<3, 1>(4, 0).data());
                    ordering->AddElementToGroup(mFixedKFPose[pKFi].block<4, 1>(0, 0).data(), 1);
                    ordering->AddElementToGroup(mFixedKFPose[pKFi].block<3, 1>(4, 0).data(), 1);
                    ordering->AddElementToGroup(mLocalMPPos[pMP].data(), 0);
                }
            }
        }
    }
    // Optimize
    if(pbStopFlag && *pbStopFlag)
        return;

    ceres::Solver::Options options;
    if(pbStopFlag)
        options.callbacks.push_back(new StopFlagCallback(pbStopFlag));
    options.max_num_iterations = 5;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_ordering.reset(ordering);
    options.num_threads = 4;
    options.use_explicit_schur_complement = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Optimize again
    bool bDoMore= true;
    if(pbStopFlag && *pbStopFlag)
        bDoMore = false;

    int nEdges = problem.NumResidualBlocks();
    if(bDoMore)
    {
        // Remove all ResidualBlocks
        vector<ceres::ResidualBlockId> residualBlockIds;
        problem.GetResidualBlocks(&residualBlockIds);
        for(auto residualBlockId : residualBlockIds)
            problem.RemoveResidualBlock(residualBlockId);
        ordering = new ceres::ParameterBlockOrdering;

        // Check inlier observations
        for(auto mit = mLocalMPPos.begin(); mit != mLocalMPPos.end(); mit++)
        {
            MapPoint* pMP = mit->first;
            map<KeyFrame*, size_t> observations = pMP->GetObservations();
            for(auto it = observations.begin(); it != observations.end(); it++)
            {
                KeyFrame* pKFi = it->first;
                if(!pKFi->isBad())
                {
                    Eigen::Matrix3d K = Converter::toMatrix3d(pKFi->mK);
                    const cv::KeyPoint& kpUn = pKFi->mvKeysUn[it->second];
                    Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix2d information = Eigen::Matrix2d::Identity() * invSigma2;

                    Eigen::Vector3d pos = mLocalMPPos[pMP];
                    Eigen::Vector7d pose = Converter::toVector7d(pKFi->GetPose());
                    Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
                    Eigen::Vector3d t(pose.block<3, 1>(4, 0));
                    bool bOutlier = CheckOutlier(K, obs, invSigma2, q, t, pos, 5.991);
                    Eigen::Vector3d p3d = q * pos + t;

                    if (bOutlier || p3d.z() <= 0)
                        continue;

                    ceres::CostFunction* cost_function = ReprojectCost::Create(K, obs, information);
                    if(mLocalKFPose.count(pKFi))
                    {
                        problem.AddResidualBlock(
                                cost_function,
                                nullptr,
                                mLocalKFPose[pKFi].block<4, 1>(0, 0).data(),
                                mLocalKFPose[pKFi].block<3, 1>(4, 0).data(),
                                mLocalMPPos[pMP].data()
                        );
                        problem.SetParameterization(mLocalKFPose[pKFi].block<4, 1>(0, 0).data(), quaternion_parameterization);
                        ordering->AddElementToGroup(mLocalKFPose[pKFi].block<4, 1>(0, 0).data(), 1);
                        ordering->AddElementToGroup(mLocalKFPose[pKFi].block<3, 1>(4, 0).data(), 1);
                        ordering->AddElementToGroup(mLocalMPPos[pMP].data(), 0);
                        if(pKFi->mnId == 0)
                        {
                            problem.SetParameterBlockConstant(mLocalKFPose[pKFi].block<4, 1>(0, 0).data());
                            problem.SetParameterBlockConstant(mLocalKFPose[pKFi].block<3, 1>(4, 0).data());
                        }
                    }
                    else  // if(mFixedKFPose.count(pKFi))
                    {
                        problem.AddResidualBlock(
                                cost_function,
                                nullptr,
                                mFixedKFPose[pKFi].block<4, 1>(0, 0).data(),
                                mFixedKFPose[pKFi].block<3, 1>(4, 0).data(),
                                mLocalMPPos[pMP].data()
                        );
                        problem.SetParameterization(mFixedKFPose[pKFi].block<4, 1>(0, 0).data(), quaternion_parameterization);
                        problem.SetParameterBlockConstant(mFixedKFPose[pKFi].block<4, 1>(0, 0).data());
                        problem.SetParameterBlockConstant(mFixedKFPose[pKFi].block<3, 1>(4, 0).data());
                        ordering->AddElementToGroup(mFixedKFPose[pKFi].block<4, 1>(0, 0).data(), 1);
                        ordering->AddElementToGroup(mFixedKFPose[pKFi].block<3, 1>(4, 0).data(), 1);
                        ordering->AddElementToGroup(mLocalMPPos[pMP].data(), 0);
                    }
                }
            }
        }
        // Optimize again without the outliers
        options.max_num_iterations = 10;
        options.linear_solver_ordering.reset(ordering);
        ceres::Solve(options, &problem, &summary);
    }

    // Check inlier observations
    vector<pair<KeyFrame*, MapPoint*>> vToErase;
    vToErase.reserve(nEdges);
    for(auto mit = mLocalMPPos.begin(); mit != mLocalMPPos.end(); mit++)
    {
        MapPoint* pMP = mit->first;
        map<KeyFrame*, size_t> observations = pMP->GetObservations();
        for(auto it = observations.begin(); it != observations.end(); it++)
        {
            KeyFrame* pKFi = it->first;
            if(!pKFi->isBad())
            {
                Eigen::Matrix3d K = Converter::toMatrix3d(pKFi->mK);
                const cv::KeyPoint& kpUn = pKFi->mvKeysUn[it->second];
                Eigen::Vector2d obs(kpUn.pt.x, kpUn.pt.y);
                const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                Eigen::Vector3d pos = Converter::toVector3d(pMP->GetWorldPos());
                Eigen::Vector7d pose = Converter::toVector7d(pKFi->GetPose());
                Eigen::Quaterniond q(pose.block<4, 1>(0, 0));
                Eigen::Vector3d t(pose.block<3, 1>(4, 0));
                bool bOutlier = CheckOutlier(K, obs, invSigma2, q, t, pos, 5.991);
                Eigen::Vector3d p3d = q * pos + t;

                if (bOutlier || p3d.z() <= 0)
                    vToErase.push_back(make_pair(pKFi, pMP));
            }
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    for(size_t i = 0; i < vToErase.size(); i++)
    {
        KeyFrame* pKFi = vToErase[i].first;
        MapPoint* pMPi = vToErase[i].second;
        pKFi->EraseMapPointMatch(pMPi);
        pMPi->EraseObservation(pKFi);
    }

    // Recover optimized data
    // Keyframes
    for(auto mit = mLocalKFPose.begin(); mit != mLocalKFPose.end(); mit++)
    {
        KeyFrame* pKFi = mit->first;
        Eigen::Vector7d pose = mit->second;
        pKFi->SetPose(Converter::toCvMat(pose));
    }

    // Points
    for(auto mit = mLocalMPPos.begin(); mit != mLocalMPPos.end(); mit++)
    {
        MapPoint* pMP = mit->first;
        Eigen::Vector3d pos = mit->second;
        pMP->SetWorldPos(Converter::toCvMat(pos));
        pMP->UpdateNormalAndDepth();
    }
}

bool Optimizer::CheckOutlier(const Eigen::Matrix3d& K, const Eigen::Vector2d& observation, float inv_sigma,
                                  const Eigen::Quaterniond& q, const Eigen::Vector3d& t, const Eigen::Vector3d& p3d,
                                  double thresh)
{
    Eigen::Vector3d p2d = K * (q * p3d + t);
    double error_u = observation[0] - p2d[0] / p2d[2];
    double error_v = observation[1] - p2d[1] / p2d[2];
    double error = (error_u * error_u + error_v * error_v) * inv_sigma;
    return error > thresh;
}

}  // namespace ORB_SLAM2
