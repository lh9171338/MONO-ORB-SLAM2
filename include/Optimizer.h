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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <mutex>
#include <Eigen/StdVector>
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Converter.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;

namespace ORB_SLAM2
{

class MapPoint;
class Frame;
class KeyFrame;
class Map;

class Optimizer
{
public:
    void static BundleAdjustment(const vector<KeyFrame*>& vpKF, const vector<MapPoint*>& vpMP,
                                 int nIterations=5, bool* pbStopFlag=nullptr, const bool bRobust=true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool* pbStopFlag=nullptr, const bool bRobust=true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap);
    int static PoseOptimization(Frame* pFrame);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
