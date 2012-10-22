/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
 *
 * Geoorgia Tech Graphics Lab
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DYNAMICS_JOINTLIMIT_DYNAMICS_H
#define DYNAMICS_JOINTLIMIT_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

namespace lcpsolver {
    class LCPSolver;
} //namespace lcpsolver

namespace dynamics {
    class SkeletonDynamics;

    class JointLimitDynamics {
    public:
        JointLimitDynamics(SkeletonDynamics *_skel, double _dt);
        virtual ~JointLimitDynamics() { };
        void applyJointLimitTorques();
        inline Eigen::VectorXd getConstraintForce() const { return mConstrForce; }
    private:
        void updateTauStar();        
        void fillMatrices();
        bool solve();
        void applySolution();
        
        SkeletonDynamics *mSkel;
        std::vector<int> mLimitingDofIndex; // if dof i hits upper limit, we store this information as mLimitingDofIndex.push_back(i+1), if dof i hits lower limite, mLimitingDofIndex.push_back(-(i+1));
        double mDt; // timestep

        Eigen::VectorXd mConstrForce; // solved constraint force in generalized coordinates
        Eigen::VectorXd mTauStar;

        Eigen::MatrixXd mA;
        Eigen::VectorXd mQBar;
        Eigen::VectorXd mX;
    };
} // namespace dynamics

#endif // DYNAMICS_JOINTLIMIT_DYNAMICS_H
