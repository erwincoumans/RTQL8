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

#include "Controller.h"

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/Dof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace kinematics;
using namespace dynamics;
using namespace utils;


Controller::Controller(dynamics::SkeletonDynamics *_skel) {

    mSkel = _skel;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    mTorques.setZero();
    for (int i = 0; i < nDof; i++)
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    
    for (int i = 3; i < nDof; i++) {
        mKp(i, i) = 500.0;
        mKd(i, i) = 50.0;
    }
    mFrame = 0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // track a pose using PD
    mTorques.setZero();
    mTorques = -mKp * (_dof - mDesiredDofs) - mKd * _dofVel;
    for (int i = 0; i < 3; i++)
        mTorques[i] = 0.0;
    mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass

    
    mFrame++;
}
