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
#include "kinematics/FileInfoDof.h"

using namespace Eigen;

Controller::Controller(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, double _t) {
    mMotion = _motion;
    mSkel = _skel;
    mTimestep = _t;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);;
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

#ifdef SPD
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 15.0;
        mKd(i, i) = 2.0;
    }
#else
    for (int i = 0; i < nDof; i++) {
        mKp(i, i) = 800.0;
        mKd(i, i) = 15;
    }
#endif

    mSimFrame = 0;
    mInterval = (1.0 / mMotion->getFPS()) / mTimestep;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // feedback control force
    int motionFrame = mSimFrame / mInterval;    
    if (motionFrame >= mMotion->getNumFrames())
        motionFrame = mMotion->getNumFrames() - 1;
    
    mDesiredDofs = mMotion->getPoseAtFrame(motionFrame);
#ifdef SPD
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d);
    mTorques = p + d - mKd * qddot * mTimestep;
#else
    mTorques = -mKp * (_dof - mDesiredDofs) - mKd * _dofVel;
    mTorques = mSkel->getMassMatrix() * mTorques; // scaled by accumulated mass
#endif

    mSimFrame++; 
}
