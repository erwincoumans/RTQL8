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

#include "Analyzer.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/FileInfoDof.h"
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace dynamics;

Analyzer::Analyzer(kinematics::FileInfoDof *_motion, dynamics::SkeletonDynamics *_skel, Vector3d _grav) {
    mMotion = _motion;
    mSkel = _skel;
    mGravity = _grav;
    int nDof = mSkel->getNumDofs();
    int nFrame = mMotion->getNumFrames();
    mTorques.resize(nFrame);
    for (int i = 0; i < nFrame; i++) {
        mTorques[i].resize(nDof);
        mTorques[i].setZero();
    }
}


void Analyzer::analyze() {
    ofstream outFile("output.txt", ios::out);
    outFile.precision(10);
    
    computeTorques();
    int nFrame = mMotion->getNumFrames();
    double timestep = 1.0 / mMotion->getFPS();
    for (int i = 0; i < nFrame - 1; i++) {
        mSkel->setPose(mMotion->getPoseAtFrame(i), true, true);
        VectorXd qdot = (mMotion->getPoseAtFrame(i + 1) - mMotion->getPoseAtFrame(i)) / timestep;
        Vector3d lin = evalLinMomentum(qdot);
        Vector3d ang = evalAngMomentum(qdot);
        mLinMomentum.push_back(lin);
        mAngMomentum.push_back(ang);
    }
    // last frame
    Vector3d zero(0, 0, 0);
    mLinMomentum.push_back(zero);
    mAngMomentum.push_back(zero);

    outFile << "#Frame = " << nFrame << endl;
    outFile << "Torque" << endl;
    for (int i = 0; i < nFrame; i++) {
        for (int j = 0; j < mTorques[i].size(); j++)
            outFile << mTorques[i][j] << " ";
        outFile << endl;
    }
    outFile << "Linear Momentum" << endl;
    for (int i = 0; i < nFrame; i++)
        outFile << mLinMomentum[i][0] << " " << mLinMomentum[i][1] << " "  << mLinMomentum[i][2] << " " << endl;
    outFile << "Angular Momentum" << endl;
    for (int i = 0; i < nFrame; i++)
        outFile << mAngMomentum[i][0] << " " << mAngMomentum[i][1] << " " << mAngMomentum[i][2] << endl;
    outFile.close();
}

void Analyzer::computeTorques() {
    int nFrame = mMotion->getNumFrames();
    double timestep = 1.0 / mMotion->getFPS();
    for (int i = 0; i < nFrame - 2; i++) {
        VectorXd qdot = (mMotion->getPoseAtFrame(i + 1) - mMotion->getPoseAtFrame(i)) / timestep;

        VectorXd qddot = (mMotion->getPoseAtFrame(i + 2) - 2 * mMotion->getPoseAtFrame(i + 1)+ mMotion->getPoseAtFrame(i)) / (timestep * timestep);
        mSkel->setPose(mMotion->getPoseAtFrame(i), true, false);
        mTorques[i] = mSkel->computeInverseDynamicsLinear(mGravity, &qdot, &qddot, false, false);
    }
}

Vector3d Analyzer::evalLinMomentum(const VectorXd& _dofVel) {
    MatrixXd J(MatrixXd::Zero(3, mSkel->getNumDofs()));
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        MatrixXd localJ = node->getJacobianLinear() * node->getMass();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int index = node->getDependentDof(j);
            J.col(index) += localJ.col(j);
        }
    }
    Vector3d cDot = J * _dofVel;
    return cDot / mSkel->getMass();
}

Vector3d Analyzer::evalAngMomentum(const VectorXd& _dofVel) {
    Vector3d c = mSkel->getWorldCOM();
    Vector3d sum = Vector3d::Zero();
    Vector3d temp = Vector3d::Zero();
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        node->evalVelocity(_dofVel);
        node->evalOmega(_dofVel);
        sum += node->getInertia() * node->mOmega;
        sum += node->getMass() * (node->getWorldCOM() - c).cross(node->mVel);
    }
    return sum;
}
