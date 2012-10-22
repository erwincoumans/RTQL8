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
#include "dynamics/ContactDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "kinematics/Shape.h"
#include "utils/UtilsMath.h"
#include "collision/CollisionSkeleton.h"

using namespace kinematics;
using namespace dynamics;
using namespace utils;

Controller::Controller(dynamics::SkeletonDynamics *_skel, dynamics::ContactDynamics *_collisionHandle, double _t) {
    mSkel = _skel;
    mCollisionHandle = _collisionHandle;
    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumDofs();
    mKp = MatrixXd::Identity(nDof, nDof);
    mKd = MatrixXd::Identity(nDof, nDof);
    mConstrForces = VectorXd::Zero(nDof);
        
    mTorques.resize(nDof);
    mDesiredDofs.resize(nDof);
    for (int i = 0; i < nDof; i++){
        mTorques[i] = 0.0;
        mDesiredDofs[i] = mSkel->getDof(i)->getValue();
    }

    // using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++) {
        mKp(i, i) = 0.0;
        mKd(i, i) = 0.0;
    }
    for (int i = 6; i < 22; i++)
        mKp(i, i) = 200.0; // lower body + lower back
    for (int i = 22; i < nDof; i++)
        mKp(i, i) = 20.0;
    for (int i = 6; i < 22; i++) 
        mKd(i, i) = 100.0;
    for (int i = 22; i < nDof; i++) 
        mKd(i, i) = 10.0;
        
    mPreOffset = 0.0;
}

void Controller::computeTorques(const VectorXd& _dof, const VectorXd& _dofVel) {
    // SPD tracking
    int nDof = mSkel->getNumDofs();
    MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    VectorXd d = -mKd * _dofVel;
    VectorXd qddot = invM * (-mSkel->getCombinedVector() + p + d + mConstrForces);
    mTorques = p + d - mKd * qddot * mTimestep;

    // angular momentum control for upper body
    Vector3d ang = evalAngMomentum(_dofVel);
    VectorXd controlledAxis(2);
    VectorXd deltaMomentum(2);
    controlledAxis[0] = 3;
    controlledAxis[1] = 5;
    deltaMomentum[0] = 1000 * -ang[0];    
    deltaMomentum[1] = 1000 * -ang[2];    
    mTorques += adjustAngMomentum(deltaMomentum, controlledAxis);

    // ankle strategy for sagital plane
    Vector3d com = mSkel->getWorldCOM();
    double cop = 0.02;
    double offset = com[0] - cop;
    double k1 = 20.0;
    double k2 = 10.0;
    double kd = 100.0;
    mTorques[10] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[12] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;

    // Just to make sure no illegal torque is used    
    for (int i = 0; i < 6; i++){        
        mTorques[i] = 0.0;
    }
    mFrame++;
}

Vector3d Controller::evalLinMomentum(const VectorXd& _dofVel) {
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

Vector3d Controller::evalAngMomentum(const VectorXd& _dofVel) {
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

VectorXd Controller::adjustAngMomentum(VectorXd _deltaMomentum, VectorXd _controlledAxis) {
    int nDof = mSkel->getNumDofs();
    double mass = mSkel->getMass();
    Matrix3d c = utils::makeSkewSymmetric(mSkel->getWorldCOM());
    MatrixXd A(MatrixXd::Zero(6, nDof));
    MatrixXd Jv(MatrixXd::Zero(3, nDof));
    MatrixXd Jw(MatrixXd::Zero(3, nDof));
    for (int i = 0; i < mSkel->getNumNodes(); i++) {
        BodyNodeDynamics *node = (BodyNodeDynamics*)mSkel->getNode(i);
        Matrix3d c_i = utils::makeSkewSymmetric(node->getWorldCOM());
        double m_i = node->getMass();
        MatrixXd localJv = node->getJacobianLinear();
        MatrixXd localJw = node->getJacobianAngular();
        Jv.setZero();
        Jw.setZero();
        for (int j = 0; j < node->getNumDependentDofs(); j++) {
            int dofIndex = node->getDependentDof(j);
            Jv.col(dofIndex) += localJv.col(j);
            Jw.col(dofIndex) += localJw.col(j);
        }
        A.block(0, 0, 3, nDof) += m_i * Jv / mass;
        A.block(3, 0, 3, nDof) += m_i * (c_i - c) * Jv + node->getInertia() * Jw;
    }
    for ( int i = 0; i < 6; i++)
        A.col(i).setZero(); // try to realize momentum without using root acceleration
    for (int i = 6; i < 20; i++)
        A.col(i) *= 0;
    
    MatrixXd aggregateMat(_controlledAxis.size(), nDof);     
    for (int i = 0; i < _controlledAxis.size(); i++) {
        aggregateMat.row(i) = A.row(_controlledAxis[i]);
    }

    VectorXd deltaQdot = aggregateMat.transpose() * (aggregateMat * aggregateMat.transpose()).inverse() * _deltaMomentum;
    return deltaQdot;
}
