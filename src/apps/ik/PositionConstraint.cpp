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

#include "PositionConstraint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Skeleton.h"
#include "optimizer/Var.h"
#include "utils/UtilsMath.h"

using namespace kinematics;
using namespace utils;

namespace optimizer {
    
    PositionConstraint::PositionConstraint(vector<Var *>& var, Skeleton* skel, BodyNode* node, const Vector3d& offset, const Vector3d& val) : Constraint(var), mSkel(skel), mNode(node), mTarget(val), mOffset(offset) {
        mNumRows = 3;

        mWeight = VectorXd::Ones(mNumRows);
        mConstTerm = VectorXd::Zero(mNumRows);
        mCompletion = VectorXd::Zero(mNumRows);
    }

    VectorXd PositionConstraint::evalCon() {
        Vector3d wp = mNode->evalWorldPos(mOffset);
        Vector3d c = wp - mTarget;
        VectorXd ret(c);
        return ret;
    }

    void PositionConstraint::fillJac(VVD jEntry, VVB jMap, int index) {
        for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
            int dofindex = mNode->getDependentDof(i);
            const Var* v = mVariables[dofindex];
            VectorXd J = xformHom(mNode->getDerivWorldTransform(i), mOffset);
            for (int k = 0; k < 3; k++) {
                (*jEntry)[index + k]->at(dofindex) = J[k];
                (*jMap)[index + k]->at(dofindex) = true;
            }
        }
    }

    void PositionConstraint::fillObjGrad(std::vector<double>& dG) {
        VectorXd dP = evalCon();
        for(int i = 0; i < mNode->getNumDependentDofs(); i++) {
            int dofindex = mNode->getDependentDof(i);       
            VectorXd J = xformHom(mNode->getDerivWorldTransform(i), mOffset);
            dG[dofindex] += 2 * dP.dot(J);
        }
    }

    void PositionConstraint::setTarget(const Vector3d& target) {
        mTarget = target;
    }

    Vector3d PositionConstraint::getTarget() const {
        return mTarget;
    }
} // namespace optimizer
