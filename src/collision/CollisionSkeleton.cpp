/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>, Karen Liu <karenliu@cc.gatech.edu>
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

#include "CollisionSkeleton.h"
#include "CollisionShapes.h"
#include "kinematics/Shape.h"
#include <cmath>
#include "utils/LoadOpengl.h"
#include "utils/UtilsMath.h"

using namespace std;
using namespace Eigen;
using namespace utils;

namespace collision_checking{
    
    CollisionSkeletonNode::CollisionSkeletonNode(kinematics::BodyNode* _bodyNode)
    {
        mBodyNode = _bodyNode;
        if (mBodyNode->getShape()->getShapeType() == kinematics::Shape::P_ELLIPSOID) {
            mMesh = createEllipsoid<RSS>(mBodyNode->getShape()->getDim()[0], mBodyNode->getShape()->getDim()[1], mBodyNode->getShape()->getDim()[2]);
        } else if (mBodyNode->getShape()->getShapeType() == kinematics::Shape::P_CUBE) {
            mMesh = createCube<RSS>(mBodyNode->getShape()->getDim()[0], mBodyNode->getShape()->getDim()[1], mBodyNode->getShape()->getDim()[2]);
        } else {
            mMesh = createMesh<RSS>(mBodyNode->getShape()->getDim()[0], mBodyNode->getShape()->getDim()[1], mBodyNode->getShape()->getDim()[2], _bodyNode->getShape()->getVizMesh());
        }
    }
    CollisionSkeletonNode::~CollisionSkeletonNode()
    {
        delete mMesh;
    }

    int CollisionSkeletonNode::checkCollision(CollisionSkeletonNode* _otherNode, vector<ContactPoint>& _result, int _num_max_contact)
    {    
        BVH_CollideResult res;
        evalRT();
        _otherNode->evalRT();

        res.num_max_contacts = _num_max_contact;
        //        static Timer tSim("Simulation");
        //tSim.startTimer();
        collide(*mMesh , mR, mT, *_otherNode->mMesh, _otherNode->mR, _otherNode->mT, &res);
        //        tSim.stopTimer();
        //        tSim.printScreen();

        int start, size;
        start = _result.size();
        size = 0;

        int numCoplanarContacts = 0;
        int numNoContacts = 0;
        int numContacts = 0;

        for(int i = 0; i < res.numPairs();i++) {
            // for each pair of intersecting triangles, we create two contact points
            ContactPoint pair1, pair2;
            pair1.bd1 = mBodyNode;
            pair1.bd2 = _otherNode->mBodyNode;
            pair1.bdID1 = this->mBodynodeID;
            pair1.bdID2 = _otherNode->mBodynodeID;
            pair1.collisionSkeletonNode1 = this;
            pair1.collisionSkeletonNode2 = _otherNode;
            Vec3f v;
            pair1.triID1 = res.id1(i);
            pair1.triID2 = res.id2(i);
            pair1.penetrationDepth = res.collidePairs()[i].penetration_depth;
            pair2 = pair1;
            int contactResult = evalContactPosition(res, _otherNode, i, pair1.point, pair2.point, pair1.contactTri);
            pair2.contactTri = pair1.contactTri;
            if(contactResult == COPLANAR_CONTACT) {
                numCoplanarContacts++;      
                //                if(numContacts != 0 || numCoplanarContacts > 1)
                if (numContacts > 2)
                  continue; 
            } else if(contactResult == NO_CONTACT) {
                numNoContacts++;
                continue;
            } else {
                numContacts++;
            }
            v = res.collidePairs()[i].normal;
            pair1.normal = Vector3d(v[0], v[1], v[2]);
            pair2.normal = Vector3d(v[0], v[1], v[2]);
        
            _result.push_back(pair1);
            _result.push_back(pair2);

            size+=2;
        }

        if(res.numPairs()==0)
            return 0;

        const double ZERO = 0.000001;
        const double ZERO2 = ZERO*ZERO;

        vector<int> deleteIDs;
        size = _result.size() - start;
        deleteIDs.clear();

        // mark all the repeated points
        for (int i = start; i < start + size; i++)
            for (int j = i + 1; j < start + size; j++) {
                Vector3d diff = _result[i].point - _result[j].point;
                if (diff.dot(diff) < 3 * ZERO2) {
                    deleteIDs.push_back(i);
                    break;
                }
            }
        // delete repeated points
        for (int i = deleteIDs.size() - 1; i >= 0; i--)
            _result.erase(_result.begin() + deleteIDs[i]);
        
        size = _result.size() - start;
        deleteIDs.clear();

        // remove all the co-linear contact points
        bool bremove;
        for (int i = start; i < start + size; i++) {
            bremove = false;
            for (int j = start; j < start + size; j++) {
                if (j == i)
                    continue;
                if (bremove)
                    break;
                for (int k = j + 1; k < start + size; k++) {
                    if (i == k)
                        continue;
                    Vector3d  v = (_result[i].point - _result[j].point).cross(_result[i].point - _result[k].point);
                    if (v.dot(v) < ZERO2 && ((_result[i].point - _result[j].point).dot(_result[i].point - _result[k].point) < 0)) {
                        bremove = true;
                        break;
                    }
                }
            }
            if (bremove)
                deleteIDs.push_back(i);
        }
    
        for (int i = deleteIDs.size() - 1; i >= 0; i--)
            _result.erase(_result.begin() + deleteIDs[i]);

        int collisionNum = res.numPairs();
        //return numCoplanarContacts*100+numNoContacts;
        return collisionNum;
    }

    void CollisionSkeletonNode::evalRT() {
        mWorldTrans = mBodyNode->getWorldTransform();
        Vector3d p = xformHom(mBodyNode->getWorldTransform(), mBodyNode->getLocalCOM());
        for (int i = 0; i < 3; i++) {
            mT[i] = p[i];
            mWorldTrans(i, 3) = p[i];
        }
        
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                mR[i][j] = mBodyNode->getWorldTransform()(i, j);
    }

    int CollisionSkeletonNode::evalContactPosition(BVH_CollideResult& _result,  CollisionSkeletonNode* _other, int _idx, Vector3d& _contactPosition1, Vector3d& _contactPosition2, ContactTriangle& _contactTri ) {
        int id1, id2;
        Triangle tri1, tri2;
        CollisionSkeletonNode* node1 = this;
        CollisionSkeletonNode* node2 = _other;
        id1 = _result.id1(_idx);
        id2 = _result.id2(_idx);
        tri1 = node1->mMesh->tri_indices[id1];
        tri2 = node2->mMesh->tri_indices[id2];

        Vec3f v1, v2, v3, p1, p2, p3;
        v1 = node1->mMesh->vertices[tri1[0]];
        v2 = node1->mMesh->vertices[tri1[1]];
        v3 = node1->mMesh->vertices[tri1[2]];

        p1 = node2->mMesh->vertices[tri2[0]];
        p2 = node2->mMesh->vertices[tri2[1]];
        p3 = node2->mMesh->vertices[tri2[2]];

        Vec3f contact1, contact2;
        v1 = node1->TransformVertex(v1);
        v2 = node1->TransformVertex(v2);
        v3 = node1->TransformVertex(v3);
        p1 = node2->TransformVertex(p1);
        p2 = node2->TransformVertex(p2);
        p3 = node2->TransformVertex(p3);
        int testRes = FFtest(v1, v2, v3, p1, p2, p3, contact1, contact2);

        _contactTri.v1 = v1;
        _contactTri.v2 = v2;
        _contactTri.v3 = v3;
        _contactTri.u1 = p1;
        _contactTri.u2 = p2;
        _contactTri.u3 = p3;
        if (testRes == COPLANAR_CONTACT) {
            double area1 = triArea(v1, v2, v3);
            double area2 = triArea(p1, p2, p3);
            //            cout << "this node = " << this->mBodynodeID << " other node = " << _other->mBodynodeID << endl;
            if (area1 < area2)                 
                contact1 = v1 + v2 + v3;
            else 
                contact1 = p1 + p2 + p3;
            contact1[0] /= 3.0;
            contact1[1] /= 3.0;
            contact1[2] /= 3.0;
            contact2 = contact1;
            //            cout << contact1[0] << " " << contact1[1] << " " << contact1[2] << endl;
        }
        _contactPosition1 = Vector3d(contact1[0], contact1[1], contact1[2]);
        _contactPosition2 = Vector3d(contact2[0], contact2[1], contact2[2]);
        return testRes;
    }

    void CollisionSkeletonNode::drawCollisionTriangle(int _tri) {
        Triangle Tri = mMesh->tri_indices[_tri];
        glVertex3f(mMesh->vertices[Tri[0]][0], mMesh->vertices[Tri[0]][1], mMesh->vertices[Tri[0]][2]);
        glVertex3f(mMesh->vertices[Tri[1]][0], mMesh->vertices[Tri[1]][1], mMesh->vertices[Tri[1]][2]);
        glVertex3f(mMesh->vertices[Tri[2]][0], mMesh->vertices[Tri[2]][1], mMesh->vertices[Tri[2]][2]);
    }

    void CollisionSkeletonNode::drawCollisionSkeletonNode(bool _bTrans) {
        evalRT();        
        double M[16];
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                M[j*4+i] = mWorldTrans(i, j);
        Vec3f v1, v2, v3;
        glPushMatrix();
        if(_bTrans)
            glMultMatrixd(M);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_TRIANGLES);
        for(int i = 0;i < mMesh->num_tris; i++) {
            Triangle tri = mMesh->tri_indices[i];
            glVertex3f(mMesh->vertices[tri[0]][0], mMesh->vertices[tri[0]][1], mMesh->vertices[tri[0]][2]);
            glVertex3f(mMesh->vertices[tri[1]][0], mMesh->vertices[tri[1]][1], mMesh->vertices[tri[1]][2]);
            glVertex3f(mMesh->vertices[tri[2]][0], mMesh->vertices[tri[2]][1], mMesh->vertices[tri[2]][2]);

        }
        glEnd();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glPopMatrix();
    }

    SkeletonCollision::~SkeletonCollision() {
        for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
            delete mCollisionSkeletonNodeList[i];
    }

    void SkeletonCollision::addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive) {
        if (_bRecursive == false || _bd->getNumChildJoints() == 0) {
            CollisionSkeletonNode* csnode = new CollisionSkeletonNode(_bd);
            csnode->mBodynodeID = mCollisionSkeletonNodeList.size();
            mCollisionSkeletonNodeList.push_back(csnode);
            mBodyNodeHash[_bd] = csnode;
        } else {
            addCollisionSkeletonNode(_bd, false);
            for (int i = 0; i < _bd->getNumChildJoints(); i++)
                addCollisionSkeletonNode(_bd->getChildNode(i), true);
        }
    }

    void SkeletonCollision::checkCollision(bool bConsiderGround) {
        int num_max_contact = 100;
        clearAllContacts();
        int numCollision = 0;
        for (int i = 0; i < mCollisionSkeletonNodeList.size(); i++) {
            for (int j = i + 1; j < mCollisionSkeletonNodeList.size(); j++) {
                if (mCollisionSkeletonNodeList[i]->mBodyNode->getParentNode() == mCollisionSkeletonNodeList[j]->mBodyNode || mCollisionSkeletonNodeList[j]->mBodyNode->getParentNode() == mCollisionSkeletonNodeList[i]->mBodyNode)
                    continue;
                if (mCollisionSkeletonNodeList[i]->mBodyNode->getSkel() == mCollisionSkeletonNodeList[j]->mBodyNode->getSkel())
                    continue;
                numCollision += mCollisionSkeletonNodeList[i]->checkCollision(mCollisionSkeletonNodeList[j], mContactPointList, num_max_contact);
            }
        }
        mNumTriIntersection = numCollision;
    }

    void SkeletonCollision::draw() {
        for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
            mCollisionSkeletonNodeList[i]->drawCollisionSkeletonNode();
    }
}
