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

#include "MyWindow.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"
#include "utils/Timer.h"
#include "yui/GLFuncs.h"
#include "Analyzer.h"

using namespace dynamics;

void MyWindow::initDyn()
{
    mTimeStep = 1.0 / mMotion->getFPS();

    mSkel->initDynamics();
    mSkel->setPose(mMotion->getPoseAtFrame(0), false, false);
    
    mAnalyzer = new Analyzer(mMotion, mSkel, mGravity);
}

void MyWindow::displayTimer(int _val)
{
    if (mPlay) {
        mPlayFrame++;
        if (mPlayFrame >= mMotion->getNumFrames())
            mPlayFrame = 0;
        glutPostRedisplay();
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
    }
}

void MyWindow::draw()
{
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    if (mPlayFrame < mMotion->getNumFrames())
        mSkel->setPose(mMotion->getPoseAtFrame(mPlayFrame), false, false);
    
    mSkel->draw(mRI);
            
    // display the frame count in 2D text
    char buff[64];
    sprintf(buff, "%d", mPlayFrame);
    string frame(buff);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ':
        mAnalyzer->analyze();
        cout << "See output.txt" << endl;
        break;
    case 'p': // playBack
        mPlay = !mPlay;
        if (mPlay)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        
        break;
    case '[': // step backward
        mPlayFrame--;
        if(mPlayFrame < 0)
            mPlayFrame = 0;
        glutPostRedisplay();
        
        break;
    case ']': // step forwardward
        mPlayFrame++;
        if(mPlayFrame >= mMotion->getNumFrames())
            mPlayFrame = 0;
        glutPostRedisplay();
        
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}
