/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Yuting Ye <yuting.ye@gmail.com>
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
#include "yui/GLFuncs.h"
#include "kinematics/Skeleton.h"
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace kinematics;

void MyWindow::displayTimer(int _val)
{
    mFrame += 4;
    if(mFrame == mMaxFrame){
        mFrame = 0;
        mPlaying = false;
    }
    glutPostRedisplay();
    if(mPlaying)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::computeMax()
{
    mMaxFrame = mMainMotion.getNumFrames();
}

void MyWindow::draw()
{
    Skeleton* model = mMainMotion.getSkel();

    // validate the frame index
    int nFrames = mMainMotion.getNumFrames();
    int fr = mFrame;
    if (fr >= nFrames) fr = nFrames-1;
    
    // update and draw the motion
    model->setPose(mMainMotion.getPoseAtFrame(fr));
    model->draw(mRI);
    
    Skeleton* model2 = mCompareMotion.getSkel();
    if (fr >= mCompareMotion.getNumFrames()) 
        fr = mCompareMotion.getNumFrames() - 1;
    model2->setPose(mCompareMotion.getPoseAtFrame(fr));
    model2->draw(mRI);       

    if(mShowMarker) model->drawMarkers(mRI);
    if(mShowProgress) yui::drawProgressBar(fr,mMaxFrame);

    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d/%d",mFrame,mMaxFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mPlaying = !mPlaying;
        if(mPlaying)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    case 'a': // move one frame forward
        if(mFrame<mMaxFrame-1)
            mFrame ++;
        break;
    case 'd': // move one frame backward
        if(mFrame>0)
            mFrame --;
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}

// display the progress bar when the mouse moves to the bar region
void MyWindow::move(int _x, int _y)
{
    double Xport = (double)_x/mWinWidth;
    double Yport = (double)(mWinHeight-_y)/mWinHeight;

    bool oldShow = mShowProgress;

    if(!mRotate && !mTranslate && !mZooming){
        if(Xport>=0.15 && Xport<=0.85 && Yport>=0.02 && Yport<=0.08)
            mShowProgress = true;
        else if(mShowProgress) mShowProgress = false;
    }

    if( oldShow != mShowProgress)
        glutPostRedisplay();
}
