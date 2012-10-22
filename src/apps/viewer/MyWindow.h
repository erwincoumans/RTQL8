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

#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "yui/Win3D.h"
#include "kinematics/FileInfoDof.h"

class MyWindow : public yui::Win3D {
public:
 MyWindow(kinematics::FileInfoDof& _mot): Win3D(), mMainMotion(_mot), mCompareMotion(_mot)
 {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlaying = false;
        mShowMarker = false;
        mShowProgress = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;
        mDisplayTimeout = 5;        
    }

 MyWindow(kinematics::FileInfoDof& _mot1, kinematics::FileInfoDof& _mot2): Win3D(), mMainMotion(_mot1), mCompareMotion(_mot2)
 {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;
		
        mPlaying = false;
        mShowMarker = false;
        mShowProgress = false;

        mPersp = 30.f;
        mTrans[2] = -1.f;
        mFrame = 0;
        mDisplayTimeout = 5;        
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);
    virtual void move(int _x, int _y);
    void computeMax();
	
protected:
    int mMaxFrame;
    bool mPlaying;
    int mFrame;
    bool mShowMarker;
    bool mShowProgress;
    kinematics::FileInfoDof& mMainMotion;
    kinematics::FileInfoDof& mCompareMotion;
};

#endif
