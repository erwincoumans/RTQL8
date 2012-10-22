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

#include "kinematics/Skeleton.h"
#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/FileInfoDof.h"

#include "MyWindow.h"

#include <iostream>

using namespace std;
using namespace kinematics;
#include "utils/Paths.h"

int main(int argc, char* argv[])
{
    const char* modelfile;
    const char* doffile;
    const char* doffile1;
    if(argc!=3){
        modelfile = RTQL8_DATA_PATH"skel/fixedHand.skel";        
        doffile = RTQL8_DATA_PATH"dof/fixedHand.dof";
    }else{
        modelfile = argv[1];
        doffile = argv[2];
    }
	
    FileInfoSkel<Skeleton> model;
    model.loadFile(modelfile);
    // model.getSkel();

    FileInfoDof motion(model.getSkel());
    motion.loadFile(doffile);
    //    FileInfoDof motion1(model.getSkel());
    //motion1.loadFile(doffile1);

    MyWindow window(motion);
    //    MyWindow window(motion, motion1);
    window.computeMax();

    glutInit(&argc, argv);
    window.initWindow(640, 480, modelfile);
    glutMainLoop();

    return 0;
}
