RTQL8 (pronounced “articulate”): Dynamic Multibody Simulator

RTQL8 is an open source library for developing kinematic and dynamic applications in robotics and computer animation. The multibody simulator is designed to aid development of motion control algorithms. RTQL8 uses generalized coordinates to represent articulated rigid body systems and computes Lagrange’s equations derived from D’Alemberts principle to describe the dynamics of motion. In contrast to many popular physics engines which view the simulator as a black box, RTQL8 provides full access to internal kinematic and dynamic states, such as mass matrix, Coriolis and centrifugal force, transformation matrices and their derivatives, etc. RTQL8 also provides efficient computation of Jacobian matrices for arbitrary points in arbitrary coordinate frames.

RTQL8 is written in C++. Currently, we only provide installation instructions for Mac OSX/Unix users. However, RTQL8 does not depend on any platform-specific libraries and can be installed on Windows. We will provide the installation instructions for Visual Studio soon.

Download: 
RTQL8.zip from http://www.cc.gatech.edu/~karenliu/RTQL8.html
(48MB, including all source code and tutorials)

Tutoriali: 
RTQL8 tutorial: docs/rtql8-tutorial.pdf
Multibody dynamics tutorial: docs/dynamics-tutorial.pdf
LCP contact handling tutorial: docs/lcp-tutorial.pdf

Contributors:
Karen Liu
Sumit Jain
Yuting Ye
Sehoon Ha
Jie Tan
Chen Tang
Kristin Siu

License:
RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
All rights reserved.
Geoorgia Tech Graphics Lab
This file is provided under the following "BSD-style" License:
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
Redistributions of source code must retain the above copyright
Redistributions in binary form must reproduce the abovecopyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provide with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


