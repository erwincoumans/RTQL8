/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>
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

#include "Lemke.h"
#include <iostream>
#include <cmath>
using namespace std;

#include <math.h>

#ifndef isnan
# define isnan(x) \
    (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
    : sizeof (x) == sizeof (double) ? isnan_d (x) \
    : isnan_f (x))
static inline int isnan_f  (float       x) { return x != x; }
static inline int isnan_d  (double      x) { return x != x; }
static inline int isnan_ld (long double x) { return x != x; }
#endif

#ifndef isinf
# define isinf(x) \
    (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
    : sizeof (x) == sizeof (double) ? isinf_d (x) \
    : isinf_f (x))
static inline int isinf_f  (float       x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_d  (double      x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_ld (long double x)
{ return !isnan (x) && isnan (x - x); }
#endif

namespace lcpsolver {

    double RandDouble(double _low, double _high)
    {
        double temp;

        /* swap low & high around if the user makes no sense */
        if (_low > _high)
        {
            temp = _low;
            _low = _high;
            _high = temp;
        }

        /* calculate the random number & return it */
        temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
            * (_high - _low) + _low;
        return temp;
    }


    int Lemke(const MatrixXd& _M, const VectorXd& _q, VectorXd& _z)
    {
        int n = _q.size();
        if (_M.rows() == n && _M.cols() == n)
            cout << "Matrices are not compatible" << endl;
        
	    const double zer_tol = 1e-5;
	    const double piv_tol = 1e-8;
	    int maxiter = 1000;
	    int err = 0;

	    if (_q.minCoeff() > 0)
	    {
		    _z = VectorXd::Zero(n);
		    return err;	
	    }

	    _z = VectorXd::Zero(2 * n);
	    int iter = 0;
	    double theta = 0;
	    double ratio = 0;
	    int leaving  = 0;
	    VectorXd Be = VectorXd::Constant(n, 1);
	    VectorXd x = _q;
	    vector<int> bas;
	    vector<int> nonbas;

	    int t = 2 * n + 1;
	    int entering = t;


	    bas.clear();
	    for (int i = 0; i < n; ++i)
	    {
		    nonbas.push_back(i);
		    bas.push_back(i + n);
	    }

	    MatrixXd B = -MatrixXd::Identity(n, n);
	    VectorXd minuxX = -x;
	    int lvindex;
	    double tval = minuxX.maxCoeff(&lvindex);
	    leaving = bas[lvindex];

	    bas[lvindex] = t;
	    VectorXd U = VectorXd::Zero(n);
	    for (int i = 0; i < n; ++i)
	    {
		    if (x[i] < 0)
			    U[i] = 1;
	    }
	    Be = -(B * U);
	    x += tval * U;
	    x[lvindex] = tval;
	    B.col(lvindex) = Be;

	    for (iter = 0; iter < maxiter; ++iter)
	    {
		    if (leaving == t)
		    {
			    break;
		    }
		    else if (leaving < n)
		    {
			    entering = n + leaving;
			    Be = VectorXd::Zero(n);
			    Be[leaving] = -1;
		    }
		    else
		    {
			    entering = leaving - n;
			    Be = _M.col(entering);
		    }
		    VectorXd d = B.fullPivLu().solve(Be);

		    vector<int> j;
		    for (int i = 0; i < n; ++i)
		    {
			    if (d[i] > piv_tol)
				    j.push_back(i);
		    }
		    if (j.empty())
		    {
			    err = 2;
			    break;
		    }
		    int jSize = static_cast<int>(j.size());
		    VectorXd minRatio(jSize);
		    for (int i = 0; i < jSize; ++i)
		    {
			    minRatio[i] = (x[j[i]] + zer_tol) / d[j[i]];
		    }
		    double theta = minRatio.minCoeff();
		    vector<int> tmpJ;
                    vector<double> tmpMinRatio;
		    for (int i = 0; i < jSize; ++i)
		    {
                if (x[j[i]] / d[j[i]] <= theta)
                {
                    tmpJ.push_back(j[i]);
                    tmpMinRatio.push_back(minRatio[i]);
                }
		    }

		    j = tmpJ;
		    jSize = static_cast<int>(j.size());
            if (jSize == 0)
            {
                err = 4;
                break;
            }
		    lvindex = -1;
		    for (int i = 0; i < jSize; ++i)
		    {
			    if (bas[j[i]] == t)
				    lvindex = i;
		    }
		    if (lvindex != -1)
		    {
			    lvindex = j[lvindex]; 
		    }
		    else
		    {
                theta = tmpMinRatio[0];
                for (int i = 0; i < jSize; ++i)
                {
                    if (tmpMinRatio[i] <= theta)
                    {
                        theta = tmpMinRatio[i];
                        lvindex = i;
                    }
                }
// 			    VectorXd dj(jSize);
// 			    for (int i = 0; i < jSize; ++i)
// 			    {
// 				    dj[i] = d[j[i]];
// 			    }
// 			    theta = dj.maxCoeff(&lvindex);
// 			    vector<int> lvindexSet;
// 			    for (int i = 0; i < jSize; ++i)
// 			    {
// 				    if (dj[i] == theta)
// 					    lvindexSet.push_back(i);
// 			    }
//			    lvindex = lvindexSet[static_cast<int>((lvindexSet.size() * RandDouble(0, 1)))];
			    lvindex = j[lvindex];

		    }

		    leaving = bas[lvindex];

		    ratio = x[lvindex] / d[lvindex];

            bool bDiverged = false;
            for (int i = 0; i < n; ++i)
            {
                if (isnan(x[i]) || isinf(x[i]))
                {
                    bDiverged = true;
                    break;
                }
            }
            if (bDiverged)
            {
                err = 4;
                break;
            }
		    x = x - ratio * d;
		    x[lvindex] = ratio;
		    B.col(lvindex) = Be;
		    bas[lvindex] = entering;
    		
	    }
            //            cout << "check 6" << endl;

	    if (iter >= maxiter && leaving != t)
		    err = 1;

	    //for (size_t i = 0; i < bas.size(); ++i)
     //           cout << "bas[i] = " << bas[i] << " ";
     //       cout << endl;

		if (err == 0)
		{
			for (size_t i = 0; i < bas.size(); ++i) {
				_z[bas[i]] = x[i];
			}

			VectorXd realZ = _z.segment(0, n);
			_z = realZ;

            if (!validate(_M, _z, _q))
            {
                _z = VectorXd::Zero(n);
                err = 3;
            }
		}
		else
		{
			_z = VectorXd::Zero(n); //solve failed, return a 0 vector
		}

	    if (err == 1)
		    cout << "LCP Solver: Iterations exceeded limit";
	    else if (err == 2)
		    cout << "LCP Solver: Unbounded ray";
        else if (err == 3)
            cout << "LCP Solver: Solver converged with numerical issues. Validation failed.";
        else if (err == 4)
            cout << "LCP Solver: Iteration diverged.";
	    return err;
    }

    bool validate(const MatrixXd& _M, const VectorXd& _z, const VectorXd& _q)
    {
        const double threshold = 1e-4;
        int n = _z.size();

        VectorXd w = _M * _z + _q;
        for (int i = 0; i < n; ++i)
        {
            if (w(i) < -threshold || _z(i) < -threshold)
                return false;
            if (abs(w(i) * _z(i)) > threshold)
                return false;
        }
        return true;
    }

} //namespace lcpsolver
