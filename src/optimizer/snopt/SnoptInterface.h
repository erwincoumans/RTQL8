/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef OPTIMIZER_SNOPT_SNOPT_INTERFACE_H
#define OPTIMIZER_SNOPT_SNOPT_INTERFACE_H

#include <vector>
#include <Eigen/Dense>
#include "optimizer/OptimizerArrayTypes.h"

#ifndef	ZERO
#define	ZERO 1.0e-30
#endif  // ifndef ZERO

namespace optimizer {
    namespace snopt {

        class SnoptInterface {
        public:
            enum Return { Solution, UserStop, Error, Stop, Infeasible};
            enum UpdateType { Obj = 1, Constr = 2 };
            enum SlackType { NoSlack = 0, Vslack = 1, Wslack = 2 };
            enum AbnormalType { NONE = 0, INFEASIBLE = 1, HESSIAN_UPDATE = 2, HESSIAN_RESET = 3 }; 

            struct Slack {
                int constr_idx;
                SlackType type;
                double val;
            };

            typedef int (*updateFunc) (long mask, int compute_gradients, double *coef_values, void *update_data);
		
            SnoptInterface(int constr_total, int coef_total, 
                           int nonlin_constr_total, int nonlin_obj_coef, int nonlin_jac_coef,
                           int *constr_eqns, int has_objective, VVD J, VVB JMap, std::vector<double> *constraints,
                           double *objective, std::vector<double> *gradient,	updateFunc update_f, void *update_d);
		
            SnoptInterface(int has_objective, VVD J, VVB JMap, std::vector<double> *constraints, double *objective,
                           std::vector<double> *gradient, updateFunc update_f, void *update_d);
            ~SnoptInterface();

            Return solve(double *x, double *lo_bounds, double *hi_bounds, int unit = 4);

            void clear(long mask, int compute_derivs);

            void resizeJacobian(int coef_total, int nonlin_coef_total, int constr_total, int nonlin_constr_total);
            void resizeCoef();

            int mNumConstr;
            int mNumCoef;
            int mNumNonlinConstr;
            int mNumNonlinObjCoef;
            int mNumNonlinJacCoef;

            double *mSolverX;
            double *mProblemX;
            double *mBoundsLo;
            double *mBoundsHi;
            int *mConstrEqns;

            int mHasObjective;

            static SnoptInterface *mRef;

            double *mObj;
            std::vector<double> *mdObjdCoef;

            std::vector<double> *mConstr;
            VVD mdConstrdCoef;

            Eigen::VectorXd mConstrScale;
            Eigen::VectorXd mCoefScale;

            VVB mCoefMap;
            double mReturnedObj;

            int mOutput;
            int mSum;
            bool mCheckTerm;
            bool mTermination;
            AbnormalType mAbnormal;
            int mBreak;

            void updateSolverX();
            void update(long mask, int compute_derivs, double *x);
            static void checkTermination(int *iAbort, double *xs);


        protected:
            SnoptInterface::updateFunc mUpdateFunc;
            void *mUpdateData;

            void scaleValues(long update_type, int compute_derivs);

        private:
            static void snoptObj(long int *mode, long int *nn_obj, double *x, 
                                 double *f_obj, double *g_obj, long int *nstate, 
                                 char *cu, long int *lencu, long int *iu, long int *leniu, 
                                 double *ru, long int *lenru);
            static void snoptJac(long int *mode, long int *nn_con, long int *nn_jac, long int *ne_jac,
                                 double *x, double *f_con, double *g_con, long int *nstate, 
                                 char *cu, long int *lencu, long int *iu, long int *leniu, 
                                 double *ru, long int *lenru);
            void fillUpSnoptFormat(VVD jacobian, double **a, long int **ha, long int **ka);
            int sparseCount(int col);

        };
        
    } // namespace snopt
} // namespace optimizer

#endif // #ifndef OPTIMIZER_SNOPT_SNOPT_INTERFACE_H

