/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Quadratic Programming
 */

#include <visp3/core/vpQuadProgProxQP.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_14)
BEGIN_VISP_NAMESPACE
/*!
  Solves a Quadratic Program under equality and inequality constraints.

  If the equality constraints \f$(\mathbf{A}, \mathbf{b})\f$ are the same between two calls, it is more efficient to use
  setEqualityConstraint() and solveQPi().

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2 \\
               & \text{s.t.}& \mathbf{A}\mathbf{x} = \mathbf{b} \\
               & \text{s.t.}& \mathbf{C}\mathbf{x} \leq \mathbf{d}
  \end{array}
  \f$
  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param A : equality matrix (dimension m x n)
  \param b : equality vector (dimension m)
  \param C : inequality matrix (dimension p x n)
  \param d : inequality vector (dimension p)
  \param x : solution (dimension n)
  \param tol : tolerance to test the ranks

  \return True if the solution was found.

  Here is an example:

  \f$\begin{array}{lll}
  \mathbf{x} = &  \arg\min & (x_1-1)^2 + x_2^2  \\
               & \text{s.t.}& x_1 + x_2 = 1  \\
                & \text{s.t.} & x_2 \geq 1\end{array}\f$
  \code
  #include <visp3/core/vpLinProg.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix Q(2,2), A(1,2), C(1,2);
    vpColVector r(2), b(1), d(1);

    Q[0][0] = Q[1][1] = 1;    r[0] = 1;
    A[0][0] = A[0][1] = 1;    b[0] = 1;
    C[0][1] = -1; d[0] = -1;

    vpColVector x;
    vpQuadProgNative qp;

    qp.solveQP(Q, r, A, b, C, d, x);
    std::cout << x.t() << std::endl;  // prints (0 1)
  }
  \endcode

  \sa resetActiveSet()
*/
bool vpQuadProgNative::solveQP(const vpMatrix &Q, const vpColVector &r, vpMatrix A, vpColVector b, const vpMatrix &C,
                         const vpColVector &d, vpColVector &x, const double &tol)
{
  const vpMatrix H = Q.transpose() * Q;
  const vpColVector g = -Q.transpose() * r;

  return solveQPCanonicalCost(H, g, A, b, C, d, x, tol);
}

bool vpQuadProgNative::solveQPCanonicalCost(const vpMatrix &H, const vpColVector &g, vpMatrix A, vpColVector b, const vpMatrix &C,
                          const vpColVector &d, vpColVector &x, const double &tol)
{
  Eigen::MatrixXd H_eigen;
  Eigen::VectorXd g_eigen;
  Eigen::MatrixXd A_eigen;
  Eigen::VectorXd g_eigen;
  Eigen::MatrixXd C_eigen;
  Eigen::VectorXd d_eigen;

  visp2eigen(H, H_eigen);
  visp2eigen(g, g_eigen);
  visp2eigen(A, A_eigen);
  visp2eigen(b, b_eigen);
  visp2eigen(C, C_eigen);
  visp2eigen(d, d_eigen);

  proxsuite::proxqp::isize dim = H.getRows();
  proxsuite::proxqp::isize n_eq = A.getRows();
  proxsuite::proxqp::isize n_in = C.getRows();

  Eigen::VectorXd l ;

  Eigen::VectorXd l_box(n);
  l_box.setZero();
  l_box.array() -= proxsuite::helpers::infinite_bouds::value();
  
  proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in);
  qp.init(H_eigen, g_eigen, A_eigen, b_eigen, C_eigen, l_box, d_eigen);
  qp.settings.eps_abs = tol;
  proxsuite::proxqp::Results<double> results = qp.solve();    

  if(result.info.status == proxsuite::proxqp::PROXQP_SOLVED)
  {
    eigen2visp(result.x, x);
    return true;
  }
  else
  {
    // TODO: log errors
    return false;
  }
}
