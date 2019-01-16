/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Linear Programming with simplex
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/

#ifndef vpLinProgh
#define vpLinProgh

#include <cmath>     // For std::abs() on iOS
#include <cstdlib>   // For std::abs() on iOS

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

/*!
  \file vpLinProg.h
  \brief Implementation of Linear Program with simplex algorithm.
*/

/*!
  \class vpLinProg
  \ingroup group_core_optim
  \brief This class provides two solvers for Linear Programs.

  One is a classical simplex, the other can deal with various inequality or bound constraints.

  Utility functions to reduce or check linear equalities or inequalities are also available.

  \warning The solvers are only available if C++11 is activated during compilation. Configure ViSP using cmake -DUSE_CPP11=ON.
*/
class VISP_EXPORT vpLinProg
{
public:

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
/*!
  Used to pass a list of bounded variables to solveLP(), as a list of (index, bound).

The type is compatible with C++11's braced initialization.
 Construction can be done in the call to solveLP or before, as shown in this example:

  \f$\begin{array}{lll}
  (x,y,z) = &  \arg\min & -2x -3y -4z\\
               & \text{s.t.}& 3x + 2y + z \leq 10\\
               & \text{s.t.}& 2x + 5y + 3z \leq 15\\
                & \text{s.t.}& x, y, z \geq 0\\
                & \text{s.t.}& z \leq 6\end{array}\f$

  Here the lower bound is built explicitely while the upper one is built during the call to solveLP():

  \warning This function is only available if C++11 is activated during compilation. Configure ViSP using cmake -DUSE_CPP11=ON.

  \code
  #include <visp3/core/vpLinProg.h>

  int main()
  {
    vpColVector c(3), x;
    vpMatrix C(2, 3);
    vpColVector d(2);
    c[0] = -2; c[1] = -3; c[2] = -4;
    C[0][0] = 3;    C[0][1] = 2; C[0][2] = 1; d[0] = 10;
    C[1][0] = 2; C[1][1] = 5; C[1][2] = 3;  d[1] = 15;

    // build lower bounds explicitely as a std::vector of std::pair<int, double>
    std::vector<vpLinProg::BoundedIndex> lower_bound;
    for(unsigned int i = 0; i < 3; ++i)
    {
      vpLinProg::BoundedIndex bound;
      bound.first = i;    // index
      bound.second = 0;   // lower bound for this index
      lower_bound.push_back(bound);
    }

    if(vpLinProg::solveLP(c, vpMatrix(0,0), vpColVector(0), C, d, x,
                          lower_bound,
                          {{2,6}})) // upper bound is passed with braced initialization
    {
        std::cout << "x: " << x.t() << std::endl;
        std::cout << "cost: " << c.t()*x << std::endl;
    }
  }
  \endcode

    \sa solveLP()
  */
  typedef std::pair<unsigned int, double> BoundedIndex;

  /** @name Solvers  */
  //@{
  static bool simplex(const vpColVector &c, vpMatrix A, vpColVector b,
                      vpColVector &x, const double &tol = 1e-6);

  static bool solveLP(const vpColVector &c, vpMatrix A, vpColVector b,
                      const vpMatrix &C, const vpColVector &d, vpColVector &x,
                      std::vector<BoundedIndex> l = {},
                      std::vector<BoundedIndex> u = {},
                      const double &tol = 1e-6);

  //@}
#endif

  /** @name Dimension reduction for equality constraints  */
  //@{
  static bool colReduction(vpMatrix &A, vpColVector &b, bool full_rank = false, const double &tol = 1e-6);

  static bool rowReduction(vpMatrix &A, vpColVector &b, const double &tol = 1e-6);
  //@}

  /** @name Vector and equality checking  */
  //@{
  /*!
    Check if all elements of \f$x\f$ are near zero.

    \param x : vector to be checked
    \param tol : tolerance

    \return True if \f$\forall i, |\mathbf{x}_i| < \text{~tol} \f$
  */
  static bool allZero(const vpColVector &x, const double &tol = 1e-6)
  {
    for(unsigned int i = 0; i < x.getRows(); ++i)
    {
      if(std::abs(x[i]) > tol)
        return false;
    }
    return true;
  }

  /*!
    Check if \f$\mathbf{A}\mathbf{x}\f$ is near \f$\mathbf{b}\f$.

    \param A : matrix (dimension m x n)
    \param x : vector (dimension n)
    \param b : vector (dimension m)
    \param tol : tolerance

    \return True if \f$ \forall i, |\mathbf{A}_i\mathbf{x} - \mathbf{b}_i| < \text{~tol}\f$
  */
  static bool allClose(const vpMatrix &A, const vpColVector &x, const vpColVector &b, const double &tol = 1e-6)
  {
    for(unsigned int i = 0; i < b.getRows(); ++i)
    {
      if(std::abs(A.getRow(i)*x - b[i]) > tol)
        return false;
    }
    return true;
  }

  /*!
    Check if all elements of \f$\mathbf{C}\mathbf{x} - \mathbf{d}\f$ are lesser or equal to threshold.
    \param C : matrix (dimension m x n)
    \param x : vector (dimension n)
    \param d : vector (dimension m)
    \param thr : threshold

    \return True if \f$ \forall i, \mathbf{C}_i\mathbf{x} - \mathbf{d}_i \leq \text{~thr}\f$
  */
  static bool allLesser(const vpMatrix &C, const vpColVector &x, const vpColVector &d, const double &thr = 1e-6)
  {
    for(unsigned int i = 0; i < d.getRows(); ++i)
    {
      if(C.getRow(i)*x - d[i] > thr)
        return false;
    }
    return true;
  }

  /*!
    Check if all elements of \f$\mathbf{x}\f$ are lesser or equal to threshold.

    \param x : vector (dimension n)
    \param thr : threshold

    \return True if \f$ \forall i, \mathbf{x}_i \leq \text{~thr}\f$
  */
  static bool allLesser(const vpColVector &x, const double &thr = 1e-6)
  {
    for(unsigned int i = 0; i < x.getRows(); ++i)
    {
      if(x[i] > thr)
        return false;
    }
    return true;
  }

  /*!
    Check if all elements of \f$\mathbf{x}\f$ are greater or equal to threshold.

    \param x : vector (dimension n)
    \param thr : threshold

    \return True if \f$ \forall i, \mathbf{x}_i \geq \text{~thr}\f$
  */
  static bool allGreater(const vpColVector &x, const double &thr = 1e-6)
  {
    for(unsigned int i = 0; i < x.getRows(); ++i)
    {
      if(x[i] < thr)
        return false;
    }
    return true;
  }
  //@}
};
#endif // vpLinProgh
