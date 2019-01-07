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
 * Quadratic Programming
 *
 * Authors:
 * Olivier Kermorgant
 *
 *****************************************************************************/

#ifndef vpQuadProgh
#define vpQuadProgh

#include <vector>
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpLinProg.h>

/*!
  \file vpQuadProg.h
  \brief Implementation of Quadratic Program with Active Sets.
*/

/*!
  \class vpQuadProg
  \ingroup group_core_optim
  \brief This class provides a solver for Quadratic Programs.

  The cost function is written under the form \f$ \min ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2\f$.

  If a cost function is written under the canonical form \f$\min \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{c}^T\mathbf{x}\f$
  then fromCanonicalCost() can be used to retrieve Q and r from H and c.

  Equality constraints are solved through projection into the kernel.

  Inequality constraints are solved with active sets.

  In order to be used sequentially, the decomposition of the equality constraint may be stored.
  The last active set is always stored and used to warm start the next call.

  \warning The solvers are only available if C++11 is activated during compilation. Configure ViSP using cmake -DUSE_CPP11=ON.
*/
class VISP_EXPORT vpQuadProg
{
public:
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  /** @name Instanciated solvers  */
  //@{
  bool solveQPe(const vpMatrix &Q, const vpColVector &r,
                vpColVector &x, const double &tol = 1e-6) const;

  bool solveQPi(const vpMatrix &Q, const vpColVector &r,
                const vpMatrix &C, const vpColVector &d,
                vpColVector &x,
                const bool use_equality = false,
                const double &tol = 1e-6);

  bool solveQP(const vpMatrix &Q, const vpColVector &r,
               vpMatrix A, vpColVector b,
               const vpMatrix &C, const vpColVector &d,
               vpColVector &x, const double &tol = 1e-6);
  //@}

  /** @name Managing sequential calls to solvers  */
  //@{
  bool setEqualityConstraint(const vpMatrix &A, const vpColVector &b, const double &tol = 1e-6);
  /*!
    Resets the active set that was found by a previous call to solveQP() or solveQPi(), if any.
  */
  void resetActiveSet()
  {
    active.clear();
  }
  //@}

  static void fromCanonicalCost(const vpMatrix &H, const vpColVector &c, vpMatrix &Q, vpColVector &r, const double &tol = 1e-6);
  static bool solveQPe(const vpMatrix &Q, const vpColVector &r,
                vpMatrix A, vpColVector b,
                vpColVector &x, const double &tol = 1e-6);

protected:
  /*!
    Active set from the last call to solveQP() or solveQPi(). Used for warm starting the next call.
  */
  std::vector<unsigned int> active;
  /*!
    Inactive set from the last call to solveQP() or solveQPi(). Used for warm starting the next call.
  */
  std::vector<unsigned int> inactive;
  /*!
    Stored particular solution from the last call to setEqualityConstraint().
  */
  vpColVector x1;
  /*!
    Stored projection to the kernel from the last call to setEqualityConstraint().
  */
  vpMatrix Z;

  static bool solveByProjection(const vpMatrix &Q, const vpColVector &r,
                                vpMatrix &A, vpColVector &b,
                                vpColVector &x, const double &tol = 1e-6);

  /*!
    Performs a dimension check of passed QP matrices and vectors.

    If any inconsistency is detected, displays a summary and throws an exception.

  \param Q : cost matrix (dimension c x n)
  \param r : cost vector (dimension c)
  \param A : pointer to the equality matrix (if any, dimension m x n)
  \param b : pointer to the equality vector (if any, dimension m)
  \param C : pointer to the inequality matrix (if any, dimension p x n)
  \param d : pointer to the inequality vector (if any, dimension p)
  \param fct : name of the solver that called this function

  \return the dimension of the search space.
  */
  static unsigned int checkDimensions(const vpMatrix &Q, const vpColVector &r,
                                      const vpMatrix* A, const vpColVector* b,
                                      const vpMatrix* C, const vpColVector* d,
                                      const std::string fct)
  {
    // check data consistency
    const unsigned int n = Q.getCols();
    const bool Ab = (A != NULL && b != NULL && A->getRows());
    const bool Cd = (C != NULL && d != NULL && C->getRows());

    if (  (Ab && n != A->getCols()) ||
          (Cd && n != C->getCols()) ||
          (Ab && A->getRows() != b->getRows()) ||
          (Cd && C->getRows() != d->getRows()) ||
          Q.getRows() != r.getRows())
    {
      std::cout << "vpQuadProg::" << fct << ": wrong dimension\n" <<
                   "Q: " << Q.getRows() << "x" << Q.getCols() << " - r: " << r.getRows() << std::endl;
      if(Ab)
        std::cout << "A: " << A->getRows() << "x" << A->getCols() << " - b: " << b->getRows() << std::endl;
      if(Cd)
        std::cout << "C: " << C->getRows() << "x" << C->getCols() << " - d: " << d->getRows() << std::endl;
      throw vpMatrixException::dimensionError;
    }
    return n;
  }
#endif
};
#endif
