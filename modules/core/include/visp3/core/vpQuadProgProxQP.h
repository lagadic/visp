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

/*!
 * \file vpQuadProgProxQP.h
 * \brief Implementation of Quadratic Program with Active Sets.
 */

#ifndef _vpQuadProgProxQP_h_
#define _vpQuadProgProxQP_h_

#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpQuadProg.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpQuadProgProxQP
 * \ingroup group_core_optim
 * \brief This class provides a solver for Quadratic Programs.
 *
 * The cost function is written under the form \f$ \min ||\mathbf{Q}\mathbf{x} - \mathbf{r}||^2\f$.
 *
 * If a cost function is written under the canonical form \f$\min \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} +
 * \mathbf{c}^T\mathbf{x}\f$ then fromCanonicalCost() can be used to retrieve Q and r from H and c.
 *
 * Equality constraints are solved through projection into the kernel.
 *
 * Inequality constraints are solved with active sets.
 *
 * In order to be used sequentially, the decomposition of the equality constraint may be stored.
 * The last active set is always stored and used to warm start the next call.
 *
 * \warning The solvers are only available if c++14 or higher is activated during build.
 * Configure ViSP using cmake -DUSE_CXX_STANDARD=14.
*/
class VISP_EXPORT vpQuadProgProxQP : public vpQuadProg
{
public:
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_14)
  /** @name Interface implementation */
  //@{
  virtual bool solveQP(const vpMatrix &Q, const vpColVector &r, vpMatrix A, vpColVector b, const vpMatrix &C,
               const vpColVector &d, vpColVector &x, const double &tol = 1e-6) override;

  virtual bool solveQPCanonicalCost(const vpMatrix &H, const vpColVector &g, vpMatrix A, vpColVector b, const vpMatrix &C,
                            const vpColVector &d, vpColVector &x, const double &tol = 1e-6) override;
  //@}

protected:
  // TODO store solver for re-use if warm start
#endif
};
END_VISP_NAMESPACE
#endif
