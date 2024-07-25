/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
*****************************************************************************/

#include "vpTutoMeanSquareFitting.h"

namespace tutorial
{
#ifdef ENABLE_VISP_NAMESPACE
using VISP_NAMESPACE_NAME;
#endif

vpTutoMeanSquareFitting::vpTutoMeanSquareFitting()
  : m_a(0.f)
  , m_b(0.f)
  , m_c(0.f)
{ }

float vpTutoMeanSquareFitting::fit(const std::vector<vpImagePoint> &pts)
{
  unsigned int nbPts = pts.size();
  vpMatrix A(nbPts, 3, 1.); // The matrix that contains the u^2, u and 1s
  vpMatrix X(3, 1); // The matrix we want to estimate, that contains the a, b and c coefficients.
  vpMatrix b(nbPts, 1); // The matrix that contains the v values

  // Fill the matrices that form the system we want to solve
  for (unsigned int i = 0; i < nbPts; ++i) {
    float u = pts[i].get_u();
    float v = pts[i].get_v();
    A[i][0] = u *u;
    A[i][1] = u;
    A[i][2] = 1.f;
    b[i][0] = v;
  }

  // Compute the parabolla coefficients using the least-mean-square method.
  X = A.pseudoInverse() * b;
  m_a = X[0][0];
  m_b = X[1][0];
  m_c = X[2][0];

  // Compute the mean absolute error
  float meanError = 0.f;
  for (unsigned int i = 0; i < nbPts; ++i) {
    float u = pts[i].get_u();
    float v = pts[i].get_v();
    meanError += std::abs(v - ((m_a * u * u) + (m_b * u) + m_c));
  }
  meanError /= static_cast<float>(nbPts);
  return meanError;
}
}
