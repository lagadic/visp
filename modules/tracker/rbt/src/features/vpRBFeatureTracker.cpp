/*
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
 */

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

BEGIN_VISP_NAMESPACE

vpRBFeatureTracker::vpRBFeatureTracker()
{
  m_numFeatures = 0;
  m_userVvsWeight = 1.0;
  m_vvsConverged = false;
  m_enableDisplay = true;
}

void vpRBFeatureTracker::updateCovariance(const double lambda)
{
  vpMatrix  D;
  D.diag(m_covWeightDiag);
  m_cov = computeCovarianceMatrix(m_L, lambda * m_error, D);
}

void vpRBFeatureTracker::computeJTR(const vpMatrix &interaction, const vpColVector &error, vpColVector &JTR)
{
  if (interaction.getRows() != error.getRows() || interaction.getCols() != 6) {
    throw vpMatrixException(vpMatrixException::incorrectMatrixSizeError, "Incorrect matrices size in computeJTR.");
  }

  JTR.resize(6, false);
#if defined(VISP_HAVE_SIMDLIB)
  SimdComputeJtR(interaction.data, interaction.getRows(), error.data, JTR.data);
#else
  const unsigned int N = interaction.getRows();

  for (unsigned int i = 0; i < 6; ++i) {
    double ssum = 0;
    for (unsigned int j = 0; j < N; ++j) {
      ssum += interaction[j][i] * error[j];
    }
    JTR[i] = ssum;
}
#endif
}

vpMatrix vpRBFeatureTracker::computeCovarianceMatrix(const vpMatrix &DJ, const vpColVector &e, const vpMatrix &covDiag)
{
  const vpColVector covDiagE = covDiag * e;
  double sigma2 = (covDiagE.t() * covDiag * e) / ((double)e.getRows());
  return (DJ.t() * covDiag * DJ).pseudoInverse() * sigma2;
}

END_VISP_NAMESPACE
