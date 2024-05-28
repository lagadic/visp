/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/

#include <limits> // numeric_limits

#include <visp3/core/vpImageTools.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardAdditional.h>

BEGIN_VISP_NAMESPACE
vpTemplateTrackerSSDForwardAdditional::vpTemplateTrackerSSDForwardAdditional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), minimizationMethod(USE_NEWTON), p_prec(), G_prec(), KQuasiNewton()
{
  useCompositionnal = false;
}

void vpTemplateTrackerSSDForwardAdditional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  dW = 0;

  double lambda = lambdaDep;
  double IW, dIWx, dIWy;
  double Tij;
  unsigned int iteration = 0;
  int i, j;
  double i2, j2;
  double alpha = 2.;

  initPosEvalRMS(p);

  double evolRMS_init = 0;
  double evolRMS_prec = 0;
  double evolRMS_delta;
  double *tempt = new double[nbParam];

  do {
    unsigned int Nbpoint = 0;
    double erreur = 0;
    G = 0;
    H = 0;
    Warp->computeCoeff(p);
    for (unsigned int point = 0; point < templateSize; point++) {
      i = ptTemplate[point].y;
      j = ptTemplate[point].x;
      X1[0] = j;
      X1[1] = i;

      Warp->computeDenom(X1, p);
      Warp->warpX(X1, X2, p);

      j2 = X2[0];
      i2 = X2[1];
      if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
        Tij = ptTemplate[point].val;

        if (!blur)
          IW = I.getValue(i2, j2);
        else
          IW = BI.getValue(i2, j2);

        dIWx = dIx.getValue(i2, j2);
        dIWy = dIy.getValue(i2, j2);
        Nbpoint++;
        // Calcul du Hessien
        Warp->dWarp(X1, X2, p, dW);
        for (unsigned int it = 0; it < nbParam; it++)
          tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

        for (unsigned int it = 0; it < nbParam; it++)
          for (unsigned int jt = 0; jt < nbParam; jt++)
            H[it][jt] += tempt[it] * tempt[jt];

        double er = (Tij - IW);
        for (unsigned int it = 0; it < nbParam; it++)
          G[it] += er * tempt[it];

        erreur += (er * er);
      }
    }
    if (Nbpoint == 0) {
      delete[] tempt;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }

    vpMatrix::computeHLM(H, lambda, HLM);
    try {
      dp = HLM.inverseByLU() * G;
    }
    catch (const vpException &e) {
      delete[] tempt;
      throw(e);
    }

    switch (minimizationMethod) {
    case vpTemplateTrackerSSDForwardAdditional::USE_LMA: {
      vpColVector p_test_LMA(nbParam);
      p_test_LMA = p + dp;
      erreur = -getCost(I, p);
      double erreur_LMA = -getCost(I, p_test_LMA);
      if (erreur_LMA < erreur) {
        p = p_test_LMA;
        lambda = (lambda / 10. < 1e-6) ? lambda / 10. : 1e-6;
      }
      else {
        lambda = (lambda * 10. < 1e6) ? 1e6 : lambda * 10.;
      }
    } break;
    case vpTemplateTrackerSSDForwardAdditional::USE_GRADIENT: {
      dp = gain * 0.000001 * G;
      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, erreur, dp, alpha);
        dp = alpha * dp;
      }
      p += dp;
      break;
    }

    case vpTemplateTrackerSSDForwardAdditional::USE_QUASINEWTON: {
      if (iterationGlobale != 0) {
        vpColVector s_quasi = p - p_prec;
        vpColVector y_quasi = G - G_prec;
        double s_scal_y = s_quasi.t() * y_quasi;
        if (std::fabs(s_scal_y) > std::numeric_limits<double>::epsilon()) // DFP
          KQuasiNewton = KQuasiNewton + 0.001 * (s_quasi * s_quasi.t() / s_scal_y -
                                                 KQuasiNewton * y_quasi * y_quasi.t() * KQuasiNewton /
                                                     (y_quasi.t() * KQuasiNewton * y_quasi));
      }
      dp = -KQuasiNewton * G;
      p_prec = p;
      G_prec = G;
      p -= 1.01 * dp;
    } break;

    case vpTemplateTrackerSSDForwardAdditional::USE_NEWTON:
    default: {
      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, erreur, dp, alpha);
        dp = alpha * dp;
      }

      p += dp;
      break;
    }
    }

    computeEvalRMS(p);

    if (iteration == 0) {
      evolRMS_init = evolRMS;
    }

    iteration++;
    iterationGlobale++;

    evolRMS_delta = std::fabs(evolRMS - evolRMS_prec);
    evolRMS_prec = evolRMS;

  } while ((iteration < iterationMax) && (evolRMS_delta > std::fabs(evolRMS_init) * evolRMS_eps));
  delete[] tempt;

  nbIteration = iteration;
}
END_VISP_NAMESPACE
