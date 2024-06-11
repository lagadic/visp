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
#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTrackerSSDESM.h>

BEGIN_VISP_NAMESPACE
vpTemplateTrackerSSDESM::vpTemplateTrackerSSDESM(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), compoInitialised(false), HDir(), HInv(), HLMDir(), HLMInv(), GDir(), GInv()
{
  useCompositionnal = false;
  useInverse = false;

  if (!Warp->isESMcompatible()) {
    throw(vpException(vpException::badValue, "The selected warp function is not appropriate for the ESM algorithm..."));
  }

  HInv.resize(nbParam, nbParam);
  HDir.resize(nbParam, nbParam);
  HLMInv.resize(nbParam, nbParam);
  HLMDir.resize(nbParam, nbParam);
  GInv.resize(nbParam);
  GDir.resize(nbParam);
}

void vpTemplateTrackerSSDESM::initHessienDesired(const vpImage<unsigned char> &I) { initCompInverse(I); }

void vpTemplateTrackerSSDESM::initCompInverse(const vpImage<unsigned char> & /*I*/)
{
  ptTemplateCompo = new vpTemplateTrackerPointCompo[templateSize];
  int i, j;
  // direct
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    ptTemplateCompo[point].dW = new double[2 * nbParam];
    Warp->getdWdp0(i, j, ptTemplateCompo[point].dW);
  }

  // inverse
  HInv = 0;
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;

    ptTemplate[point].dW = new double[nbParam];
    Warp->getdW0(i, j, ptTemplate[point].dy, ptTemplate[point].dx, ptTemplate[point].dW);

    for (unsigned int it = 0; it < nbParam; it++)
      for (unsigned int jt = 0; jt < nbParam; jt++)
        HInv[it][jt] += ptTemplate[point].dW[it] * ptTemplate[point].dW[jt];
  }
  vpMatrix::computeHLM(HInv, lambdaDep, HLMInv);

  compoInitialised = true;
}

void vpTemplateTrackerSSDESM::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

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
    dp = 0;
    HDir = 0;
    GDir = 0;
    GInv = 0;
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
        // INVERSE
        Tij = ptTemplate[point].val;
        if (!blur)
          IW = I.getValue(i2, j2);
        else
          IW = BI.getValue(i2, j2);
        Nbpoint++;
        double er = (Tij - IW);
        for (unsigned int it = 0; it < nbParam; it++)
          GInv[it] += er * ptTemplate[point].dW[it];

        erreur += er * er;

        dIWx = dIx.getValue(i2, j2) + ptTemplate[point].dx;
        dIWy = dIy.getValue(i2, j2) + ptTemplate[point].dy;

        // Calcul du Hessien
        Warp->dWarpCompo(X1, X2, p, ptTemplateCompo[point].dW, dW);

        for (unsigned int it = 0; it < nbParam; it++)
          tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

        for (unsigned int it = 0; it < nbParam; it++)
          for (unsigned int jt = 0; jt < nbParam; jt++)
            HDir[it][jt] += tempt[it] * tempt[jt];

        for (unsigned int it = 0; it < nbParam; it++)
          GDir[it] += er * tempt[it];
      }
    }
    if (Nbpoint == 0) {
      delete[] tempt;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }

    vpMatrix::computeHLM(HDir, lambdaDep, HLMDir);

    try {
      dp = (HLMDir).inverseByLU() * (GDir);
    }
    catch (const vpException &e) {
      delete[] tempt;
      throw(e);
    }

    dp = gain * dp;
    if (useBrent) {
      alpha = 2.;
      computeOptimalBrentGain(I, p, erreur / Nbpoint, dp, alpha);
      dp = alpha * dp;
    }

    p += dp;

    computeEvalRMS(p);

    if (iteration == 0) {
      evolRMS_init = evolRMS;
    }

    iteration++;

    evolRMS_delta = std::fabs(evolRMS - evolRMS_prec);
    evolRMS_prec = evolRMS;

  } while ((iteration < iterationMax) && (evolRMS_delta > std::fabs(evolRMS_init) * evolRMS_eps));
  delete[] tempt;

  nbIteration = iteration;
}
END_VISP_NAMESPACE
