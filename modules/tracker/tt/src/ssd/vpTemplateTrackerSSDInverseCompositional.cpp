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
#include <visp3/core/vpImageTools.h>
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>

BEGIN_VISP_NAMESPACE
vpTemplateTrackerSSDInverseCompositional::vpTemplateTrackerSSDInverseCompositional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), compoInitialised(false), HInv(), HCompInverse(), useTemplateSelect(false)
{
  useInverse = true;
  HInv.resize(nbParam, nbParam);
  HCompInverse.resize(nbParam, nbParam);
}

void vpTemplateTrackerSSDInverseCompositional::initCompInverse(const vpImage<unsigned char> & /*I*/)
{
  H = 0;

  for (unsigned int point = 0; point < templateSize; point++) {
    if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
      int i = ptTemplate[point].y;
      int j = ptTemplate[point].x;
      ptTemplate[point].dW = new double[nbParam];

      Warp->getdW0(i, j, ptTemplate[point].dy, ptTemplate[point].dx, ptTemplate[point].dW);

      for (unsigned int it = 0; it < nbParam; it++)
        for (unsigned int jt = 0; jt < nbParam; jt++)
          H[it][jt] += ptTemplate[point].dW[it] * ptTemplate[point].dW[jt];
    }
  }

  vpMatrix HLMtemp;
  vpMatrix::computeHLM(H, lambdaDep, HLMtemp);

  HCompInverse = HLMtemp.inverseByLU();

  for (unsigned int point = 0; point < templateSize; point++) {
    if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
      ptTemplate[point].HiG = new double[nbParam];

      for (unsigned int i = 0; i < HCompInverse.getRows(); i++) {
        ptTemplate[point].HiG[i] = 0;
        for (unsigned int j = 0; j < HCompInverse.getCols(); j++) {
          ptTemplate[point].HiG[i] -= HCompInverse[i][j] * ptTemplate[point].dW[j];
        }
      }
    }
  }
  compoInitialised = true;
}

void vpTemplateTrackerSSDInverseCompositional::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompInverse(I);
}

void vpTemplateTrackerSSDInverseCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);

  vpColVector dpinv(nbParam);
  double IW;
  double Tij;
  unsigned int iteration = 0;
  int i, j;
  double i2, j2;
  double alpha = 2.;
  initPosEvalRMS(p);

  vpTemplateTrackerPoint *pt;

  double evolRMS_init = 0;
  double evolRMS_prec = 0;
  double evolRMS_delta;

  do {
    unsigned int Nbpoint = 0;
    double erreur = 0;
    dp = 0;
    Warp->computeCoeff(p);
    for (unsigned int point = 0; point < templateSize; point++) {
      if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
        pt = &ptTemplate[point];
        i = pt->y;
        j = pt->x;
        X1[0] = j;
        X1[1] = i;
        Warp->computeDenom(X1, p);
        Warp->warpX(X1, X2, p);
        j2 = X2[0];
        i2 = X2[1];

        if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
          Tij = pt->val;
          if (!blur)
            IW = I.getValue(i2, j2);
          else
            IW = BI.getValue(i2, j2);
          Nbpoint++;
          double er = (Tij - IW);
          for (unsigned int it = 0; it < nbParam; it++)
            dp[it] += er * pt->HiG[it];

          erreur += er * er;
        }
      }
    }
    if (Nbpoint == 0) {
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }
    dp = gain * dp;
    if (useBrent) {
      alpha = 2.;
      computeOptimalBrentGain(I, p, erreur / Nbpoint, dp, alpha);
      dp = alpha * dp;
    }
    Warp->getParamInverse(dp, dpinv);
    Warp->pRondp(p, dpinv, p);

    computeEvalRMS(p);

    if (iteration == 0) {
      evolRMS_init = evolRMS;
    }
    iteration++;

    evolRMS_delta = std::fabs(evolRMS - evolRMS_prec);
    evolRMS_prec = evolRMS;

  } while ((iteration < iterationMax) && (evolRMS_delta > std::fabs(evolRMS_init) * evolRMS_eps));
  nbIteration = iteration;
}
END_VISP_NAMESPACE
