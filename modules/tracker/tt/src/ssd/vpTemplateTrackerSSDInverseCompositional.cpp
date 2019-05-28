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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/core/vpImageTools.h>
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>

vpTemplateTrackerSSDInverseCompositional::vpTemplateTrackerSSDInverseCompositional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), compoInitialised(false), HInv(), HCompInverse(), useTemplateSelect(false), evolRMS(0),
    x_pos(), y_pos(), threshold_RMS(1e-8)
{
  useInverse = true;
  HInv.resize(nbParam, nbParam);
  HCompInverse.resize(nbParam, nbParam);
}

void vpTemplateTrackerSSDInverseCompositional::initCompInverse(const vpImage<unsigned char> & /*I*/)
{

  H = 0;
  int i, j;

  for (unsigned int point = 0; point < templateSize; point++) {
    if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
      i = ptTemplate[point].y;
      j = ptTemplate[point].x;
      X1[0] = j;
      X1[1] = i;
      Warp->computeDenom(X1, p);
      ptTemplate[point].dW = new double[nbParam];

      Warp->getdW0(i, j, ptTemplate[point].dy, ptTemplate[point].dx, ptTemplate[point].dW);

      for (unsigned int it = 0; it < nbParam; it++)
        for (unsigned int jt = 0; jt < nbParam; jt++)
          H[it][jt] += ptTemplate[point].dW[it] * ptTemplate[point].dW[jt];
    }
  }
  HInv = H;
  vpMatrix HLMtemp(nbParam, nbParam);
  vpMatrix::computeHLM(H, lambdaDep, HLMtemp);

  HCompInverse.resize(nbParam, nbParam);
  HCompInverse = HLMtemp.inverseByLU();
  // std::cout<<Hinverse<<std::endl;
  vpColVector dWtemp(nbParam);
  vpColVector HiGtemp(nbParam);

  for (unsigned int point = 0; point < templateSize; point++) {
    if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
      // i=ptTemplate[point].y;
      // j=ptTemplate[point].x;
      for (unsigned int it = 0; it < nbParam; it++)
        dWtemp[it] = ptTemplate[point].dW[it];

      HiGtemp = -1. * HCompInverse * dWtemp;
      ptTemplate[point].HiG = new double[nbParam];

      for (unsigned int it = 0; it < nbParam; it++)
        ptTemplate[point].HiG[it] = HiGtemp[it];
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
  // vpTemplateTrackerPointtest *pt;
  initPosEvalRMS(p);

  vpTemplateTrackerPoint *pt;
  do {
    unsigned int Nbpoint = 0;
    double erreur = 0;
    dp = 0;
    Warp->computeCoeff(p);
    for (unsigned int point = 0; point < templateSize; point++) {
      if ((!useTemplateSelect) || (ptTemplateSelect[point])) {
        // pt=&ptTemplatetest[point];
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
    // std::cout << "npoint: " << Nbpoint << std::endl;
    if (Nbpoint == 0) {
      // std::cout<<"plus de point dans template suivi"<<std::endl;
      deletePosEvalRMS();
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }
    dp = gain * dp;
    // std::cout<<erreur/Nbpoint<<","<<GetCost(I,p)<<std::endl;
    if (useBrent) {
      alpha = 2.;
      computeOptimalBrentGain(I, p, erreur / Nbpoint, dp, alpha);
      dp = alpha * dp;
    }
    Warp->getParamInverse(dp, dpinv);
    Warp->pRondp(p, dpinv, p);
    iteration++;

    computeEvalRMS(p);
    // std::cout << "iteration: " << iteration << " max: " << iterationMax <<
    // std::endl;  std::cout << "evolRMS: " <<  evolRMS << " threshold: " <<
    // threshold_RMS << std::endl;
  } while (/*( erreur_prec-erreur<50) &&*/ (iteration < iterationMax) && (evolRMS > threshold_RMS));

  nbIteration = iteration;
  deletePosEvalRMS();
}

void vpTemplateTrackerSSDInverseCompositional::initPosEvalRMS(const vpColVector &p_)
{
  unsigned int nb_corners = zoneTracked->getNbTriangle() * 3;
  x_pos.resize(nb_corners);
  y_pos.resize(nb_corners);

  Warp->computeCoeff(p_);
  vpTemplateTrackerTriangle triangle;

  for (unsigned int i = 0; i < zoneTracked->getNbTriangle(); i++) {
    zoneTracked->getTriangle(i, triangle);
    for (unsigned int j = 0; j < 3; j++) {
      triangle.getCorner(j, X1[0], X1[1]);

      Warp->computeDenom(X1, p_);
      Warp->warpX(X1, X2, p_);
      x_pos[i * 3 + j] = X2[0];
      y_pos[i * 3 + j] = X2[1];
    }
  }
}

void vpTemplateTrackerSSDInverseCompositional::computeEvalRMS(const vpColVector &p_)
{
  unsigned int nb_corners = zoneTracked->getNbTriangle() * 3;

  Warp->computeCoeff(p_);
  evolRMS = 0;
  vpTemplateTrackerTriangle triangle;

  for (unsigned int i = 0; i < zoneTracked->getNbTriangle(); i++) {
    zoneTracked->getTriangle(i, triangle);
    for (unsigned int j = 0; j < 3; j++) {
      triangle.getCorner(j, X1[0], X1[1]);

      Warp->computeDenom(X1, p_);
      Warp->warpX(X1, X2, p_);
      evolRMS += (x_pos[i * 3 + j] - X2[0]) * (x_pos[i * 3 + j] - X2[0]) +
                 (y_pos[i * 3 + j] - X2[1]) * (y_pos[i * 3 + j] - X2[1]);
      x_pos[i * 3 + j] = X2[0];
      y_pos[i * 3 + j] = X2[1];
    }
  }
  evolRMS = evolRMS / nb_corners;
}

void vpTemplateTrackerSSDInverseCompositional::deletePosEvalRMS() {}
