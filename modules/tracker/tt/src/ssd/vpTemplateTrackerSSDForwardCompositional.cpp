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
#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardCompositional.h>

vpTemplateTrackerSSDForwardCompositional::vpTemplateTrackerSSDForwardCompositional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), compoInitialised(false)
{
}

void vpTemplateTrackerSSDForwardCompositional::initCompo(const vpImage<unsigned char> & /*I*/)
{
  // std::cout<<"Initialise precomputed value of Compositionnal
  // Direct"<<std::endl;
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    ptTemplate[point].dW = new double[2 * nbParam];
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);
    Warp->getdWdp0(i, j, ptTemplate[point].dW);
  }
  compoInitialised = true;
}

void vpTemplateTrackerSSDForwardCompositional::initHessienDesired(const vpImage<unsigned char> &I) { initCompo(I); }

void vpTemplateTrackerSSDForwardCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (!compoInitialised)
    std::cout << "Compositionnal tracking no initialised\nUse "
                 "InitCompo(vpImage<unsigned char> &I) function"
              << std::endl;

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
        /*Warp->dWarp(X1,X2,p,dW);
        double *tempt=new double[nbParam];
        for(int it=0;it<nbParam;it++)
        tempt[it]=dW[0][it]*dIWx+dW[1][it]*dIWy;*/

        Warp->dWarpCompo(X1, X2, p, ptTemplate[point].dW, dW);

        double *tempt = new double[nbParam];
        for (unsigned int it = 0; it < nbParam; it++)
          tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

        for (unsigned int it = 0; it < nbParam; it++)
          for (unsigned int jt = 0; jt < nbParam; jt++)
            H[it][jt] += tempt[it] * tempt[jt];

        double er = (Tij - IW);
        for (unsigned int it = 0; it < nbParam; it++)
          G[it] += er * tempt[it];

        erreur += (er * er);
        delete[] tempt;
      }
    }
    if (Nbpoint == 0) {
      // std::cout<<"plus de point dans template suivi"<<std::endl;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }

    vpMatrix::computeHLM(H, lambda, HLM);

    try {
      dp = 1. * HLM.inverseByLU() * G;
    } catch (const vpException &e) {
      // std::cout<<"probleme inversion"<<std::endl;
      throw(e);
    }

    dp = gain * dp;
    if (useBrent) {
      alpha = 2.;
      computeOptimalBrentGain(I, p, erreur / Nbpoint, dp, alpha);
      dp = alpha * dp;
    }
    Warp->pRondp(p, dp, p);
    // p+=Gain*dp;
    iteration++;
  } while (/*( erreur_prec-erreur<50) &&*/ (iteration < iterationMax));

  nbIteration = iteration;
}
