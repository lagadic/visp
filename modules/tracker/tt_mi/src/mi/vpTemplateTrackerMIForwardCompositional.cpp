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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/tt_mi/vpTemplateTrackerMIForwardCompositional.h>

vpTemplateTrackerMIForwardCompositional::vpTemplateTrackerMIForwardCompositional(vpTemplateTrackerWarp *_warp)
  : vpTemplateTrackerMI(_warp), CompoInitialised(false)
{
}

void vpTemplateTrackerMIForwardCompositional::initCompo()
{
  std::cout << "Initialise precomputed value of Compositionnal Direct" << std::endl;
  ptTemplateSupp = new vpTemplateTrackerPointSuppMIInv[templateSize];
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);
    ptTemplate[point].dW = new double[2 * nbParam];
    Warp->getdWdp0(i, j, ptTemplate[point].dW);

    double Tij = ptTemplate[point].val;
    int ct = (int)((Tij * (Nc - 1)) / 255.);
    double et = (Tij * (Nc - 1)) / 255. - ct;
    ptTemplateSupp[point].et = et;
    ptTemplateSupp[point].ct = ct;
    ptTemplateSupp[point].Bt = new double[4];
    ptTemplateSupp[point].dBt = new double[4];
    for (char it = -1; it <= 2; it++) {
      ptTemplateSupp[point].Bt[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
      ptTemplateSupp[point].dBt[it + 1] = vpTemplateTrackerMIBSpline::dBspline4(-it + et);
    }
  }
  CompoInitialised = true;
}
void vpTemplateTrackerMIForwardCompositional::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompo();

  // std::cout<<"Initialise Hessian at Desired position..."<<std::endl;

  dW = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  // double erreur=0;
  int Nbpoint = 0;

  // double Tij;
  double IW, dx, dy;
  int cr, ct;
  double er, et;

  Nbpoint = 0;
  // erreur=0;

  zeroProbabilities();

  Warp->computeCoeff(p);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;

    Warp->computeDenom(X1, p);
    Warp->warpX(X1, X2, p);

    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;
      // Tij=ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      dx = 1. * dIx.getValue(i2, j2) * (Nc - 1) / 255.;
      dy = 1. * dIy.getValue(i2, j2) * (Nc - 1) / 255.;

      cr = ptTemplateSupp[point].ct;
      er = ptTemplateSupp[point].et;
      ct = (int)((IW * (Nc - 1)) / 255.);
      et = ((double)IW * (Nc - 1)) / 255. - ct;

      Warp->dWarpCompo(X1, X2, p, ptTemplate[point].dW, dW);

      double *tptemp = new double[nbParam];
      for (unsigned int it = 0; it < nbParam; it++)
        tptemp[it] = dW[0][it] * dx + dW[1][it] * dy;

      // calcul de l'erreur
      // erreur+=(Tij-IW)*(Tij-IW);

      vpTemplateTrackerMIBSpline::PutTotPVBspline(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);

      delete[] tptemp;
    }
  }
  double MI;
  computeProba(Nbpoint);
  computeMI(MI);
  computeHessien(Hdesire);

  lambda = lambdaDep;

  vpMatrix::computeHLM(Hdesire, lambda, HLMdesire);
  HLMdesireInverse = HLMdesire.inverseByLU();
  // std::cout<<"Hdesire = "<<Hdesire<<std::endl;
  // std::cout<<"\tEnd initialisation..."<<std::endl;
}

void vpTemplateTrackerMIForwardCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (!CompoInitialised)
    std::cout << "Compositionnal tracking no initialised\nUse "
                 "initCompo(vpImage<unsigned char> &I) function"
              << std::endl;
  dW = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  // double erreur=0;

  lambda = lambdaDep;
  double MI = 0, MIprec = -1000;

  MI_preEstimation = -getCost(I, p);

  double i2, j2;
  // double Tij;
  double IW;
  // unsigned
  int cr, ct;
  double er, et;
  double dx, dy;

  vpColVector dpinv(nbParam);
  double alpha = 2.;

  int i, j;
  unsigned int iteration = 0;
  do {
    int Nbpoint = 0;
    MIprec = MI;
    MI = 0;
    // erreur=0;

    zeroProbabilities();

    Warp->computeCoeff(p);

    for (unsigned int point = 0; point < templateSize; point++) {
      i = ptTemplate[point].y;
      j = ptTemplate[point].x;
      X1[0] = j;
      X1[1] = i;
      Warp->warpX(i, j, i2, j2, p);
      X2[0] = j2;
      X2[1] = i2;

      Warp->computeDenom(X1, p);
      if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
        Nbpoint++;
        // Tij=ptTemplate[point].val;
        if (!blur)
          IW = I.getValue(i2, j2);
        else
          IW = BI.getValue(i2, j2);

        dx = 1. * dIx.getValue(i2, j2) * (Nc - 1) / 255.;
        dy = 1. * dIy.getValue(i2, j2) * (Nc - 1) / 255.;

        ct = (int)((IW * (Nc - 1)) / 255.);
        et = ((double)IW * (Nc - 1)) / 255. - ct;
        cr = ptTemplateSupp[point].ct;
        er = ptTemplateSupp[point].et;

        Warp->dWarpCompo(X1, X2, p, ptTemplate[point].dW, dW);

        double *tptemp = new double[nbParam];
        for (unsigned int it = 0; it < nbParam; it++)
          tptemp[it] = dW[0][it] * dx + dW[1][it] * dy;

        // calcul de l'erreur
        // erreur+=(Tij-IW)*(Tij-IW);

        if (ApproxHessian == HESSIAN_NONSECOND || hessianComputation == vpTemplateTrackerMI::USE_HESSIEN_DESIRE)
          vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);
        else if (ApproxHessian == HESSIAN_0 || ApproxHessian == HESSIAN_NEW)
          vpTemplateTrackerMIBSpline::PutTotPVBspline(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);

        delete[] tptemp;
      }
    }
    if (Nbpoint == 0) {
      // std::cout<<"plus de point dans template suivi"<<std::endl;
      diverge = true;
      MI = 0;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    } else {
      computeProba(Nbpoint);
      computeMI(MI);
      if (hessianComputation != vpTemplateTrackerMI::USE_HESSIEN_DESIRE)
        computeHessien(H);
      computeGradient();

      vpMatrix::computeHLM(H, lambda, HLM);

      try {
        switch (hessianComputation) {
        case vpTemplateTrackerMI::USE_HESSIEN_DESIRE:
          dp = gain * HLMdesireInverse * G;
          break;
        case vpTemplateTrackerMI::USE_HESSIEN_BEST_COND:
          if (HLM.cond() > HLMdesire.cond())
            dp = gain * HLMdesireInverse * G;
          else
            dp = gain * 0.2 * HLM.inverseByLU() * G;
          break;
        default:
          dp = gain * 0.2 * HLM.inverseByLU() * G;
          break;
        }
      } catch (const vpException &e) {
        // std::cerr<<"probleme inversion"<<std::endl;
        throw(e);
      }
    }

    if (ApproxHessian == HESSIAN_NONSECOND)
      dp = -0.04 * dp;
    else
      dp = 1. * dp;

    if (useBrent) {
      alpha = 2.;
      computeOptimalBrentGain(I, p, -MI, dp, alpha);
      dp = alpha * dp;
    }
    Warp->pRondp(p, dp, p);

    iteration++;

  } while ((std::fabs(MI - MIprec) > std::fabs(MI) * std::numeric_limits<double>::epsilon()) &&
           (iteration < iterationMax));
  // while( (MI!=MIprec) && (iteration< iterationMax) );
  nbIteration = iteration;

  MI_postEstimation = -getCost(I, p);
  if (MI_preEstimation > MI_postEstimation) {
    MI_postEstimation = -1;
  }
}
