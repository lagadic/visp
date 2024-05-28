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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/

#include <visp3/tt_mi/vpTemplateTrackerMIESM.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
vpTemplateTrackerMIESM::vpTemplateTrackerMIESM(vpTemplateTrackerWarp *_warp)
  : vpTemplateTrackerMI(_warp), minimizationMethod(USE_NEWTON), CompoInitialised(false), HDirect(), HInverse(),
  HdesireDirect(), HdesireInverse(), GDirect(), GInverse()
{
  useCompositionnal = false;
  useInverse = false;
  if (!Warp->isESMcompatible()) {
    throw(vpException(vpException::badValue, "The selected warp function is not appropriate for the ESM algorithm..."));
  }
}

void vpTemplateTrackerMIESM::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompInverse();

  dW = 0;

  int Nbpoint = 0;

  double i2, j2;
  double IW, dx, dy;
  int i, j;
  int cr, ct;
  double er, et;

  Nbpoint = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);

  zeroProbabilities();

  vpColVector tptemp(nbParam);

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
      Nbpoint++;

      if (blur)
        IW = BI.getValue(i2, j2);
      else
        IW = I.getValue(i2, j2);

      ct = ptTemplateSupp[point].ct;
      et = ptTemplateSupp[point].et;
      cr = static_cast<int>((IW * (Nc - 1)) / 255.);
      er = (IW * (Nc - 1)) / 255. - cr;

      vpTemplateTrackerMIBSpline::computeProbabilities(PrtTout, cr, er, ct, et, Nc, ptTemplate[point].dW, nbParam,
                                                       bspline, ApproxHessian, false);
    }
  }

  double MI;
  computeProba(Nbpoint);
  computeMI(MI);
  computeHessien(HdesireInverse);

  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);
  if (ApproxHessian != HESSIAN_NONSECOND && ApproxHessian != HESSIAN_0 && ApproxHessian != HESSIAN_NEW &&
      ApproxHessian != HESSIAN_YOUCEF) {
    vpImageFilter::getGradX(dIx, d2Ix, fgdG, taillef);
    vpImageFilter::getGradY(dIx, d2Ixy, fgdG, taillef);
    vpImageFilter::getGradY(dIy, d2Iy, fgdG, taillef);
  }

  Nbpoint = 0;
  zeroProbabilities();

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

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight()) && (j2 < I.getWidth())) {
      Nbpoint++;

      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      dx = dIx.getValue(i2, j2) * (Nc - 1) / 255.;
      dy = dIy.getValue(i2, j2) * (Nc - 1) / 255.;

      cr = ptTemplateSupp[point].ct;
      er = ptTemplateSupp[point].et;
      ct = static_cast<int>((IW * (Nc - 1)) / 255.);
      et = (IW * (Nc - 1)) / 255. - ct;

      Warp->dWarpCompo(X1, X2, p, ptTemplateCompo[point].dW, dW);

      for (unsigned int it = 0; it < nbParam; it++)
        tptemp[it] = dW[0][it] * dx + dW[1][it] * dy;

      vpTemplateTrackerMIBSpline::computeProbabilities(PrtTout, cr, er, ct, et, Nc, tptemp.data, nbParam, bspline,
                                                       ApproxHessian, false);
    }
  }

  computeProba(Nbpoint);
  computeMI(MI);
  computeHessien(HdesireDirect);

  lambda = lambdaDep;

  Hdesire = HdesireDirect + HdesireInverse;

  vpMatrix::computeHLM(Hdesire, lambda, HLMdesire);

  HLMdesireInverse = HLMdesire.inverseByLU();
}

void vpTemplateTrackerMIESM::initCompInverse()
{
  HDirect.resize(nbParam, nbParam);
  HInverse.resize(nbParam, nbParam);
  HdesireDirect.resize(nbParam, nbParam);
  HdesireInverse.resize(nbParam, nbParam);
  GDirect.resize(nbParam);
  GInverse.resize(nbParam);

  ptTemplateSupp = new vpTemplateTrackerPointSuppMIInv[templateSize];
  ptTemplateCompo = new vpTemplateTrackerPointCompo[templateSize];
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);

    ptTemplateCompo[point].dW = new double[2 * nbParam];
    Warp->getdWdp0(i, j, ptTemplateCompo[point].dW);

    ptTemplate[point].dW = new double[nbParam];
    double dx = ptTemplate[point].dx * (Nc - 1) / 255.;
    double dy = ptTemplate[point].dy * (Nc - 1) / 255.;
    Warp->getdW0(i, j, dy, dx, ptTemplate[point].dW);

    double Tij = ptTemplate[point].val;
    int ct = static_cast<int>((Tij * (Nc - 1)) / 255.);
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

void vpTemplateTrackerMIESM::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (!CompoInitialised) {
    std::cout << "Compositionnal tracking not initialised.\nUse initCompInverse() function." << std::endl;
  }
  dW = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  int point;

  MI_preEstimation = -getCost(I, p);

  lambda = lambdaDep;

  double i2, j2;
  double IW;
  int cr, ct;
  double er, et;

  vpColVector dpinv(nbParam);

  double alpha = 2.;

  int i, j;
  unsigned int iteration = 0;
  vpColVector tptemp(nbParam);

  do {
    int Nbpoint = 0;
    double MI = 0;

    zeroProbabilities();

    /////////////////////////////////////////////////////////////////////////
    // Inverse
    Warp->computeCoeff(p);
    for (point = 0; point < static_cast<int>(templateSize); point++) {
      i = ptTemplate[point].y;
      j = ptTemplate[point].x;
      X1[0] = j;
      X1[1] = i;

      Warp->computeDenom(X1, p);
      Warp->warpX(X1, X2, p);

      j2 = X2[0];
      i2 = X2[1];

      if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
        Nbpoint++;

        if (!blur)
          IW = I.getValue(i2, j2);
        else
          IW = BI.getValue(i2, j2);

        ct = ptTemplateSupp[point].ct;
        et = ptTemplateSupp[point].et;
        cr = static_cast<int>((IW * (Nc - 1)) / 255.);
        er = (IW * (Nc - 1)) / 255. - cr;

        vpTemplateTrackerMIBSpline::computeProbabilities(PrtTout, cr, er, ct, et, Nc, ptTemplate[point].dW, nbParam,
                                                         bspline, ApproxHessian,
                                                         hessianComputation == USE_HESSIEN_DESIRE);
      }
    }

    if (Nbpoint == 0) {
      diverge = true;
      MI = 0;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }
    else {
      computeProba(Nbpoint);
      computeMI(MI);
      if (hessianComputation != vpTemplateTrackerMI::USE_HESSIEN_DESIRE) {
        computeHessien(HInverse);
      }
      computeGradient();
      GInverse = G;

      /////////////////////////////////////////////////////////////////////////
      // DIRECT

      Nbpoint = 0;
      MI = 0;

      zeroProbabilities();

      Warp->computeCoeff(p);
#ifdef VISP_HAVE_OPENMP
      int nthreads = omp_get_num_procs();
      // std::cout << "file: " __FILE__ << " line: " << __LINE__ << "
      // function: " << __FUNCTION__ << " nthread: " << nthreads << std::endl;
      omp_set_num_threads(nthreads);
#pragma omp parallel for private(i, j, i2, j2) default(shared)
#endif
      for (point = 0; point < static_cast<int>(templateSize); point++) {
        i = ptTemplate[point].y;
        j = ptTemplate[point].x;
        X1[0] = j;
        X1[1] = i;
        Warp->computeDenom(X1, p);
        Warp->warpX(i, j, i2, j2, p);
        X2[0] = j2;
        X2[1] = i2;

        if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
          Nbpoint++;

          if (!blur)
            IW = I.getValue(i2, j2);
          else
            IW = BI.getValue(i2, j2);

          double dx = dIx.getValue(i2, j2) * (Nc - 1) / 255.;
          double dy = dIy.getValue(i2, j2) * (Nc - 1) / 255.;

          ct = static_cast<int>((IW * (Nc - 1)) / 255.);
          et = (IW * (Nc - 1)) / 255. - ct;
          cr = ptTemplateSupp[point].ct;
          er = ptTemplateSupp[point].et;
          Warp->dWarpCompo(X1, X2, p, ptTemplateCompo[point].dW, dW);

          for (unsigned int it = 0; it < nbParam; it++)
            tptemp[it] = dW[0][it] * dx + dW[1][it] * dy;

          vpTemplateTrackerMIBSpline::computeProbabilities(PrtTout, cr, er, ct, et, Nc, tptemp.data, nbParam, bspline,
                                                           ApproxHessian, hessianComputation == USE_HESSIEN_DESIRE);
        }
      }

      computeProba(Nbpoint);
      computeMI(MI);
      if (hessianComputation != vpTemplateTrackerMI::USE_HESSIEN_DESIRE)
        computeHessien(HDirect);
      computeGradient();
      GDirect = G;

      if (hessianComputation != vpTemplateTrackerMI::USE_HESSIEN_DESIRE) {
        H = HDirect + HInverse;
        vpMatrix::computeHLM(H, lambda, HLM);
      }
      G = GDirect - GInverse;

      try {
        if (minimizationMethod == vpTemplateTrackerMIESM::USE_GRADIENT)
          dp = -gain * 0.3 * G;
        else {
          switch (hessianComputation) {
          case vpTemplateTrackerMI::USE_HESSIEN_DESIRE:
            dp = gain * HLMdesireInverse * G;
            break;
          case vpTemplateTrackerMI::USE_HESSIEN_BEST_COND:
            if (HLM.cond() > HLMdesire.cond()) {
              dp = gain * HLMdesireInverse * G;
            }
            else {
              dp = gain * 0.3 * HLM.inverseByLU() * G;
            }
            break;
          default:
            dp = gain * 0.3 * HLM.inverseByLU() * G;
            break;
          }
        }
      }
      catch (const vpException &e) {
        throw(e);
      }

      if (ApproxHessian == HESSIAN_NONSECOND)
        dp = -dp;

      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, -MI, dp, alpha);
        dp = alpha * dp;
      }
      p += dp;

      iteration++;
    }
  } while ((iteration < iterationMax));

  MI_postEstimation = -getCost(I, p);
  if (MI_preEstimation > MI_postEstimation) {
    MI_postEstimation = -1;
  }

  nbIteration = iteration;
}
END_VISP_NAMESPACE
