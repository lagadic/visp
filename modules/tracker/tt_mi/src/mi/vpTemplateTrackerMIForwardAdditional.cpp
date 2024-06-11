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

#include <visp3/tt_mi/vpTemplateTrackerMIForwardAdditional.h>

#ifdef VISP_HAVE_OPENMP
#include <omp.h>
#endif

BEGIN_VISP_NAMESPACE
vpTemplateTrackerMIForwardAdditional::vpTemplateTrackerMIForwardAdditional(vpTemplateTrackerWarp *_warp)
  : vpTemplateTrackerMI(_warp), minimizationMethod(USE_NEWTON), p_prec(), G_prec(), KQuasiNewton()
{
  useCompositionnal = false;
}

void vpTemplateTrackerMIForwardAdditional::initHessienDesired(const vpImage<unsigned char> &I)
{
  dW = 0;

  int Nbpoint = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  double Tij;
  double IW, dx, dy;
  int cr, ct;
  double er, et;

  Nbpoint = 0;

  zeroProbabilities();
  Warp->computeCoeff(p);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    X2[0] = j;
    X2[1] = i;

    Warp->computeDenom(X1, p);
    Warp->warpX(X1, X2, p);

    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;
      Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      dx = dIx.getValue(i2, j2) * (Nc - 1) / 255.;
      dy = dIy.getValue(i2, j2) * (Nc - 1) / 255.;

      ct = static_cast<int>((IW * (Nc - 1)) / 255.);
      cr = static_cast<int>((Tij * (Nc - 1)) / 255.);
      et = (IW * (Nc - 1)) / 255. - ct;
      er = (static_cast<double>(Tij) * (Nc - 1)) / 255. - cr;
      Warp->dWarp(X1, X2, p, dW);

      double *tptemp = new double[nbParam];
      for (unsigned int it = 0; it < nbParam; it++)
        tptemp[it] = dW[0][it] * dx + dW[1][it] * dy;

      if (ApproxHessian == HESSIAN_NONSECOND)
        vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);
      else if (ApproxHessian == HESSIAN_0 || ApproxHessian == HESSIAN_NEW)
        vpTemplateTrackerMIBSpline::PutTotPVBspline(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);

      delete[] tptemp;
    }
  }

  if (Nbpoint > 0) {
    double MI;
    computeProba(Nbpoint);
    computeMI(MI);
    computeHessien(Hdesire);

    vpMatrix::computeHLM(Hdesire, lambda, HLMdesire);
    try {
      HLMdesireInverse = HLMdesire.inverseByLU();
    }
    catch (const vpException &e) {
      throw(e);
    }
  }
}

void vpTemplateTrackerMIForwardAdditional::trackNoPyr(const vpImage<unsigned char> &I)
{
  dW = 0;

  int Nbpoint = 0;
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  double MI = 0, MIprec = -1000;

  MI_preEstimation = -getCost(I, p);

  double alpha = 2.;

  unsigned int iteration = 0;

  initPosEvalRMS(p);
  double evolRMS_init = 0;
  double evolRMS_prec = 0;
  double evolRMS_delta;

  do {
    if (iteration % 5 == 0)
      initHessienDesired(I);
    Nbpoint = 0;
    MIprec = MI;
    MI = 0;
    // erreur=0;

    zeroProbabilities();

    Warp->computeCoeff(p);
#ifdef VISP_HAVE_OPENMP
    int nthreads = omp_get_num_procs();
    // std::cout << "file: " __FILE__ << " line: " << __LINE__ << " function:
    // " << __FUNCTION__ << " nthread: " << nthreads << std::endl;
    omp_set_num_threads(nthreads);
#pragma omp parallel for default(shared)
#endif
    for (int point = 0; point < (int)templateSize; point++) {
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
        double Tij = ptTemplate[point].val;
        double IW;
        if (!blur)
          IW = I.getValue(i2, j2);
        else
          IW = BI.getValue(i2, j2);

        double dx = dIx.getValue(i2, j2) * (Nc - 1) / 255.;
        double dy = dIy.getValue(i2, j2) * (Nc - 1) / 255.;

        int ct = (int)((IW * (Nc - 1)) / 255.);
        int cr = (int)((Tij * (Nc - 1)) / 255.);
        double et = (IW * (Nc - 1)) / 255. - ct;
        double er = ((double)Tij * (Nc - 1)) / 255. - cr;

        Warp->dWarp(X1, X2, p, dW);

        double *tptemp = new double[nbParam];
        for (unsigned int it = 0; it < nbParam; it++)
          tptemp[it] = (dW[0][it] * dx + dW[1][it] * dy);
        if (ApproxHessian == HESSIAN_NONSECOND || hessianComputation == vpTemplateTrackerMI::USE_HESSIEN_DESIRE)
          vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);
        else if (ApproxHessian == HESSIAN_0 || ApproxHessian == HESSIAN_NEW)
          vpTemplateTrackerMIBSpline::PutTotPVBspline(PrtTout, cr, er, ct, et, Nc, tptemp, nbParam, bspline);

        delete[] tptemp;
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
      }
      catch (const vpException &e) {
        throw(e);
      }
    }

    switch (minimizationMethod) {
    case vpTemplateTrackerMIForwardAdditional::USE_LMA: {
      vpColVector p_test_LMA(nbParam);
      if (ApproxHessian == HESSIAN_NONSECOND)
        p_test_LMA = p - 100000.1 * dp;
      else
        p_test_LMA = p + dp;
      MI = -getCost(I, p);
      double MI_LMA = -getCost(I, p_test_LMA);
      if (MI_LMA > MI) {
        p = p_test_LMA;
        lambda = (lambda / 10. < 1e-6) ? lambda / 10. : 1e-6;
      }
      else {
        lambda = (lambda * 10. < 1e6) ? 1e6 : lambda * 10.;
      }
    } break;
    case vpTemplateTrackerMIForwardAdditional::USE_GRADIENT: {
      dp = -gain * 6.0 * G;
      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, -MI, dp, alpha);
        dp = alpha * dp;
      }
      p += dp;
      break;
    }

    case vpTemplateTrackerMIForwardAdditional::USE_QUASINEWTON: {
      if (iterationGlobale != 0) {
        vpColVector s_quasi = p - p_prec;
        vpColVector y_quasi = G - G_prec;
        double s_scal_y = s_quasi.t() * y_quasi;
        if (std::fabs(s_scal_y) > std::numeric_limits<double>::epsilon()) {
          KQuasiNewton = KQuasiNewton + 0.001 * (s_quasi * s_quasi.t() / s_scal_y -
                                                 KQuasiNewton * y_quasi * y_quasi.t() * KQuasiNewton /
                                                     (y_quasi.t() * KQuasiNewton * y_quasi));
        }
      }
      dp = -KQuasiNewton * G;
      p_prec = p;
      G_prec = G;
      p -= 1.01 * dp;
    } break;

    default: {
      if (ApproxHessian == HESSIAN_NONSECOND)
        dp = -0.1 * dp;
      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, -MI, dp, alpha);
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

  } while ((std::fabs(MI - MIprec) > std::fabs(MI) * std::numeric_limits<double>::epsilon()) &&
           (iteration < iterationMax) && (evolRMS_delta > std::fabs(evolRMS_init) * evolRMS_eps));

  if (Nbpoint == 0) {
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
  }

  nbIteration = iteration;
  MI_postEstimation = -getCost(I, p);
  if (MI_preEstimation > MI_postEstimation) {
    MI_postEstimation = -1;
  }
}
END_VISP_NAMESPACE
