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
#include <visp3/core/vpTrackingException.h>
#include <visp3/tt_mi/vpTemplateTrackerMIInverseCompositional.h>

#include <memory>

BEGIN_VISP_NAMESPACE
vpTemplateTrackerMIInverseCompositional::vpTemplateTrackerMIInverseCompositional(vpTemplateTrackerWarp *_warp)
  : vpTemplateTrackerMI(_warp), minimizationMethod(USE_LMA), CompoInitialised(false), useTemplateSelect(false),
  p_prec(), G_prec(), KQuasiNewton()
{
  useInverse = true;
}

void vpTemplateTrackerMIInverseCompositional::initTemplateRefBspline(unsigned int ptIndex, double &et) // AY : Optim
{
  ptTemplateSupp[ptIndex].BtInit = new double[(1 + nbParam + nbParam * nbParam) * static_cast<unsigned int>(bspline)];

  unsigned int index = 0;
  int endIndex = 1;

  double (*ptBspFct)(double);
  double (*ptdBspFct)(double);
  double (*ptd2BspFct)(double);
  if (bspline == 3) {
    if (et > 0.5) {
      et = et - 1;
    }
    ptBspFct = &vpTemplateTrackerMIBSpline::Bspline3;
    ptdBspFct = &vpTemplateTrackerMIBSpline::dBspline3;
    ptd2BspFct = &vpTemplateTrackerMIBSpline::d2Bspline3;
  }
  else {
    ptBspFct = &vpTemplateTrackerBSpline::Bspline4;
    ptdBspFct = &vpTemplateTrackerMIBSpline::dBspline4;
    ptd2BspFct = &vpTemplateTrackerMIBSpline::d2Bspline4;
    endIndex = 2;
  }

  for (int it = -1; it <= endIndex; it++) {
    ptTemplateSupp[ptIndex].BtInit[index++] = (*ptBspFct)(static_cast<double>(-it) + et);

    for (unsigned int ip = 0; ip < nbParam; ++ip) {
      ptTemplateSupp[ptIndex].BtInit[index++] =
        (*ptdBspFct)(static_cast<double>(-it) + et) * ptTemplate[ptIndex].dW[ip] * (-1.0);
      for (unsigned int ip2 = 0; ip2 < nbParam; ++ip2) {
        ptTemplateSupp[ptIndex].BtInit[index++] =
          (*ptd2BspFct)(static_cast<double>(-it) + et) * ptTemplate[ptIndex].dW[ip] * ptTemplate[ptIndex].dW[ip2];
      }
    }
  }
}

void vpTemplateTrackerMIInverseCompositional::initCompInverse(const vpImage<unsigned char> &I)
{
  ptTemplateSupp = new vpTemplateTrackerPointSuppMIInv[templateSize];

  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  if (ApproxHessian != HESSIAN_NONSECOND && ApproxHessian != HESSIAN_0 && ApproxHessian != HESSIAN_NEW &&
      ApproxHessian != HESSIAN_YOUCEF) {
    vpImageFilter::getGradX(dIx, d2Ix, fgdG, taillef);
    vpImageFilter::getGradY(dIx, d2Ixy, fgdG, taillef);
    vpImageFilter::getGradY(dIy, d2Iy, fgdG, taillef);
  }
  double Nc_255_ = (Nc - 1) / 255.;
  Warp->computeCoeff(p);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;

    ptTemplate[point].dW = new double[nbParam];

    double dx = ptTemplate[point].dx * Nc_255_;
    double dy = ptTemplate[point].dy * Nc_255_;

    Warp->getdW0(i, j, dy, dx, ptTemplate[point].dW);
    double Tij = ptTemplate[point].val;
    int ct = static_cast<int>(Tij * Nc_255_);
    double et = (Tij * Nc_255_) - ct;
    ptTemplateSupp[point].et = et;
    ptTemplateSupp[point].ct = ct;
  }
  CompoInitialised = true;
}

void vpTemplateTrackerMIInverseCompositional::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompInverse(I);

  int Nbpoint = 0;

  double IW;
  int cr, ct;
  double er, et;

  Nbpoint = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);

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

      if (blur)
        IW = BI.getValue(i2, j2);
      else
        IW = I.getValue(i2, j2);

      ct = ptTemplateSupp[point].ct;
      et = ptTemplateSupp[point].et;
      cr = static_cast<int>((IW * (Nc - 1)) / 255.);
      er = (IW * (Nc - 1)) / 255. - cr;

      if (ApproxHessian == HESSIAN_NONSECOND && (ptTemplateSelect[point] || !useTemplateSelect)) {
        vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(PrtTout, cr, er, ct, et, Nc, ptTemplate[point].dW, nbParam,
                                                            bspline);
      }
      else if ((ApproxHessian == HESSIAN_0 || ApproxHessian == HESSIAN_NEW) &&
              (ptTemplateSelect[point] || !useTemplateSelect)) {
        if (bspline == 3) {
          vpTemplateTrackerMIBSpline::PutTotPVBspline3(PrtTout, cr, er, ct, et, Nc, ptTemplate[point].dW, nbParam);
        }
        else {
          vpTemplateTrackerMIBSpline::PutTotPVBspline4(PrtTout, cr, er, ct, et, Nc, ptTemplate[point].dW, nbParam);
        }
      }
      else if (ptTemplateSelect[point] || !useTemplateSelect)
        vpTemplateTrackerMIBSpline::PutTotPVBsplinePrt(PrtTout, cr, er, ct, et, Nc, nbParam, bspline);
    }
  }

  double MI;
  computeProba(Nbpoint);
  computeMI(MI);
  computeHessien(Hdesire);

  lambda = lambdaDep;

  vpMatrix::computeHLM(Hdesire, lambda, HLMdesire);

  HLMdesireInverse = HLMdesire.inverseByLU();
  KQuasiNewton = HLMdesireInverse;
}

void vpTemplateTrackerMIInverseCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (!CompoInitialised) {
    std::cout << "Compositionnal tracking not initialised.\nUse initCompInverse() function." << std::endl;
  }

  dW = 0;

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);

  lambda = lambdaDep;
  double MI = 0, MIprec = -1000;

  vpColVector p_avant_estimation;
  p_avant_estimation = p;
  MI_preEstimation = -getCost(I, p);
  NMI_preEstimation = -getNormalizedCost(I, p);

  initPosEvalRMS(p);

  vpColVector dpinv(nbParam);
  double alpha = 2.;

  unsigned int iteration = 0;

  vpMatrix Hnorm(nbParam, nbParam);
  double evolRMS_init = 0;
  double evolRMS_prec = 0;
  double evolRMS_delta;

  vpColVector dp_test_LMA(nbParam);
  vpColVector dpinv_test_LMA(nbParam);
  vpColVector p_test_LMA(nbParam);

  do {
    int Nbpoint = 0;
    MIprec = MI;
    MI = 0;

    zeroProbabilities();

    Warp->computeCoeff(p);

    for (int point = 0; point < static_cast<int>(templateSize); point++) {
      X1[0] = static_cast<double>(ptTemplate[point].x);
      X1[1] = static_cast<double>(ptTemplate[point].y);

      Warp->computeDenom(X1, p);
      Warp->warpX(X1, X2, p);

      double j2 = X2[0];
      double i2 = X2[1];

      if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {

        Nbpoint++;
        double IW;
        if (!blur)
          IW = static_cast<double>(I.getValue(i2, j2));
        else
          IW = BI.getValue(i2, j2);

        int ct = ptTemplateSupp[point].ct;
        double et = ptTemplateSupp[point].et;
        double tmp = IW * (static_cast<double>(Nc) - 1.) / 255.;
        int cr = static_cast<int>(tmp);
        double er = tmp - static_cast<double>(cr);

        if ((ApproxHessian == HESSIAN_NONSECOND || hessianComputation == vpTemplateTrackerMI::USE_HESSIEN_DESIRE) &&
            (ptTemplateSelect[point] || !useTemplateSelect)) {
          vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(Prt, dPrt, cr, er, ct, et, Ncb, ptTemplate[point].dW,
                                                              nbParam, bspline);
        }
        else if (ptTemplateSelect[point] || !useTemplateSelect) {
          if (bspline == 3) {
            vpTemplateTrackerMIBSpline::PutTotPVBspline3(Prt, dPrt, d2Prt, cr, er, ct, et, Ncb, ptTemplate[point].dW,
                                                         nbParam);
          }
          else {
            vpTemplateTrackerMIBSpline::PutTotPVBspline4(Prt, dPrt, d2Prt, cr, er, ct, et, Ncb, ptTemplate[point].dW,
                                                         nbParam);
          }
        }
        else {
          vpTemplateTrackerMIBSpline::PutTotPVBsplinePrt(Prt, cr, er, ct, et, Ncb, nbParam, bspline);
        }
      }
    }

    if (Nbpoint == 0) {
      diverge = true;
      MI = 0;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));

    }
    else {
      unsigned int indd, indd2;
      indd = indd2 = 0;
      unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
      for (unsigned int i = 0; i < Ncb_ * Ncb_; i++) {
        Prt[i] /= Nbpoint;
        for (unsigned int j = 0; j < nbParam; j++) {
          dPrt[indd] /= Nbpoint;
          indd++;
          for (unsigned int k = 0; k < nbParam; k++) {
            d2Prt[indd2] /= Nbpoint;
            indd2++;
          }
        }
      }

      computeMI(MI);

      if (hessianComputation != vpTemplateTrackerMI::USE_HESSIEN_DESIRE) {
        computeHessienNormalized(Hnorm);
        computeHessien(H);
      }
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
    case vpTemplateTrackerMIInverseCompositional::USE_LMA: {
      if (ApproxHessian == HESSIAN_NONSECOND)
        dp_test_LMA = -100000.1 * dp;
      else
        dp_test_LMA = dp;
      Warp->getParamInverse(dp_test_LMA, dpinv_test_LMA);
      Warp->pRondp(p, dpinv_test_LMA, p_test_LMA);

      MI = -getCost(I, p);
      double MI_LMA = -getCost(I, p_test_LMA);
      if (MI_LMA > MI) {
        dp = dp_test_LMA;
        lambda = (lambda / 10. < 1e-6) ? lambda / 10. : 1e-6;
      }
      else {
        dp = 0;
        lambda = (lambda * 10. < 1e6) ? 1e6 : lambda * 10.;
      }
    } break;
    case vpTemplateTrackerMIInverseCompositional::USE_GRADIENT:
      dp = -gain * 0.3 * G * 20;
      break;

    case vpTemplateTrackerMIInverseCompositional::USE_QUASINEWTON: {
      if (iterationGlobale != 0) {
        vpColVector s_quasi = p - p_prec;
        vpColVector y_quasi = G - G_prec;
        double s_scal_y = s_quasi.t() * y_quasi;
        if (std::fabs(s_scal_y) > std::numeric_limits<double>::epsilon()) {
          KQuasiNewton = KQuasiNewton + 0.0001 * (s_quasi * s_quasi.t() / s_scal_y -
                                                  KQuasiNewton * y_quasi * y_quasi.t() * KQuasiNewton /
                                                      (y_quasi.t() * KQuasiNewton * y_quasi));
        }
      }
      dp = gain * KQuasiNewton * G;
      p_prec = p;
      G_prec = G;
    } break;

    default: {
      if (useBrent) {
        alpha = 2.;
        computeOptimalBrentGain(I, p, -MI, dp, alpha);
        dp = alpha * dp;
      }
      if (ApproxHessian == HESSIAN_NONSECOND)
        dp = -dp;

      break;
    }
    }

    Warp->getParamInverse(dp, dpinv);
    Warp->pRondp(p, dpinv, p);
    computeEvalRMS(p);

    if (iteration == 0) {
      evolRMS_init = evolRMS;
    }
    iteration++;
    iterationGlobale++;

    evolRMS_delta = std::fabs(evolRMS - evolRMS_prec);
    evolRMS_prec = evolRMS;

  } while ((!diverge) && (std::fabs(MI - MIprec) > std::fabs(MI) * std::numeric_limits<double>::epsilon()) &&
           (iteration < iterationMax) && (evolRMS_delta > std::fabs(evolRMS_init) * evolRMS_eps));

  nbIteration = iteration;

  if (diverge) {
    if (computeCovariance) {
      covarianceMatrix = vpMatrix(Warp->getNbParam(), Warp->getNbParam());
      covarianceMatrix = -1;
      MI_postEstimation = -1;
      NMI_postEstimation = -1;
    }
  }
  else {
    MI_postEstimation = -getCost(I, p);
    NMI_postEstimation = -getNormalizedCost(I, p);

    if (MI_preEstimation > MI_postEstimation) {
      p = p_avant_estimation;
      MI_postEstimation = MI_preEstimation;
      NMI_postEstimation = NMI_preEstimation;
      covarianceMatrix = vpMatrix(Warp->getNbParam(), Warp->getNbParam());
      covarianceMatrix = -1;
    }

    if (computeCovariance) {
      try {
        covarianceMatrix = (-H).inverseByLU();
      }
      catch (...) {
        covarianceMatrix = vpMatrix(Warp->getNbParam(), Warp->getNbParam());
        covarianceMatrix = -1;
        MI_postEstimation = -1;
        NMI_postEstimation = -1;
      }
    }
  }
}
END_VISP_NAMESPACE
