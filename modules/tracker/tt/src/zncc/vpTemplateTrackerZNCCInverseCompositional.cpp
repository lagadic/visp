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
#include <limits> // numeric_limits

#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTrackerZNCCInverseCompositional.h>

vpTemplateTrackerZNCCInverseCompositional::vpTemplateTrackerZNCCInverseCompositional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerZNCC(warp), compoInitialised(false), evolRMS(0), x_pos(), y_pos(), threshold_RMS(1e-8),
    moydIrefdp()
{
  useInverse = true;
}

void vpTemplateTrackerZNCCInverseCompositional::initCompInverse(const vpImage<unsigned char> &I)
{
  // std::cout<<"Initialise precomputed value of Compositionnal
  // Inverse"<<std::endl;
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;

    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);
    ptTemplate[point].dW = new double[nbParam];

    double dx = ptTemplate[point].dx;
    double dy = ptTemplate[point].dy;
    // std::cout<<ptTemplate[point].dx<<","<<ptTemplate[point].dy<<std::endl;

    Warp->getdW0(i, j, dy, dx, ptTemplate[point].dW);
  }
  // vpTRACE("fin Comp Inverse");
  compoInitialised = true;
}

void vpTemplateTrackerZNCCInverseCompositional::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompInverse(I);

  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  vpImage<double> dIxx, dIxy, dIyx, dIyy;
  vpImageFilter::getGradX(dIx, dIxx, fgdG, taillef);
  vpImageFilter::getGradY(dIx, dIxy, fgdG, taillef);

  vpImageFilter::getGradX(dIy, dIyx, fgdG, taillef);
  vpImageFilter::getGradY(dIy, dIyy, fgdG, taillef);

  Warp->computeCoeff(p);
  double Ic, dIcx = 0., dIcy = 0.;
  double Iref;
  int i, j;
  double i2, j2;
  int Nbpoint = 0;

  double moyIref = 0;
  double moyIc = 0;
  double denom = 0;
  moydIrefdp.resize(nbParam);
  moydIrefdp = 0;
  vpMatrix moyd2Iref(nbParam, nbParam);
  moyd2Iref = 0;

  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    X2[0] = j;
    X2[1] = i;

    Warp->computeDenom(X1, p);

    j2 = X2[0];
    i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Iref = ptTemplate[point].val;

      if (!blur)
        Ic = I.getValue(i2, j2);
      else
        Ic = BI.getValue(i2, j2);

      Nbpoint++;
      moyIref += Iref;
      moyIc += Ic;

      for (unsigned int it = 0; it < nbParam; it++)
        moydIrefdp[it] += ptTemplate[point].dW[it];

      Warp->dWarp(X1, X2, p, dW);
      double *tempt = new double[nbParam];
      for (unsigned int it = 0; it < nbParam; it++)
        tempt[it] = dW[0][it] * dIcx + dW[1][it] * dIcy;
      double d_Ixx = dIxx.getValue(i2, j2);
      double d_Iyy = dIyy.getValue(i2, j2);
      double d_Ixy = dIxy.getValue(i2, j2);

      for (unsigned int it = 0; it < nbParam; it++)
        for (unsigned int jt = 0; jt < nbParam; jt++) {
          moyd2Iref[it][jt] += (dW[0][it] * (dW[0][jt] * d_Ixx + dW[1][jt] * d_Ixy) +
                                dW[1][it] * (dW[0][jt] * d_Ixy + dW[1][jt] * d_Iyy));
        }

      delete[] tempt;
    }
  }

  moyIref = moyIref / Nbpoint;
  moydIrefdp = moydIrefdp / Nbpoint;
  moyd2Iref = moyd2Iref / Nbpoint;
  moyIc = moyIc / Nbpoint;
  Hdesire = 0;
  double covarIref = 0, covarIc = 0;
  double sIcIref = 0;
  vpColVector sIcdIref(nbParam);
  sIcdIref = 0;
  vpMatrix sIcd2Iref(nbParam, nbParam);
  sIcd2Iref = 0;
  vpMatrix sdIrefdIref(nbParam, nbParam);
  sdIrefdIref = 0;
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    X2[0] = j;
    X2[1] = i;

    Warp->computeDenom(X1, p);

    j2 = X2[0];
    i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Iref = ptTemplate[point].val;

      if (!blur)
        Ic = I.getValue(i2, j2);
      else
        Ic = BI.getValue(i2, j2);

      dIcx = dIx.getValue(i2, j2);
      dIcy = dIy.getValue(i2, j2);

      Warp->dWarp(X1, X2, p, dW);

      double *tempt = new double[nbParam];
      for (unsigned int it = 0; it < nbParam; it++)
        tempt[it] = dW[0][it] * dIcx + dW[1][it] * dIcy;

      double prodIc = (Ic - moyIc);

      double d_Ixx = dIxx.getValue(i2, j2);
      double d_Iyy = dIyy.getValue(i2, j2);
      double d_Ixy = dIxy.getValue(i2, j2);

      for (unsigned int it = 0; it < nbParam; it++)
        for (unsigned int jt = 0; jt < nbParam; jt++) {
          sIcd2Iref[it][jt] += prodIc * (dW[0][it] * (dW[0][jt] * d_Ixx + dW[1][jt] * d_Ixy) +
                                         dW[1][it] * (dW[0][jt] * d_Ixy + dW[1][jt] * d_Iyy) - moyd2Iref[it][jt]);
          sdIrefdIref[it][jt] +=
              (ptTemplate[point].dW[it] - moydIrefdp[it]) * (ptTemplate[point].dW[jt] - moydIrefdp[jt]);
        }

      delete[] tempt;

      for (unsigned int it = 0; it < nbParam; it++)
        sIcdIref[it] += prodIc * (ptTemplate[point].dW[it] - moydIrefdp[it]);

      covarIref += (Iref - moyIref) * (Iref - moyIref);
      covarIc += (Ic - moyIc) * (Ic - moyIc);
      sIcIref += (Iref - moyIref) * (Ic - moyIc);
    }
  }
  covarIref = sqrt(covarIref);
  covarIc = sqrt(covarIc);

  denom = covarIref * covarIc;

  double NCC = sIcIref / denom;
  // std::cout<<"NCC = "<<NCC<<std::endl;
  vpColVector dcovarIref(nbParam);
  dcovarIref = -sIcdIref / covarIref;

  vpColVector dNCC(nbParam);
  dNCC = (sIcdIref / denom - NCC * dcovarIref / covarIref);
  vpMatrix d2covarIref(nbParam, nbParam);
  d2covarIref = -(sIcd2Iref - sdIrefdIref + dcovarIref * dcovarIref.t()) / covarIref;
#ifdef APPROX_NCC
  Hdesire = sIcd2Iref / denom;
#else
  Hdesire = (sIcd2Iref - sdIrefdIref + dcovarIref * dcovarIref.t()) / denom;
#endif
  vpMatrix::computeHLM(Hdesire, lambdaDep, HLMdesire);
  HLMdesireInverse = HLMdesire.inverseByLU();
  // std::cout<<"Hdesire = "<<Hdesire<<std::endl;
}

void vpTemplateTrackerZNCCInverseCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);

  // double erreur=0;
  vpColVector dpinv(nbParam);
  double Ic;
  double Iref;
  unsigned int iteration = 0;
  int i, j;
  double i2, j2;
  initPosEvalRMS(p);
  do {
    unsigned int Nbpoint = 0;
    // erreur=0;
    G = 0;
    Warp->computeCoeff(p);
    double moyIref = 0;
    double moyIc = 0;
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
        Iref = ptTemplate[point].val;

        if (!blur)
          Ic = I.getValue(i2, j2);
        else
          Ic = BI.getValue(i2, j2);

        Nbpoint++;
        moyIref += Iref;
        moyIc += Ic;
      }
    }
    if (Nbpoint > 0) {
      moyIref = moyIref / Nbpoint;
      moyIc = moyIc / Nbpoint;
      double sIcIref = 0;
      double covarIref = 0, covarIc = 0;
      vpColVector sIcdIref(nbParam);
      sIcdIref = 0;
      vpColVector sIrefdIref(nbParam);
      sIrefdIref = 0;

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
          Iref = ptTemplate[point].val;

          if (!blur)
            Ic = I.getValue(i2, j2);
          else
            Ic = BI.getValue(i2, j2);

          double prod = (Ic - moyIc);
          for (unsigned int it = 0; it < nbParam; it++)
            sIcdIref[it] += prod * (ptTemplate[point].dW[it] - moydIrefdp[it]);
          for (unsigned int it = 0; it < nbParam; it++)
            sIrefdIref[it] += (Iref - moyIref) * (ptTemplate[point].dW[it] - moydIrefdp[it]);

          // double er=(Iref-Ic);
          // erreur+=(er*er);
          // denom+=(Iref-moyIref)*(Iref-moyIref)*(Ic-moyIc)*(Ic-moyIc);
          covarIref += (Iref - moyIref) * (Iref - moyIref);
          covarIc += (Ic - moyIc) * (Ic - moyIc);
          sIcIref += (Iref - moyIref) * (Ic - moyIc);
        }
      }
      covarIref = sqrt(covarIref);
      covarIc = sqrt(covarIc);
      double denom = covarIref * covarIc;

      // if(denom==0.0)
      if (std::fabs(denom) <= std::numeric_limits<double>::epsilon()) {
        diverge = true;
      } else {
        double NCC = sIcIref / denom;
        vpColVector dcovarIref(nbParam);
        dcovarIref = sIrefdIref / covarIref;
        G = 1. * (sIcdIref / denom - NCC * dcovarIref / covarIref);

        try {
          dp = -1. * HLMdesireInverse * G;
        } catch (...) {
          std::cout << "probleme inversion" << std::endl;
          break;
        }

        Warp->getParamInverse(dp, dpinv);
        Warp->pRondp(p, dpinv, p);

        computeEvalRMS(p);
      }
    } else
      diverge = true;

    iteration++;
  } while ((!diverge && (evolRMS > threshold_RMS) && (iteration < iterationMax)));

  // std::cout<<"erreur "<<erreur<<std::endl;
  nbIteration = iteration;

  deletePosEvalRMS();
}

void vpTemplateTrackerZNCCInverseCompositional::initPosEvalRMS(const vpColVector &p_)
{
  unsigned int nb_corners = zoneTracked->getNbTriangle() * 3;
  x_pos.resize(nb_corners);
  y_pos.resize(nb_corners);

  Warp->computeCoeff(p);
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

void vpTemplateTrackerZNCCInverseCompositional::computeEvalRMS(const vpColVector &p_)
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

void vpTemplateTrackerZNCCInverseCompositional::deletePosEvalRMS() {}
