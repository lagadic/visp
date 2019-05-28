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
#include <visp3/tt/vpTemplateTrackerZNCCForwardAdditional.h>

vpTemplateTrackerZNCCForwardAdditional::vpTemplateTrackerZNCCForwardAdditional(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerZNCC(warp)
{
  useCompositionnal = false;
}

void vpTemplateTrackerZNCCForwardAdditional::initHessienDesired(const vpImage<unsigned char> &I)
{
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
  double IW, dIWx, dIWy;
  double Tij;
  int i, j;
  double i2, j2;
  int Nbpoint = 0;

  double moyTij = 0;
  double moyIW = 0;
  double denom = 0;
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
      Tij = ptTemplate[point].val;

      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      Nbpoint++;
      moyTij += Tij;
      moyIW += IW;
    }
  }
  moyTij = moyTij / Nbpoint;
  moyIW = moyIW / Nbpoint;
  Hdesire = 0;
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
      Tij = ptTemplate[point].val;

      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      dIWx = dIx.getValue(i2, j2);
      dIWy = dIy.getValue(i2, j2);
      // Calcul du Hessien
      Warp->dWarp(X1, X2, p, dW);
      double *tempt = new double[nbParam];
      for (unsigned int it = 0; it < nbParam; it++)
        tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

      double prod = (Tij - moyTij);

      double d_Ixx = dIxx.getValue(i2, j2);
      double d_Iyy = dIyy.getValue(i2, j2);
      double d_Ixy = dIxy.getValue(i2, j2);

      for (unsigned int it = 0; it < nbParam; it++)
        for (unsigned int jt = 0; jt < nbParam; jt++)
          Hdesire[it][jt] += prod * (dW[0][it] * (dW[0][jt] * d_Ixx + dW[1][jt] * d_Ixy) +
                                     dW[1][it] * (dW[0][jt] * d_Ixy + dW[1][jt] * d_Iyy));
      /*Hdesire[0][0]+=prod*d_Ixx;
      Hdesire[1][0]+=prod*d_Ixy;
      Hdesire[0][1]+=prod*d_Ixy;
      Hdesire[1][1]+=prod*d_Iyy;*/

      denom += (Tij - moyTij) * (Tij - moyTij) * (IW - moyIW) * (IW - moyIW);
      delete[] tempt;
    }
  }

  Hdesire = Hdesire / sqrt(denom);
  vpMatrix::computeHLM(Hdesire, lambdaDep, HLMdesire);
  HLMdesireInverse = HLMdesire.inverseByLU();
  // std::cout<<"Hdesire = "<<Hdesire<<std::endl;
}

void vpTemplateTrackerZNCCForwardAdditional::trackNoPyr(const vpImage<unsigned char> &I)
{
  if (blur)
    vpImageFilter::filter(I, BI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  /*vpImage<double> dIxx,dIxy,dIyx,dIyy;
  getGradX(dIx, dIxx, fgdG,taillef);
  getGradY(dIx, dIxy, fgdG,taillef);

  getGradX(dIy, dIyx, fgdG,taillef);
  getGradY(dIy, dIyy, fgdG,taillef);*/

  dW = 0;

  // double lambda=lambdaDep;
  double IW, dIWx, dIWy;
  double Tij;
  unsigned int iteration = 0;
  int i, j;
  double i2, j2;
  double alpha = 2.;
  do {
    int Nbpoint = 0;
    double erreur = 0;
    G = 0;
    H = 0;
    Warp->computeCoeff(p);
    double moyTij = 0;
    double moyIW = 0;
    double denom = 0;
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

        Nbpoint++;
        moyTij += Tij;
        moyIW += IW;
      }
    }

    if (!Nbpoint) {
      throw(vpException(vpException::divideByZeroError, "Cannot track the template: no point"));
    }

    moyTij = moyTij / Nbpoint;
    moyIW = moyIW / Nbpoint;
    // vpMatrix d2Wx(nbParam,nbParam);
    // vpMatrix d2Wy(nbParam,nbParam);
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
        // Calcul du Hessien
        Warp->dWarp(X1, X2, p, dW);
        double *tempt = new double[nbParam];
        for (unsigned int it = 0; it < nbParam; it++)
          tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

        double prod = (Tij - moyTij);
        for (unsigned int it = 0; it < nbParam; it++)
          G[it] += prod * tempt[it];

        /*	Warp->d2Warp(X1,X2,p,d2Wx,d2Wy);
        for(int it=0;it<nbParam;it++)
          for(int jt=0;jt<nbParam;jt++)
            H[it][jt]+=prod*(d2Wx[it][jt]*dIWx+d2Wx[it][jt]*dIWy);*/
        /*double d_Ixx=dIxx.getValue(i2,j2);
        double d_Iyy=dIyy.getValue(i2,j2);
        double d_Ixy=dIxy.getValue(i2,j2);

        for(int it=0;it<nbParam;it++)
          for(int jt=0;jt<nbParam;jt++)
            H[it][jt] +=prod*(dW[0][it]*(dW[0][jt]*d_Ixx+dW[1][jt]*d_Ixy)
                +dW[1][it]*(dW[0][jt]*d_Ixy+dW[1][jt]*d_Iyy));*/
        /*H[0][0]+=prod*d_Ixx;
        H[1][0]+=prod*d_Ixy;
        H[0][1]+=prod*d_Ixy;
        H[1][1]+=prod*d_Iyy;*/

        double er = (Tij - IW);
        erreur += (er * er);
        denom += (Tij - moyTij) * (Tij - moyTij) * (IW - moyIW) * (IW - moyIW);
        delete[] tempt;
      }
    }
    /*std::cout<<"G="<<G<<std::endl;
    std::cout<<"H="<<H<<std::endl;
    std::cout<<" denom="<<denom<<std::endl;*/
    G = G / sqrt(denom);
    // std::cout<<G<<std::endl;
    H = H / sqrt(denom);

    // if(Nbpoint==0)std::cout<<"plus de point dans template
    // suivi"<<std::endl; // cannot occur

    try {
      // vpMatrix::computeHLM(H,lambda,HLM);
      // dp=1.*HLM.inverseByLU()*G;
      dp = 1. * HLMdesireInverse * G;
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
    p -= dp;
    iteration++;
  } while (/*( erreur_prec-erreur<50) && */ (iteration < iterationMax));

  // std::cout<<"erreur "<<erreur<<std::endl;
  nbIteration = iteration;
}
