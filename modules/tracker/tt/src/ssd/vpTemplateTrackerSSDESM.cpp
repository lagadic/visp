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
#include <visp3/tt/vpTemplateTrackerSSDESM.h>

vpTemplateTrackerSSDESM::vpTemplateTrackerSSDESM(vpTemplateTrackerWarp *warp)
  : vpTemplateTrackerSSD(warp), compoInitialised(false), HDir(), HInv(), HLMDir(), HLMInv(), GDir(), GInv()
{
  useCompositionnal = false;
  useInverse = false;

  if (!Warp->isESMcompatible())
    std::cerr << "The selected warp function is not appropriate for the ESM "
                 "algorithm..."
              << std::endl;

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
  // std::cout<<"Initialise precomputed value of ESM with templateSize: "<<
  // templateSize<<std::endl;
  ptTemplateCompo = new vpTemplateTrackerPointCompo[templateSize];
  int i, j;
  // direct
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    ptTemplateCompo[point].dW = new double[2 * nbParam];
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);
    Warp->getdWdp0(i, j, ptTemplateCompo[point].dW);
  }

  // inverse
  HInv = 0;
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;

    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, p);
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

        // DIRECT
        // dIWx=dIx.getValue(i2,j2);
        // dIWy=dIy.getValue(i2,j2);

        dIWx = dIx.getValue(i2, j2) + ptTemplate[point].dx;
        dIWy = dIy.getValue(i2, j2) + ptTemplate[point].dy;

        // Calcul du Hessien
        // Warp->dWarp(X1,X2,p,dW);
        Warp->dWarpCompo(X1, X2, p, ptTemplateCompo[point].dW, dW);

        double *tempt = new double[nbParam];
        for (unsigned int it = 0; it < nbParam; it++)
          tempt[it] = dW[0][it] * dIWx + dW[1][it] * dIWy;

        for (unsigned int it = 0; it < nbParam; it++)
          for (unsigned int jt = 0; jt < nbParam; jt++)
            HDir[it][jt] += tempt[it] * tempt[jt];

        for (unsigned int it = 0; it < nbParam; it++)
          GDir[it] += er * tempt[it];
        delete[] tempt;
      }
    }
    if (Nbpoint == 0) {
      // std::cout<<"plus de point dans template suivi"<<std::endl;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
    }

    vpMatrix::computeHLM(HDir, lambdaDep, HLMDir);

    try {
      // dp=(HLMInv+HLMDir).inverseByLU()*(GInv+GDir);
      // dp=HLMInv.inverseByLU()*GInv+HLMDir.inverseByLU()*GDir;
      // dp=HLMInv.inverseByLU()*GInv;
      dp = (HLMDir).inverseByLU() * (GDir);
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

    // Warp->pRondp(p,dp,p);
    p += dp;
    iteration++;

  } while ((iteration < iterationMax));

  nbIteration = iteration;
}

/*void vpTemplateTrackerSSDESM::InitCompInverse(vpImage<unsigned char> &I)
{
  ptTempateCompo=new vpTemplateTrackerPointCompo[taille_template];
  int i,j;
  for(int point=0;point<taille_template;point++)
  {
    i=ptTemplate[point].y;
    j=ptTemplate[point].x;
    ptTempateCompo[point].dW=new double[2*nbParam];
    Warp->getdWdp0(i,j,ptTempateCompo[point].dW);
  }

}

void vpTemplateTrackerSSDESM::track(vpImage<unsigned char> &I)
{
  double erreur=0,erreur_prec=1e38;
  int Nbpoint=0;

  int taillefiltre=taille_filtre_dgaussien;
  double *fg=new double[taillefiltre+1] ;
  getGaussianDerivativeKernel(fg, taillefiltre) ;
  getGradX(I, dIx,fg,taillefiltre);
  getGradY(I, dIy,fg,taillefiltre);
  delete[] fg;

  vpColVector dpinv(nbParam);
  double lambda=lambdaDep;
  double IW,dIWx,dIWy;
  double Tij;
  int iteration=0;
  int i,j;
  double i2,j2;
  vpTemplateTrackerPoint *pt;
  do
  {
    Nbpoint=0;
    erreur_prec=erreur;
    erreur=0;
    dp=0;
    HDir=0;
    GDir=0;
    GInv=0;
    Warp->ComputeCoeff(p);
    for(int point=0;point<taille_template;point++)
    {
      pt=&ptTemplate[point];
      i=pt->y;
      j=pt->x;
      X1[0]=j;X1[1]=i;

      Warp->ComputeDenom(X1,p);
      Warp->warpX(X1,X2,p);

      j2=X2[0];i2=X2[1];
      if((j2<I.getWidth())&&(i2<I.getHeight())&&(i2>0)&&(j2>0))
      {
        //INVERSE
        Tij=pt->val;
        IW=I.getPixelBI(j2,i2);
        Nbpoint++;
        double er=(Tij-IW);
        for(int it=0;it<nbParam;it++)
          GInv[it]+=er*ptTemplate[point].dW[it];

        erreur+=er*er;

        //DIRECT COMPO
        Tij=ptTemplate[point].val;
        IW=I.getPixelBI(j2,i2);
        dIWx=dIx.getPixelBI(j2,i2);
        dIWy=dIy.getPixelBI(j2,i2);
        Nbpoint++;
        Warp->dWarpCompo(X1,X2,p,ptTempateCompo[point].dW,dW);
        double *tempt=new double[nbParam];
        for(int it=0;it<nbParam;it++)
          tempt[it] =dW[0][it]*dIWx+dW[1][it]*dIWy;

        for(int it=0;it<nbParam;it++)
          for(int jt=0;jt<nbParam;jt++)
            HDir[it][jt]+=tempt[it]*tempt[jt];

        for(int it=0;it<nbParam;it++)
          GDir[it]+=er*tempt[it];

        delete[] tempt;
      }


    }
    if(Nbpoint==0)std::cout<<"plus de point dans template suivi"<<std::endl;
    try
    {
      dp=(HInv+HDir).inverseByLU()*(GInv+GDir);
    }
    catch(...)
    {
      std::cout<<"probleme inversion"<<std::endl;
      break;
    }

    if(Compare)write_infos(p,erreur/Nbpoint);

    p+=Gain*dp;
    iteration++;

  }
  while( (iteration < IterationMax));

  NbIteration=iteration;
}*/
