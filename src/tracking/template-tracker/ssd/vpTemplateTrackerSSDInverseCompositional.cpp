/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
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
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpImageTools.h>

vpTemplateTrackerSSDInverseCompositional::vpTemplateTrackerSSDInverseCompositional(vpTemplateTrackerWarp *warp):vpTemplateTrackerSSD(warp)
{
  useInverse=true;
  compoInitialised=false;
  HInv.resize(nbParam,nbParam);
  HCompInverse.resize(nbParam,nbParam);
  useTemplateSelect=false;
  threshold_RMS=1e-20;
}

void vpTemplateTrackerSSDInverseCompositional::initCompInverse(const vpImage<unsigned char> &/*I*/)
{

  H=0;
  int i,j;

  for(unsigned int point=0;point<templateSize;point++)
  {
    if((!useTemplateSelect)||(ptTemplateSelect[point]))
    {
      i=ptTemplate[point].y;
      j=ptTemplate[point].x;
      X1[0]=j;X1[1]=i;
      Warp->computeDenom(X1,p);
      ptTemplate[point].dW=new double[nbParam];

      Warp->getdW0(i,j,ptTemplate[point].dy,ptTemplate[point].dx,ptTemplate[point].dW);

      for(unsigned int it=0;it<nbParam;it++)
        for(unsigned int jt=0;jt<nbParam;jt++)
          H[it][jt]+=ptTemplate[point].dW[it]*ptTemplate[point].dW[jt];
    }

  }
  HInv=H;
  vpMatrix HLMtemp(nbParam,nbParam);
  vpMatrix::computeHLM(H,lambdaDep,HLMtemp);

  HCompInverse.resize(nbParam,nbParam);
  HCompInverse=HLMtemp.inverseByLU();
  //std::cout<<Hinverse<<std::endl;
  vpColVector dWtemp(nbParam);
  vpColVector HiGtemp(nbParam);

  for(unsigned int point=0;point<templateSize;point++)
  {
    if((!useTemplateSelect)||(ptTemplateSelect[point]))
    {
      i=ptTemplate[point].y;
      j=ptTemplate[point].x;
      for(unsigned int it=0;it<nbParam;it++)
        dWtemp[it]=ptTemplate[point].dW[it];
      
      HiGtemp	= -1.*HCompInverse*dWtemp;
      ptTemplate[point].HiG=new double[nbParam];

      for(unsigned int it=0;it<nbParam;it++)
        ptTemplate[point].HiG[it]=HiGtemp[it];
    }
  }
  compoInitialised=true;
}

void vpTemplateTrackerSSDInverseCompositional::initHessienDesired(const vpImage<unsigned char> &I)
{
  initCompInverse(I);
}

void vpTemplateTrackerSSDInverseCompositional::trackNoPyr(const vpImage<unsigned char> &I)
{
  double erreur=0;
  int Nbpoint=0;
  if(blur)
    vpImageFilter::filter(I, BI,fgG,taillef);

  vpColVector dpinv(nbParam);
  double IW;
  double Tij;
  unsigned int iteration=0;
  int i,j;
  double i2,j2;
  double alpha=2.;
  //vpTemplateTrackerPointtest *pt;
  initPosEvalRMS(p);

  vpTemplateTrackerPoint *pt;
  do
  {
    Nbpoint=0;
    erreur=0;
    dp=0;
    Warp->computeCoeff(p);
    for(unsigned int point=0;point<templateSize;point++)
    {
      if((!useTemplateSelect)||(ptTemplateSelect[point]))
      {
        //pt=&ptTemplatetest[point];
        pt=&ptTemplate[point];
        i=pt->y;
        j=pt->x;
        X1[0]=j;X1[1]=i;
        Warp->computeDenom(X1,p);
        Warp->warpX(X1,X2,p);
        j2=X2[0];i2=X2[1];

        if((i2>=0)&&(j2>=0)&&(i2<I.getHeight()-1)&&(j2<I.getWidth()-1))
        {
          Tij=pt->val;
          if(!blur)
            IW=I.getValue(i2,j2);
          else
            IW=BI.getValue(i2,j2);
          Nbpoint++;
          double er=(Tij-IW);
          for(unsigned int it=0;it<nbParam;it++)
            dp[it]+=er*pt->HiG[it];

          erreur+=er*er;
        }
      }
    }
    if(Nbpoint==0)std::cout<<"plus de point dans template suivi"<<std::endl;
    dp=gain*dp;
    //std::cout<<erreur/Nbpoint<<","<<GetCost(I,p)<<std::endl;
    if(useBrent)
    {
      alpha=2.;
      computeOptimalBrentGain(I,p,erreur/Nbpoint,dp,alpha);
      dp=alpha*dp;
    }
    Warp->Param_inv(dp,dpinv);
    Warp->pRondp(p,dpinv,p);
    iteration++;

    computeEvalRMS(p);
  }
  while(/*( erreur_prec-erreur<50) &&*/ (iteration < iterationMax)&&(evolRMS>threshold_RMS));

  nbIteration=iteration;
  deletePosEvalRMS();
}

void vpTemplateTrackerSSDInverseCompositional::initPosEvalRMS(vpColVector &p)
{
  x_pos=new double[zoneTracked->getNbSommetDiff()];
  y_pos=new double[zoneTracked->getNbSommetDiff()];

  Warp->computeCoeff(p);
  for(unsigned int i=0;i<zoneTracked->getNbSommetDiff();i++)
  {
    int x,y;
    zoneTracked->getCornerDiff((int)i,x,y);
    X1[0]=x;X1[1]=y;
    Warp->computeDenom(X1,p);
    Warp->warpX(X1,X2,p);
    x_pos[i]=X2[0];
    y_pos[i]=X2[1];
  }
}

void vpTemplateTrackerSSDInverseCompositional::computeEvalRMS(const vpColVector &p)
{

  Warp->computeCoeff(p);
  evolRMS=0;
  for(unsigned int i=0;i<zoneTracked->getNbSommetDiff();i++)
  {
    int x,y;
    zoneTracked->getCornerDiff((int)i,x,y);
    X1[0]=x;X1[1]=y;
    Warp->computeDenom(X1,p);
    Warp->warpX(X1,X2,p);
    evolRMS+=(x_pos[i]-X2[0])*(x_pos[i]-X2[0])+(y_pos[i]-X2[1])*(y_pos[i]-X2[1]);
    x_pos[i]=X2[0];
    y_pos[i]=X2[1];
  }
  evolRMS=evolRMS/zoneTracked->getNbSommetDiff();
}

void vpTemplateTrackerSSDInverseCompositional::deletePosEvalRMS()
{
  delete[] x_pos;
  delete[] y_pos;
}
