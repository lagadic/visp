/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
#include <visp/vpTemplateTrackerWarp.h>

//warp Tr en TT avec le deplacement p
void vpTemplateTrackerWarp::warpTriangle(const vpTemplateTrackerTriangle &TR,const vpColVector &p, vpTemplateTrackerTriangle &TT)
{
  vpColVector S1(2),S2(2),S3(2);
  vpColVector rS1(2),rS2(2),rS3(2);
  TR.getCorners(S1,S2,S3);
  warpX(S1,rS1,p);
  warpX(S2,rS2,p);
  warpX(S3,rS3,p);
  TT.init(rS1,rS2,rS3);
}
void vpTemplateTrackerWarp::warpZone(const vpTemplateTrackerZone &ZR,const vpColVector &p, vpTemplateTrackerZone &ZT)
{
  vpTemplateTrackerTriangle TR,TT;
  for(int i=0;i<ZR.getNbTriangle();i++)
  {
    ZR.getTriangle(i,TR);
    warpTriangle(TR,p,TT);
    ZT.add(TT);
  }
}
void vpTemplateTrackerWarp::warpZone(const vpTemplateTrackerZone &Z,const vpColVector &p)
{
  computeCoeff(p);
  vpColVector X1(2),X2(2);
  vpTemplateTrackerZPoint *pt=Z.getListPt();
  vpTemplateTrackerZPoint *ptWarpes=Z.getListPtWarpes();
  
  for(unsigned int i=0;i<Z.getNbToutSommets();i++)
  {
    X1[0]=pt[i].x;
    X1[1]=pt[i].y;
    computeDenom(X1,p);
    warpX(X1,X2,p);
    ptWarpes[i].x=vpMath::round(X2[0]);
    ptWarpes[i].y=vpMath::round(X2[1]);
  }
}
double vpTemplateTrackerWarp::getDistanceBetweenZoneAndWarpedZone(const vpTemplateTrackerZone &Z, const vpColVector &p)
{
  int nb_pt_diff=(int)Z.getNbSommetDiff();
  computeCoeff(p);
  vpColVector X1(2),X2(2);
  vpTemplateTrackerZPoint pt;
  double res=0;
  for(int i=0;i<nb_pt_diff;i++)
  {
    pt=Z.getCorner(i);
    X1[0]=pt.x;
    X1[1]=pt.y;
    computeDenom(X1,p);
    warpX(X1,X2,p);
    res+=sqrt((X2[0]-X1[0])*(X2[0]-X1[0])+(X2[1]-X1[1])*(X2[1]-X1[1]));
  }
  return res/nb_pt_diff;
}

void vpTemplateTrackerWarp::warp(const double *ut0,const double *vt0,int nb_pt,const vpColVector& p,double *u,double *v)
{
  computeCoeff(p);
  vpColVector X1(2),X2(2);
  for(int i=0;i<nb_pt;i++)
  {
    X1[0]=ut0[i];
    X1[1]=vt0[i];
    computeDenom(X1,p);
    warpX(X1,X2,p);
    u[i]=X2[0];
    v[i]=X2[1];
    //std::cout<<"warp "<<X2[0]<<","<<X2[1]<<std::endl;
  }
}
void vpTemplateTrackerWarp::find_warp(const double *ut0,const double *vt0,const double *u,const double *v,int nb_pt,vpColVector& p)
{
  vpMatrix dW(2,NbParam);
  vpMatrix dX(2,1);
  vpMatrix H(NbParam,NbParam);
  vpMatrix G(NbParam,1);

  int cpt=0;
  vpColVector X1(2);
  vpColVector fX1(2);
  vpColVector X2(2);
  double erreur=0;
  double erreur_prec;
  double lambda=0.01;
  do
  {
    erreur_prec=erreur;
    H=0;
    G=0;
    erreur=0;
    computeCoeff(p);
    for(int i=0;i<nb_pt;i++)
    {
      X1[0]=ut0[i];
      X1[1]=vt0[i];
      computeDenom(X1,p);
      warpX(X1,fX1,p);
      dWarp(X1,fX1,p,dW);
      H+=dW.AtA();

      X2[0]=u[i];
      X2[1]=v[i];

      dX=X2-fX1;
      G+=dW.t()*dX;

      erreur+=((u[i]-fX1[0])*(u[i]-fX1[0])+(v[i]-fX1[1])*(v[i]-fX1[1]));

    }

    vpMatrix::computeHLM(H,lambda,H);
    try{p+=H.inverseByLU()*G;}
    catch(...){std::cerr<<"probleme inversion find homography"<<std::endl;break;}
    cpt++;
  }
  while((cpt<150)&&(sqrt((erreur_prec-erreur)*(erreur_prec-erreur))>1e-20));
  //std::cout<<"erreur apres transformation="<<erreur<<std::endl;

}
