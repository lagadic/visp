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
#include <visp/vpTemplateTrackerTriangle.h>


vpTemplateTrackerTriangle::vpTemplateTrackerTriangle()
{
  minx_temp=0;
  miny_temp=0;

  sommet_mileu_top=false;
  xp1=0;
  yp2=0;
  yp3=0;

  l_t=0;
  h_t=0;

  marge_triangle = 0.00001;

  //ptempo.resize(2);ptempo=0;
  //p_ds_uv.resize(2);p_ds_uv=0;
  //uvinv.resize(2,2);uvinv=0;
  /*std::cout<<"Constructeur"<<std::endl;
  S1.resize(2);S2.resize(2);S3.resize(2);
  Sommet.resize(2);*/
}
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(const vpTemplateTrackerTriangle& T)
{
  *this = T;
}
vpTemplateTrackerTriangle & vpTemplateTrackerTriangle::operator=(const vpTemplateTrackerTriangle& T)
{
  minx_temp=T.minx_temp;
  miny_temp=T.miny_temp;

  sommet_mileu_top=T.sommet_mileu_top;
  xp1=T.xp1;
  yp2=T.yp2;
  yp3=T.yp3;

  l_t=T.l_t;
  h_t=T.h_t;
  S1.x=T.S1.x;
  S1.y=T.S1.y;
  S2.x=T.S2.x;
  S2.y=T.S2.y;
  S3.x=T.S3.x;
  S3.y=T.S3.y;
  //uvinv.resize(2,2);
  //uvinv=T.uvinv;
  //p_ds_uv.resize(2);
  //p_ds_uv=T.p_ds_uv;
  //ptempo.resize(2);
  //ptempo=T.ptempo;
  pas_bon=T.pas_bon;

  uvinv00=T.uvinv00;
  uvinv01=T.uvinv01;
  uvinv10=T.uvinv10;
  uvinv11=T.uvinv11;

  marge_triangle = T.marge_triangle;

  return (*this);
}

vpTemplateTrackerTriangle vpTemplateTrackerTriangle::getPyramidDown() const
{
  vpTemplateTrackerTriangle Ttemp;
  Ttemp.init(S1.x/2.,S1.y/2.,S2.x/2.,S2.y/2.,S3.x/2.,S3.y/2.);
  return Ttemp;
}

vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(int x1,int y1, int x2,int y2, int x3,int y3)
{
  vpTemplateTrackerTriangle();
  init(x1,y1,x2,y2,x3,y3);
}
vpTemplateTrackerTriangle::vpTemplateTrackerTriangle(double x1,double y1, double x2,double y2, double x3,double y3)
{
  vpTemplateTrackerTriangle();
  init(x1,y1,x2,y2,x3,y3);
}
void vpTemplateTrackerTriangle::init(const vpColVector &rS1,const vpColVector &rS2,const vpColVector &rS3)
{
  vpTemplateTrackerTriangle();
  init(rS1[0],rS1[1],rS2[0],rS2[1],rS3[0],rS3[1]);
}
void vpTemplateTrackerTriangle::init(int x1,int y1, int x2,int y2, int x3,int y3)
{
  vpTemplateTrackerTriangle();
  init((double)x1,(double)y1,(double)x2,(double)y2,(double)x3,(double)y3);
}
void vpTemplateTrackerTriangle::init(double x1,double y1, double x2,double y2, double x3,double y3)
{

  S1.x=x1;S1.y=y1;
  S2.x=x2;S2.y=y2;
  S3.x=x3;S3.y=y3;

  double minx,miny,maxx,maxy;
  //calcul du rectangle minimal contenant le triangle seletionn�
  minx=(x1<x2)?x1:x2;
  miny=(y1<y2)?y1:y2;
  minx=(minx<x3)?minx:x3;
  miny=(miny<y3)?miny:y3;
  maxx=(x1>x2)?x1:x2;
  maxy=(y1>y2)?y1:y2;
  maxx=(maxx>x3)?maxx:x3;
  maxy=(maxy>y3)?maxy:y3;

  vpColVector u;
  vpColVector v;
  u.resize(2);
  v.resize(2);
  vpMatrix uv(2,2);
  vpMatrix uvinv(2,2);

  u[0]=S2.x-S1.x;
  u[1]=S2.y-S1.y;

  v[0]=S3.x-S1.x;
  v[1]=S3.y-S1.y;

  uv[0][0]=u[0];uv[1][0]=v[0];
  uv[0][1]=u[1];uv[1][1]=v[1];
  try
  {
    uvinv=uv.inverseByLU();
    pas_bon=false;
  }
  catch(...)
  {
    pas_bon=true;
    std::cout<<"Triangle vide"<<std::endl;

  }
  uvinv00=uvinv[0][0];
  uvinv01=uvinv[0][1];
  uvinv10=uvinv[1][0];
  uvinv11=uvinv[1][1];

  l_t=maxx-minx;
  h_t=maxy-miny;
  minx_temp=minx;
  miny_temp=miny;

  marge_triangle = 0.00001;
}

//marge ajoutee a zone pour que sommet soit pris en compte

bool vpTemplateTrackerTriangle::inTriangle(const int &i, const int &j) const
{
  if(pas_bon)
    return false;

  /*ptempo[0]=j-S1.x;
  ptempo[1]=i-S1.y;

  p_ds_uv=ptempo*uvinv;
  return (p_ds_uv[0]+p_ds_uv[1]<1. && p_ds_uv[0]>0 && p_ds_uv[1]>0);*/

  double ptempo0=j-S1.x;
  double ptempo1=i-S1.y;
  double p_ds_uv0=ptempo0*uvinv00+ptempo1*uvinv10;
  double p_ds_uv1=ptempo0*uvinv01+ptempo1*uvinv11;
  return (p_ds_uv0+p_ds_uv1<1.+marge_triangle && p_ds_uv0>-marge_triangle && p_ds_uv1>-marge_triangle);
}

bool vpTemplateTrackerTriangle::inTriangle(const double &i, const double &j) const
{
  if(pas_bon)
    return false;
  /*ptempo[0]=j-S1.x;
  ptempo[1]=i-S1.y;

  p_ds_uv=ptempo*uvinv;
  return (p_ds_uv[0]+p_ds_uv[1]<1. && p_ds_uv[0]>0 && p_ds_uv[1]>0);*/
  double ptempo0=j-S1.x;
  double ptempo1=i-S1.y;
  double p_ds_uv0=ptempo0*uvinv00+ptempo1*uvinv10;
  double p_ds_uv1=ptempo0*uvinv01+ptempo1*uvinv11;
  return (p_ds_uv0+p_ds_uv1<1.+marge_triangle && p_ds_uv0>-marge_triangle && p_ds_uv1>-marge_triangle);
}

void vpTemplateTrackerTriangle::getCorners(vpColVector &rS1,vpColVector &rS2,vpColVector &rS3) const
{
  rS1=getS1();
  rS2=getS2();
  rS3=getS3();
}

vpColVector vpTemplateTrackerTriangle::getS1() const
{
  vpColVector rS1(2);
  rS1[0]=S1.x;
  rS1[1]=S1.y;

  return rS1;
}
vpColVector vpTemplateTrackerTriangle::getS2() const
{
  vpColVector rS2(2);
  rS2[0]=S2.x;
  rS2[1]=S2.y;
  return rS2;
}

vpColVector vpTemplateTrackerTriangle::getS3() const
{
  vpColVector rS3(2);
  rS3[0]=S3.x;
  rS3[1]=S3.y;
  return rS3;
}

void vpTemplateTrackerTriangle::getSize(double *lt,double *ht) const
{
  *lt=l_t;
  *ht=h_t;
}
void vpTemplateTrackerTriangle::getSize(int *lt,int *ht) const
{
  /*
  *lt=vpMath::round(l_t);
  *ht=vpMath::round(h_t);*/
  *lt=(int)l_t+1;
  *ht=(int)h_t+1;
}

double vpTemplateTrackerTriangle::getMinx() const
{
  return minx_temp-1;
}
double vpTemplateTrackerTriangle::getMiny() const
{
  return miny_temp-1;
}
double vpTemplateTrackerTriangle::getMaxx() const
{
  return minx_temp+l_t+1;
}
double vpTemplateTrackerTriangle::getMaxy() const
{
  return miny_temp+h_t+1;
}



