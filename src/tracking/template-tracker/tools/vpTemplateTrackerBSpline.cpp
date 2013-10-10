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
#include <visp/vpTemplateTrackerBSpline.h>


double vpTemplateTrackerBSpline::getSubPixBspline4(const vpImage<double> &I, double r, double t)
{
  double res=0;
  int cr=(int)(r);
  int ct=(int)(t);
  double er=(double)r-cr;
  double et=(double)t-ct;
  int height=(int)I.getHeight();//r
  int width=(int)I.getWidth();//t
  int tr,tt;
  for(int ir=-1;ir<=2;ir++)
  {
    tr=ir+cr;
    for(int it=-1;it<=2;it++)
    {
      tt=it+ct;
      if(tr>=0 && tr <height && tt>=0 && tt <width)
        res+=Bspline4((double)ir-er)*Bspline4((double)it-et)*I[tr][tt];
    }
  }
  return res;
}

void vpTemplateTrackerBSpline::PutPVBsplineD(double *Prt, int cr, double er, int ct, double et,int Nc, double val, int &degre)
{
  switch(degre)
  {
  case 2:
    PutPVBsplineD2(Prt, cr, er, ct, et, Nc, val);break;
  case 3:
    PutPVBsplineD3(Prt, cr, er, ct, et, Nc, val);break;
  case 4:
    PutPVBsplineD4(Prt, cr, er, ct, et, Nc, val);break;
  default:
    PutPVBsplineD2(Prt, cr, er, ct, et, Nc, val);//std::cout<<"DEFAUT"<<std::endl;
  }
}

void vpTemplateTrackerBSpline::PutPVBsplineD2(double *Prt, int cr, double er, int ct, double et,int Nc, double val)
{
  double *pt=&Prt[(cr*Nc+ct)*4];
  for(int ir=0;ir<=1;ir++)
    for(int it=0;it<=1;it++)
    {
      *pt++ +=Bspline2(ir-er)*Bspline2(it-et)*val;
      //pt++;
    }
}

void vpTemplateTrackerBSpline::PutPVBsplineD3(double *Prt, int cr, double er, int ct, double et,int Nc, double val)
{
  int sr=0;
  int st=0;
  if(er>0.5){sr=1;er=er-1;}
  if(et>0.5){st=1;et=et-1;}
  double *pt=&Prt[((cr+sr)*Nc+(ct+st))*9];
  for(int ir=-1;ir<=1;ir++)
    for(int it=-1;it<=1;it++)
    {
      *pt++ +=Bspline3(ir-er)*Bspline3(it-et)*val;
      //pt++;
    }
}

void vpTemplateTrackerBSpline::PutPVBsplineD4(double *Prt, int cr, double er, int ct, double et,int Nc, double val)
{
  double Bti[4];

  double Br;
  double *ptBti=&Bti[0];
  for(int it=-1;it<=2;it++)
  {
    *ptBti++ =Bspline4i(it-et,it);
    //pt++;
  }
  double *pt=&Prt[(cr*Nc+ct)*16];
  for(int ir=-1;ir<=2;ir++)
  {
    Br=Bspline4i(ir-er,ir);
    ptBti=&Bti[0];
    for(int it=-1;it<=2;it++)
    {
      *pt++ +=Br* *ptBti++ *val;
      //*pt++ +=Br*Bspline4i(it-et,it)*val;
      //pt++;
    }
  }
}

double vpTemplateTrackerBSpline::getSubPixBspline4dx(const vpImage<unsigned char> &I, double r, double t)
{
  double res=0;
  int cr=(int)(r);
  int ct=(int)(t);
  double er=(double)r-cr;
  double et=(double)t-ct;
  int height=(int)I.getHeight();//r
  int width=(int)I.getWidth();//t

  for(int ir=-1;ir<=2;ir++)
    for(int it=-1;it<=2;it++)
    {
      int tr=ir+cr;
      int tt=it+ct;
      if(tr>=0 && tr <height && tt>=0 && tt <width)
        res+=Bspline4((double)ir-er)*dBspline4((double)it-et)*I[tr][tt];
    }
  return -res;
}

double vpTemplateTrackerBSpline::getSubPixBspline4dy(const vpImage<unsigned char> &I, double r, double t)
{
  double res=0;
  int cr=(int)(r);
  int ct=(int)(t);
  double er=(double)r-cr;
  double et=(double)t-ct;
  int height=(int)I.getHeight();//r
  int width=(int)I.getWidth();//t

  for(int ir=-1;ir<=2;ir++)
    for(int it=-1;it<=2;it++)
    {
      int tr=ir+cr;
      int tt=it+ct;
      if(tr>=0 && tr <height && tt>=0 && tt <width)
        res+=dBspline4((double)ir-er)*Bspline4((double)it-et)*I[tr][tt];
    }
  return -res;
}

double vpTemplateTrackerBSpline::Bspline2(double diff)
{
  //double result;
  double aDiff=vpMath::abs(diff);
  if(aDiff<1.)
    return (1.-aDiff);
  else
    return 0;
}

double vpTemplateTrackerBSpline::Bspline3(double diff)
{
  //double result;
  double aDiff=vpMath::abs(diff);
  if(aDiff<0.5)
    return (-(aDiff-0.5)*(aDiff-0.5)+(-aDiff+0.5)+0.5);
  else if(aDiff<1.5)
    return (0.5*(1.5-aDiff)*(1.5-aDiff));
  else
    return 0;
}

double vpTemplateTrackerBSpline::Bspline4(double diff)
{
  //double result;
  double aDiff=vpMath::abs(diff);
  if(aDiff<1.)
    return (aDiff*aDiff*aDiff/2.-aDiff*aDiff+4./6.);
  //return (0.5*(1.-aDiff)*(1.-aDiff)*(1.-aDiff)+0.5*(1.-aDiff)*(1.-aDiff)-0.5*(1.-aDiff)+1./6.);
  else if(aDiff<2.)
    return ((2.-aDiff)*(2.-aDiff)*(2.-aDiff)/6.);
  else
    return 0;
}

double vpTemplateTrackerBSpline::dBspline4(double diff)
{
  if((diff>-2.)&&(diff<=-1.))
    return (diff+2.)*(diff+2.)/2.;
  else if((diff>-1.)&&(diff<=0.))
    return -3.*diff*diff/2.-2.*diff;
  else if((diff>0.)&&(diff<=1.))
    return 3.*diff*diff/2.-2.*diff;
  else if((diff>1.)&&(diff<=2.))
    return -(diff-2.)*(diff-2.)/2.;
  else
    return 0;
}
