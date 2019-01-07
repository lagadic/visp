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
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void vpTemplateTrackerMIBSpline::PutPVBsplineD(double *Prt, int cr, double er, int ct, double et, int Nc, double val,
                                               const int &degre)
{
  switch (degre) {
  case 4:
    PutPVBsplineD4(Prt, cr, er, ct, et, Nc, val);
    break;
  default:
    PutPVBsplineD3(Prt, cr, er, ct, et, Nc,
                   val); // std::cout<<"DEFAUT"<<std::endl;
  }
}

void vpTemplateTrackerMIBSpline::PutPVBsplineD3(double *Prt, int cr, double er, int ct, double et, int Nc, double val)
{
  int sr = 0;
  int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }
  double *pt = &Prt[((cr + sr) * Nc + (ct + st)) * 9];
  for (int ir = -1; ir <= 1; ir++)
    for (int it = -1; it <= 1; it++) {
      *pt++ += Bspline3(ir - er) * Bspline3(it - et) * val;
      // pt++;
    }
}

void vpTemplateTrackerMIBSpline::PutPVBsplineD4(double *Prt, int cr, double er, int ct, double et, int Nc, double val)
{
  double Bti[4];

  double *ptBti = &Bti[0];
  for (int it = -1; it <= 2; it++) {
    *ptBti++ = Bspline4i(it - et, it);
    // pt++;
  }
  double *pt = &Prt[(cr * Nc + ct) * 16];
  for (int ir = -1; ir <= 2; ir++) {
    double Br = Bspline4i(ir - er, ir);
    ptBti = &Bti[0];
    for (int it = -1; it <= 2; it++) {
      *pt++ += Br * *ptBti++ * val;
    }
  }
}

double vpTemplateTrackerMIBSpline::Bspline3(double diff)
{
  // double result;
  double aDiff = std::fabs(diff);
  //  if(aDiff<0.5)
  ////    return (-(aDiff-0.5)*(aDiff-0.5)+(-aDiff+0.5)+0.5);
  //      return (-(aDiff * aDiff) + 0.75);
  //  else if(aDiff<1.5)
  //    return (0.5*(1.5-aDiff)*(1.5-aDiff));

  if (aDiff < 1.5) {
    if (aDiff < 0.5)
      return (-(aDiff * aDiff) + 0.75);
    return (0.5 * (1.5 - aDiff) * (1.5 - aDiff));
  }

  return 0;
}

double vpTemplateTrackerMIBSpline::Bspline4i(double diff, int &interv)
{
  switch (interv) {
  case -1:
    return ((2. + diff) * (2. + diff) * (2. + diff) / 6.);
  case 0:
    return (-diff * diff * diff / 2. - diff * diff + 4. / 6.);
  case 1:
    return (diff * diff * diff / 2. - diff * diff + 4. / 6.);
  case 2:
    return ((2. - diff) * (2. - diff) * (2. - diff) / 6.);
  default:
    return 0;
  }
}

double vpTemplateTrackerMIBSpline::dBspline3(double diff)
{
  if ((diff > -1.5) && (diff <= -0.5))
    return diff + 1.5;
  else if ((diff > -0.5) && (diff <= 0.5))
    return -2. * diff;
  else if ((diff > 0.5) && (diff <= 1.5))
    return diff - 1.5;

  //  if(fabs(diff + 1.5) <= (-0.5 + 1.5))
  //    return diff+1.5;
  //  else if(fabs(diff + 0.5) <= (0.5 + 0.5))
  //    return -2.*diff;
  //  else if(fabs(diff - 0.5) <= (1.5 - 0.5))
  //    return diff-1.5;

  return 0;
}

double vpTemplateTrackerMIBSpline::dBspline4(double diff)
{
  if ((diff > -2.) && (diff <= -1.))
    return (diff + 2.) * (diff + 2.) / 2.;
  else if ((diff > -1.) && (diff <= 0.))
    return -3. * diff * diff / 2. - 2. * diff;
  else if ((diff > 0.) && (diff <= 1.))
    return 3. * diff * diff / 2. - 2. * diff;
  else if ((diff > 1.) && (diff <= 2.))
    return -(diff - 2.) * (diff - 2.) / 2.;
  else
    return 0;
}

double vpTemplateTrackerMIBSpline::d2Bspline3(double diff)
{
  if ((diff > -1.5) && (diff <= -0.5))
    return 1.;
  else if ((diff > -0.5) && (diff <= 0.5))
    return -2.;
  else if ((diff > 0.5) && (diff <= 1.5))
    return 1.;
  else
    return 0;
}

double vpTemplateTrackerMIBSpline::d2Bspline4(double diff)
{
  if ((diff > -2.) && (diff <= -1.))
    return (diff + 2.);
  else if ((diff > -1.) && (diff <= 0.))
    return -3. * diff - 2.;
  else if ((diff > 0.) && (diff <= 1.))
    return 3. * diff - 2.;
  else if ((diff > 1.) && (diff <= 2.))
    return -(diff - 2.);
  else
    return 0;
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline(double *Prt, int cr, double &er, int ct, double &et, int Nc,
                                                 double *val, unsigned int &NbParam, int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4(Prt, cr, er, ct, et, Nc, val, NbParam);
    break;
  default:
    PutTotPVBspline3(Prt, cr, er, ct, et, Nc, val, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline(double *Prt, double *dPrt, double *d2Prt, int cr, double &er, int ct,
                                                 double &et, int Ncb, double *val, unsigned int &NbParam, int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4(Prt, dPrt, d2Prt, cr, er, ct, et, Ncb, val, NbParam);
    break;
  default:
    PutTotPVBspline3(Prt, dPrt, d2Prt, cr, er, ct, et, Ncb, val, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3(double *Prt, int cr, double &er, int ct, double &et, int Nc,
                                                  double *val, unsigned int &NbParam)
{
  short int sr = 0;
  short int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3];
  double dBti[3];
  double d2Bti[3];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  double *ptd2Bti = &d2Bti[0];

  for (short int it = 1; it >= -1; it--) {
    *ptBti++ = Bspline3(it + et);
    *ptdBti++ = dBspline3(it + et);
    *ptd2Bti++ = d2Bspline3(it + et);
  }

  double *pt = &Prt[((cr + sr) * Nc + (ct + st)) * 9 * (1 + (int)(NbParam + NbParam * NbParam))];
  for (short int ir = -1; ir <= 1; ++ir) {
    double Br = Bspline3(-ir + er);

    for (short unsigned int it = 0; it <= 2; ++it) {
      *pt++ += Br * (Bti[it]);

      double v1 = Br * (dBti[it]);
      for (short unsigned int ip = 0; ip < NbParam; ++ip) {
        *pt++ -= v1 * val[ip];
        double v2 = Br * (d2Bti[it]) * val[ip];
        for (short unsigned int ip2 = 0; ip2 < NbParam; ++ip2)
          *pt++ += v2 * val[ip2];
      }
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3(double *Prt, double *dPrt, double *d2Prt, int cr, double &er, int ct,
                                                  double &et, int Ncb, double *val, unsigned int &NbParam)
{
  short int sr = 0;
  short int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3];
  double dBti[3];
  double d2Bti[3];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  double *ptd2Bti = &d2Bti[0];

  for (short int it = 1; it >= -1; it--) {
    *ptBti++ = Bspline3(it + et);
    *ptdBti++ = dBspline3(it + et);
    *ptd2Bti++ = d2Bspline3(it + et);
  }

  int NbParam_ = (int)NbParam;
  for (short int ir = -1; ir <= 1; ++ir) {
    double Br = Bspline3(-ir + er);
    short int irInd = ir + 1;
    short int ind = (cr + sr + irInd) * Ncb;
    for (short int it = 0; it <= 2; ++it) {
      Prt[ind + (ct + st + it)] += Br * (Bti[it]);

      double v1 = Br * (dBti[it]);
      int ind1 = ((cr + sr + irInd) * Ncb + (ct + st + it)) * NbParam_;
      for (int ip = 0; ip < NbParam_; ++ip) {
        dPrt[ind1 + ip] -= v1 * val[ip];
        double v2 = Br * (d2Bti[it]) * val[ip];
        int ind2 = ((cr + sr + irInd) * Ncb + (ct + st + it)) * NbParam_ * NbParam_ + ip * NbParam_;
        for (short int ip2 = 0; ip2 < NbParam_; ++ip2)
          d2Prt[ind2 + ip2] += v2 * val[ip2];
      }
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3(double *Prt, double &er, double *bt, unsigned int size)
{
//  double Br;
//  for(short int ir=-1;ir<=1;++ir)
//  {
//    Br=Bspline3(-ir+er);

//    for(short unsigned int it=0;it < size;++it)
//        *Prt++ += Br * bt[it];
//  }
#define LSIZE 12

  double *bt0 = &bt[0];
  if (er > 0.5) {
    er = er - 1.0;
  }

  for (int ir = -1; ir <= 1; ++ir) {
    double Br = Bspline3(-ir + er);
    const double *btend = bt0 + size;
    bt = bt0;

    if (size >= LSIZE) {
      btend -= LSIZE - 1;
      for (; bt < btend; bt += LSIZE) {
        *Prt++ += Br * bt[0];
        *Prt++ += Br * bt[1];
        *Prt++ += Br * bt[2];
        *Prt++ += Br * bt[3];
        *Prt++ += Br * bt[4];
        *Prt++ += Br * bt[5];
        *Prt++ += Br * bt[6];
        *Prt++ += Br * bt[7];
        *Prt++ += Br * bt[8];
        *Prt++ += Br * bt[9];
        *Prt++ += Br * bt[10];
        *Prt++ += Br * bt[11];
      }
      btend += LSIZE - 1;
    }
    for (; bt < btend; *Prt++ += Br * *bt++) {
    };
  }
#undef LSIZE
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4(double *Prt, int cr, double er, int ct, double et, int Nc,
                                                  double *val, unsigned int &NbParam)
{
  double Bti[4];
  double dBti[4];
  double d2Bti[4];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  double *ptd2Bti = &d2Bti[0];
  for (char it = -1; it <= 2; it++) {
    *ptBti++ = vpTemplateTrackerBSpline::Bspline4(-it + et);
    *ptdBti++ = dBspline4(-it + et);
    *ptd2Bti++ = d2Bspline4(-it + et);
  }

  int NbParam_ = (int)NbParam;

  double *pt = &Prt[(cr * Nc + ct) * 16 * (1 + NbParam_ + NbParam_ * NbParam_)];
  for (char ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    ptBti = &Bti[0];
    ptdBti = &dBti[0];
    ptd2Bti = &d2Bti[0];
    for (char it = -1; it <= 2; it++) {
      *pt++ += Br * *ptBti;
      for (short int ip = 0; ip < NbParam_; ip++) {
        *pt++ -= Br * *ptdBti * val[ip];
        for (short int ip2 = 0; ip2 < NbParam_; ip2++)
          *pt++ += Br * *ptd2Bti * val[ip] * val[ip2];
      }
      ptBti++;
      ptdBti++;
      ptd2Bti++;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4(double *Prt, double *dPrt, double *d2Prt, int cr, double er, int ct,
                                                  double et, int Ncb, double *val, unsigned int &NbParam)
{
  double Bti[4];
  double dBti[4];
  double d2Bti[4];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  double *ptd2Bti = &d2Bti[0];
  for (char it = -1; it <= 2; it++) {
    *ptBti++ = vpTemplateTrackerBSpline::Bspline4(-it + et);
    *ptdBti++ = dBspline4(-it + et);
    *ptd2Bti++ = d2Bspline4(-it + et);
  }

  int NbParam_ = (int)NbParam;

  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    int irInd = ir + 1;
    int ind = (cr + irInd) * Ncb + ct;

    ptBti = &Bti[0];
    ptdBti = &dBti[0];
    ptd2Bti = &d2Bti[0];

    for (int it = -1; it <= 2; it++) {
      Prt[ind + it] += Br * *ptBti;
      int ind1 = ((cr + irInd) * Ncb + (ct + it)) * NbParam_;
      for (int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind1 + ip] -= Br * *ptdBti * val[ip];
        int ind2 = ((cr + irInd) * Ncb + (ct + it)) * NbParam_ * NbParam_ + ip * NbParam_;
        for (int ip2 = 0; ip2 < NbParam_; ip2++)
          d2Prt[ind2 + ip2] += Br * *ptd2Bti * val[ip] * val[ip2];
      }
      ptBti++;
      ptdBti++;
      ptd2Bti++;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4(double *Prt, double &er, double *bt, unsigned int size)
{
#define LSIZE 12
  double *bt0 = &bt[0];

  for (int ir = -1; ir <= 2; ++ir) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    const double *btend = bt0 + size;
    bt = bt0;

    if (size >= LSIZE) {
      btend -= LSIZE - 1;
      for (; bt < btend; bt += LSIZE) {
        *Prt++ += Br * bt[0];
        *Prt++ += Br * bt[1];
        *Prt++ += Br * bt[2];
        *Prt++ += Br * bt[3];
        *Prt++ += Br * bt[4];
        *Prt++ += Br * bt[5];
        *Prt++ += Br * bt[6];
        *Prt++ += Br * bt[7];
        *Prt++ += Br * bt[8];
        *Prt++ += Br * bt[9];
        *Prt++ += Br * bt[10];
        *Prt++ += Br * bt[11];
      }
      btend += LSIZE - 1;
    }
    for (; bt < btend; *Prt++ += Br * *bt++) {
    };
  }
#undef LSIZE
}

void vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc,
                                                         double *val, unsigned int &NbParam, int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4NoSecond(Prt, cr, er, ct, et, Nc, val, NbParam);
    break;
  default:
    PutTotPVBspline3NoSecond(Prt, cr, er, ct, et, Nc, val, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBsplineNoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct,
                                                         double &et, int &Ncb, double *val, unsigned int &NbParam,
                                                         int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4NoSecond(Prt, dPrt, cr, er, ct, et, Ncb, val, NbParam);
    break;
  default:
    PutTotPVBspline3NoSecond(Prt, dPrt, cr, er, ct, et, Ncb, val, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3NoSecond(double *Prt, int &cr, double &er, int &ct, double &et,
                                                          int &Nc, double *val, unsigned int &NbParam)
{
  int sr = 0;
  int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3];
  double dBti[3];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  for (char it = -1; it <= 1; it++) {
    *ptBti++ = Bspline3(-it + et);
    *ptdBti++ = dBspline3(-it + et);
  }

  int NbParam_ = (int)NbParam;

  double *pt = &Prt[((cr + sr) * Nc + (ct + st)) * 9 * (1 + NbParam_ + NbParam_ * NbParam_)];
  for (char ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);
    ptBti = &Bti[0];
    ptdBti = &dBti[0];
    for (char it = -1; it <= 1; it++) {
      *pt++ += Br * *ptBti;
      for (unsigned int ip = 0; ip < NbParam; ip++) {
        *pt++ -= Br * *ptdBti * val[ip];
        pt = pt + NbParam; // Modif AY
      }
      //      pt=pt+NbParam*NbParam; // Modif AY
      ptBti++;
      ptdBti++;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3NoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct,
                                                          double &et, int &Ncb, double *val, unsigned int &NbParam)
{
  int sr = 0;
  int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3];
  double dBti[3];

  double *ptBti = &Bti[0];
  double *ptdBti = &dBti[0];
  for (char it = -1; it <= 1; it++) {
    *ptBti++ = Bspline3(-it + et);
    *ptdBti++ = dBspline3(-it + et);
  }

  int NbParam_ = (int)NbParam;

  for (char ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);

    int irInd = ir + 1;
    int ind = (cr + sr + irInd) * Ncb;

    ptBti = &Bti[0];
    ptdBti = &dBti[0];
    for (char it = -1; it <= 1; it++) {
      Prt[ind + (ct + st + it)] += Br * *ptBti;
      int ind1 = ((cr + sr + irInd) * Ncb + (ct + st + it)) * NbParam_;
      for (short int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind1 + ip] -= Br * *ptdBti * val[ip];
      }
      ptBti++;
      ptdBti++;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4NoSecond(double *Prt, int &cr, double &er, int &ct, double &et,
                                                          int &Nc, double *val, unsigned int &NbParam)
{
  double Bti[4] = {0, 0, 0, 0};
  double dBti[4] = {0, 0, 0, 0};

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
    dBti[it + 1] = dBspline4(-it + et);
  }

  int NbParam_ = (int)NbParam;

  double *pt = &Prt[(cr * Nc + ct) * 16 * (1 + NbParam_ + NbParam_ * NbParam_)];
  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    for (int it = 0; it <= 3; it++) {
      (*pt++) += Br * Bti[it];

      for (int ip = 0; ip < NbParam_; ip++) {
        (*pt++) -= Br * dBti[it] * val[ip];
        pt = pt + NbParam_; // Modif AY
      }
      //      pt=pt+NbParam*NbParam; // Modif AY
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4NoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct,
                                                          double &et, int &Ncb, double *val, unsigned int &NbParam)
{
  double Bti[4] = {0, 0, 0, 0};
  double dBti[4] = {0, 0, 0, 0};

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
    dBti[it + 1] = dBspline4(-it + et);
  }

  int NbParam_ = (int)NbParam;

  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    int irInd = ir + 1;
    int ind = (cr + irInd) * Ncb + ct;

    for (int it = 0; it <= 3; it++) {
      Prt[ind + it] += Br * Bti[it];
      int ind1 = ((cr + irInd) * Ncb + (ct + it)) * NbParam_;

      for (int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind1 + ip] -= Br * dBti[it] * val[ip];
      }
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBsplinePrtTout(double *PrtTout, int &cr, double &er, int &ct, double &et,
                                                        int &Nc, unsigned int &NbParam, int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4PrtTout(PrtTout, cr, er, ct, et, Nc, NbParam);
    break;
  default:
    PutTotPVBspline3PrtTout(PrtTout, cr, er, ct, et, Nc, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3PrtTout(double *PrtTout, int &cr, double &er, int &ct, double &et,
                                                         int &Nc, unsigned int &NbParam)
{
  int sr = 0;
  int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3] = {0, 0, 0};

  for (char it = -1; it <= 1; it++) {
    Bti[it + 1] = Bspline3(-it + et);
  }

  int NbParam_ = (int)NbParam;
  int NbParam_val = NbParam_ + NbParam_ * NbParam_;

  double *pt = &PrtTout[(unsigned int)(((cr + sr) * Nc + (ct + st)) * 9 * (1 + NbParam_val))];
  for (int ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);
    for (int it = 0; it <= 2; it++) {
      (*pt++) += Br * Bti[it];
      pt = pt + NbParam_val;
    }
  }
}
void vpTemplateTrackerMIBSpline::PutTotPVBspline4PrtTout(double *PrtTout, int &cr, double &er, int &ct, double &et,
                                                         int &Nc, unsigned int &NbParam)
{
  double Bti[4] = {0, 0, 0, 0};

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
  }

  int NbParam_ = (int)NbParam;
  int NbParam_val = NbParam_ + NbParam_ * NbParam_;
  double *pt = &PrtTout[(unsigned int)((cr * Nc + ct) * 16 * (1 + NbParam_val))];
  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    for (int it = 0; it <= 3; it++) {
      (*pt++) += Br * Bti[it];
      pt = pt + NbParam_val;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBsplinePrt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb,
                                                    unsigned int &NbParam, int &degree)
{
  switch (degree) {
  case 4:
    PutTotPVBspline4PrtTout(Prt, cr, er, ct, et, Ncb, NbParam);
    break;
  default:
    PutTotPVBspline3PrtTout(Prt, cr, er, ct, et, Ncb, NbParam);
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline3Prt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb)
{

  int sr = 0;
  int st = 0;
  if (er > 0.5) {
    sr = 1;
    er = er - 1;
  }
  if (et > 0.5) {
    st = 1;
    et = et - 1;
  }

  double Bti[3] = {0, 0, 0};

  for (char it = -1; it <= 1; it++) {
    Bti[it + 1] = Bspline3(-it + et);
  }

  for (int ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);

    int irInd = ir + 1;
    int ind = (cr + sr + irInd) * Ncb;
    for (int it = 0; it <= 2; it++) {
      Prt[ind + (ct + st + it)] += Br * Bti[it];
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4Prt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb)
{
  double Bti[4] = {0, 0, 0, 0};

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
  }

  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    int irInd = ir + 1;
    int ind = (cr + irInd) * Ncb + ct;

    for (int it = 0; it <= 3; it++) {
      Prt[ind + it] += Br * Bti[it];
    }
  }
}

#endif
