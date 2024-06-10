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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
BEGIN_VISP_NAMESPACE
void vpTemplateTrackerMIBSpline::PutPVBsplineD(double *Prt, int cr, double er, int ct, double et, int Nc, double val,
                                               const int &degre)
{
  switch (degre) {
  case 4:
    PutPVBsplineD4(Prt, cr, er, ct, et, Nc, val);
    break;
  default:
    PutPVBsplineD3(Prt, cr, er, ct, et, Nc, val);
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
  if (std::fabs(val - 1.) > std::numeric_limits<double>::epsilon()) {
    for (int ir = -1; ir <= 1; ir++) {
      double Bspline3_diff_r_ = Bspline3(ir - er);
      for (int it = -1; it <= 1; it++) {
        *pt++ += Bspline3_diff_r_ * Bspline3(it - et) * val;
      }
    }
  }
  else {
    for (int ir = -1; ir <= 1; ir++) {
      double Bspline3_diff_r_ = Bspline3(ir - er);
      for (int it = -1; it <= 1; it++) {
        *pt++ += Bspline3_diff_r_ * Bspline3(it - et);
      }
    }
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
  if (std::fabs(val - 1.) > std::numeric_limits<double>::epsilon()) {
    for (int ir = -1; ir <= 2; ir++) {
      double Br = Bspline4i(ir - er, ir);
      ptBti = &Bti[0];
      for (int it = -1; it <= 2; it++) {
        *pt++ += Br * (*ptBti++) * val;
      }
    }
  }
  else {
    for (int ir = -1; ir <= 2; ir++) {
      double Br = Bspline4i(ir - er, ir);
      ptBti = &Bti[0];
      for (int it = -1; it <= 2; it++) {
        *pt++ += Br * (*ptBti++);
      }
    }
  }
}

double vpTemplateTrackerMIBSpline::Bspline3(double diff)
{
  double aDiff = std::fabs(diff);

  if (aDiff < 1.5) {
    if (aDiff < 0.5) {
      return (-(aDiff * aDiff) + 0.75);
    }
    double tmp_ = 1.5 - aDiff;
    return (0.5 * tmp_ * tmp_);
  }

  return 0;
}

double vpTemplateTrackerMIBSpline::Bspline4i(double diff, int &interv)
{
  switch (interv) {
  case 2:
  case -1: {
    double tmp_ = 2. + diff;
    return (tmp_ * tmp_ * tmp_ / 6.);
  }
  case 0: {
    double diff2_ = diff * diff;
    return (-diff2_ * diff / 2. - diff2_ + 4. / 6.);
  }
  case 1: {
    double diff2_ = diff * diff;
    return (diff2_ * diff / 2. - diff2_ + 4. / 6.);
  }
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

  return 0;
}

double vpTemplateTrackerMIBSpline::dBspline4(double diff)
{
  if ((diff > -2.) && (diff <= -1.)) {
    double diff_2_ = diff + 2.;
    return (diff_2_ * diff_2_ * 0.5);
  }
  else if ((diff > -1.) && (diff <= 0.)) {
    return -1.5 * diff * diff - 2. * diff;
  }
  else if ((diff > 0.) && (diff <= 1.)) {
    return 1.5 * diff * diff - 2. * diff;
  }
  else if ((diff > 1.) && (diff <= 2.)) {
    double diff_2_ = diff - 2.;
    return (-0.5 * diff_2_ * diff_2_);
  }
  else {
    return 0;
  }
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
      double Br_ptBti_ = Br * (*ptBti);
      double Br_ptdBti_ = Br * (*ptdBti);
      double Br_ptd2Bti_ = Br * (*ptd2Bti);
      *pt++ += Br_ptBti_;
      for (short int ip = 0; ip < NbParam_; ip++) {
        *pt++ -= Br_ptdBti_ * val[ip];
        for (short int ip2 = 0; ip2 < NbParam_; ip2++) {
          *pt++ += Br_ptd2Bti_ * val[ip] * val[ip2];
        }
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

  for (size_t i = 0; i < 4; ++i) {
    Bti[i] = 0.;
    dBti[i] = 0.;
    d2Bti[i] = 0.;
  }

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
      int ind2 = ind1 * NbParam_;
      double Br_ptdBti_ = Br * (*ptdBti);
      double Br_ptd2Bti_ = Br * (*ptd2Bti);
      for (int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind1 + ip] -= Br_ptdBti_ * val[ip];
        int ind3 = ind2 + ip * NbParam_;
        for (int ip2 = 0; ip2 < NbParam_; ip2++)
          d2Prt[ind3 + ip2] += Br_ptd2Bti_ * val[ip] * val[ip2];
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
      double Br_ptdBti_ = Br * (*ptdBti);
      for (unsigned int ip = 0; ip < NbParam; ip++) {
        *pt++ -= Br_ptdBti_ * val[ip];
        pt += NbParam; // Modif AY
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

  int NbParam_ = static_cast<int>(NbParam);
  int ct_st_ = ct + st;
  int cr_sr_ = cr + sr;

  for (char ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);

    int irInd = ir + 1;
    int ind = (cr_sr_ + irInd) * Ncb;
    ptBti = &Bti[0];
    ptdBti = &dBti[0];

    double Br_ptBti_ = Br * (*ptBti);
    double Br_ptdBti_ = Br * (*ptdBti);
    for (char it = -1; it <= 1; it++) {
      Prt[ind + (ct_st_ + it)] += Br_ptBti_;
      int ind1 = (ind + (ct_st_ + it)) * NbParam_;
      for (short int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind1 + ip] -= Br_ptdBti_ * val[ip];
      }
      ptBti++;
      ptdBti++;
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4NoSecond(double *Prt, int &cr, double &er, int &ct, double &et,
                                                          int &Nc, double *val, unsigned int &NbParam)
{
  double Bti[4] = { 0, 0, 0, 0 };
  double dBti[4] = { 0, 0, 0, 0 };

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

      double Br_dBti_ = Br * dBti[it];
      for (int ip = 0; ip < NbParam_; ip++) {
        (*pt++) -= Br_dBti_ * val[ip];
        pt += NbParam_; // Modif AY
      }
      //      pt=pt+NbParam*NbParam; // Modif AY
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4NoSecond(double *Prt, double *dPrt, int &cr, double &er, int &ct,
                                                          double &et, int &Ncb, double *val, unsigned int &NbParam)
{
  double Bti[4] = { 0, 0, 0, 0 };
  double dBti[4] = { 0, 0, 0, 0 };

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
    dBti[it + 1] = dBspline4(-it + et);
  }

  int NbParam_ = static_cast<int>(NbParam);

  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    int irInd = ir + 1;
    int ind = (cr + irInd) * Ncb;
    int ind1 = ind + ct;

    for (int it = 0; it <= 3; it++) {
      Prt[ind1 + it] += Br * Bti[it];
      int ind2 = (ind + (ct + it)) * NbParam_;

      double Br_dBti_ = Br * dBti[it];
      for (int ip = 0; ip < NbParam_; ip++) {
        dPrt[ind2 + ip] -= Br_dBti_ * val[ip];
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

  double Bti[3] = { 0, 0, 0 };

  for (char it = -1; it <= 1; it++) {
    Bti[it + 1] = Bspline3(-it + et);
  }

  int NbParam_ = static_cast<int>(NbParam);
  int NbParam_val = NbParam_ + NbParam_ * NbParam_;

  double *pt = &PrtTout[(unsigned int)(((cr + sr) * Nc + (ct + st)) * 9 * (1 + NbParam_val))];
  for (int ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);
    for (int it = 0; it <= 2; it++) {
      (*pt++) += Br * Bti[it];
      pt += NbParam_val;
    }
  }
}
void vpTemplateTrackerMIBSpline::PutTotPVBspline4PrtTout(double *PrtTout, int &cr, double &er, int &ct, double &et,
                                                         int &Nc, unsigned int &NbParam)
{
  double Bti[4] = { 0, 0, 0, 0 };

  for (char it = -1; it <= 2; it++) {
    Bti[it + 1] = vpTemplateTrackerBSpline::Bspline4(-it + et);
  }

  int NbParam_ = static_cast<int>(NbParam);
  int NbParam_val = NbParam_ + NbParam_ * NbParam_;
  double *pt = &PrtTout[(unsigned int)((cr * Nc + ct) * 16 * (1 + NbParam_val))];
  for (int ir = -1; ir <= 2; ir++) {
    double Br = vpTemplateTrackerBSpline::Bspline4(-ir + er);
    for (int it = 0; it <= 3; it++) {
      (*pt++) += Br * Bti[it];
      pt += NbParam_val;
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

  double Bti[3] = { 0, 0, 0 };

  for (char it = -1; it <= 1; it++) {
    Bti[it + 1] = Bspline3(-it + et);
  }

  int ct_st_ = ct + st;
  int cr_sr_ = cr + sr;
  for (int ir = -1; ir <= 1; ir++) {
    double Br = Bspline3(-ir + er);

    int irInd = ir + 1;
    int ind = (cr_sr_ + irInd) * Ncb;
    for (int it = 0; it <= 2; it++) {
      Prt[ind + (ct_st_ + it)] += Br * Bti[it];
    }
  }
}

void vpTemplateTrackerMIBSpline::PutTotPVBspline4Prt(double *Prt, int &cr, double &er, int &ct, double &et, int &Ncb)
{
  double Bti[4] = { 0, 0, 0, 0 };

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

void vpTemplateTrackerMIBSpline::computeProbabilities(double *Prt, int &cr, double &er, int &ct, double &et, int &Nc,
                                                      double *dW, unsigned int &NbParam, int &bspline,
                                                      vpTemplateTrackerMI::vpHessienApproximationType &approx,
                                                      bool use_hessien_des)
{
  if (approx == vpTemplateTrackerMI::HESSIAN_NONSECOND || use_hessien_des) {
    if (bspline == 3)
      PutTotPVBspline3NoSecond(Prt, cr, er, ct, et, Nc, dW, NbParam);
    else
      PutTotPVBspline4NoSecond(Prt, cr, er, ct, et, Nc, dW, NbParam);
  }
  else {
    if (bspline == 3)
      PutTotPVBspline3(Prt, cr, er, ct, et, Nc, dW, NbParam);
    else
      PutTotPVBspline4(Prt, cr, er, ct, et, Nc, dW, NbParam);
  }
}
END_VISP_NAMESPACE
#endif
