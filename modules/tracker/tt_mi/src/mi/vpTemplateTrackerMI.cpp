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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/core/vpException.h>
#include <visp3/tt_mi/vpTemplateTrackerMI.h>
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

void vpTemplateTrackerMI::setBspline(const vpBsplineType &newbs)
{
  bspline = (int)newbs;
  influBspline = bspline * bspline;
  Ncb = Nc + bspline;
  if (Pt)
    delete[] Pt;
  if (Pr)
    delete[] Pr;
  if (Prt)
    delete[] Prt;
  if (dPrt)
    delete[] dPrt;
  if (d2Prt)
    delete[] d2Prt;
  if (PrtD)
    delete[] PrtD;
  if (dPrtD)
    delete[] dPrtD;
  if (PrtTout)
    delete[] PrtTout;

  Pt = new double[Ncb];
  Pr = new double[Ncb];

  Prt = new double[Ncb * Ncb];
  dPrt = new double[Ncb * Ncb * (int)(nbParam)];
  d2Prt = new double[Ncb * Ncb * (int)(nbParam * nbParam)];

  /*std::cout<<Nc*Nc*influBspline<<std::endl;std::cout<<Nc*Nc*nbParam*influBspline<<std::endl;*/
  PrtD = new double[Nc * Nc * influBspline];
  dPrtD = new double[Nc * Nc * (int)(nbParam)*influBspline];
  PrtTout = new double[Nc * Nc * influBspline * (1 + (int)(nbParam + nbParam * nbParam))];

  hessianComputation = USE_HESSIEN_DESIRE;
}

vpTemplateTrackerMI::vpTemplateTrackerMI(vpTemplateTrackerWarp *_warp)
  : vpTemplateTracker(_warp), hessianComputation(USE_HESSIEN_NORMAL), ApproxHessian(HESSIAN_NEW), lambda(0), temp(NULL),
    Prt(NULL), dPrt(NULL), Pt(NULL), Pr(NULL), d2Prt(NULL), PrtTout(NULL), dprtemp(NULL), PrtD(NULL), dPrtD(NULL),
    influBspline(0), bspline(3), Nc(8), Ncb(0), d2Ix(), d2Iy(), d2Ixy(), MI_preEstimation(0), MI_postEstimation(0),
    NMI_preEstimation(0), NMI_postEstimation(0), covarianceMatrix(), computeCovariance(false)
{
  Ncb = Nc + bspline;
  influBspline = bspline * bspline;

  dW.resize(2, nbParam);
  H.resize(nbParam, nbParam);
  G.resize(nbParam);
  Hdesire.resize(nbParam, nbParam);
  HLM.resize(nbParam, nbParam);
  HLMdesire.resize(nbParam, nbParam);
  dprtemp = new double[nbParam];
  temp = new double[nbParam];

  X1.resize(2);
  X2.resize(2);

  PrtD = new double[Nc * Nc * influBspline]; //(r,t)
  dPrtD = new double[Nc * Nc * (int)(nbParam)*influBspline];

  Prt = new double[Ncb * Ncb]; //(r,t)
  Pt = new double[Ncb];
  Pr = new double[Ncb];
  dPrt = new double[Ncb * Ncb * (int)(nbParam)];
  d2Prt = new double[Ncb * Ncb * (int)(nbParam * nbParam)];

  PrtTout = new double[Nc * Nc * influBspline * (1 + (int)(nbParam + nbParam * nbParam))];

  lambda = lambdaDep;
}

void vpTemplateTrackerMI::setNc(int nc)
{
  Nc = nc;
  Ncb = Nc + bspline;

  if (Pt)
    delete[] Pt;
  if (Pr)
    delete[] Pr;
  if (Prt)
    delete[] Prt;
  if (dPrt)
    delete[] dPrt;
  if (d2Prt)
    delete[] d2Prt;
  if (PrtD)
    delete[] PrtD;
  if (dPrtD)
    delete[] dPrtD;
  if (PrtTout)
    delete[] PrtTout;

  PrtD = new double[Nc * Nc * influBspline]; //(r,t)
  dPrtD = new double[Nc * Nc * (int)(nbParam)*influBspline];
  Prt = new double[Ncb * Ncb]; //(r,t)
  dPrt = new double[Ncb * Ncb * (int)(nbParam)];
  Pt = new double[Ncb];
  Pr = new double[Ncb];
  d2Prt = new double[Ncb * Ncb * (int)(nbParam * nbParam)]; //(r,t)
  PrtTout = new double[Nc * Nc * influBspline * (1 + (int)(nbParam + nbParam * nbParam))];
}

double vpTemplateTrackerMI::getCost(const vpImage<unsigned char> &I, const vpColVector &tp)
{
  double MI = 0;
  int Nbpoint = 0;
  double IW;

  unsigned int Ncb_ = (unsigned int)Ncb;
  unsigned int Nc_ = (unsigned int)Nc;
  unsigned int influBspline_ = (unsigned int)influBspline;

  memset(Prt, 0, Ncb_ * Ncb_ * sizeof(double));
  memset(PrtD, 0, Nc_ * Nc_ * influBspline_ * sizeof(double));

  // Warp->ComputeMAtWarp(tp);
  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    // Tij=Templ[i-(int)Triangle->GetMiny()][j-(int)Triangle->GetMinx()];
    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;

      double Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      int cr = (int)((IW * (Nc - 1)) / 255.);
      int ct = (int)((Tij * (Nc - 1)) / 255.);
      double er = (IW * (Nc - 1)) / 255. - cr;
      double et = ((double)Tij * (Nc - 1)) / 255. - ct;

      // Calcul de l'histogramme joint par interpolation bilinÃaire
      // (Bspline ordre 1)
      vpTemplateTrackerMIBSpline::PutPVBsplineD(PrtD, cr, er, ct, et, Nc, 1., bspline);
    }
  }

  ratioPixelIn = (double)Nbpoint / (double)templateSize;

  double *pt = PrtD;
  for (int r = 0; r < Nc; r++)
    for (int t = 0; t < Nc; t++) {
      for (int i = 0; i < influBspline; i++) {
        int r2, t2;
        r2 = r + i / bspline;
        t2 = t + i % bspline;
        Prt[r2 * Ncb + t2] += *pt;

        pt++;
      }
    }

  if (Nbpoint == 0)
    return 0;
  for (unsigned int r = 0; r < Ncb_; r++)
    for (unsigned int t = 0; t < Ncb_; t++)
      // printf("%f ",Prt[r*Ncb+t]);
      Prt[r * Ncb_ + t] = Prt[r * Ncb_ + t] / Nbpoint;
  // calcul Pr;
  memset(Pr, 0, Ncb_ * sizeof(double));
  for (unsigned int r = 0; r < Ncb_; r++) {
    for (unsigned int t = 0; t < Ncb_; t++)
      Pr[r] += Prt[r * Ncb_ + t];
  }

  // calcul Pt;
  memset(Pt, 0, Ncb_ * sizeof(double));
  for (unsigned int t = 0; t < Ncb_; t++) {
    for (unsigned int r = 0; r < Ncb_; r++)
      Pt[t] += Prt[r * Ncb_ + t];
  }
  for (unsigned int r = 0; r < Ncb_; r++)
    // if(Pr[r]!=0)
    if (std::fabs(Pr[r]) > std::numeric_limits<double>::epsilon())
      MI -= Pr[r] * log(Pr[r]);

  for (unsigned int t = 0; t < Ncb_; t++)
    // if(Pt[t]!=0)
    if (std::fabs(Pt[t]) > std::numeric_limits<double>::epsilon())
      MI -= Pt[t] * log(Pt[t]);

  for (unsigned int r = 0; r < Ncb_; r++)
    for (unsigned int t = 0; t < Ncb_; t++)
      // if(Prt[r*Ncb+t]!=0)
      if (std::fabs(Prt[r * Ncb_ + t]) > std::numeric_limits<double>::epsilon())
        MI += Prt[r * Ncb_ + t] * log(Prt[r * Ncb_ + t]);

  return -MI;
}

double vpTemplateTrackerMI::getNormalizedCost(const vpImage<unsigned char> &I, const vpColVector &tp)
{
  // Attention, cette version calculée de la NMI ne pourra pas atteindre le
  // maximum de 2 Ceci est du au fait que par defaut, l'image est floutée dans
  // vpTemplateTracker::initTracking()

  double MI = 0;
  double Nbpoint = 0;
  double IW;

  double Pr_[256];
  double Pt_[256];
  double Prt_[256][256];

  memset(Pr_, 0, 256 * sizeof(double));
  memset(Pt_, 0, 256 * sizeof(double));
  memset(Prt_, 0, 256 * 256 * sizeof(double));

  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;
      double Tij = ptTemplate[point].val;
      if (!blur)
        IW = I[(int)i2][(int)j2];
      else
        IW = BI.getValue(i2, j2);

      Pr_[(int)Tij]++;
      Pt_[(int)IW]++;
      Prt_[(int)Tij][(int)IW]++;
    }
  }

  for (int i = 0; i < 256; i++) {
    Pr_[i] /= Nbpoint;
    Pt_[i] /= Nbpoint;
    for (int j = 0; j < 256; j++)
      Prt_[i][j] /= Nbpoint;
  }

  for (int r = 0; r < 256; r++)
    // if(Pr_[r]!=0)
    if (std::fabs(Pr_[r]) > std::numeric_limits<double>::epsilon())
      MI -= Pr_[r] * log(Pr_[r]);

  for (int t = 0; t < 256; t++)
    // if(Pt_[t]!=0)
    if (std::fabs(Pt_[t]) > std::numeric_limits<double>::epsilon())
      MI -= Pt_[t] * log(Pt_[t]);

  double denom = 0;
  for (int r = 0; r < 256; r++)
    for (int t = 0; t < 256; t++)
      // if(Prt_[r][t]!=0)
      if (std::fabs(Prt_[r][t]) > std::numeric_limits<double>::epsilon())
        denom -= (Prt_[r][t] * log(Prt_[r][t]));

  // if(denom != 0)
  if (std::fabs(denom) > std::numeric_limits<double>::epsilon())
    MI = MI / denom;
  else
    MI = 0;

  return -MI;
}

vpTemplateTrackerMI::~vpTemplateTrackerMI()
{
  if (Pt)
    delete[] Pt;
  if (Pr)
    delete[] Pr;
  if (Prt)
    delete[] Prt;
  if (dPrt)
    delete[] dPrt;
  if (d2Prt)
    delete[] d2Prt;
  if (PrtD)
    delete[] PrtD;
  if (dPrtD)
    delete[] dPrtD;
  if (PrtTout)
    delete[] PrtTout;
  if (temp)
    delete[] temp;
  if (dprtemp)
    delete[] dprtemp;
}

void vpTemplateTrackerMI::computeProba(int &nbpoint)
{
  double *pt = PrtTout;
  unsigned int Nc_ = (unsigned int)Nc;
  unsigned int Ncb_ = (unsigned int)Ncb;
  unsigned int bspline_ = (unsigned int)bspline;

  for (unsigned int r = 0; r < Nc_; r++) {
    for (unsigned int t = 0; t < Nc_; t++) {
      for (unsigned int r2 = 0; r2 < bspline_; r2++) {
        for (unsigned int t2 = 0; t2 < bspline_; t2++) {
          Prt[((r2 + r) * Ncb_ + (t2 + t))] += *pt++;
          for (unsigned int ip = 0; ip < nbParam; ip++) {
            dPrt[((r2 + r) * Ncb_ + (t2 + t)) * nbParam + ip] += *pt++;
            for (unsigned int it = 0; it < nbParam; it++) {
              d2Prt[((r2 + r) * Ncb_ + (t2 + t)) * nbParam * nbParam + ip * nbParam + it] += *pt++;
            }
          }
        }
      }
    }
  }

  //    for(unsigned int r=0;r<Nc;r++)
  //        for(unsigned int t=0;t<Nc;t++)
  //    {
  //        for(int r2=0;r2<bspline;r2++)
  //            for(int t2=0;t2<bspline;t2++)
  //        {
  //            Prt[((r2+r)*Ncb+(t2+t))]+=*pt++;
  //            for(int ip=0;ip<nbParam;ip++)
  //            {
  //                dPrt[((r2+r)*Ncb+(t2+t))*nbParam+ip]+=*pt++;
  //            }
  //            for(int ip=0;ip<nbParam;ip++)
  //            {
  //                for(int it=0;it<nbParam;it++)
  //                    d2Prt[((r2+r)*Ncb+(t2+t))*nbParam*nbParam+ip*nbParam+it]+=*pt++;
  //            }
  //        }
  //    }

  //    int vr2, vt2, vt2ip;
  //    for(unsigned int r=0;r<Nc;r++)
  //        for(unsigned int t=0;t<Nc;t++)
  //        {
  //            for(int r2=0;r2<bspline;r2++)
  //            {
  //                vr2 = (r2+r)*Ncb;
  //                for(int t2=0;t2<bspline;t2++)
  //                {
  //                    vt2 = vr2+(t2+t);
  //                    Prt[vt2]+=*pt++;
  //                    vt2 *= nbParam;

  //                    for(int ip=0;ip<nbParam;ip++)
  //                        dPrt[vt2+ip]+=*pt++;

  //                    for(int ip=0;ip<nbParam;ip++)
  //                    {
  //                        vt2ip = vt2*nbParam+ip*nbParam;
  //                        for(int it=0;it<nbParam;it++)
  //                            d2Prt[vt2ip+it]+=*pt++;
  //                    }
  //                }
  //            }
  //        }

  if (nbpoint == 0) {
    // std::cout<<"plus de point dans template suivi"<<std::endl;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
  }
  unsigned int indd, indd2;
  indd = indd2 = 0;
  for (volatile int i = 0; i < Ncb * Ncb; i++) {
    Prt[i] = Prt[i] / nbpoint;
    for (unsigned int j = 0; j < nbParam; j++) {
      dPrt[indd] = dPrt[indd] / nbpoint;
      indd++;
      for (unsigned int k = 0; k < nbParam; k++) {
        d2Prt[indd2] = d2Prt[indd2] / nbpoint;
        indd2++;
      }
    }
  }
}

void vpTemplateTrackerMI::computeMI(double &MI)
{
  unsigned int Ncb_ = (unsigned int)Ncb;

  // calcul Pr;
  memset(Pr, 0, Ncb_ * sizeof(double));
  for (unsigned int r = 0; r < Ncb_; r++) {
    for (unsigned int t = 0; t < Ncb_; t++)
      Pr[r] += Prt[r * Ncb_ + t];
  }

  // calcul Pt;
  memset(Pt, 0, Ncb_ * sizeof(double));
  for (unsigned int t = 0; t < Ncb_; t++) {
    for (unsigned int r = 0; r < Ncb_; r++)
      Pt[t] += Prt[r * Ncb_ + t];
  }

  // calcul Entropies;
  // double entropieI=0;
  for (unsigned int r = 0; r < Ncb_; r++) {
    // if(Pr[r]!=0)
    if (std::fabs(Pr[r]) > std::numeric_limits<double>::epsilon()) {
      // entropieI-=Pr[r]*log(Pr[r]);
      MI -= Pr[r] * log(Pr[r]);
    }
  }

  // double entropieT=0;
  for (unsigned int t = 0; t < Ncb_; t++) {
    // if(Pt[t]!=0)
    if (std::fabs(Pt[t]) > std::numeric_limits<double>::epsilon()) {
      // entropieT-=Pt[t]*log(Pt[t]);
      MI -= Pt[t] * log(Pt[t]);
    }
  }

  for (unsigned int r = 0; r < Ncb_; r++)
    for (unsigned int t = 0; t < Ncb_; t++)
      // if(Prt[r*Ncb+t]!=0)
      if (std::fabs(Prt[r * Ncb_ + t]) > std::numeric_limits<double>::epsilon())
        MI += Prt[r * Ncb_ + t] * log(Prt[r * Ncb_ + t]);
}

void vpTemplateTrackerMI::computeHessien(vpMatrix &Hessian)
{
  double seuilevitinf = 1e-200;
  Hessian = 0;
  double dtemp;
  unsigned int Ncb_ = (unsigned int)Ncb;
  for (unsigned int t = 0; t < Ncb_; t++) {
    // if(Pt[t]!=0)
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        // if(Prt[r*Ncb+t]!=0)
        if (Prt[r * Ncb_ + t] > seuilevitinf) {
          for (unsigned int it = 0; it < nbParam; it++)
            dprtemp[it] = dPrt[(r * Ncb_ + t) * nbParam + it];

          dtemp = 1. + log(Prt[r * Ncb_ + t] / Pt[t]);
          for (unsigned int it = 0; it < nbParam; it++)
            for (unsigned int jt = 0; jt < nbParam; jt++)
              if (ApproxHessian != HESSIAN_NONSECOND && ApproxHessian != HESSIAN_NEW)
                Hessian[it][jt] += dprtemp[it] * dprtemp[jt] * (1. / Prt[r * Ncb_ + t] - 1. / Pt[t]) +
                                   d2Prt[(r * Ncb_ + t) * nbParam * nbParam + it * nbParam + jt] * dtemp;
              else if (ApproxHessian == HESSIAN_NEW)
                Hessian[it][jt] += d2Prt[(r * Ncb_ + t) * nbParam * nbParam + it * nbParam + jt] * dtemp;
              else
                Hessian[it][jt] += dprtemp[it] * dprtemp[jt] * (1. / Prt[r * Ncb_ + t] - 1. / Pt[t]);
          /*std::cout<<"Prt[r*Ncb+t]="<<Prt[r*Ncb+t]<<std::endl;
          std::cout<<"Pt[t]="<<Pt[t]<<std::endl;

          std::cout<<"Hessian="<<Hessian<<std::endl;*/
        }
      }
    }
  }
  //    std::cout<<"Hessian="<<Hessian<<std::endl;
}

void vpTemplateTrackerMI::computeHessienNormalized(vpMatrix &Hessian)
{
  double seuilevitinf = 1e-200;
  // double dtemp;
  double u = 0, v = 0, B = 0;
  double *du = new double[nbParam];
  double *dv = new double[nbParam];
  double *A = new double[nbParam];
  double *dB = new double[nbParam];
  double **d2u = new double *[nbParam];
  double **d2v = new double *[nbParam];
  double **dA = new double *[nbParam];
  for (unsigned int i = 0; i < nbParam; i++) {
    d2u[i] = new double[nbParam];
    d2v[i] = new double[nbParam];
    dA[i] = new double[nbParam];
  }

  memset(du, 0, nbParam * sizeof(double));
  memset(dv, 0, nbParam * sizeof(double));
  memset(A, 0, nbParam * sizeof(double));
  memset(dB, 0, nbParam * sizeof(double));
  memset(dprtemp, 0, nbParam * sizeof(double));

  for (unsigned int it = 0; it < nbParam; it++) {
    for (unsigned int jt = 0; jt < nbParam; jt++) {
      memset(d2u[it], 0, nbParam * sizeof(double));
      memset(d2v[it], 0, nbParam * sizeof(double));
      memset(dA[it], 0, nbParam * sizeof(double));
    }
  }

  unsigned int Ncb_ = (unsigned int)Ncb;

  for (unsigned int t = 0; t < Ncb_; t++) {
    // if(Pt[t]!=0)
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        // if(Prt[r*Ncb+t]!=0)
        if (Prt[r * Ncb_ + t] > seuilevitinf) {
          for (unsigned int it = 0; it < nbParam; it++) {
            // dPxy/dt
            dprtemp[it] = dPrt[(r * Ncb_ + t) * nbParam + it];
          }
          // dtemp=1.+log(Prt[r*Ncb+t]/Pt[t]);
          // u = som(Pxy.logPxPy)
          u += Prt[r * Ncb_ + t] * log(Pt[t] * Pr[r]);
          // v = som(Pxy.logPxy)
          v += Prt[r * Ncb_ + t] * log(Prt[r * Ncb_ + t]);

          for (unsigned int it = 0; it < nbParam; it++) {
            // u' = som dPxylog(PxPy)
            du[it] += dprtemp[it] * log(Pt[t] * Pr[r]);
            // v' = som dPxy(1+log(Pxy))
            dv[it] += dprtemp[it] * (1 + log(Prt[r * Ncb_ + t]));
          }
          for (unsigned int it = 0; it < nbParam; it++) {
            for (unsigned int jt = 0; jt < nbParam; jt++) {
              d2u[it][jt] += d2Prt[(r * Ncb_ + t) * nbParam * nbParam + it * nbParam + jt] * log(Pt[t] * Pr[r]) +
                             (1.0 / Prt[r * Ncb_ + t]) * (dprtemp[it] * dprtemp[it]);
              d2v[it][jt] +=
                  d2Prt[(r * Ncb_ + t) * nbParam * nbParam + it * nbParam + jt] * (1 + log(Prt[r * Ncb_ + t])) +
                  (1.0 / Prt[r * Ncb_ + t]) * (dprtemp[it] * dprtemp[it]);
            }
          }
        }
      }
    }
  }
  // B = v2
  B = (v * v);
  for (unsigned int it = 0; it < nbParam; it++) {
    // A = u'v-uv'
    A[it] = du[it] * v - u * dv[it];
    // B' = 2vv'
    dB[it] = 2 * v * dv[it];
    // G = (u'v-v'u)/v2
    //        G[it] = A[it]/B;
    for (unsigned int jt = 0; jt < nbParam; jt++) {
      // A' = u''v-v''u
      dA[it][jt] = d2u[it][jt] * v - d2v[it][jt] * u;
      // Hessian = (A'B-AB')/B2
      Hessian[it][jt] = (dA[it][jt] * B - A[it] * dB[it]) / (B * B);
    }
  }

  delete[] du;
  delete[] dv;
  delete[] A;
  delete[] dB;
  for (unsigned int i = 0; i < nbParam; i++) {
    delete[] d2u[i];
    delete[] d2v[i];
    delete[] dA[i];
  }
  delete[] d2u;
  delete[] d2v;
  delete[] dA;

  //    std::cout<<"Hdes - compute Hessien\n"<<u<<"\n"<<v<<"\n"<<du[0]<<"
  //    "<<du[1]<<"
  //    "<<du[2]<<"\n"<<dv[2]<<"\n"<<d2u[2][2]<<"\n"<<d2v[2][2]<<"\n"<<H<<std::endl;
}

void vpTemplateTrackerMI::computeGradient()
{

  double seuilevitinf = 1e-200;
  G = 0;
  unsigned int Ncb_ = (unsigned int)Ncb;
  double dtemp;
  for (unsigned int t = 0; t < Ncb_; t++) {
    // if(Pt[t]!=0)
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        if (Prt[r * Ncb_ + t] > seuilevitinf)
        // if(Prt[r*Ncb+t]!=0)
        {
          for (unsigned int it = 0; it < nbParam; it++)
            dprtemp[it] = dPrt[(r * Ncb_ + t) * nbParam + it];

          dtemp = 1. + log(Prt[r * Ncb_ + t] / Pt[t]);

          for (unsigned int it = 0; it < nbParam; it++)
            G[it] += dtemp * dprtemp[it];
        }
      }
    }
  }
}
void vpTemplateTrackerMI::zeroProbabilities()
{
  unsigned int Ncb_ = (unsigned int)Ncb;
  unsigned int Nc_ = (unsigned int)Nc;
  unsigned int influBspline_ = (unsigned int)influBspline;

  memset(Prt, 0, Ncb_ * Ncb_ * sizeof(double));
  memset(dPrt, 0, Ncb_ * Ncb_ * nbParam * sizeof(double));
  memset(d2Prt, 0, Ncb_ * Ncb_ * nbParam * nbParam * sizeof(double));
  memset(PrtTout, 0, Nc_ * Nc_ * influBspline_ * (1 + nbParam + nbParam * nbParam) * sizeof(double));

  //    std::cout << Ncb*Ncb << std::endl;
  //    std::cout << Ncb*Ncb*nbParam << std::endl;
  //    std::cout << Ncb*Ncb*nbParam*nbParam << std::endl;
  //    std::cout << Ncb*Ncb*influBspline*(1+nbParam+nbParam*nbParam) <<
  //    std::endl;
}

double vpTemplateTrackerMI::getMI(const vpImage<unsigned char> &I, int &nc, const int &bspline_, vpColVector &tp)
{
  unsigned int tNcb = (unsigned int)(nc + bspline_);
  unsigned int tinfluBspline = (unsigned int)(bspline_ * bspline_);
  unsigned int nc_ = (unsigned int)nc;
  double *tPrtD = new double[nc_ * nc_ * tinfluBspline];
  double *tPrt = new double[tNcb * tNcb];
  double *tPr = new double[tNcb];
  double *tPt = new double[tNcb];

  double MI = 0;
  volatile int Nbpoint = 0;
  double IW;

  vpImage<double> GaussI;
  vpImageFilter::filter(I, GaussI, fgG, taillef);

  memset(tPrt, 0, tNcb * tNcb * sizeof(double));
  memset(tPrtD, 0, nc_ * nc_ * tinfluBspline * sizeof(double));

  // Warp->ComputeMAtWarp(tp);
  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    // Tij=Templ[i-(int)Triangle->GetMiny()][j-(int)Triangle->GetMinx()];
    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth()) - 1) {
      Nbpoint++;

      double Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = GaussI.getValue(i2, j2);

      int cr = (int)((IW * (nc - 1)) / 255.);
      int ct = (int)((Tij * (nc - 1)) / 255.);
      double er = (IW * (nc - 1)) / 255. - cr;
      double et = ((double)Tij * (nc - 1)) / 255. - ct;

      // Calcul de l'histogramme joint par interpolation bilineaire (Bspline_
      // ordre 1)
      vpTemplateTrackerMIBSpline::PutPVBsplineD(tPrtD, cr, er, ct, et, nc, 1., bspline_);
    }
  }
  double *pt = tPrtD;
  int tNcb_ = (int)tNcb;
  int tinfluBspline_ = (int)tinfluBspline;
  for (int r = 0; r < nc; r++)
    for (int t = 0; t < nc; t++) {
      for (volatile int i = 0; i < tinfluBspline_; i++) {
        int r2, t2;
        r2 = r + i / bspline_;
        t2 = t + i % bspline_;
        tPrt[r2 * tNcb_ + t2] += *pt;

        pt++;
      }
    }

  if (Nbpoint == 0) {
    delete[] tPrtD;
    delete[] tPrt;
    delete[] tPr;
    delete[] tPt;

    return 0;
  } else {
    for (unsigned int r = 0; r < tNcb; r++)
      for (unsigned int t = 0; t < tNcb; t++)
        // printf("%f ",tPrt[r*tNcb+t]);
        tPrt[r * tNcb + t] = tPrt[r * tNcb + t] / Nbpoint;
    // calcul Pr;
    memset(tPr, 0, tNcb * sizeof(double));
    for (unsigned int r = 0; r < tNcb; r++) {
      for (unsigned int t = 0; t < tNcb; t++)
        tPr[r] += tPrt[r * tNcb + t];
    }

    // calcul Pt;
    memset(tPt, 0, (size_t)(tNcb * sizeof(double)));
    for (unsigned int t = 0; t < tNcb; t++) {
      for (unsigned int r = 0; r < tNcb; r++)
        tPt[t] += tPrt[r * tNcb + t];
    }
    for (unsigned int r = 0; r < tNcb; r++)
      // if(tPr[r]!=0)
      if (std::fabs(tPr[r]) > std::numeric_limits<double>::epsilon())
        MI -= tPr[r] * log(tPr[r]);

    for (unsigned int t = 0; t < tNcb; t++)
      // if(tPt[t]!=0)
      if (std::fabs(tPt[t]) > std::numeric_limits<double>::epsilon())
        MI -= tPt[t] * log(tPt[t]);

    for (unsigned int r = 0; r < tNcb; r++)
      for (unsigned int t = 0; t < tNcb; t++)
        // if(tPrt[r*tNcb+t]!=0)
        if (std::fabs(tPrt[r * tNcb + t]) > std::numeric_limits<double>::epsilon())
          MI += tPrt[r * tNcb + t] * log(tPrt[r * tNcb + t]);
  }
  delete[] tPrtD;
  delete[] tPrt;
  delete[] tPr;
  delete[] tPt;

  return MI;
}

double vpTemplateTrackerMI::getMI256(const vpImage<unsigned char> &I, const vpColVector &tp)
{
  vpMatrix Prt256(256, 256);
  Prt256 = 0;
  vpColVector Pr256(256);
  Pr256 = 0;
  vpColVector Pt256(256);
  Pt256 = 0;

  volatile int Nbpoint = 0;
  unsigned int Tij, IW;

  vpImage<double> GaussI;
  if (blur)
    vpImageFilter::filter(I, GaussI, fgG, taillef);

  // Warp->ComputeMAtWarp(tp);
  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    int i = ptTemplate[point].y;
    int j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    // Tij=Templ[i-(int)Triangle->GetMiny()][j-(int)Triangle->GetMinx()];
    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth()) - 1) {
      Nbpoint++;

      Tij = (unsigned int)ptTemplate[point].val;
      if (!blur)
        IW = (unsigned int)I.getValue(i2, j2);
      else
        IW = (unsigned int)GaussI.getValue(i2, j2);

      Prt256[Tij][IW]++;
      Pr256[Tij]++;
      Pt256[IW]++;
    }
  }

  if (Nbpoint == 0) {
    throw(vpException(vpException::divideByZeroError, "Cannot get MI; number of points = 0"));
  }
  Prt256 = Prt256 / Nbpoint;
  Pr256 = Pr256 / Nbpoint;
  Pt256 = Pt256 / Nbpoint;

  double MI = 0;

  for (unsigned int t = 0; t < 256; t++) {
    for (unsigned int r = 0; r < 256; r++) {
      // if(Prt256[r][t]!=0)
      if (std::fabs(Prt256[r][t]) > std::numeric_limits<double>::epsilon())
        MI += Prt256[r][t] * log(Prt256[r][t]);
    }
    // if(Pt256[t]!=0)
    if (std::fabs(Pt256[t]) > std::numeric_limits<double>::epsilon())
      MI += -Pt256[t] * log(Pt256[t]);
    // if(Pr256[t]!=0)
    if (std::fabs(Pr256[t]) > std::numeric_limits<double>::epsilon())
      MI += -Pr256[t] * log(Pr256[t]);
  }
  return MI;
}
