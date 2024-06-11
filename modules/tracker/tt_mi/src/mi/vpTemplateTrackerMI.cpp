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
 * Example of template tracking.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/
#include <visp3/core/vpException.h>
#include <visp3/tt_mi/vpTemplateTrackerMI.h>
#include <visp3/tt_mi/vpTemplateTrackerMIBSpline.h>

BEGIN_VISP_NAMESPACE
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

  PrtD = new double[Nc * Nc * influBspline];
  dPrtD = new double[Nc * Nc * (int)(nbParam)*influBspline];
  PrtTout = new double[Nc * Nc * influBspline * (1 + (int)(nbParam + nbParam * nbParam))];

  hessianComputation = USE_HESSIEN_DESIRE;
}

vpTemplateTrackerMI::vpTemplateTrackerMI(vpTemplateTrackerWarp *_warp)
  : vpTemplateTracker(_warp), hessianComputation(USE_HESSIEN_NORMAL), ApproxHessian(HESSIAN_NEW), lambda(0), temp(nullptr),
  Prt(nullptr), dPrt(nullptr), Pt(nullptr), Pr(nullptr), d2Prt(nullptr), PrtTout(nullptr), dprtemp(nullptr), PrtD(nullptr), dPrtD(nullptr),
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

  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    X1[0] = ptTemplate[point].x;
    X1[1] = ptTemplate[point].y;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;

      double Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);

      double Nc_1 = (Nc - 1.) / 255.;
      double IW_Nc = IW * Nc_1;
      double Tij_Nc = Tij * Nc_1;
      int cr = static_cast<int>(IW_Nc);
      int ct = static_cast<int>(Tij_Nc);
      double er = IW_Nc - cr;
      double et = Tij_Nc - ct;

      // Calcul de l'histogramme joint par interpolation bilineaire
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
  for (unsigned int r = 0; r < Ncb_; r++) {
    for (unsigned int t = 0; t < Ncb_; t++) {
      Prt[r * Ncb_ + t] /= Nbpoint;
    }
  }
  // Compute Pr, Pt
  memset(Pr, 0, Ncb_ * sizeof(double));
  memset(Pt, 0, Ncb_ * sizeof(double));
  for (unsigned int r = 0; r < Ncb_; r++) {
    for (unsigned int t = 0; t < Ncb_; t++) {
      Pr[r] += Prt[r * Ncb_ + t];
      Pt[r] += Prt[t * Ncb_ + r];
    }
  }

  for (unsigned int r = 0; r < Ncb_; r++) {
    if (std::fabs(Pr[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pr[r] * log(Pr[r]);
    }
    if (std::fabs(Pt[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pt[r] * log(Pt[r]);
    }
    for (unsigned int t = 0; t < Ncb_; t++) {
      unsigned int r_Ncb_t_ = r * Ncb_ + t;
      if (std::fabs(Prt[r_Ncb_t_]) > std::numeric_limits<double>::epsilon()) {
        MI += Prt[r_Ncb_t_] * log(Prt[r_Ncb_t_]);
      }
    }
  }

  return -MI;
}

double vpTemplateTrackerMI::getNormalizedCost(const vpImage<unsigned char> &I, const vpColVector &tp)
{
  // Attention, cette version calculee de la NMI ne pourra pas atteindre le
  // maximum de 2. Ceci est du au fait que par defaut, l'image est floutee dans
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
    X1[0] = ptTemplate[point].x;
    X1[1] = ptTemplate[point].y;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth() - 1)) {
      Nbpoint++;
      double Tij = ptTemplate[point].val;
      int Tij_ = static_cast<int>(Tij);
      if (!blur) {
        IW = I[(int)i2][(int)j2];
      }
      else {
        IW = BI.getValue(i2, j2);
      }
      int IW_ = static_cast<int>(IW);

      Pr_[Tij_]++;
      Pt_[IW_]++;
      Prt_[Tij_][IW_]++;
    }
  }

  double denom = 0;
  for (int r = 0; r < 256; r++) {
    Pr_[r] /= Nbpoint;
    Pt_[r] /= Nbpoint;
    if (std::fabs(Pr_[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pr_[r] * log(Pr_[r]);
    }
    if (std::fabs(Pt_[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pt_[r] * log(Pt_[r]);
    }
    for (int t = 0; t < 256; t++) {
      Prt_[r][t] /= Nbpoint;
      if (std::fabs(Prt_[r][t]) > std::numeric_limits<double>::epsilon()) {
        denom -= (Prt_[r][t] * log(Prt_[r][t]));
      }
    }
  }

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
  unsigned int Nc_ = static_cast<unsigned int>(Nc);
  unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
  unsigned int bspline_ = static_cast<unsigned int>(bspline);

  for (unsigned int r = 0; r < Nc_; r++) {
    for (unsigned int t = 0; t < Nc_; t++) {
      for (unsigned int r2 = 0; r2 < bspline_; r2++) {
        unsigned int r2_r_Ncb_ = (r2 + r) * Ncb_;
        for (unsigned int t2 = 0; t2 < bspline_; t2++) {
          unsigned int t2_t_ = t2 + t;
          unsigned int r2_r_Ncb_t2_t_nbParam_ = (r2_r_Ncb_ + t2_t_) * nbParam;
          Prt[r2_r_Ncb_ + t2_t_] += *pt++;
          for (unsigned int ip = 0; ip < nbParam; ip++) {
            dPrt[r2_r_Ncb_t2_t_nbParam_ + ip] += *pt++;
            unsigned int ip_nbParam_ = ip * nbParam;
            for (unsigned int it = 0; it < nbParam; it++) {
              d2Prt[r2_r_Ncb_t2_t_nbParam_ * nbParam + ip_nbParam_ + it] += *pt++;
            }
          }
        }
      }
    }
  }

  if (nbpoint == 0) {
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "No points in the template"));
  }
  unsigned int indd, indd2;
  indd = indd2 = 0;
  for (volatile int i = 0; i < Ncb * Ncb; i++) {
    Prt[i] = Prt[i] / nbpoint;
    for (unsigned int j = 0; j < nbParam; j++) {
      dPrt[indd] /= nbpoint;
      indd++;
      for (unsigned int k = 0; k < nbParam; k++) {
        d2Prt[indd2] /= nbpoint;
        indd2++;
      }
    }
  }
}

void vpTemplateTrackerMI::computeMI(double &MI)
{
  unsigned int Ncb_ = (unsigned int)Ncb;

  // Compute Pr and Pt
  memset(Pr, 0, Ncb_ * sizeof(double));
  memset(Pt, 0, Ncb_ * sizeof(double));
  for (unsigned int r = 0; r < Ncb_; r++) {
    unsigned int r_Nbc_ = r * Ncb_;
    for (unsigned int t = 0; t < Ncb_; t++) {
      Pr[r] += Prt[r_Nbc_ + t];
      Pt[r] += Prt[r + Ncb_ * t];
    }
  }

  // Compute Entropy
  for (unsigned int r = 0; r < Ncb_; r++) {
    if (std::fabs(Pr[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pr[r] * log(Pr[r]);
    }
    if (std::fabs(Pt[r]) > std::numeric_limits<double>::epsilon()) {
      MI -= Pt[r] * log(Pt[r]);
    }
    unsigned int r_Nbc_ = r * Ncb_;
    for (unsigned int t = 0; t < Ncb_; t++) {
      unsigned int r_Nbc_t_ = r_Nbc_ + t;
      if (std::fabs(Prt[r_Nbc_t_]) > std::numeric_limits<double>::epsilon()) {
        MI += Prt[r_Nbc_t_] * log(Prt[r_Nbc_t_]);
      }
    }
  }
}

void vpTemplateTrackerMI::computeHessien(vpMatrix &Hessian)
{
  double seuilevitinf = 1e-200;
  Hessian = 0;
  double dtemp;
  unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
  unsigned int nbParam2 = nbParam * nbParam;

  for (unsigned int t = 0; t < Ncb_; t++) {
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        if (Prt[r * Ncb_ + t] > seuilevitinf) {
          unsigned int r_Ncb_t_ = r * Ncb_ + t;
          unsigned int r_Ncb_t_nbParam_ = r_Ncb_t_ * nbParam;
          for (unsigned int it = 0; it < nbParam; it++) {
            dprtemp[it] = dPrt[r_Ncb_t_nbParam_ + it];
          }

          dtemp = 1. + log(Prt[r * Ncb_ + t] / Pt[t]);

          double Prt_Pt_ = 1. / Prt[r_Ncb_t_] - 1. / Pt[t];
          unsigned int r_Ncb_t_nbParam2_ = r_Ncb_t_ * nbParam2;
          for (unsigned int it = 0; it < nbParam; it++) {
            unsigned int r_Ncb_t_nbParam2_it_nbParam_ = r_Ncb_t_nbParam2_ + it * nbParam;
            for (unsigned int jt = 0; jt < nbParam; jt++) {
              if (ApproxHessian != HESSIAN_NONSECOND && ApproxHessian != HESSIAN_NEW)
                Hessian[it][jt] +=
                dprtemp[it] * dprtemp[jt] * Prt_Pt_ + d2Prt[r_Ncb_t_nbParam2_it_nbParam_ + jt] * dtemp;
              else if (ApproxHessian == HESSIAN_NEW)
                Hessian[it][jt] += d2Prt[r_Ncb_t_nbParam2_it_nbParam_ + jt] * dtemp;
              else
                Hessian[it][jt] += dprtemp[it] * dprtemp[jt] * Prt_Pt_;
            }
          }
        }
      }
    }
  }
}

void vpTemplateTrackerMI::computeHessienNormalized(vpMatrix &Hessian)
{
  double seuilevitinf = 1e-200;
  double u = 0, v = 0, B = 0;
  m_du.resize(nbParam);
  m_dv.resize(nbParam);
  m_A.resize(nbParam);
  m_dB.resize(nbParam);
  m_d2u.resize(nbParam);
  m_d2v.resize(nbParam);
  m_dA.resize(nbParam);

  for (unsigned int i = 0; i < nbParam; i++) {
    m_d2u[i].resize(nbParam);
    m_d2v[i].resize(nbParam);
    m_dA[i].resize(nbParam);
  }

  std::fill(m_du.begin(), m_du.end(), 0);
  std::fill(m_dv.begin(), m_dv.end(), 0);
  std::fill(m_A.begin(), m_A.end(), 0);
  std::fill(m_dB.begin(), m_dB.end(), 0);
  for (unsigned int it = 0; it < nbParam; it++) {
    std::fill(m_d2u[0].begin(), m_d2u[0].end(), 0);
    std::fill(m_d2v[0].begin(), m_d2v[0].end(), 0);
    std::fill(m_dA[0].begin(), m_dA[0].end(), 0);
  }

  memset(dprtemp, 0, nbParam * sizeof(double));

  unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
  unsigned int nbParam2 = nbParam * nbParam;

  for (unsigned int t = 0; t < Ncb_; t++) {
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        unsigned int r_Ncb_t_ = r * Ncb_ + t;
        if (Prt[r_Ncb_t_] > seuilevitinf) {
          unsigned int r_Ncb_t_nbParam_ = r_Ncb_t_ * nbParam;
          for (unsigned int it = 0; it < nbParam; it++) {
            // dPxy/dt
            dprtemp[it] = dPrt[r_Ncb_t_nbParam_ + it];
          }
          double log_Pt_Pr_ = log(Pt[t] * Pr[r]);
          double log_Prt_ = log(Prt[r_Ncb_t_]);
          // u = som(Pxy.logPxPy)
          u += Prt[r_Ncb_t_] * log_Pt_Pr_;
          // v = som(Pxy.logPxy)
          v += Prt[r_Ncb_t_] * log_Prt_;

          double log_Prt_1_ = 1 + log(Prt[r_Ncb_t_]);
          for (unsigned int it = 0; it < nbParam; it++) {
            // u' = som dPxylog(PxPy)
            m_du[it] += dprtemp[it] * log_Pt_Pr_;
            // v' = som dPxy(1+log(Pxy))
            m_dv[it] += dprtemp[it] * log_Prt_1_;
          }
          double Prt_ = 1.0 / Prt[r_Ncb_t_];
          unsigned int r_Ncb_t_nbParam2_ = r_Ncb_t_ * nbParam2;
          for (unsigned int it = 0; it < nbParam; it++) {
            double dprtemp_it2_ = Prt_ * dprtemp[it] * dprtemp[it];
            unsigned int r_Ncb_t_nbParam2_it_nbParam_ = r_Ncb_t_nbParam2_ + it * nbParam;
            for (unsigned int jt = 0; jt < nbParam; jt++) {
              unsigned int r_Ncb_t_nbParam2_it_nbParam_jt_ = r_Ncb_t_nbParam2_it_nbParam_ + jt;
              m_d2u[it][jt] += d2Prt[r_Ncb_t_nbParam2_it_nbParam_jt_] * log_Pt_Pr_ + dprtemp_it2_;
              m_d2v[it][jt] += d2Prt[r_Ncb_t_nbParam2_it_nbParam_jt_] * log_Prt_1_ + dprtemp_it2_;
            }
          }
        }
      }
    }
  }
  // B = v2
  B = (v * v);
  double B2 = B * B;
  for (unsigned int it = 0; it < nbParam; it++) {
    // A = u'v-uv'
    m_A[it] = m_du[it] * v - u * m_dv[it];
    // B' = 2vv'
    m_dB[it] = 2 * v * m_dv[it];
    double A_it_dB_it_ = m_A[it] * m_dB[it];
    for (unsigned int jt = 0; jt < nbParam; jt++) {
      // A' = u''v-v''u
      m_dA[it][jt] = m_d2u[it][jt] * v - m_d2v[it][jt] * u;
      // Hessian = (A'B-AB')/B2
      Hessian[it][jt] = (m_dA[it][jt] * B - A_it_dB_it_) / B2;
    }
  }
}

void vpTemplateTrackerMI::computeGradient()
{
  double seuilevitinf = 1e-200;
  G = 0;
  unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
  double dtemp;
  for (unsigned int t = 0; t < Ncb_; t++) {
    if (Pt[t] > seuilevitinf) {
      for (unsigned int r = 0; r < Ncb_; r++) {
        unsigned int r_Ncb_t_ = r * Ncb_ + t;
        if (Prt[r_Ncb_t_] > seuilevitinf) {
          unsigned int r_Ncb_t_nbParam_ = r_Ncb_t_ * nbParam;
          for (unsigned int it = 0; it < nbParam; it++) {
            dprtemp[it] = dPrt[r_Ncb_t_nbParam_ + it];
          }

          dtemp = 1. + log(Prt[r_Ncb_t_] / Pt[t]);

          for (unsigned int it = 0; it < nbParam; it++) {
            G[it] += dtemp * dprtemp[it];
          }
        }
      }
    }
  }
}

void vpTemplateTrackerMI::zeroProbabilities()
{
  unsigned int Ncb_ = static_cast<unsigned int>(Ncb);
  unsigned int Nc_ = static_cast<unsigned int>(Nc);
  unsigned int influBspline_ = static_cast<unsigned int>(influBspline);

  unsigned int Ncb2_ = Ncb_ * Ncb_ * sizeof(double);
  unsigned int Ncb2_nbParam_ = Ncb2_ * nbParam;
  unsigned int Ncb2_nbParam2_ = Ncb2_nbParam_ * nbParam;

  memset(Prt, 0, Ncb2_);
  memset(dPrt, 0, Ncb2_nbParam_);
  memset(d2Prt, 0, Ncb2_nbParam2_);
  memset(PrtTout, 0, Nc_ * Nc_ * influBspline_ * (1 + nbParam + nbParam * nbParam) * sizeof(double));
}

double vpTemplateTrackerMI::getMI(const vpImage<unsigned char> &I, int &nc, const int &bspline_, vpColVector &tp)
{
  unsigned int tNcb = static_cast<unsigned int>(nc + bspline_);
  unsigned int tinfluBspline = static_cast<unsigned int>(bspline_ * bspline_);
  unsigned int nc_ = static_cast<unsigned int>(nc);

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

  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    X1[0] = ptTemplate[point].x;
    X1[1] = ptTemplate[point].y;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth()) - 1) {
      Nbpoint++;

      double Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = GaussI.getValue(i2, j2);

      int cr = static_cast<int>((IW * (nc - 1)) / 255.);
      int ct = static_cast<int>((Tij * (nc - 1)) / 255.);
      double er = (IW * (nc - 1)) / 255. - cr;
      double et = (Tij * (nc - 1)) / 255. - ct;

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
  }
  else {
    for (unsigned int r = 0; r < tNcb; r++)
      for (unsigned int t = 0; t < tNcb; t++)
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
      if (std::fabs(tPr[r]) > std::numeric_limits<double>::epsilon())
        MI -= tPr[r] * log(tPr[r]);

    for (unsigned int t = 0; t < tNcb; t++)
      if (std::fabs(tPt[t]) > std::numeric_limits<double>::epsilon())
        MI -= tPt[t] * log(tPt[t]);

    for (unsigned int r = 0; r < tNcb; r++)
      for (unsigned int t = 0; t < tNcb; t++)
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

  Warp->computeCoeff(tp);
  for (unsigned int point = 0; point < templateSize; point++) {
    X1[0] = ptTemplate[point].x;
    X1[1] = ptTemplate[point].y;

    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);
    double j2 = X2[0];
    double i2 = X2[1];

    if ((i2 >= 0) && (j2 >= 0) && (i2 < I.getHeight() - 1) && (j2 < I.getWidth()) - 1) {
      Nbpoint++;

      Tij = static_cast<unsigned int>(ptTemplate[point].val);
      if (!blur)
        IW = static_cast<unsigned int>(I.getValue(i2, j2));
      else
        IW = static_cast<unsigned int>(GaussI.getValue(i2, j2));

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
      if (std::fabs(Prt256[r][t]) > std::numeric_limits<double>::epsilon())
        MI += Prt256[r][t] * log(Prt256[r][t]);
    }
    if (std::fabs(Pt256[t]) > std::numeric_limits<double>::epsilon())
      MI += -Pt256[t] * log(Pt256[t]);
    if (std::fabs(Pr256[t]) > std::numeric_limits<double>::epsilon())
      MI += -Pr256[t] * log(Pr256[t]);
  }
  return MI;
}
END_VISP_NAMESPACE
