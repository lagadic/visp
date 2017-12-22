/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
#ifndef vpTemplateTrackerMI_hh
#define vpTemplateTrackerMI_hh

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpImageFilter.h>
#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>

/*!
  \class vpTemplateTrackerMI
  \ingroup group_tt_mi_tracker
*/
class VISP_EXPORT vpTemplateTrackerMI : public vpTemplateTracker
{
public:
  /*! Hessian approximation. */
  typedef enum {
    HESSIAN_NONSECOND = -1,
    HESSIAN_0,
    HESSIAN_d2I,
    HESSIAN_YOUCEF,
    HESSIAN_EXACT,
    HESSIAN_NEW
  } vpHessienApproximationType;

  /*! Hessian computation. */
  typedef enum { USE_HESSIEN_NORMAL, USE_HESSIEN_DESIRE, USE_HESSIEN_BEST_COND } vpHessienType;

  /*! Hessian computation. */
  typedef enum { BSPLINE_THIRD_ORDER = 3, BSPLINE_FOURTH_ORDER = 4 } vpBsplineType;

protected:
  vpHessienType hessianComputation;
  vpHessienApproximationType ApproxHessian;
  double lambda;

  double *temp;
  double *Prt;
  double *dPrt;
  double *Pt;
  double *Pr;
  double *d2Prt;
  double *PrtTout;
  double *dprtemp;

  double *PrtD;
  double *dPrtD;
  int influBspline;

  int bspline;
  // Nombre de couleur concidere dans l'histogramme
  int Nc;
  int Ncb;

  vpImage<double> d2Ix;
  vpImage<double> d2Iy;
  vpImage<double> d2Ixy;

  double MI_preEstimation;
  double MI_postEstimation;

  double NMI_preEstimation;
  double NMI_postEstimation;

  vpMatrix covarianceMatrix;
  bool computeCovariance;

protected:
  void computeGradient();
  void computeHessien(vpMatrix &H);
  void computeHessienNormalized(vpMatrix &H);
  void computeMI(double &MI);
  void computeProba(int &nbpoint);
  double getCost(const vpImage<unsigned char> &I, const vpColVector &tp);
  double getCost(const vpImage<unsigned char> &I) { return getCost(I, p); }
  double getNormalizedCost(const vpImage<unsigned char> &I, const vpColVector &tp);
  double getNormalizedCost(const vpImage<unsigned char> &I) { return getNormalizedCost(I, p); }
  virtual void initHessienDesired(const vpImage<unsigned char> &I) = 0;
  virtual void trackNoPyr(const vpImage<unsigned char> &I) = 0;
  void zeroProbabilities();

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpTemplateTrackerMI(const vpTemplateTrackerMI &)
  //    : vpTemplateTracker(), hessianComputation(USE_HESSIEN_NORMAL),
  //    ApproxHessian(HESSIAN_0), lambda(0),
  //      temp(NULL), Prt(NULL), dPrt(NULL), Pt(NULL), Pr(NULL), d2Prt(NULL),
  //      PrtTout(NULL), dprtemp(NULL), PrtD(NULL), dPrtD(NULL),
  //      influBspline(0), bspline(0), Nc(0), Ncb(0), d2Ix(), d2Iy(), d2Ixy(),
  //      MI_preEstimation(0), MI_postEstimation(0), NMI_preEstimation(0),
  //      NMI_postEstimation(0), covarianceMatrix(), computeCovariance(false)
  //  {
  //    throw vpException(vpException::functionNotImplementedError, "Not
  //    implemented!");
  //  }
  //    vpTemplateTrackerMI &operator=(const vpTemplateTrackerMI &){
  //      throw vpException(vpException::functionNotImplementedError, "Not
  //      implemented!"); return *this;
  //    }
  //#endif

public:
  // constructeur
  //! Default constructor.
  vpTemplateTrackerMI()
    : vpTemplateTracker(), hessianComputation(USE_HESSIEN_NORMAL), ApproxHessian(HESSIAN_0), lambda(0), temp(NULL),
      Prt(NULL), dPrt(NULL), Pt(NULL), Pr(NULL), d2Prt(NULL), PrtTout(NULL), dprtemp(NULL), PrtD(NULL), dPrtD(NULL),
      influBspline(0), bspline(0), Nc(0), Ncb(0), d2Ix(), d2Iy(), d2Ixy(), MI_preEstimation(0), MI_postEstimation(0),
      NMI_preEstimation(0), NMI_postEstimation(0), covarianceMatrix(), computeCovariance(false)
  {
  }
  explicit vpTemplateTrackerMI(vpTemplateTrackerWarp *_warp);
  ~vpTemplateTrackerMI();
  vpMatrix getCovarianceMatrix() const { return covarianceMatrix; }
  double getMI() const { return MI_postEstimation; }
  double getMI(const vpImage<unsigned char> &I, int &nc, const int &bspline, vpColVector &tp);
  double getMI256(const vpImage<unsigned char> &I, const vpColVector &tp);
  double getNMI() const { return NMI_postEstimation; }
  // initialisation du Hessien en position desiree
  void setApprocHessian(vpHessienApproximationType approx) { ApproxHessian = approx; }
  void setCovarianceComputation(const bool &flag) { computeCovariance = flag; }
  void setHessianComputation(vpHessienType type) { hessianComputation = type; }
  void setBspline(const vpBsplineType &newbs);
  void setLambda(double _l) { lambda = _l; }
  void setNc(int newNc);
};

#endif
