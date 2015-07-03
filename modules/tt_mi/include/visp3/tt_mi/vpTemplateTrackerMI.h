/****************************************************************************
 *
 * $Id: templateTracker.cpp 4428 2013-09-06 13:51:34Z fspindle $
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

#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerHeader.h>
#include <visp3/core/vpImageFilter.h>

class VISP_EXPORT vpTemplateTrackerMI: public vpTemplateTracker
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
  typedef enum {
    USE_HESSIEN_NORMAL,
    USE_HESSIEN_DESIRE,
    USE_HESSIEN_BEST_COND
  } vpHessienType;

  /*! Hessian computation. */
  typedef enum {
    BSPLINE_THIRD_ORDER = 3,
    BSPLINE_FOURTH_ORDER = 4
  } vpBsplineType;

  protected:
    vpHessienType              hessianComputation;
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
    //Nombre de couleur concid�r� dans l'histogramme
    int Nc;
    int Ncb;

    vpImage<double> d2Ix ;
    vpImage<double> d2Iy ;
    vpImage<double> d2Ixy ;

    double MI_preEstimation;
    double MI_postEstimation;

    double NMI_preEstimation;
    double NMI_postEstimation;

    vpMatrix    covarianceMatrix;
    bool        computeCovariance;

  protected:
            void    computeGradient();
            void    computeHessien(vpMatrix &H);
            void    computeHessienNormalized(vpMatrix &H);
            void    computeMI(double &MI);
            void    computeProba(int &nbpoint);
            double  getCost(const vpImage<unsigned char> &I,vpColVector &tp);
            double  getCost(vpImage<unsigned char> &I){return getCost(I,p);}
            double  getNormalizedCost(const vpImage<unsigned char> &I,vpColVector &tp);
            double  getNormalizedCost(vpImage<unsigned char> &I){return getNormalizedCost(I,p);}
    virtual void    initHessienDesired(const vpImage<unsigned char> &I)=0;
    virtual void    trackNoPyr(const vpImage<unsigned char> &I)=0;
            void    zeroProbabilities();

  public:
  	//constructeur
    vpTemplateTrackerMI(vpTemplateTrackerWarp *_warp);
    ~vpTemplateTrackerMI();
    vpMatrix getCovarianceMatrix(){ return covarianceMatrix; }
    double getMI() const {return MI_postEstimation;}
    double getMI(vpImage<unsigned char> &I,int &nc,int &bspline,vpColVector &tp);
    double getMI256(vpImage<unsigned char> &I,vpColVector &tp);
    double getNMI() const {return NMI_postEstimation;}
    //initialisation du Hessien en position desiree
    void setApprocHessian(vpHessienApproximationType approx){ApproxHessian=approx;}
    void setCovarianceComputation(const bool & flag){ computeCovariance = flag; }
    void setHessianComputation(vpHessienType type){hessianComputation=type;}
    void setBspline(const vpBsplineType &newbs);
    void setLambda(double _l) {lambda = _l ; }
    void setNc(int newNc);
};
		
#endif

