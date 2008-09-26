/****************************************************************************
 *
 * $Id: vpRobust.h,v 1.7 2008-09-26 15:20:55 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * M-Estimator and various influence function.
 *
 * Authors:
 * Andrew Comport
 *
 *****************************************************************************/

/*!
 \file vpRobust.h
*/



#ifndef CROBUST_HH
#define CROBUST_HH

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>


/*!
  \class vpRobust
  \ingroup Robust
  \brief Contains an M-Estimator and various influence function.

  Supported methods: M-estimation, Tukey, Cauchy and Huber
*/
class VISP_EXPORT vpRobust
{
public:
  typedef enum
  {
    TUKEY,
    CAUCHY,
    //    MCLURE,
    HUBER
  } vpRobustEstimatorType;
private:

  double sig_prev;
  int it;

public:
  vpColVector w;

  double NoiseThreshold;

  vpRobust(int n_data);
  virtual ~vpRobust(void);

  int MEstimator(const vpRobustEstimatorType method,
		 const vpColVector &residues,
		 vpColVector &weights);

  int MEstimator(const vpRobustEstimatorType method,
		 const vpColVector &residues,
		 const vpColVector& all_residues,
		 vpColVector &weights);

  vpColVector simultMEstimator(vpColVector &residues);

  void setIteration(const int iter);
  void setThreshold(const double x);

 public :
  double residualMedian ;
  double normalizedResidualMedian ;
 private:

  double median(vpColVector &x);
  double median(vpColVector &x, vpColVector &weights);

  double computeNormalizedMedian(vpColVector &all_normres,
				 const vpColVector &residues,
				 const vpColVector &all_residues);


  //! Calculate various scale estimates
  double scale(vpRobustEstimatorType method, vpColVector &x);
  double simultscale(vpColVector &x);

  //! Partial derivative of loss function
  //! with respect to the residue
  int psiTukey(double sigma, vpColVector &x);
  int psiCauchy(double sigma, vpColVector &x);
  int psiMcLure(double sigma, vpColVector &x);
  int psiHuber(double sigma, vpColVector &x);

  //! Partial derivative of loss function
  //! with respect to the scale
  double simult_chi_huber(double x);

  //! Constrained Partial derivative of loss function
  //! with respect to the scale
  double constrainedChi(vpRobustEstimatorType method, double x);
  double constrainedChiTukey(double x);
  double constrainedChiCauchy(double x);
  double constrainedChiHuber(double x);

  //! Used to calculate the Expectation
  double erf(double x);
  double gammp(double a, double x);
  void gser(double *gamser, double a, double x, double *gln);
  void gcf(double *gammcf, double a, double x, double *gln);
  double gammln(double xx);

  // pour le calcul de la mediane
  void exch(double &A, double &B);
  int partition(vpColVector &a, int l, int r);
  double select(vpColVector &a, int l, int r, int k);
};

#endif
