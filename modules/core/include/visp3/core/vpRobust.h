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
 * M-Estimator and various influence function.
 *
 * Authors:
 * Andrew Comport
 * Jean Laneurit
 *
 *****************************************************************************/

/*!
 \file vpRobust.h
*/

#ifndef CROBUST_HH
#define CROBUST_HH

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>

/*!
  \class vpRobust
  \ingroup group_core_robust
  \brief Contains an M-Estimator and various influence function.

  Supported methods: M-estimation, Tukey, Cauchy and Huber
*/
class VISP_EXPORT vpRobust
{

public:
  //! Enumeration of influence functions
  typedef enum { TUKEY, CAUCHY, HUBER } vpRobustEstimatorType;

private:
  //! Normalized residue
  vpColVector normres;
  //! Sorted normalized Residues
  vpColVector sorted_normres;
  //! Sorted residues
  vpColVector sorted_residues;

  //! Noise threshold
  double NoiseThreshold;
  //!
  double sig_prev;
  //!
  unsigned int it;
  //! Vairiable used in swap method
  double swap;
  //! Size of the containers
  unsigned int size;

public:
  //! Default Constructor
  explicit vpRobust(unsigned int n_data);
  vpRobust();
  vpRobust(const vpRobust &other);

  //! Destructor
  virtual ~vpRobust(){};

  //! Compute the weights according a residue vector and a PsiFunction
  void MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, vpColVector &weights);

  //! Compute the weights according a residue vector and a PsiFunction
  void MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, const vpColVector &all_residues,
                  vpColVector &weights);

  vpRobust &operator=(const vpRobust &other);
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  vpRobust &operator=(const vpRobust &&other);
#endif

  //! Resize containers for sort methods
  void resize(unsigned int n_data);

  //! Set iteration
  void setIteration(const unsigned int iter) { it = iter; }

  /*!
    Set maximal noise threshold.
    \param noise_threshold : Maximal noise threshold.
  */
  inline void setThreshold(const double noise_threshold) { NoiseThreshold = noise_threshold; }

  //! Simult Mestimator
  vpColVector simultMEstimator(vpColVector &residues);

  // public :
  // double residualMedian ;
  // double normalizedResidualMedian ;
  //  private:
  //   double median(const vpColVector &x);
  //   double median(const vpColVector &x, vpColVector &weights);

private:
  //! Compute normalized median
  double computeNormalizedMedian(vpColVector &all_normres, const vpColVector &residues, const vpColVector &all_residues,
                                 const vpColVector &weights);

  //! Calculate various scale estimates
  double simultscale(vpColVector &x);

  //---------------------------------
  //  Partial derivative of loss function with respect to the residue
  //---------------------------------
  /** @name PsiFunctions  */
  //@{
  //! Tuckey influence function
  void psiTukey(double sigma, vpColVector &x, vpColVector &w);
  //! Caucht influence function
  void psiCauchy(double sigma, vpColVector &x, vpColVector &w);
  //! Huber influence function
  void psiHuber(double sigma, vpColVector &x, vpColVector &w);
  //@}

  //! Partial derivative of loss function
  //! with respect to the scale
  double simult_chi_huber(double x);

  //---------------------------------
  // Constrained Partial derivative of loss function with respect to the scale
  //---------------------------------
  /** @name Constrained Chi Functions  */
  //@{
  //! Constrained Chi Function
  double constrainedChi(vpRobustEstimatorType method, double x);
  //! Constrained Chi Tukey Function
  double constrainedChiTukey(double x);
  //! Constrained Chi Cauchy Function
  double constrainedChiCauchy(double x);
  //! Constrained Chi Huber Function
  double constrainedChiHuber(double x);
//@}

#if !defined(VISP_HAVE_FUNC_ERFC) && !defined(VISP_HAVE_FUNC_STD_ERFC)
  //---------------------------------
  // Mathematic functions used to calculate the Expectation
  //---------------------------------
  /** @name Some math function  */
  //@{
  double erf(double x);
  double gammp(double a, double x);
  void gser(double *gamser, double a, double x, double *gln);
  void gcf(double *gammcf, double a, double x, double *gln);
  double gammln(double xx);
//@}
#endif

  /** @name Sort function  */
  //@{
  //! Swap two value
  void exch(double &A, double &B)
  {
    swap = A;
    A = B;
    B = swap;
  }
  //! Sort function using partition method
  int partition(vpColVector &a, int l, int r);
  //! Sort the vector and select a value in the sorted vector
  double select(vpColVector &a, int l, int r, int k);
  //@}
};

#endif
