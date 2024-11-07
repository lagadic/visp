/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * M-Estimator and various influence function.
 */

/*!
  \file vpRobust.h
*/

#ifndef VP_ROBUST_H
#define VP_ROBUST_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpRobust
  \ingroup group_core_robust
  \brief Contains an M-estimator and various influence function.

  This class implements an M-estimator with Tukey, Cauchy or Huber influence function \cite PhDComport
  which allow uncertain measures to be less likely considered and in some cases completely
  rejected, thus inferring that the data is not normally distributed.

  When using a robust estimate of the mean, it is usual to normalize the distribution to center
  the data around zero. In the case of a median operator, the normalized residue is given by:

  \f$\overline{r_i} = r_i - {Med}(r_i) \f$ where \f${Med}(r_i)\f$ is the median value of the residue vector \f$r\f$.

  The Median Absolute Deviation (MAD) representing one standard deviation of the normal distribution is given by:
  \f[ \sigma = 1.48 \; {Med}(|\overline{r_i}|) \f]

  This class allows to set the minimum value of \f$ \sigma \f$ using setMinMedianAbsoluteDeviation().

  This estimated standard deviation \f$\sigma\f$ can accordingly be used with a tuning variable for different
  influence functions.

  Let us consider the weight function \f$w(r)\f$ with \f$r\f$ the residual vector of the parameters to estimate.
  - Using Huber influence function, with \f$a\f$ a constant such as \f$a=1.21 \; \sigma \f$ we have
  \f[ w(r_i) = \left\{ \begin{array}{ll} 1 &
  \mbox{if } |r_i| \leq a \\ \frac{a}{|r_i|} & \mbox{else} \end{array} \right. \f]

  - Using Tukey influence function, with \f$b\f$ a constant such as \f$b=4.68 \; \sigma \f$ we have
  \f[ w(r_i) = \left\{ \begin{array}{ll} {\left(1 - {\left(\frac{r_i}{b}\right)}^2 \right)}^2 &
  \mbox{if } |r_i| \leq b \\ 0 & \mbox{else} \end{array} \right. \f]

  - Using Cauchy influence function, with \f$c\f$ a constant such as \f$c=2.38 \; \sigma \f$ we have
  \f[ w(r_i) = \frac{1}{(1 + {(r_i/c)}^2)} \f]

  Given the influence function and the residual vector, the weights are updated in MEstimator().

*/
class VISP_EXPORT vpRobust
{
public:
  //! Enumeration of influence functions
  typedef enum
  {
    TUKEY,  //!< Tukey influence function.
    CAUCHY, //!< Cauchy influence function.
    HUBER   //!< Huber influence function.
  } vpRobustEstimatorType;

public:
  vpRobust();
  vpRobust(const vpRobust &other);

  //! Destructor
  virtual ~vpRobust() { };

  /*!
   * Return residual vector Median Absolute Deviation (MAD).
   * This value is updated after a call to MEstimator(). It corresponds to
   * value of \f$ \sigma = 1.48{Med}(|r_i - {Med}(r_i)|) \f$.
   * This value cannot be lower than the min value returned by getMinMedianAbsoluteDeviation()
   * or set with setMinMedianAbsoluteDeviation().
   *
   * \sa setMinMedianAbsoluteDeviation()
   */
  double getMedianAbsoluteDeviation() { return m_mad; };

  /*!
   * Return the min value used to threshold residual vector Median Absolute Deviation (MAD).
   * This value corresponds to the minimal value of \f$\sigma\f$ computed in MEstimator().
   *
   * \sa setMinMedianAbsoluteDeviation()
   */
  double getMinMedianAbsoluteDeviation() { return m_mad_min; };

  void MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, vpColVector &weights);

  vpRobust &operator=(const vpRobust &other);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRobust &operator=(const vpRobust &&other);
#endif

  /*!
   * Set minimal median absolute deviation (MAD) value corresponding to the minimal value of
   * \f$\sigma\f$ computed in MEstimator() with
   * \f$ \sigma = 1.48{Med}(|r_i - {Med}(r_i)|) \f$.
   * \param mad_min : Minimal Median Absolute Deviation value.
   * Default value is set to 0.0017 in the default constructor.
   *
   * \sa getMinMedianAbsoluteDeviation()
   */
  inline void setMinMedianAbsoluteDeviation(double mad_min) { m_mad_min = mad_min; }

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  VP_DEPRECATED VP_EXPLICIT vpRobust(unsigned int n_data);
  //! Compute the weights according a residue vector and a PsiFunction
  void MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, const vpColVector &all_residues,
                  vpColVector &weights);
  /*!
   * \deprecated Set iteration. This function is to call before simultMEstimator().
   * \param iter : The first call iter should be set to 0.
   */
  VP_DEPRECATED void setIteration(unsigned int iter) { m_iter = iter; }
  /*!
    \deprecated You should rather use setMinMedianAbsoluteDeviation().
    Set minimal median absolute deviation (MAD) value.
    Given the input vector or residual, when MAD(residual) < mad_min
    we set MAD(residual) = mad_min.
    \param mad_min : Minimal Median Absolute Deviation value.
    Default value is set to 0.0017 in the default constructor.
  */
  VP_DEPRECATED inline void setThreshold(double mad_min) { m_mad_min = mad_min; }
  VP_DEPRECATED vpColVector simultMEstimator(vpColVector &residues);
  //@}
#endif
private:
  //! Normalized residue
  vpColVector m_normres;
  //! Sorted normalized Residues
  vpColVector m_sorted_normres;
  //! Sorted residues
  vpColVector m_sorted_residues;

  //! Min admissible value of residual vector Median Absolute Deviation
  double m_mad_min;
  //! Previous value of residual vector Median Absolute Deviation
  double m_mad_prev;
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  //! Iteration, only used in deprecated simultMEstimator()
  unsigned int m_iter;
#endif
  //! Size of the containers
  unsigned int m_size;
  //! Residual vector Median Absolute Deviation
  double m_mad;

private:
  //! Resize containers for sort methods
  void resize(unsigned int n_data);

  //---------------------------------
  //  Partial derivative of loss function with respect to the residue
  //---------------------------------
  /** @name PsiFunctions  */
  //@{
  //! Tuckey influence function
  void psiTukey(double sigma, const vpColVector &x, vpColVector &w);
  //! Caucht influence function
  void psiCauchy(double sigma, const vpColVector &x, vpColVector &w);
  //! Huber influence function
  void psiHuber(double sigma, const vpColVector &x, vpColVector &w);
  //@}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  double computeNormalizedMedian(vpColVector &all_normres, const vpColVector &residues, const vpColVector &all_residues,
                                 const vpColVector &weights);
  //! Calculate various scale estimates
  double simultscale(const vpColVector &x);
  //! Partial derivative of loss function with respect to the scale
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
#endif

  /** @name Sort function  */
  //@{
  //! Sort function using partition method
  int partition(vpColVector &a, int l, int r);
  //! Sort the vector and select a value in the sorted vector
  double select(vpColVector &a, int l, int r, int k);
  //@}
};
END_VISP_NAMESPACE
#endif
