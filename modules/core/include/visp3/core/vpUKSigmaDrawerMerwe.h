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
 * Display a point cloud using PCL library.
 */

#ifndef VP_UK_SIGMA_DRAWER_MERWE_H
#define VP_UK_SIGMA_DRAWER_MERWE_H

#include <vector>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpUKSigmaDrawerAbstract.h>
#include <visp3/core/vpUnscentedKalman.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
BEGIN_VISP_NAMESPACE
/*!
  \class vpUKSigmaDrawerMerwe
  \ingroup group_core_kalman
  This class defines a class to draw sigma points following the E. A. Wan and R. van der Merwe's method proposed in
  \cite Merwe00.

  The method has four parameters: \f$ n \f$, which is the dimension of the input, and \f$ \alpha \f$, \f$ \beta \f$ and
  \f$ \kappa \f$, which are three reals. For notational convenience, we define \f$ \lambda = \alpha^2 (n - \kappa) - n \f$.

  Be \f$ \boldsymbol{\mu} \in {R}^n \f$ the mean and \f$ \boldsymbol{\Sigma} \in {R}^{n x n} \f$ the covariance matrix of the
  input of the algorithm. The algorithm will draw \f$ 2n + 1 \f$ sigma points \f$ \chi_i \in {R}^n \f$ such as:

  \f[
  \begin{array}{lcl}
    \chi_0 &=& \boldsymbol{\mu} \\
    \chi_i &=& \begin{cases} \boldsymbol{\mu} + \left[ \sqrt{(n + \lambda) \boldsymbol{\Sigma}} \right]_i^T & i = 1 .. n \\
                             \boldsymbol{\mu} - \left[ \sqrt{(n + \lambda) \boldsymbol{\Sigma}} \right]_{i - n}^T & i = n + 1 .. 2n
               \end{cases}
  \end{array}
  \f]

  where the subscript \f$ i \f$ denotes that we keep the \f$ i^{th} \f$ of the matrix.

  Several definitions of the square root of a matrix exists. We decided to use the following definition: \f$ \textbf{L} \f$
  is the square root of the matrix \f$ \boldsymbol{\Sigma} \f$ if \f$ \boldsymbol{\Sigma} \f$ can be written as:

  \f$ \boldsymbol{\Sigma} = \textbf{L} \textbf{L}^T \f$

  This definition is favored because it can be computed using the Cholesky's decomposition.

  The computation of the weights that go along the sigma points is the following. The weight used for the
  computation of the mean of \f$ \chi_0 \f$ is computed such as:

  \f$ w_0^m = \frac{\lambda}{n + \lambda} \f$

  The weight used for the computation of the mean of \f$ \chi_0 \f$ is computed such as:

  \f$ w_0^c = \frac{\lambda}{n + \lambda} + 1 - \alpha^2 + \beta \f$

  The weights for the other sigma points \f$ \chi_1 ... \chi_{2n} \f$ are the same for the mean and covariance
  and are computed as follow:

  \f$ w_i^m = w_i^c = \frac{1}{2(n + \lambda)}  i = 1..2n \f$

  \b Note: the weights do not sum to one. Negative values can even be expected.

  \b Additionnal \b note: the original author recommended to set \f$ \beta = 2 \f$ for Gaussian problems, \f$ \kappa = 3 - n \f$
  and \f$ 0 \leq \alpha \leq 1 \f$, where a larger value for \f$ \alpha \f$ spreads the sigma points further from the mean,
  which can be a problem for highly non-linear problems.
*/
class VISP_EXPORT vpUKSigmaDrawerMerwe : public vpUKSigmaDrawerAbstract
{
public:
  typedef vpUnscentedKalman::vpAddSubFunction vpAddSubFunction;

  /**
   * \brief Construct a new vpUKSigmaDrawerMerwe object.
   *
   * \param[in] n The size of the state vector.
   * \param[in] alpha A factor, which should be a real in the interval [0; 1]. The larger alpha is,
   * the further the sigma points are spread from the mean.
   * \param[in] beta Another factor, which should be set to 2 if the problem is Gaussian.
   * \param[in] kappa A third factor, whose value should be set to 3 - n for most problems.
   * \param[in] resFunc Residual function expressed in the state space.
   * \param[in] addFunc Addition function expressed in the state space.
   */
  vpUKSigmaDrawerMerwe(const int &n, const double &alpha, const double &beta, const double &kappa,
                       const vpAddSubFunction &resFunc = vpUnscentedKalman::simpleResidual,
                       const vpAddSubFunction &addFunc = vpUnscentedKalman::simpleAdd);

  /**
   * Destructor that does nothing.
   */
  virtual ~vpUKSigmaDrawerMerwe() { }

  /**
   * \brief Draw the sigma points according to the current mean and covariance of the state
   * of the Unscented Kalman filter.
   *
   * \param[in] mean The current mean of the state of the UKF.
   * \param[in] covariance The current process covariance of the UKF.
   * @return std::vector<vpColVector> The sigma points.
   */
  virtual std::vector<vpColVector> drawSigmaPoints(const vpColVector &mean, const vpMatrix &covariance) override;

  /**
   * \brief Computed the weigths that correspond to the sigma poitns that have been drawn.
   *
   * \return vpSigmaPointsWeights The weights that correspond to the sigma points.
   */
  virtual vpSigmaPointsWeights computeWeights() override;
protected:
  inline void computeLambda()
  {
    m_lambda = m_alpha * m_alpha * (static_cast<double>(m_n) + m_kappa) - static_cast<double>(m_n);
  }

  double m_alpha; /*!< A factor, which should be a real in the interval [0; 1]. The larger alpha is,
                       the further the sigma points are spread from the mean.*/
  double m_beta; /*!< Another factor, which should be set to 2 if the problem is Gaussian.*/
  double m_kappa; /*!< A third factor, whose value should be set to 3 - n for most problems.*/
  double m_lambda; /*!< \f$ \alpha^2 (n + \kappa) - n \f$, where \f$ n \f$ is the size of the state vector.*/
  vpAddSubFunction m_resFunc; /*!< Residual function expressed in the state space.*/
  vpAddSubFunction m_addFunc; /*!< Addition function expressed in the state space.*/
};
END_VISP_NAMESPACE
#endif
#endif
