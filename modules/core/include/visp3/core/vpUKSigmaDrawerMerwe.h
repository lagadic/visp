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

#ifndef _vpUKSigmaDrawerMerwe_h_
#define _vpUKSigmaDrawerMerwe_h_

#include <vector>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpUKSigmaDrawerAbstract.h>
#include <visp3/core/vpUnscentedKalman.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  \class vpUKSigmaDrawerMerwe
  \ingroup group_core_kalman
  This class defines a class to draw sigma points following the E. A. Wan and R. van der Merwe's method.
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
   */
  vpUKSigmaDrawerMerwe(const int &n, const double &alpha, const double &beta, const double &kappa,
                       const vpAddSubFunction &resFunc = vpUnscentedKalman::simpleResidual,
                       const vpAddSubFunction &addFunc = vpUnscentedKalman::simpleAdd);

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
    m_lambda = m_alpha * m_alpha * (m_n + m_kappa) - m_n;
  }

  double m_alpha;
  double m_beta;
  double m_kappa;
  double m_lambda;
  vpAddSubFunction m_resFunc;
  vpAddSubFunction m_addFunc;
};
#endif
#endif
