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

#ifndef VP_UK_SIGMA_DRAWER_ABSTRACT_H
#define VP_UK_SIGMA_DRAWER_ABSTRACT_H

#include <vector>

#include <visp3/core/vpConfig.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <visp3/core/vpColVector.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpUKSigmaDrawerAbstract
  \ingroup group_core_kalman
  This abstract class defines the interface to draw the sigma points for the Unscented Kalman filter.
*/
class VISP_EXPORT vpUKSigmaDrawerAbstract
{
public:
  /**
   * \brief The weights corresponding to the sigma points drawing.
   */
  typedef struct vpSigmaPointsWeights
  {
    std::vector<double> m_wm; /*!< The weights for the computation of the mean.*/
    std::vector<double> m_wc; /*!< The weights for the computation of the covariance.*/
  }vpSigmaPointsWeights;

  inline vpUKSigmaDrawerAbstract(const unsigned int &n) : m_n(n) { }

  /**
   * \brief Draw the sigma points according to the current mean and covariance of the state
   * of the Unscented Kalman filter.
   *
   * \param[in] mean The current mean of the state of the UKF.
   * \param[in] covariance The current process covariance of the UKF.
   * @return std::vector<vpColVector> The sigma points.
   */
  virtual std::vector<vpColVector> drawSigmaPoints(const vpColVector &mean, const vpMatrix &covariance) = 0;

  /**
   * \brief Computed the weigths that correspond to the sigma poitns that have been drawn.
   *
   * \return vpSigmaPointsWeights The weights that correspond to the sigma points.
   */
  virtual vpSigmaPointsWeights computeWeights() = 0;
protected:
  unsigned int m_n; /*!< The size of the state of the UKF.*/
};
END_VISP_NAMESPACE
#endif
#endif
