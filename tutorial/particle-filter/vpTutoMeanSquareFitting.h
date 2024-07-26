/****************************************************************************
 *
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
*****************************************************************************/

#ifndef VP_TUTO_MEAN_SQUARE_FITTING_H
#define VP_TUTO_MEAN_SQUARE_FITTING_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRobust.h>

namespace tutorial
{
/**
 * \brief Estimates the coefficients of a parabolla v = a u^2 + b u + c
 * using the least-mean-square method.
 */
class vpTutoMeanSquareFitting
{
public:
  vpTutoMeanSquareFitting();

  /**
   * \brief Estimate the parabolla coefficients that fits the best
   * the input points \b pts.
   *
   * \param[in] pts The input points for which we want to fit a parabolla model.
   */
  void fit(const std::vector<vpImagePoint> &pts);

  /**
   * \brief Compute the mean-square error between the parabolla model and
   * the input points \b pts.
   *
   * \param[in] pts The input points.
   * \return float The mean square error.
   */
  float evaluate(const std::vector<vpImagePoint> &pts);

  /**
   * \brief Compute the mean-square error between the parabolla model and
   * the input points \b pts. An M-estimator is used to reject outliers
   * when computing the mean square error.
   *
   * \param[in] pts The input points.
   * \return float The mean square error.
   */
  float evaluateRobust(const std::vector<vpImagePoint> &pts);

  /**
   * \brief Compute the square error between the parabolla model and
   * the input point \b pt.
   *
   * \param[in] pt The input point.
   * \return float The square error.
   */
  float evaluate(const vpImagePoint &pt);

  /**
   * \brief Compute v-coordinate that corresponds to the parabolla model.
   *
   * \param[in] u The u-coordinate of the input point.
   * \return float The corresponding v-coordinate.
   */
  float model(const float &u);

  /**
   * \brief Display the fitted parabolla on the image.
   *
   * \tparam T Either unsigned char or vpRGBa.
   * \param[in] I The image on which we want to display the parabolla model.
   * \param[in] color The color we want to use to display the parabolla.
   */
  template<typename T>
  void display(const VISP_NAMESPACE_ADDRESSING vpImage<T> &I, const VISP_NAMESPACE_ADDRESSING vpColor &color,
               const unsigned int &vertPosLegend, const unsigned int &horPosLegend)
  {
    unsigned int width = I.getWidth();
    for (unsigned int u = 0; u < width; ++u) {
      int v = static_cast<int>(m_a * static_cast<float>(u * u) + m_b * static_cast<float>(u)  + m_c);
      VISP_NAMESPACE_ADDRESSING vpDisplay::displayPoint(I, v, u, color, 1);
      VISP_NAMESPACE_ADDRESSING vpDisplay::displayText(I, vertPosLegend, horPosLegend, "Least-mean square model", color);
    }
  }

  /**
   * \brief Permits to reinitialize the least-mean square fitter.
   */
  inline void reinit()
  {
    m_isFitted = false;
  }

  inline vpTutoMeanSquareFitting &operator=(const vpTutoMeanSquareFitting &other)
  {
    m_a = other.m_a;
    m_b = other.m_b;
    m_c = other.m_c;
    m_isFitted = other.m_isFitted;
    return *this;
  }

  /**
   * \brief Get the coefficients of the parabolla model.
   *
   * \return vpColVector coeffs[0] = a coeffs[1] = b coeffs[2] = c
   */
  inline vpColVector getCoeffs() const
  {
    vpColVector coeffs(3);
    coeffs[0] = m_a;
    coeffs[1] = m_b;
    coeffs[2] = m_c;
    return coeffs;
  }
protected:
  float m_a; /*!< Coefficient that multiplies u^2.*/
  float m_b; /*!< Coefficient that multiplies u.*/
  float m_c; /*!< Offset*/
  bool m_isFitted; /*!< Set to true if the fit method has been called.*/
};
}

#endif
