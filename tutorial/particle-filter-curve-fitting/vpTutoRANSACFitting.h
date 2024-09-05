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

#ifndef VP_TUTO_RANSAC_FITTING_H
#define VP_TUTO_RANSAC_FITTING_H

#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpUniRand.h>

#include "vpTutoMeanSquareFitting.h"

namespace tutorial
{
class vpTutoRANSACFitting
{
public:
  /**
   * \brief Construct a new vpTutoRANSACFittingobject .
   *
   * \param[in] n The number of points to use to build the model.
   * \param[in] k The number of iterations.
   * \param[in] thresh The threshold that indicates if a point fit the model or not.
   * \param[in] ratioInliers The ratio of points that the model explain, i.e. the ratio of inliers in the set of points.
   * \param[in] degree The degree of the curve that is estimated.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  vpTutoRANSACFitting(const unsigned int &n, const unsigned int &k, const float &thresh, const float &ratioInliers, const unsigned int &degree, const unsigned int &height, const unsigned int &width);

  /**
   * \brief Fit a parabola model using the RANSAC algorithm.
   *
   * \param[in] pts The points to which a parabola model must be fit.
   */
  void fit(const std::vector<vpImagePoint> &pts);

  /**
   * \brief Compute the mean-square error between the parabola model and
   * the input points \b pts.
   *
   * \param[in] pts The input points.
   * \return float The mean square error.
   */
  inline float evaluate(const std::vector<vpImagePoint> &pts)
  {
    return m_bestModel.evaluate(pts);
  }

  /**
   * \brief Compute the mean-square error between the parabola model and
   * the input points \b pts. An M-estimator is used to reject outliers
   * when computing the mean square error.
   *
   * \param[in] pts The input points.
   * \return float The mean square error.
   */
  inline float evaluateRobust(const std::vector<vpImagePoint> &pts)
  {
    return m_bestModel.evaluateRobust(pts);
  }

  /**
   * \brief Compute the square error between the parabola model and
   * the input point \b pt.
   *
   * \param[in] pt The input point.
   * \return float The square error.
   */
  inline float evaluate(const vpImagePoint &pt)
  {
    return m_bestModel.evaluate(pt);
  }

  /**
   * \brief Compute v-coordinate that corresponds to the parabola model.
   *
   * \param[in] u The u-coordinate of the input point.
   * \return float The corresponding v-coordinate.
   */
  inline float model(const float &u)
  {
    return m_bestModel.model(u);
  }

  /**
   * \brief Display the fitted parabola on the image.
   *
   * \tparam T Either unsigned char or vpRGBa.
   * \param[in] I The image on which we want to display the parabola model.
   * \param[in] color The color we want to use to display the parabola.
   */
  template<typename T>
  void display(const VISP_NAMESPACE_ADDRESSING vpImage<T> &I, const VISP_NAMESPACE_ADDRESSING vpColor &color,
               const unsigned int &vertPosLegend, const unsigned int &horPosLegend)
  {
#ifdef VISP_HAVE_DISPLAY
    unsigned int width = I.getWidth();
    for (unsigned int u = 0; u < width; ++u) {
      float v = m_bestModel.model(u);
      VISP_NAMESPACE_ADDRESSING vpDisplay::displayPoint(I, v, u, color, 1);
      VISP_NAMESPACE_ADDRESSING vpDisplay::displayText(I, vertPosLegend, horPosLegend, "RANSAC model", color);
    }
#endif
  }

  inline const vpTutoMeanSquareFitting &getModel() const
  {
    return m_bestModel;
  }

private:
  unsigned int m_degree; /*!< The degree of the curve that is estimated*/
  double m_height; /*!< The height of the input image*/
  double m_width; /*!< The width of the input image*/
  vpTutoMeanSquareFitting m_bestModel; /*!< Object that fits a parabola from a vector of vpImagePoint .*/
  float m_bestError; /*!!< The mean square error of the best model.*/
  unsigned int m_n; /*!< The number of points to use to build the model.*/
  unsigned int m_k; /*!< The number of iterations.*/
  float m_thresh; /*!< The threshold that indicates if a point fit the model or not.*/
  float m_ratioInliers; /*!< The ratio of points that the model explain, i.e. the ratio of inliers in the set of points.*/
};
}
#endif
