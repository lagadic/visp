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
 * Image circle, i.e. circle in the image space.
 */

/*!
 * \file vpImageCircle.h
 * \brief Image circle, i.e. circle in the image space.
 */

#ifndef VP_IMAGE_CIRCLE_H
#define VP_IMAGE_CIRCLE_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

#if defined(VISP_HAVE_OPENCV)
#include <opencv2/core/core.hpp>
#endif

BEGIN_VISP_NAMESPACE
/**
 * \brief Class that defines a 2D circle in an image.
*/
class VISP_EXPORT vpImageCircle
{
public:
  /*!
  * Default constructor.
  */
  vpImageCircle();

  /*!
  * Constructor from a center and radius.
  */
  vpImageCircle(const vpImagePoint &center, const float &radius);

#ifdef HAVE_OPENCV_CORE
  /*!
  * Constructor from an OpenCV vector that contains [center_x, center_y, radius].
  */
  vpImageCircle(const cv::Vec3f &vec);
#endif

  /*!
  * Compute the angular coverage, in terms of radians, that is contained in the Region of Interest (RoI).
  * \sa computeArcLengthInRoI(), computeArcLengthInRoI(const vpRect &roi)
  * \param[in] roi The rectangular RoI in which we want to know the number of pixels of the circle that are contained.
  * \param[in] roundingTolerance The tolerance on the angle when the angle is close to a negative multiple of 2 * M_PI_FLOAT.
  * \return Returns angular coverage of a circle in a ROI as an angle value in radians.
  * More precisely, it returns 2.f * M_PI for a circle that is fully visible in the RoI, or the sum of the angles
  * of the arc(s) that is(are) visible in the RoI.
  */
  float computeAngularCoverageInRoI(const vpRect &roi, const float &roundingTolerance = 0.001f) const;

  /*!
  * Compute the arc length, in terms of number of pixels, that is contained in the Region of Interest (RoI).
  * \sa computeAngularCoverageInRoI(), computeAngularCoverageInRoI(const vpRect &roi)
  * \param[in] roi The rectangular RoI in which we want to know the number of pixels of the circle that are contained.
  * \param[in] roundingTolerance The tolerance on the angle when the angle is close to 2.f * M_PI_FLOAT .
  * \return The number of pixels of the circle that are contained in the RoI.
  */
  float computeArcLengthInRoI(const vpRect &roi, const float &roundingTolerance = 0.001f) const;

  /**
   * \brief Count the number of pixels of the circle whose value in the mask is true.
   *
   * \param mask A mask where true indicates that a pixel must be taken into account and false
   * that it must be ignored.
   * \return unsigned int The number of pixels in the mask.
   */
  unsigned int computePixelsInMask(const vpImage<bool> &mask) const;

  /*!
   * Get the center of the image (2D) circle
   * \return The center of the image (2D) circle.
   */
  vpImagePoint getCenter() const;

  /*!
  * Get the radius of the image (2D) circle.
  * \return The radius of the image (2D) circle.
  */
  float getRadius() const;

  /*!
  * Compute the bounding box, in the image, of the image (2D) circle.
  * \return the 2D circle bounding box.
  */
  vpRect getBBox() const;

  /*!
  * Compute the normalized moment \f$n_{20}\f$ of the image (2D) circle.
  * \return The normalized moment \f$n_{20}\f$.
  */
  float get_n20() const;

  /*!
  * Compute the normalized moment \f$n_{02}\f$ of the image (2D) circle.
  * \return The normalized moment \f$n_{02}\f$.
  */
  float get_n02() const;

  /*!
  * Compute the normalized moment \f$n_{11}\f$ of the image (2D) circle.
  * \return The normalized moment \f$n_{11}\f$.
  */
  float get_n11() const;

private:
  vpImagePoint m_center;
  float m_radius;
};
END_VISP_NAMESPACE
#endif
