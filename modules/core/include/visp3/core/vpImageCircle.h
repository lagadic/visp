/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
  \file vpImageCircle.h
  \brief Image circle, i.e. circle in the image space.
*/

#ifndef _vpImageCircle_h_
#define _vpImageCircle_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRect.h>

#if defined(VISP_HAVE_OPENCV)
#include <opencv2/core/core.hpp>
#endif

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
  * Default destructor.
  */
  virtual ~vpImageCircle();

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
#endif
