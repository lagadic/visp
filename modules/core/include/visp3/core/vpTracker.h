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
 * Generic tracker.
 */

/*!
 * \file vpTracker.h
 * \brief Class that defines what is a generic tracker.
 */

#ifndef VP_TRACKER_H
#define VP_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpTracker
 * \ingroup group_core_trackers
 * \brief Class that defines what is a feature generic tracker.
 *
 * A tracker is able to track features with parameters expressed in:
 * - in the camera frame \e cP. These parameters are located in the public
 *   attribute vpTracker::cP.
 * - in the image plane \e p. These parameters are located in the public
 *   attribute vpTracker::p. They correspond to normalized coordinates
 *   of the feature expressed in meters.
*/
class VISP_EXPORT vpTracker
{
public:
  /** @name Public Attributes Inherited from vpTracker */
  //@{
  /*!
   * Feature coordinates expressed in the image plane \e p. They correspond
   * to 2D normalized coordinates expressed in meters.
   */
  vpColVector p;
  /*!
   * Feature coordinates expressed in the camera frame \e cP.
   */
  vpColVector cP;

  /*!
   * Flag used to indicate if the feature parameters \e cP expressed
   * in the camera frame are available.
   */
  bool cPAvailable;
  //@}

public:
  //! Default constructor.
  vpTracker();
  //! Copy constructor.
  vpTracker(const vpTracker &tracker);
  //! Destructor.
  virtual ~vpTracker() { }

  /** @name Public Member Functions Inherited from vpTracker */
  //@{
  //! Return object parameters expressed in the 2D image plane computed by perspective projection.
  vpColVector get_p() const { return p; }
  //! Return object parameters expressed in the 3D camera frame.
  vpColVector get_cP() const { return cP; }

  //! Copy operator.
  vpTracker &operator=(const vpTracker &tracker);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpTracker */
  //@{
  //! Default initialization.
  void init();
  //@}
};
END_VISP_NAMESPACE
#endif
