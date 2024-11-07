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
 * Forward projection.
 */

/*!
 * \file vpForwardProjection.h
 * \brief  class that defines what is a generic geometric feature
 */

#ifndef VP_FORWARD_PROJECTION_H
#define VP_FORWARD_PROJECTION_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTracker.h>

#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpForwardProjection
 * \brief Class that defines what is a generic geometric feature.
 *
 * Each geometric feature has parameters expressed:
 *
 * - in the object frame \e oP. These parameters are located in the public
 *   attribute vpForwardProjection::oP.
 * - in the camera frame \e cP. These parameters are located in the public
 *   attribute vpTracker::cP.
 * - in the image plane \e p. These parameters are located in the public
 *   attribute vpTracker::p. They correspond to normalized coordinates
 *   of the feature expressed in meters.
*/
class VISP_EXPORT vpForwardProjection : public vpTracker
{
public:
  /*!
   * Used for memory issue especially in the vpServo class.
   */
  typedef enum { user, vpDisplayForwardProjection } vpForwardProjectionDeallocatorType;

  /** @name Public Member Functions Inherited from vpForwardProjection */
  //@{
  vpForwardProjection() : oP(), deallocate(user) { }

  /*!
   * Computes the features parameters in the camera frame (\e cP) thanks
   * to the parameters given in the object frame
   * (vpForwardProjection::oP) and the homogeneous matrix relative to
   * the pose (\e cMo) between the object frame and the camera frame.
   *
   * To set the parameters in the object frame you need to call
   * setWorldCoordinates().
   *
   * \param cMo : The homogeneous matrix corresponding to the pose
   * between the camera frame and the object frame.
   *
   * \param cP : The vector which will contain the feature parameters
   * expressed in the camera frame.
   *
   * With this method, the vpTracker::cP public attribute is not updated.
   */
  virtual void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) const = 0;

  /*!
   * Computes the features parameters in the camera frame (\e cP) thanks
   * to the parameters given in the object frame
   * (vpForwardProjection::oP) and the homogeneous matrix relative to
   * the pose (\e cMo) between the object frame and the camera frame.
   *
   * To set the parameters in the object frame you need to call
   * setWorldCoordinates().
   *
   * \param cMo : The homogeneous matrix corresponding to the pose
   * between the camera frame and the object frame.
   *
   * The features parameters in the camera frame (cP) are updated in
   * the vpTracker::cP public attribute.
   */
  virtual void changeFrame(const vpHomogeneousMatrix &cMo) = 0;

  /*!
   * Displays the feature in the image \e I thanks to the 2D feature
   * parameters in the image plane (vpTracker::p) and the camera
   * parameters which enable to convert the features from meter to pixel.
   *
   * \param I : The image where the feature must be displayed in overlay.
   *
   * \param cam : The camera parameters to enable the conversion from
   * meter to pixel.
   *
   * \param color : The desired color to display the line in the image.
   * \param thickness : Thickness of the feature representation.
   */
  virtual void display(const vpImage<unsigned char> &I, const vpCameraParameters &cam,
                       const vpColor &color = vpColor::green, unsigned int thickness = 1) = 0;

  /*!
   * Displays the feature in the image \e I thanks to the features in
   * the object frame (vpForwardProjection::oP), the homogeneous matrix
   * relative to the pose between the object frame and the camera frame and the
   * camera parameters which enable to convert the features from meter
   * to pixel.
   *
   * \param I : The image where the line must be displayed in overlay.
   *
   * \param cMo : The homogeneous matrix corresponding to the pose
   * between the camera frame and the object frame.
   *
   * \param cam : The camera parameters to enable the conversion from
   * meter to pixel.
   *
   * \param color : The desired color to display the line in the image.
   * \param thickness : Thickness of the feature representation.
   */
  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color = vpColor::green, unsigned int thickness = 1) = 0;

  /*!
   * Create an object with the same type.
   */
  virtual vpForwardProjection *duplicate() const = 0;

  //! Return object parameters expressed in the 3D object frame.
  vpColVector get_oP() const { return oP; }

  vpForwardProjectionDeallocatorType getDeallocate() { return deallocate; }

  virtual void print() const;

  /*!
   * Computes the feature parameters in the image plane from the
   * parameters expressed in the camera frame.
   *
   * \param cP [input] : Feature parameters expressed in the camera frame.
   *
   * \param p [output] : Feature parameters expressed in the image plane.
   */
  virtual void projection(const vpColVector &cP, vpColVector &p) const = 0;

  /*!
   * Computes the feature parameters in the image plane. These
   * parameters are than updated in the vpTracker::p public attribute.
   *
   * \warning To compute these parameters, the method exploit the
   * feature parameters in the camera frame. Thus, vpTracker::cP need
   * to be updated before the call of this method.  For that, a call to
   * changeFrame(const vpHomogeneousMatrix &) is requested.
   */
  virtual void projection() = 0;

  void project();
  void project(const vpHomogeneousMatrix &cMo);

  void setDeallocate(vpForwardProjectionDeallocatorType d) { deallocate = d; }

  /*!
   * Sets the parameters which define the feature in the object frame.
   *
   * \param oP : Feature parameters expressed in the object frame used
   * to set the vpForwardProjection::oP public attribute.
   */
  virtual void setWorldCoordinates(const vpColVector &oP) = 0;

  void track(const vpHomogeneousMatrix &cMo);
  //@}


public:
  /** @name Public Attributes Inherited from vpForwardProjection */
  //@{
  /*!
   * Feature coordinates expressed in the object frame.
   */
  vpColVector oP;
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpForwardProjection */
  //@{
  /*!
   * Default initialisation of the feature parameters:
   * - in the object frame: \e oP
   * - in the camera frame: \e cP
   * - in the image plane: \e p.
   */
  virtual void init() = 0;
  //@}

private:
  vpForwardProjectionDeallocatorType deallocate;
};
END_VISP_NAMESPACE
#endif
