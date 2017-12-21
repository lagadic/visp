/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Common features for unicycle mobile robots.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef VPUNICYCLE_H
#define VPUNICYCLE_H

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

/*!

  \class vpUnicycle

  \ingroup group_robot_simu_unicycle

  \brief Generic functions for unicycle mobile robots.

  This class provides common features for unicycle mobile robots.

*/
class VISP_EXPORT vpUnicycle
{
public:
  /*!
    Default constructor that does nothing.
    */
  vpUnicycle() : cMe_(), eJe_(){};
  /*!
    Destructor that does nothing.
    */
  virtual ~vpUnicycle(){};

  /** @name Inherited functionalities from vpUnicycle */
  //@{
  /*!
    Return the tranformation \f${^c}{\bf M}_e\f$ between the camera frame
    and the mobile robot end effector frame.
    */
  vpHomogeneousMatrix get_cMe() const { return cMe_; }

  /*!

    Return the twist transformation from camera frame to the mobile robot
    end effector frame.  This transformation allows to compute a velocity
    expressed in the end effector frame into the camera frame.
  */
  vpVelocityTwistMatrix get_cVe() const
  {
    vpVelocityTwistMatrix cVe;
    cVe.buildFrom(cMe_);
    return cVe;
  }

  /*!

    Return the twist transformation from camera frame to the mobile robot
    end effector frame.  This transformation allows to compute a velocity
    expressed in the end effector frame into the camera frame.

    \sa get_cVe()
  */
  void get_cVe(vpVelocityTwistMatrix &cVe) const { cVe = get_cVe(); }

  /*!
    Return the robot jacobian \f${^e}{\bf J}_e\f$ expressed in the end
    effector frame.

    \return The robot jacobian such as \f${\bf v} = {^e}{\bf J}_e \; \dot{\bf
    q}\f$ with \f$\dot{\bf q} = (v_x, w_z)\f$ the robot control velocities and
    \f$\bf v\f$ the six dimention velocity skew.
  */
  vpMatrix get_eJe() const { return eJe_; }

  /*!
    Set the transformation between the camera frame and the end effector
    frame.
    */
  void set_cMe(const vpHomogeneousMatrix &cMe) { cMe_ = cMe; }

  /*!
    Set the robot jacobian \f${^e}{\bf J}_e\f$ expressed in the end effector
    frame.

    \param eJe : The robot jacobian to set such as \f${\bf v} = {^e}{\bf J}_e
    \; \dot{\bf q}\f$ with \f$\dot{\bf q} = (v_x, w_z)\f$ the robot control
    velocities and \f$\bf v\f$ the six dimention velocity skew.
  */
  void set_eJe(const vpMatrix &eJe) { eJe_ = eJe; }
  //@}

protected:
  vpHomogeneousMatrix cMe_; // Camera frame to mobile platform frame
  vpMatrix eJe_;            // Robot jacobian
};

#endif
