/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpVelocityTwistMatrix.h>


/*!

  \class vpUnicycle

  \ingroup Pioneer

  \brief Generic functions for unicycle mobile robots.

  This class provides common features for unicycle mobile robots.

*/
class VISP_EXPORT vpUnicycle
{
public:
  /*!
    Default constructor that does nothing.
    */
  vpUnicycle()
  {
  };
  /*!
    Destructor that does nothing.
    */
  virtual ~vpUnicycle() {};

  /*!
    Return the tranformation \f${^c}{\bf M}_e\f$ between the camera frame
    and the mobile robot end effector frame.
    */
  vpHomogeneousMatrix get_cMe() const
  {
    return cMe_;
  }

  /*!

    Return the twist transformation from camera frame to the mobile robot
    end effector frame.  This transformation allows to compute a velocity expressed
    in the end effector frame into the camera frame.
  */
  vpVelocityTwistMatrix get_cVe() const
  {
    vpVelocityTwistMatrix cVe;
    cVe.buildFrom(cMe_) ;
    return cVe;
  }

  /*!

    Return the twist transformation from camera frame to the mobile robot
    end effector frame.  This transformation allows to compute a velocity expressed
    in the end effector frame into the camera frame.

    \sa get_cVe()
  */
  void get_cVe(vpVelocityTwistMatrix &cVe) const
  {
    cVe = vpUnicycle::get_cVe();
  }

  /*!
    Return the robot jacobian \f${^e}{\bf J}_e\f$ expressed in the end effector frame.

    \return The robot jacobian such as \f${\bf v} = {^e}{\bf J}_e \; \dot{\bf q}\f$ with
    \f$\dot{\bf q} = (v_x, w_z)\f$ the robot control velocities and \f$\bf v\f$ the six dimention velocity skew.
  */
  vpMatrix get_eJe() const
  {
    return eJe_;
  }

  /*!
    Set the transformation between the camera frame and the end effector frame.
    */
  void set_cMe(const vpHomogeneousMatrix &cMe)
  {
    cMe_ = cMe;
  }

  /*!
    Set the robot jacobian \f${^e}{\bf J}_e\f$ expressed in the end effector frame.

    \param eJe : The robot jacobian to set such as \f${\bf v} = {^e}{\bf J}_e \; \dot{\bf q}\f$ with
    \f$\dot{\bf q} = (v_x, w_z)\f$ the robot control velocities and \f$\bf v\f$ the six dimention velocity skew.
  */
  void set_eJe(const vpMatrix &eJe)
  {
    eJe_ = eJe;
  }

protected:
  vpHomogeneousMatrix cMe_; // Camera frame to mobile platform frame
  vpMatrix            eJe_; // Robot jacobian
};

#endif


