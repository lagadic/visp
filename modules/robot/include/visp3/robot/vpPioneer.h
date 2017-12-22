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
 * Common features for Pioneer unicycle mobile robots.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef VPPIONEER_H
#define VPPIONEER_H

#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/robot/vpUnicycle.h>

/*!

  \class vpPioneer

  \ingroup group_robot_real_unicycle group_robot_simu_unicycle

  \brief Generic functions for Pioneer mobile robots.

  This class provides common features for Pioneer mobile robots.
  This robot has two control velocities \f$(v_x, w_z)\f$, the translational
  and rotational velocities of the mobile platform respectively.

  The figure below shows the position of the frames that are used to model the
  robot. The end effector frame is here located at the middle point between
  the two wheels.

  \image html pioneer.png

  The robot jacobian at the end effector frame, the point located at the
  middle between the two wheels is given by:

  \f[
  {^e}{\bf J}_e = \left(\begin{array}{cc}
  1  &   0   \\
  0  &   0   \\
  0  &   0   \\
  0  &   0   \\
  0  &   0   \\
  0  &   1   \\
  \end{array}
  \right)
  \f]

  Considering \f$(v_x, w_z)\f$, it is possible to compute \f$\bf v\f$ the six
  dimention velocity skew expressed at the end effector frame by:

  \f[
  {\bf v} = {^e}{\bf J}_e \;
  \left(\begin{array}{c}
  v_x \\
  w_z \\
  \end{array}
  \right)
  \f].

*/
class VISP_EXPORT vpPioneer : public vpUnicycle
{
public:
  /*!
    Create a default Pioneer robot.
    */
  vpPioneer() : vpUnicycle()
  {
    set_cMe();
    set_eJe();
  }

  /*!
    Destructor that does nothing.
    */
  virtual ~vpPioneer(){};

private:
  /*!
    Set the transformation between the camera frame and the mobile platform
    end effector frame.
    */
  void set_cMe()
  {
    // Position of the camera in the mobile platform frame
    double l = 0.13;         // distance between the camera frame and the mobile robot frame
    vpTranslationVector cte; // meters
    vpRxyzVector cre;        // radian
    cte.set(0, 0, -l);
    cre.buildFrom(vpMath::rad(90.), 0, vpMath::rad(90.));
    cMe_.buildFrom(cte, vpRotationMatrix(cre));
  }

  /*!
    Set the robot jacobian at the end effector frame, the point located at the
    middle between the two wheels.

    Considering \f${\bf v} = {^e}{\bf J}_e \; [v_x, w_z]\f$ with
    \f$(v_x, w_z)\f$ respectively the translational and rotational control
    velocities of the mobile robot and \f$\bf v\f$ the six dimention velocity
    skew expressed at the end effector frame, the robot jacobian is given by:

    \f[
    {^e}{\bf J}_e = \left(\begin{array}{cc}
    1  &   0   \\
    0  &   0   \\
    0  &   0   \\
    0  &   0   \\
    0  &   0   \\
    0  &   1   \\
    \end{array}
    \right)
    \f]

  */
  void set_eJe()
  {
    eJe_.resize(6, 2); // pioneer jacobian expressed at point M
    eJe_ = 0;
    eJe_[0][0] = 1; // vx
    eJe_[5][1] = 1; // wz
  }
};

#endif
