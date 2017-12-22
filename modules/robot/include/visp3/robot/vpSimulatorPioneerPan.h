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
 * Pioneer mobile robot equipped with a pan head simulator without display.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpSimulatorPioneerPan_H
#define vpSimulatorPioneerPan_H

/*!
  \file vpSimulatorPioneerPan.h
  \brief class that defines the Pioneer mobile robot simulator equipped
  with a camera able to move in pan.
*/

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/robot/vpPioneerPan.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotSimulator.h>

/*!
  \class vpSimulatorPioneerPan

  \ingroup group_robot_simu_unicycle

  \brief Class that defines the Pioneer mobile robot simulator equipped
  with a camera able to move in pan.

  It intends to simulate the mobile robot described in vpPioneerPan class.
  This robot has 3 dof: \f$(v_x, w_z, \dot{q_1})\f$, the translational and
  rotational velocities of the mobile platform, the pan head velocity
respectively.

  The robot position evolves with respect to a world frame; wMc. When a new
joint velocity is applied to the robot using setVelocity(), the position of
the camera wrt the world frame is updated.

  \image html pioneer-pan.png

  The following code shows how to control this robot in position and velocity.
  \code
#include <visp3/robot/vpSimulatorPioneerPan.h>

int main()
{
  vpHomogeneousMatrix wMc;
  vpSimulatorPioneerPan robot;

  robot.getPosition(wMc); // Position of the camera in the world frame
  std::cout << "Default position of the camera in the world frame wMc:\n" << wMc << std::endl;

  robot.setSamplingTime(0.100); // Modify the default sampling time to 0.1 second
  robot.setMaxTranslationVelocity(1.); // vx max set to 1 m/s
  robot.setMaxRotationVelocity(vpMath::rad(90)); // wz max set to 90 deg/s

  vpColVector v(3); // we control vx, wz and q_pan
  v = 0;
  v[0] = 1.; // set vx to 1 m/s
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);
  // The robot has moved from 0.1 meters along the z axis
  robot.getPosition(wMc); // Position of the camera in the world frame
  std::cout << "New position of the camera wMc:\n" << wMc << std::endl;
}
  \endcode

  The usage of this class is also highlighted in \ref
tutorial-simu-robot-pioneer.

*/
class VISP_EXPORT vpSimulatorPioneerPan : public vpPioneerPan, public vpRobotSimulator
{

protected:
  //! robot / camera location in the world frame
  vpHomogeneousMatrix wMc_; // world to camera
  vpHomogeneousMatrix wMm_; // world to mobile robot frame located between the two weels
  // mMp_ mobile robot to pan frame is a protected member of vpPioneerPan
  // pMe_ pan head to end effector frame is a protected member of vpPioneerPan
  // cMe_ is a protected member of vpUnicycle

  double xm_;
  double ym_;
  double theta_;
  double q_pan_;

public:
  vpSimulatorPioneerPan();
  virtual ~vpSimulatorPioneerPan();

public:
  /** @name Inherited functionalities from vpSimulatorPioneerPan */
  //@{
  void get_eJe(vpMatrix &eJe);

  void getPosition(vpHomogeneousMatrix &wMc) const;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
  //@}

private:
  void init();

  // Non implemented virtual pure functions
  void get_fJe(vpMatrix & /*_fJe */){};
  void getDisplacement(const vpRobot::vpControlFrameType /* frame */, vpColVector & /* q */){};
  void setPosition(const vpRobot::vpControlFrameType /* frame */, const vpColVector & /* q */){};
};

#endif
