/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Defines the simplest robot : a free flying camera.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRobotCamera_H
#define vpRobotCamera_H

/*!
  \file vpRobotCamera.h
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/robot/vpRobotSimulator.h>

/*!
  \class vpRobotCamera
  \ingroup group_robot_simu_camera

  \deprecated This class is deprecated since ViSP 3.0.0.

  \brief Class that defines the simplest robot: a free flying camera. We
  recommend to use vpSimulatorCamera instead.

  This free flying camera has 6 dof; 3 in translation and 3 in rotation.
  It evolves as a gantry robot with respect to a world frame. This class
  is similar to vpSimulatorCamera class except that here the position of the
  robot is provided as the transformation from camera frame to world frame; cMw.
  Since the position of the camera frame evolves, this representation is less
  intuitive than the one implemented in vpSimulatorCamera where the
  transformation from world to camera frame is considered; wMc.

  For this particular simulated robot, the end-effector and camera frame are
  confused. That means that the cMe transformation is equal to identity.

  The robot jacobian expressed in the end-effector frame
  \f$ {^e}{\bf J}_e \f$ is also set to identity (see get_eJe()).

  The following code shows how to control this robot in position and velocity.
  \code
#include <visp3/robot/vpRobotCamera.h>

int main()
{
  vpHomogeneousMatrix cMw;
  vpRobotCamera robot;

  robot.getPosition(cMw); // Position of the world frame in the camera frame
  std::cout << "Default position of the world frame in the camera frame cMw:\n" << cMw << std::endl;

  cMw[2][3] = 1.; // World frame is 1 meter along z axis in front of the camera frame
  robot.setPosition(cMw); // Set the new position of the world frame in the camera frame
  std::cout << "New position of the world frame in the camera frame cMw:\n" << cMw << std::endl;

  robot.setSamplingTime(0.100); // Modify the default sampling time to 0.1 second
  robot.setMaxTranslationVelocity(1.); // vx, vy and vz max set to 1 m/s
  robot.setMaxRotationVelocity(vpMath::rad(90)); // wx, wy and wz max set to 90 deg/s

  vpColVector v(6);
  v = 0;
  v[2] = 1.; // set v_z to 1 m/s
  robot.setVelocity(vpRobot::CAMERA_FRAME, v);
  // The robot has moved from 0.1 meters along the z axis
  robot.getPosition(cMw); // Position of the world frame in the camera frame
  std::cout << "New position of the camera cMw:\n" << cMw << std::endl;
}
  \endcode

*/
class VISP_EXPORT vpRobotCamera : public vpRobotSimulator
{
protected:
  vpHomogeneousMatrix cMw_; // camera to world

public:
  vpRobotCamera();
  virtual ~vpRobotCamera();

  /** @name Inherited functionalities from vpRobotCamera */
  //@{
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_eJe(vpMatrix &eJe);

  void getPosition(vpHomogeneousMatrix &cMw) const;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);

  void setPosition(const vpHomogeneousMatrix &cMw);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  //@}

private:
  void init();

  // Non implemented virtual pure functions
  void get_fJe(vpMatrix & /*_fJe */){};
  void getDisplacement(const vpRobot::vpControlFrameType /* frame */, vpColVector & /* q */){};
  void setPosition(const vpRobot::vpControlFrameType /* frame */, const vpColVector & /* q */){};
};

#endif
#endif
