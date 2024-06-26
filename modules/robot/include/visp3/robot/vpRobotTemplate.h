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
 * Defines a robot just to show which function you must implement.
 */

/*!
 * \file vpRobotTemplate.h
 * Defines a robot just to show which function you must implement.
 */

#ifndef vpRobotTemplate_h
#define vpRobotTemplate_h

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpRobotTemplate
 * \ingroup group_robot_real_template
 * \brief Class that defines a robot just to show which function you must implement.
*/
class VISP_EXPORT vpRobotTemplate : public vpRobot
{
public:
  vpRobotTemplate();
  virtual ~vpRobotTemplate() VP_OVERRIDE;

  void get_eJe(vpMatrix &eJe_) VP_OVERRIDE;
  void get_fJe(vpMatrix &fJe_) VP_OVERRIDE;

  /*!
   * Return constant transformation between end-effector and tool frame.
   * If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  vpHomogeneousMatrix get_eMc() const { return m_eMc; }

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q) VP_OVERRIDE;

  /*!
   * Set constant transformation between end-effector and tool frame.
   * If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  void set_eMc(vpHomogeneousMatrix &eMc) { m_eMc = eMc; }
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q) VP_OVERRIDE;
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel) VP_OVERRIDE;

protected:
  void init() VP_OVERRIDE;
  void getJointPosition(vpColVector &q);
  void setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  void setJointVelocity(const vpColVector &qdot);

protected:
  vpHomogeneousMatrix m_eMc; //!< Constant transformation between end-effector and tool (or camera) frame
};
END_VISP_NAMESPACE
#endif
