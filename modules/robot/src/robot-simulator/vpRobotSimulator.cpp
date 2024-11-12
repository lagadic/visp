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
 * Basic class used to make robot simulators.
 */


#include <visp3/core/vpConfig.h>           // for BEGIN_VISP_NAMESPACE, END_...
#include <visp3/robot/vpRobotSimulator.h>  // for vpRobotSimulator
#include <visp3/robot/vpRobot.h>           // for vpRobot

BEGIN_VISP_NAMESPACE
/*!
  Basic constructor that sets the sampling time by default to 40ms.
*/
vpRobotSimulator::vpRobotSimulator() : vpRobot(), delta_t_(0.040f) { }
END_VISP_NAMESPACE
