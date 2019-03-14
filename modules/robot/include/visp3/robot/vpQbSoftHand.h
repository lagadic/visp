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
 * Interface for the qb robotics qbSoftHand device.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef _vpQbSoftHand_h_
#define _vpQbSoftHand_h_

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_QBDEVICE

#include <visp3/core/vpColVector.h>
#include <visp3/robot/vpQbDevice.h>

/*!

  \class vpQbSoftHand

  \ingroup group_robot_haptic

  Interface for qbSoftHand [device](https://qbrobotics.com/products/qb-softhand/).

  See https://qbrobotics.com/ for more details.

  The following example shows how to close and open the SoftHand with a given speed factor and stiffness used to stop the command applied to
  the motors when the measured current is larger than the stiffness multiplied by the maximum allowed current that can be applied to the motors.

  \code
#include <visp3/robot/vpQbSoftHand.h>

int main()
{
  vpQbSoftHand qbsofthand;

  vpColVector q(1);

  double speed_factor = 0.5; // half speed
  double stiffness = 0.7;    // 70% of the max allowed current supported by the motors
  std::cout << "** Close the hand with blocking positioning function" << std::endl;
  q[0] = 1;
  qbsofthand.setPosition(q, speed_factor, stiffness);

  std::cout << "** Open the hand with blocking positioning function" << std::endl;
  q[0] = 0;
  qbsofthand.setPosition(q, speed_factor, stiffness);
}
  \endcode

*/
class VISP_EXPORT vpQbSoftHand: public vpQbDevice
{
public:
  vpQbSoftHand();
  virtual ~vpQbSoftHand();

  void getCurrent(vpColVector &current, const int &id=1);
  void getPosition(vpColVector &position, const int &id=1);

  void setPosition(const vpColVector &position, const int &id=1);
  void setPosition(const vpColVector &position, double speed_factor, double stiffness, const int &id=1);
};


#endif
#endif
