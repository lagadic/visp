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
 * Test for qbdevice.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testQbSoftHand.cpp

  Test qb device from qbrobotics.
*/

#include <iostream>

#include <visp3/robot/vpQbSoftHand.h>
#include <visp3/core/vpTime.h>

int main()
{
#ifdef VISP_HAVE_QBDEVICE
  std::cout << "Test qbSoftHand device" << std::endl;
  try {
    vpQbSoftHand qbsofthand;

    {
      // Call blocking functions to close and open the hand
      vpColVector q(1), q_mes, current;

      double speed_factor = 0.5; // half speed
      double stiffness = 0.7;
      std::cout << "** Close the hand with blocking positioning function" << std::endl;
      q[0] = 1;
      qbsofthand.setPosition(q, speed_factor, stiffness);

      std::cout << "** Open the hand with blocking positioning function" << std::endl;
      q[0] = 0;
      qbsofthand.setPosition(q, speed_factor, stiffness);
    }

    {
      // Call non-blocking positionning functions
      vpColVector q(1), q_mes, current;
      double max_current = qbsofthand.getCurrentMax();
      int i_max = 0;
      std::cout << "** Close the hand with non-blocking positioning function" << std::endl;
      for(int i=1; i <= 10; i++) {
        qbsofthand.getPosition(q_mes);
        qbsofthand.getCurrent(current);
        if (std::fabs(current[0]) > max_current/2) {
          std::cout << "Stop closure, current > " << max_current/2 << std::endl;
          i_max = i;
          break;
        }
        q[0] = i/10.0;
        qbsofthand.setPosition(q);
        vpTime::sleepMs(500);
      }
      std::cout << "** Open the hand with non-blocking positioning function" << std::endl;
      for(int i=i_max; i >= 0; i--) {
        qbsofthand.getPosition(q_mes);
        qbsofthand.getCurrent(current);
        q[0] = i/10.0;
        qbsofthand.setPosition(q);
        vpTime::sleepMs(500);
      }
    }
    std::cout << "The end" << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "Catch exception: " << e.getStringMessage() << std::endl;
  }
#else
  std::cout << "ViSP is not build with qbdevice 3rd party" << std::endl;
#endif
  return EXIT_SUCCESS;
}

