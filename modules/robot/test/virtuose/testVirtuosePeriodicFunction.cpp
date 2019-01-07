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
 * Test for Virtuose SDK wrapper.
 *
 * Author:
 * Nicolò Pedemonte
 *
 *****************************************************************************/

/*!
  \example testVirtuosePeriodicFunction.cpp

  Test for reading the Virtuose's current position using a callback function.
*/

#include <visp3/core/vpTime.h>
#include <visp3/robot/vpVirtuose.h>

#if defined(VISP_HAVE_VIRTUOSE)

void CallBackVirtuose(VirtContext VC, void *ptr)
{
  (void)VC;
  vpVirtuose *p_virtuose = (vpVirtuose *)ptr;

  vpPoseVector localPose = p_virtuose->getPhysicalPosition();
  vpColVector vel = p_virtuose->getPhysicalVelocity();
  std::cout << "pose: " << localPose.t() << std::endl;
  std::cout << "vel: " << vel.t() << std::endl;

  return;
}

int main()
{
  try {
    vpVirtuose virtuose;
    virtuose.setVerbose(true);
    virtuose.setPeriodicFunction(CallBackVirtuose);
    virtuose.startPeriodicFunction();

    int counter = 0;
    bool swtch = true;

    while (swtch) {
      if (counter >= 2) {
        virtuose.stopPeriodicFunction();
        swtch = false;
      }
      counter++;
      vpTime::sleepMs(1000);
    }
    std::cout << "The end" << std::endl;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
  }
}

#else
int main() { std::cout << "You should install Virtuose API to use this binary..." << std::endl; }
#endif
