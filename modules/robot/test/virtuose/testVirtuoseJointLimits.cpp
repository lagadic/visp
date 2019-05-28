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
 * Authors:
 * Nicolò Pedemonte
 *
 *****************************************************************************/

/*!
  \example testVirtuoseJointLimits.cpp

  Test Haption Virtuose for testing the force feedback in articular mode.
  A force is felt when approaching to the Virtuose's joint limits (estimated
  experimentally).
*/

#include <visp3/core/vpTime.h>
#include <visp3/robot/vpVirtuose.h>

#if defined(VISP_HAVE_VIRTUOSE)

void CallBackVirtuose(VirtContext VC, void *ptr)
{
  (void)VC;
  vpVirtuose *p_virtuose = (vpVirtuose *)ptr;

  float maxQ[6] = {0.7811045051f, -0.07668215036f, 2.481732368f, 2.819076777f, 1.044736624f, 2.687076807f};
  float minQ[6] = {-0.8011951447f, -1.648244739f, 0.7439950705f, -3.022218227f, -1.260564089f, -2.054088593f};
  unsigned int numJoint = 6;

  vpColVector feedbackRegion(numJoint, 0);
  vpColVector forceFeedback(numJoint, 0);

  int feedbackRegionFactor = 10;
  float saturationForce[6] = {5, 5, 5, 2.5, 2.5, 2.5};

  for (unsigned int iter = 0; iter < numJoint; iter++)
    feedbackRegion[iter] = (maxQ[iter] - minQ[iter]) / feedbackRegionFactor;

  vpColVector currentQ = p_virtuose->getArticularPosition();

  // force feedback definition
  for (unsigned int iter = 0; iter < numJoint; iter++) {
    if (currentQ[iter] >= (maxQ[iter] - feedbackRegion[iter])) {
      forceFeedback[iter] =
          -saturationForce[iter] * pow((currentQ[iter] - maxQ[iter] + feedbackRegion[iter]) / feedbackRegion[iter], 2);
      std::cout << "WARNING! Getting close to the maximum joint limit. Joint #" << iter + 1 << std::endl;
    } else if (currentQ[iter] <= (minQ[iter] + feedbackRegion[iter])) {
      forceFeedback[iter] =
          saturationForce[iter] * pow((minQ[iter] + feedbackRegion[iter] - currentQ[iter]) / feedbackRegion[iter], 2);
      std::cout << "WARNING! Getting close to the minimum joint limit. Joint #" << iter + 1 << std::endl;
    } else {
      forceFeedback[iter] = 0;
      std::cout << "Safe zone" << std::endl;
    }
  }

  // Printing force feedback
  //    std::cout << "Force feedback: " << forceFeedback.t() << std::endl;

  // Set force feedback
  p_virtuose->setArticularForce(forceFeedback);

  return;
}

int main()
{
  try {
    float period = 0.001f;
    vpVirtuose virtuose;
    virtuose.setTimeStep(period);
    virtuose.setIpAddress("localhost#5000");
    virtuose.setVerbose(true);
    virtuose.setPowerOn();

    // setArticularForce only works in COMMAND_TYPE_ARTICULAR_IMPEDANCE.
    virtuose.setCommandType(COMMAND_TYPE_ARTICULAR_IMPEDANCE);

    // -----------------------------------------------------------
    // Code to obtain (experimentally) the Virtuose joint limits
    // -----------------------------------------------------------

    /*
    // Move the Virtuose in all its workspace while running this code

    vpColVector joints(6);
    vpColVector max_joint(6,-1000);
    vpColVector min_joint(6,1000);

    for(unsigned int iter=0; iter<10000; iter++) {
      virtuose.getArticularPosition(joints);
      for(unsigned int i=0; i<6; i++) {
        if (joints[i] > max_joint[i])
            max_joint[i] = joints[i];
        if (joints[i] < min_joint[i])
          min_joint[i] = joints[i];
      }
      // Printing joint values
      std::cout << "Joint values: " << joints.t() << std::endl;
      vpTime::wait(10);
    }

    std::cout << "Max Joint values: " << max_joint.t() << std::endl;
    std::cout << "Min Joint values: " << min_joint.t() << std::endl;

    // Best Result (small errors are to be expected)
    // Max Joint values: 0.7811045051  -0.07668215036  2.481732368
    2.819076777  1.044736624  2.687076807
    //  Min Joint values: -0.8011951447  -1.648244739  0.7439950705
    -3.022218227  -1.260564089  -2.054088593
*/

    virtuose.setPeriodicFunction(CallBackVirtuose);
    virtuose.startPeriodicFunction();

    int counter = 0;
    bool swtch = true;

    while (swtch) {
      if (counter >= 10) {
        virtuose.stopPeriodicFunction();
        virtuose.setPowerOff();
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
