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
 * Firas Abi Farraj
 *
 *****************************************************************************/

/*!
  \example testVirtuoseHapticBox.cpp

  Test Haption Virtuose SDK wrapper to constrain movements inside a cube of
  pre-determined side. Hard springs are applied to rotations (only
  translations are allowed).
*/

#include <visp3/core/vpTime.h>
#include <visp3/robot/vpVirtuose.h>

#if defined(VISP_HAVE_VIRTUOSE)

void CallBackVirtuose(VirtContext VC, void *ptr)
{
  (void)VC;

  static bool firstIteration = true;
  static vpPoseVector localPosition0;

  vpPoseVector localPosition;
  vpColVector forceFeedback(6, 0);
  vpColVector finalForce(6, 0);
  vpColVector forceEe(6, 0);
  int force_limit = 15;
  int force_increase_rate = 500;
  float cube_size = 0.05f;

  // Virtual spring to let the user know where the initial position is
  // Estimated Virtuose handle mass = 0.1;
  // Estimated Virtuose handle length = 0.23;
  // In the ee frame: Virtuose Handle as a cylinder for the inertia
  // Estimated Inertia1 = m*l*l/12
  // Estimated Inertia2 = m*l*l/2 (rotation w.r.t. e-e z axis)
  double virtualStiffnessAng = 20;
  double virtualDamperAng = 0.182;   // greater than sqrt 4*Inertia1*virtualStiffnessAng
  double virtualDamperAng2 = 0.0456; // greater than sqrt 4*Inertia2*virtualStiffnessAng

  vpColVector xd(3, 0);
  vpColVector yd(3, 0);
  vpColVector zd(3, 0);
  vpColVector xee(3, 0);
  vpColVector zee(3, 0);
  vpColVector xeed(3, 0);
  vpColVector zeed(3, 0);
  vpColVector zYZ(3, 0);
  vpColVector zXZ(3, 0);
  vpColVector xXY(3, 0);
  vpTranslationVector tee;
  vpColVector omegad(3, 0);
  vpRotationMatrix Qd;
  vpRotationMatrix Qee;
  vpPoseVector pee;
  vpColVector vee(6, 0);
  vpColVector veed(6, 0);

  double alpha;

  vpColVector torque1(3, 0);
  vpColVector torque2(3, 0);
  vpColVector torque3(3, 0);

  vpVirtuose *p_virtuose = (vpVirtuose *)ptr;
  localPosition = p_virtuose->getPhysicalPosition();

  if (firstIteration) {
    localPosition0 = localPosition;
    firstIteration = false;
  }

  //  Position and velocity in of the ee expressed in the base frame
  pee = localPosition;
  vee = p_virtuose->getPhysicalVelocity();
  // Z axis = [pee_x pee_y 0]
  zd[0] = pee[0];
  zd[1] = pee[1];
  zd.normalize();
  // X axis = [0 0 1]
  xd[2] = 1;
  // Y axis from cross product
  yd = zd.skew(zd) * xd;

  // Current orientation of the ee frame
  pee.extract(Qee);
  pee.extract(tee);
  // X and Z axis of the ee frame expressed in the base frame
  xee = Qee.getCol(0);
  zee = Qee.getCol(2);

  // Rotation matrix from Desired Frame to Base Frame
  Qd[0][0] = xd[0];
  Qd[1][0] = xd[1];
  Qd[2][0] = xd[2];
  Qd[0][1] = yd[0];
  Qd[1][1] = yd[1];
  Qd[2][1] = yd[2];
  Qd[0][2] = zd[0];
  Qd[1][2] = zd[1];
  Qd[2][2] = zd[2];

  // X and Z axis of the ee frame expressed in the desired frame
  xeed = Qd.inverse() * xee;
  zeed = Qd.inverse() * zee;

  vpHomogeneousMatrix dMb(tee, Qd);
  // Velocity twist matrix for expressing velocities in the desired frame
  vpVelocityTwistMatrix dVMb(dMb.inverse());
  // Force twist matrix for expressing forces in the base frame
  vpForceTwistMatrix dFMb(dMb);

  veed = dVMb * vee;

  // Angular velocity in the desired frame
  omegad[0] = veed[3];
  omegad[1] = veed[4];
  omegad[2] = veed[5];

  // Projection of Z axis of the ee frame onto plane YZ (expressed in the
  // desired frame)
  zYZ[1] = zeed[1];
  zYZ[2] = zeed[2];

  // Projection of Z axis of the ee frame onto plane XZ (expressed in the
  // desired frame)
  zXZ[0] = zeed[0];
  zXZ[2] = zeed[2];

  // Hard spring to keep Z axis of the ee frame in the horizontal plane
  // Spring applied to the angle between the Z axis of the ee frame and its
  // projection in the YZ (horizontal) plane
  vpColVector rotzYZ(3, 0);
  rotzYZ = zeed.skew(zeed) * zYZ.normalize();
  vpColVector forceStiff1 = virtualStiffnessAng * rotzYZ;
  vpColVector forceDamp1 = virtualDamperAng * (omegad * rotzYZ.normalize()) * rotzYZ.normalize();

  for (unsigned int i = 0; i < 3; i++)
    torque1[i] = forceStiff1[i] - forceDamp1[i];

  // Hard spring to keep Z axis of the ee frame pointing at the origin
  // Spring applied to the angle between the Z axis of the ee frame and its
  // projection in the XZ (vertical) plane
  vpColVector rotzXZ(3, 0);
  rotzXZ = zeed.skew(zeed) * zXZ.normalize();
  vpColVector forceStiff2 = virtualStiffnessAng * rotzXZ;
  vpColVector forceDamp2 = virtualDamperAng * (omegad * rotzXZ.normalize()) * rotzXZ.normalize();

  for (unsigned int i = 0; i < 3; i++)
    torque2[i] = forceStiff2[i] - forceDamp2[i];

  // Hard spring for rotation around z axis of the ee
  xXY[0] = xeed[0];
  xXY[1] = xeed[1];
  vpColVector xdd(3, 0);
  xdd[0] = 1;
  vpColVector zdd(3, 0);
  zdd[2] = 1;
  vpColVector rotxXY(3, 0);
  rotxXY = xdd.skew(xdd) * xXY.normalize();
  alpha = asin(rotxXY[2]);

  vpColVector forceStiff3 = virtualStiffnessAng * alpha * zdd;
  vpColVector forceDamp3 = virtualDamperAng2 * (omegad * zdd) * zdd;
  for (unsigned int i = 0; i < 3; i++)
    torque3[i] = forceStiff3[i] - forceDamp3[i];

  for (unsigned int j = 0; j < 3; j++)
    forceEe[j + 3] = torque1[j] + torque2[j] + torque3[j];

  forceEe = dFMb * forceEe;

  // ---------------
  //  Haptic Box
  // ---------------
  vpColVector p_min(3, 0), p_max(3, 0);
  for (unsigned int i = 0; i < 3; i++) {
    p_min[i] = localPosition0[i] - cube_size / 2;
    p_max[i] = localPosition0[i] + cube_size / 2;
  }

  for (int i = 0; i < 3; i++) {
    if ((p_min[i] >= localPosition[i])) {
      forceFeedback[i] = (p_min[i] - localPosition[i]) * force_increase_rate;
      if (forceFeedback[i] >= force_limit)
        forceFeedback[i] = force_limit;
    } else if ((p_max[i] <= localPosition[i])) {
      forceFeedback[i] = (p_max[i] - localPosition[i]) * force_increase_rate;
      if (forceFeedback[i] <= -force_limit)
        forceFeedback[i] = -force_limit;
    } else
      forceFeedback[i] = 0;
  }

  for (unsigned int j = 0; j < 6; j++)
    finalForce[j] = forceFeedback[j] + forceEe[j];

  // Set force feedback
  p_virtuose->setForce(finalForce);

  return;
}

int main()
{
  try {
    vpVirtuose virtuose;
    virtuose.setIpAddress("localhost#5000");
    virtuose.setVerbose(true);
    virtuose.setPowerOn();
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
int main()
{
  std::cout << "You should install pthread and/or Virtuose API to use this "
               "binary..."
            << std::endl;
}
#endif
