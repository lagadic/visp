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
 *  Tests the control law
 */

/*!
 * \file moveBiclops.cpp
 * \example moveBiclops.cpp
 *
 * Example of a real robot control, the Biclops robot (pan-tilt turret) by
 * Traclabs. The robot is controlled first in position, then in velocity.
 *
 * See http://www.traclabs.com/tracbiclops.htm for more details.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "c:h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*
 * Print the program options.
 *
 * \param name : Program name.
 * \param badparam : Bad parameter name.
 * \param conf : Biclops configuration file.
 */
void usage(const char *name, const char *badparam, std::string conf)
{
  fprintf(stdout, "\n\
Move the Biclops robot\n\
\n\
SYNOPSIS\n\
  %s [-c <Biclops configuration file>] [-h]\n\
",
name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c <Biclops configuration file>                      %s\n\
     Sets the Biclops robot configuration file.\n\n",
          conf.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!
 * Set the program options.
 *
 * \param argc : Command line number of parameters.
 * \param argv : Array of command line parameters.
 * \param conf : Biclops configuration file.
 *
 * \return false if the program has to be stopped, true otherwise.
 */
bool getOptions(int argc, const char **argv, std::string &conf)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      conf = optarg_;
      break;
    case 'h':
      usage(argv[0], nullptr, conf);
      return false;

    default:
      usage(argv[0], optarg_, conf);
      return false;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], nullptr, conf);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  std::string opt_conf = "/usr/share/BiclopsDefault.cfg";

  // Read the command line options
  if (getOptions(argc, argv, opt_conf) == false) {
    return EXIT_FAILURE;
  }
  try {
    vpRobotBiclops robot(opt_conf);

    vpColVector q(vpBiclops::ndof);      // desired position
    vpColVector qdot(vpBiclops::ndof);   // desired velocity
    vpColVector qm(vpBiclops::ndof);     // measured position
    vpColVector qm_dot(vpBiclops::ndof); // measured velocity

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    q = 0;
    q[0] = vpMath::rad(-10);
    q[1] = vpMath::rad(-20);
    std::cout << "Set Joint position "
      << " pan: " << vpMath::deg(q[0]) << " deg"
      << " tilt: " << vpMath::deg(q[1]) << " deg" << std::endl;
    robot.setPositioningVelocity(30.);
    robot.setPosition(vpRobot::JOINT_STATE, q);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    q[0] = vpMath::rad(10);
    q[1] = vpMath::rad(20);
    std::cout << "Set Joint position "
      << " pan: " << vpMath::deg(q[0]) << " deg"
      << " tilt: " << vpMath::deg(q[1]) << " deg" << std::endl;
    robot.setPositioningVelocity(10);
    robot.setPosition(vpRobot::JOINT_STATE, q);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position: "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity: "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    std::cout << "Set STATE_VELOCITY_CONTROL" << std::endl;
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position: "
      << " pan: " << vpMath::deg(qm[0]) << " deg"
      << " tilt: " << vpMath::deg(qm[1]) << " deg" << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    qdot = 0;
    //  qdot[0] = vpMath::rad(0.1) ;
    qdot[1] = vpMath::rad(25);
    std::cout << "Set joint velocity for 5 sec"
      << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
      << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << std::endl;
    robot.setVelocity(vpRobot::JOINT_STATE, qdot);

    // waits 5000ms
    vpTime::wait(5000.0);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position "
      << " pan: " << vpMath::deg(qm[0]) << " deg"
      << " tilt: " << vpMath::deg(qm[1]) << " deg" << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    qdot = 0;
    //  qdot[0] = vpMath::rad(0.1) ;
    qdot[1] = -vpMath::rad(25);
    std::cout << "Set joint velocity for 3 sec"
      << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
      << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << std::endl;
    robot.setVelocity(vpRobot::JOINT_STATE, qdot);

    // waits 3000 ms
    vpTime::wait(3000.0);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position "
      << " pan: " << vpMath::deg(qm[0]) << " deg"
      << " tilt: " << vpMath::deg(qm[1]) << " deg" << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    qdot = 0;
    //  qdot[0] = vpMath::rad(0.1) ;
    qdot[1] = vpMath::rad(10);
    std::cout << "Set joint velocity for 2 sec"
      << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
      << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << std::endl;
    robot.setVelocity(vpRobot::JOINT_STATE, qdot);

    // waits 2000 ms
    vpTime::wait(2000.0);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position "
      << " pan: " << vpMath::deg(qm[0]) << " deg"
      << " tilt: " << vpMath::deg(qm[1]) << " deg" << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;

    qdot = 0;
    qdot[0] = vpMath::rad(-5);
    // qdot[1] = vpMath::rad(-5);

    std::cout << "Set joint velocity for 2 sec"
      << " pan: " << vpMath::deg(qdot[0]) << " deg/s"
      << " tilt: " << vpMath::deg(qdot[1]) << " deg/s" << std::endl;
    robot.setVelocity(vpRobot::JOINT_STATE, qdot);

    // waits 2000 ms
    vpTime::wait(2000.0);

    robot.getPosition(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint position "
      << " pan: " << vpMath::deg(qm[0]) << " deg"
      << " tilt: " << vpMath::deg(qm[1]) << " deg" << std::endl;
    robot.getVelocity(vpRobot::JOINT_STATE, qm);
    std::cout << "  Joint velocity "
      << " pan: " << vpMath::deg(qm[0]) << " tilt: " << vpMath::deg(qm[1]) << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
}
#else
int main()
{
  std::cout << "You do not have an Biclops PT robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif
