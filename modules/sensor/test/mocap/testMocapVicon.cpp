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
 * Test Vicon Motion Capture System.
 */

/*!
 * \example testMocapVicon.cpp
 */

#include <visp3/sensor/vpMocapVicon.h>

#include <iostream>

#if defined(VISP_HAVE_VICON) && defined(VISP_HAVE_THREADS)

#include <mutex>
#include <signal.h>
#include <thread>

#include <visp3/sensor/vpMocapVicon.h>

#include <visp3/core/vpTime.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

bool g_quit = false;

/*!
 * Quit signal handler : this function is called when CTRL-C is pressed.
 * \param[in] sig : Unused.
 */
void quitHandler(int sig)
{
  std::cout << std::endl << "TERMINATING AT USER REQUEST" << std::endl << std::endl;

  g_quit = true;
  (void)sig;
}

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--server-address <address>]"
    << " [--only-body]"
    << " [--all-bodies]"
    << " [--verbose] [-v]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --server-address <address>" << std::endl
    << "    Server address." << std::endl
    << "    Default: 192.168.30.1." << std::endl
    << std::endl
    << "  --only-body <name>" << std::endl
    << "    Name of the specific body you want to be displayed." << std::endl
    << "    Default: ''" << std::endl
    << std::endl
    << "  --all-bodies" << std::endl
    << "    When used, get all bodies pose including non visible bodies." << std::endl
    << std::endl
    << "  --verbose, -v" << std::endl
    << "    Enable verbose mode." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout << "USAGE" << std::endl
    << "  Example to test Vicon connection:" << std::endl
    << "    " << argv[0] << " --server-address 127.0.0.1  --verbose" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

void mocap_loop(std::mutex &lock, bool opt_verbose, bool opt_all_bodies, std::string &opt_serverAddress,
                std::string &opt_onlyBody, std::map<std::string, vpHomogeneousMatrix> &current_bodies_pose)
{
  vpMocapVicon vicon;
  vicon.setVerbose(opt_verbose);
  vicon.setServerAddress(opt_serverAddress);
  vicon.connect();
  while (!g_quit) {
    std::map<std::string, vpHomogeneousMatrix> bodies_pose;

    if (opt_onlyBody == "") {
      vicon.getBodiesPose(bodies_pose, opt_all_bodies);
    }
    else {
      vpHomogeneousMatrix pose;
      vicon.getSpecificBodyPose(opt_onlyBody, pose);
      bodies_pose[opt_onlyBody] = pose;
    }

    lock.lock();
    current_bodies_pose = bodies_pose;
    lock.unlock();

    vpTime::sleepMs(5);
  }
}

void display_loop(std::mutex &lock, const std::map<std::string, vpHomogeneousMatrix> &current_bodies_pose, bool verbose)
{
  std::map<std::string, vpHomogeneousMatrix> bodies_pose;

  while (!g_quit) {

    lock.lock();
    bodies_pose = current_bodies_pose;
    lock.unlock();
    for (std::map<std::string, vpHomogeneousMatrix>::iterator it = bodies_pose.begin(); it != bodies_pose.end(); ++it) {
      vpRxyzVector rxyz(it->second.getRotationMatrix());
      std::cout << "Found body: " << it->first << std::endl;
      if (verbose) {
        std::cout << "  Translation [m]: " << it->second.getTranslationVector().t() << std::endl
          << "  Quaternion: " << vpQuaternionVector(it->second.getRotationMatrix()).t() << std::endl;
        std::cout << "  Roll/pitch/yaw [deg]: ";
        for (unsigned int i = 0; i < 3; i++) {
          std::cout << vpMath::deg(rxyz[i]) << " ";
        }
        std::cout << std::endl;
      }
    }

    vpTime::sleepMs(200);
  }
}

int main(int argc, const char *argv[])
{
  bool opt_verbose = false;
  std::string opt_serverAddress = "192.168.30.1";
  std::string opt_onlyBody = "";
  bool opt_all_bodies = false;

  // Map containig all the current poses of the drones
  std::map<std::string, vpHomogeneousMatrix> current_bodies_pose;

  signal(SIGINT, quitHandler);

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--server-address") {
      opt_serverAddress = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--only-body") {
      opt_onlyBody = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--all-bodies") {
      opt_all_bodies = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  std::mutex lock;
  std::thread mocap_thread(
      [&lock, &opt_verbose, &opt_all_bodies, &opt_serverAddress, &opt_onlyBody, &current_bodies_pose]() {
        mocap_loop(lock, opt_verbose, opt_all_bodies, opt_serverAddress, opt_onlyBody, current_bodies_pose);
      });
  std::thread display_thread(
      [&lock, &current_bodies_pose, &opt_verbose]() { display_loop(lock, current_bodies_pose, opt_verbose); });

  mocap_thread.join();
  display_thread.join();

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "Install Vicon Datastream SDK to be able to test Vicon Mocap System using ViSP" << std::endl;

  return EXIT_SUCCESS;
}
#endif
