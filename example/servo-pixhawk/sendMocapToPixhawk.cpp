/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Example that shows how to send a pose from a motion capture system through masvsdk.
 *
 *****************************************************************************/

/*!
 * @example sendMocapToPixhawk.cpp
 *
 * Send motion capture data to a Pixhawk using mavsdk
 *
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17) &&                                        \
    (defined(VISP_HAVE_QUALISYS) || defined(VISP_HAVE_VICON))

#include <chrono>
#include <thread>

#include <visp3/robot/vpRobotMavsdk.h>
#include <visp3/sensor/vpMocapQualisys.h>
#include <visp3/sensor/vpMocapVicon.h>

using std::chrono::seconds;
using std::this_thread::sleep_for;

// ------------------------------------------------------------------------------
//   Modifications Qualisys
// ------------------------------------------------------------------------------

bool g_quit = false;

/*!
  Quit signal handler : this function is called when CTRL-C is pressed.
  \param[in] sig : Unused.
*/
void quitHandler(int sig)
{
  (void)sig;
  std::cout << std::endl << "TERMINATING AT USER REQUEST" << std::endl << std::endl;

  g_quit = true;
}

/*!
 *
 * \return true when success, false otherwise.
 */
bool mocap_sdk_loop(std::mutex &lock, bool qualisys, bool opt_verbose, bool opt_all_bodies,
                    std::string &opt_serverAddress, std::string &opt_onlyBody,
                    std::map<std::string, vpHomogeneousMatrix> &current_bodies_pose_NED, bool &mocap_failure,
                    bool &mavlink_failure)
{
  std::shared_ptr<vpMocap> mocap;
  if (qualisys) {
#ifdef VISP_HAVE_QUALISYS
    mocap = std::make_shared<vpMocapQualisys>();
#else
    std::cout << "ERROR : Qualisys not found.";
    return false;
#endif
  } else {
#ifdef VISP_HAVE_VICON
    mocap = std::make_shared<vpMocapVicon>();
#else

    std::cout << "ERROR : Vicon not found.";
    return false;
#endif
  }
  mocap->setVerbose(opt_verbose);
  mocap->setServerAddress(opt_serverAddress);
  if (mocap->connect() == false) {
    lock.lock();
    mocap_failure = true;
    lock.unlock();
    std::cout << "Mocap connexion failure. Check mocap server IP address" << std::endl;

    return false;
  }

  vpHomogeneousMatrix W_NED_Mat_W_Qualisys;
  W_NED_Mat_W_Qualisys.eye();
  W_NED_Mat_W_Qualisys[1][1] = -1;
  W_NED_Mat_W_Qualisys[2][2] = -1;

  bool internal_mavlink_failure = false;
  while (!g_quit && !internal_mavlink_failure) {
    std::map<std::string, vpHomogeneousMatrix> bodies_pose;

    if (opt_onlyBody == "") {
      if (!mocap->getBodiesPose(bodies_pose, opt_all_bodies)) {
        g_quit = true;
      }
    } else {
      vpHomogeneousMatrix pose;
      if (!mocap->getSpecificBodyPose(opt_onlyBody, pose)) {
        g_quit = true;
      } else {
        bodies_pose[opt_onlyBody] = pose;
      }
    }

    std::map<std::string, vpHomogeneousMatrix> bodies_pose_NED;
    for (std::map<std::string, vpHomogeneousMatrix>::iterator it = bodies_pose.begin(); it != bodies_pose.end(); ++it) {
      bodies_pose_NED[it->first] = W_NED_Mat_W_Qualisys * it->second;
    }

    lock.lock();
    internal_mavlink_failure = mavlink_failure;
    current_bodies_pose_NED = bodies_pose_NED; // Now we send directly the poses in the NED frame.
    lock.unlock();

    vpTime::sleepMs(5);
  }
  return true;
}

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(const std::string &connection_info, std::map<std::string, vpHomogeneousMatrix> &current_bodies_pose_NED,
        std::mutex &lock, bool &mocap_failure)
{
  std::map<std::string, vpHomogeneousMatrix> bodies_pose_NED;
  bool internal_mocap_failure = false;

  vpRobotMavsdk drone{connection_info};

  while (!g_quit && !internal_mocap_failure) {
    lock.lock();
    bodies_pose_NED = current_bodies_pose_NED;
    internal_mocap_failure = mocap_failure;
    lock.unlock();

    for (std::map<std::string, vpHomogeneousMatrix>::iterator it = bodies_pose_NED.begin(); it != bodies_pose_NED.end();
         ++it) {
      if (!drone.sendMocapData(it->second)) {
        return 1;
      }
    }
    vpTime::sleepMs(33);
  }

  return 0;
}

// ------------------------------------------------------------------------------
//   Usage function
// ------------------------------------------------------------------------------

void usage(char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
            << "  " << argv[0] << " [--only-body <name>] [-ob]"
            << " [--qualisys] [-q]"
            << " [--device <device port>] [-d]"
            << " [--server-address <server address>] [-sa]"
            << " [--baud <baudrate>] [-b]"
            << " [--read-thread] [-rt]"
            << " [--autotakeoff] [-a]"
            << " [--all-bodies]"
            << " [--verbose] [-v]"
            << " [--logs]"
            << " [--help] [-h]" << std::endl
            << std::endl;
  std::cout << "DESCRIPTION" << std::endl
            << "MANDATORY PARAMETERS :" << std::endl
            << "  --only-body <name>" << std::endl
            << "    Name of the specific body you want to be displayed." << std::endl
            << std::endl
            << "OPTIONAL PARAMETERS (DEFAULT VALUES)" << std::endl
            << "  --qualisys, -q" << std::endl
            << "    When used, sets the qualisys mode." << std::endl
            << "    Default: false (Vicon mode)." << std::endl
            << std::endl
            << "  --device <device port>, -d" << std::endl
            << "    String giving us all the informations necessary for connection." << std::endl
            << "    Default: serial:///dev/ttyUSB0 ." << std::endl
            << "    UDP example: udp://192.168.30.111:14540 (udp://IP:Port) ." << std::endl
            << std::endl
            << "  --server-address <address>, -sa" << std::endl
            << "    Mocap server address." << std::endl
            << "    Default for Qualisys: 192.168.34.42 ." << std::endl
            << "    Default for Vicon: 192.168.34.1 ." << std::endl
            << std::endl
            << "  --baud <baudrate>, -b" << std::endl
            << "    Set up the desired baudrate for the Mavlink messages." << std::endl
            << "    Default: 57600 ." << std::endl
            << std::endl
            << "  --all-bodies" << std::endl
            << "    When used, get all bodies pose including non visible bodies." << std::endl
            << std::endl
            << "  --verbose, -v" << std::endl
            << "    Enable verbose mode." << std::endl
            << std::endl
            << "  --logs" << std::endl
            << "    When active, creates a log of data sent to the drone in /tmp/." << std::endl
            << std::endl
            << "  --help, -h" << std::endl
            << "    Print this helper message." << std::endl
            << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
              << "  "
              << "Unsupported parameter " << argv[error] << std::endl;
  }
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void parse_commandline(int argc, char **argv, bool &qualisys, std::string &connection_info, std::string &server_address,
                       std::string &only_body, bool &all_bodies, bool &verbose)
{

  // Read input arguments
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--only-body" || std::string(argv[i]) == "-ob") {
      only_body = std::string(argv[i + 1]);
      i++;
    } else if (std::string(argv[i]) == "--qualisys" || std::string(argv[i]) == "-q") {
      qualisys = true;
    } else if (std::string(argv[i]) == "--device" || std::string(argv[i]) == "-d") {
      connection_info = std::string(argv[i + 1]);
      i++;
    } else if (std::string(argv[i]) == "--server-address" || std::string(argv[i]) == "-sa") {
      server_address = std::string(argv[i + 1]);
      i++;
    } else if (std::string(argv[i]) == "--all-bodies") {
      all_bodies = true;
    } else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      verbose = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      throw EXIT_SUCCESS;
    } else {
      usage(argv, i);
      throw EXIT_FAILURE;
    }
  }
  // end: for each input argument

  // Done!
  return;
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  std::map<std::string, vpHomogeneousMatrix> current_bodies_pose_NED;

  // Default input arguments
#ifdef __APPLE__
  std::string opt_connectionInfo = "/dev/tty.usbmodem1";
#else
  std::string opt_connectionInfo = "udp://127.0.0.1:14550";
#endif

  bool opt_qualisys = false;
  std::string opt_serverAddress;
  std::string opt_onlyBody = "";
  bool opt_all_bodies = false;
  bool opt_verbose = false;

  // User Input
  parse_commandline(argc, argv, opt_qualisys, opt_connectionInfo, opt_serverAddress, opt_onlyBody, opt_all_bodies,
                    opt_verbose);

  if (opt_qualisys && opt_serverAddress == "") {
    opt_serverAddress = "192.168.30.42";
  } else if (!opt_qualisys && opt_serverAddress == "") {
    opt_serverAddress = "192.168.30.1";
  }

  if (opt_onlyBody == "") {
    std::cout << "The parameter --only-body MUST be given in the command line." << std::endl;
    return EXIT_FAILURE;
  }

  // Modifications qualisys ----------------------------------------------------
  std::mutex lock;
  bool mocap_failure = false;
  bool mavlink_failure = false;
  std::thread mocap_thread([&lock, &opt_qualisys, &opt_verbose, &opt_all_bodies, &opt_serverAddress, &opt_onlyBody,
                            &current_bodies_pose_NED, &mocap_failure, &mavlink_failure]() {
    mocap_sdk_loop(lock, opt_qualisys, opt_verbose, opt_all_bodies, opt_serverAddress, opt_onlyBody,
                   current_bodies_pose_NED, mocap_failure, mavlink_failure);
  });
  if (mocap_failure) {
    std::cout << "Mocap connexion failure. Check mocap server IP address" << std::endl;
    return EXIT_FAILURE;
  }

  // This program uses throw, wrap one big try/catch here
  std::thread mavlink_thread(
      [&lock, &current_bodies_pose_NED, &opt_connectionInfo, &mocap_failure, &mavlink_failure]() {
        try {
          int result = top(opt_connectionInfo, current_bodies_pose_NED, lock, mocap_failure);
          return result;
        } catch (int error) {
          fprintf(stderr, "mavlink_control threw exception %i \n", error);
          lock.lock();
          mavlink_failure = true;
          lock.unlock();
          return error;
        }
      });

  mocap_thread.join();
  mavlink_thread.join();
  if (mocap_failure) {
    return EXIT_FAILURE;
  } else {
    return EXIT_SUCCESS;
  }
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n"
            << std::endl;
#endif
#if !(defined(VISP_HAVE_QUALISYS) || defined(VISP_HAVE_VICON))
  std::cout << "\nThis example requires data from a Qualisys or Vicon mocap system. You should install it, configure "
               "and rebuid ViSP.\n"
            << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  std::cout
      << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
         "rebuild ViSP.\n"
      << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
