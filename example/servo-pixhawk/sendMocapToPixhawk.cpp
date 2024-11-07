/****************************************************************************
 *
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
 * Example that shows how to send a pose from a motion capture system through masvsdk.
 *
*****************************************************************************/

/*!
 * @example sendMocapToPixhawk.cpp
 *
 * Send motion capture data to a Pixhawk using MAVSDK.
 *
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) &&                                        \
    (defined(VISP_HAVE_QUALISYS) || defined(VISP_HAVE_VICON)) && defined(VISP_HAVE_THREADS)

#include <chrono>
#include <thread>

#include <visp3/robot/vpRobotMavsdk.h>
#include <visp3/sensor/vpMocapQualisys.h>
#include <visp3/sensor/vpMocapVicon.h>

using std::chrono::seconds;
using std::this_thread::sleep_for;

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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
                    std::map<std::string, vpHomogeneousMatrix> &current_body_poses_enu_M_flu, bool &mocap_failure,
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
  }
  else {
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

  bool internal_mavlink_failure = false;
  while (!g_quit && !internal_mavlink_failure) {
    std::map<std::string, vpHomogeneousMatrix> body_poses_enu_M_flu;

    if (opt_onlyBody == "") {
      if (!mocap->getBodiesPose(body_poses_enu_M_flu, opt_all_bodies)) {
        g_quit = true;
      }
    }
    else {
      vpHomogeneousMatrix enu_M_flu;
      if (!mocap->getSpecificBodyPose(opt_onlyBody, enu_M_flu)) {
        g_quit = true;
      }
      else {
        body_poses_enu_M_flu[opt_onlyBody] = enu_M_flu;
      }
    }

    lock.lock();
    internal_mavlink_failure = mavlink_failure;
    current_body_poses_enu_M_flu =
      body_poses_enu_M_flu; // Now we send directly the poses in the ENU global reference frame.
    lock.unlock();
  }
  return true;
}

// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int top(const std::string &connection_info, std::map<std::string, vpHomogeneousMatrix> &current_body_poses_enu_M_flu,
        std::mutex &lock, bool &mocap_failure)
{
  std::map<std::string, vpHomogeneousMatrix> body_poses_enu_M_flu;
  bool internal_mocap_failure = false;
  const double fps = 100;

  vpRobotMavsdk drone { connection_info };

  while (!g_quit && !internal_mocap_failure) {
    double t = vpTime::measureTimeMs();
    lock.lock();
    body_poses_enu_M_flu = current_body_poses_enu_M_flu;
    internal_mocap_failure = mocap_failure;
    lock.unlock();

    for (std::map<std::string, vpHomogeneousMatrix>::iterator it = body_poses_enu_M_flu.begin();
         it != body_poses_enu_M_flu.end(); ++it) {
      if (!drone.sendMocapData(it->second)) {
        return 1;
      }
    }
    vpTime::wait(t, 1000./fps); // Stream MoCap at given framerate
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
    << " [--mocap-system <qualisys>/<vicon>] [-ms <q>/<v>]"
    << " [--device <device port>] [-d]"
    << " [--server-address <server address>] [-sa]"
    << " [--verbose] [-v]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "MANDATORY PARAMETERS :" << std::endl
    << "  --only-body <name>" << std::endl
    << "    Name of the specific body you want to be displayed." << std::endl
    << std::endl
    << "OPTIONAL PARAMETERS (DEFAULT VALUES)" << std::endl
    << "  --mocap-system, -ms" << std::endl
    << "    Specify the name of the mocap system : 'qualisys' / 'q' or 'vicon'/ 'v'." << std::endl
    << "    Default: Qualisys mode." << std::endl
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
    << "  --verbose, -v" << std::endl
    << "    Enable verbose mode." << std::endl
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
    }
    else if (std::string(argv[i]) == "--mocap-system" || std::string(argv[i]) == "-ms") {
      std::string mode = std::string(argv[i + 1]);
      if (mode == "qualisys" || mode == "q") {
        qualisys = true;
      }
      else if (mode == "vicon" || mode == "v") {
        qualisys = false;
      }
      else {
        std::cout << "ERROR : System not recognized, exiting." << std::endl;
        throw EXIT_FAILURE;
      }
      i++;
    }
    else if (std::string(argv[i]) == "--device" || std::string(argv[i]) == "-d") {
      connection_info = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--server-address" || std::string(argv[i]) == "-sa") {
      server_address = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--all-bodies") {
      all_bodies = true;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      throw EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      throw EXIT_FAILURE;
    }
  }

  return;
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  std::map<std::string, vpHomogeneousMatrix> current_body_poses_enu_M_flu;

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
  }
  else if (!opt_qualisys && opt_serverAddress == "") {
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
                            &current_body_poses_enu_M_flu, &mocap_failure, &mavlink_failure]() {
                              mocap_sdk_loop(lock, opt_qualisys, opt_verbose, opt_all_bodies, opt_serverAddress, opt_onlyBody,
                              current_body_poses_enu_M_flu, mocap_failure, mavlink_failure);
  });
  if (mocap_failure) {
    std::cout << "Mocap connexion failure. Check mocap server IP address" << std::endl;
    return EXIT_FAILURE;
  }

  // This program uses throw, wrap one big try/catch here
  std::thread mavlink_thread(
      [&lock, &current_body_poses_enu_M_flu, &opt_connectionInfo, &mocap_failure, &mavlink_failure]() {
        try {
          int result = top(opt_connectionInfo, current_body_poses_enu_M_flu, lock, mocap_failure);
          return result;
        }
        catch (int error) {
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
  }
  else {
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
#if !((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  std::cout
    << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
    "rebuild ViSP.\n"
    << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
