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
 * Interface for the Biclops robot.
 */

#include <signal.h>
#include <string.h>
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <unistd.h>
#endif
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#ifdef VISP_HAVE_BICLOPS

/* Headers */
#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/robot/vpRobotException.h>

#include "vpRobotBiclopsController_impl.h"

//#define VP_DEBUG        // Activate the debug mode
//#define VP_DEBUG_MODE 20 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

BEGIN_VISP_NAMESPACE
vpRobotBiclops::vpRobotBiclopsController::vpRobotBiclopsController()
  : m_biclops(), m_axisMask(0), m_panAxis(nullptr), m_tiltAxis(nullptr), m_vergeAxis(nullptr), m_panProfile(), m_tiltProfile(),
  m_vergeProfile(), m_shm(), m_stopControllerThread(false)
{
  m_axisMask = Biclops::PanMask + Biclops::TiltMask; //+ Biclops::VergeMask*/; // add this if you want verge.

  // Set Debug level depending on how much info you want to see about
  // the inner workings of the API. Level 2 is highest with 0 being
  // the default (i.e., no messages).
  m_biclops.SetDebugLevel(0);

  // initialize the shared data structure
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    m_shm.status[i] = STOP;
    m_shm.q_dot[i] = 0.;
    m_shm.actual_q[i] = 0.;
    m_shm.jointLimit[i] = false;
    m_shm.status[i] = STOP;
  }
}

vpRobotBiclops::vpRobotBiclopsController::~vpRobotBiclopsController() { }

void vpRobotBiclops::vpRobotBiclopsController::init(const std::string &configfile)
{
  vpDEBUG_TRACE(12, "Initialize Biclops.");
  bool binit = false;
  for (int i = 0; i < 1; i++) {
    try {
      std::cout << "Try to initialize Biclops head " << std::endl;
      binit = m_biclops.Initialize(configfile.c_str());
      usleep(100000);
      if (binit) {
        // Initialization completed successfully. Close the config file.
        std::cout << "Initialization succeed...\n";
        break;
      }
      else {
        std::cout << "Initialization failed...\n";
      }
    }
    catch (...) {
      std::cout << "Initialization failed..." << std::endl;
    }
  }

  if (!binit) {
    std::cout << "Cannot initialize Biclops head. " << std::endl;
    std::cout << "Check if the serial cable is connected." << std::endl;
    std::cout << "Check if the robot is powered on." << std::endl;
    std::cout << "Check if you try to open the good serial port." << std::endl;
    std::cout << "Try to power off/on and restart..." << std::endl;

    throw vpRobotException(vpRobotException::notInitializedError, "Cannot initialize Biclops head.");
  }

  vpDEBUG_TRACE(12, "Biclops initialized");

  // Get shortcut references to each axis.
  m_panAxis = m_biclops.GetAxis(Biclops::Pan);
  m_tiltAxis = m_biclops.GetAxis(Biclops::Tilt);
  if ((m_axisMask & Biclops::VergeMask) != 0)
    m_vergeAxis = m_biclops.GetAxis(Biclops::Verge);

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  if (!m_panAxis->GetHomedState() || !m_tiltAxis->GetHomedState()) {
    vpDEBUG_TRACE(12, "Biclops is not homed");
  }
#else // old API
  if (!m_panAxis->IsAlreadyHomed() || !m_tiltAxis->IsAlreadyHomed()) {
    vpDEBUG_TRACE(12, "Biclops is not homed");
  }
#endif

  // Execute the homing sequence for all axes.
  vpDEBUG_TRACE(12, "Execute the homing sequence for all axes");
  vpDEBUG_TRACE(12, "Execute the homing sequence for all axes");
  if (m_biclops.HomeAxes(m_axisMask))
    vpDEBUG_TRACE(12, "Homing sequence succeed.");
  else {
    vpERROR_TRACE("Homing sequence failed. Program is stopped");
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with Biclops");
  }

  // Get the currently defined (default) motion profiles.
  //      PMDAxisControl::Profile m_panProfile,m_tiltProfile,m_vergeProfile;
  m_panAxis->GetProfile(m_panProfile);
  m_tiltAxis->GetProfile(m_tiltProfile);
  if ((m_axisMask & Biclops::VergeMask) != 0)
    m_vergeAxis->GetProfile(m_vergeProfile);
}

void vpRobotBiclops::vpRobotBiclopsController::setPosition(const vpColVector &q, double percentVelocity)
{
  if (q.getRows() != vpBiclops::ndof) {
    vpERROR_TRACE("Bad dimension for positioning vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for positioning vector.");
  }

  m_panAxis->SetProfileMode(PMDTrapezoidalProfile);
  m_tiltAxis->SetProfileMode(PMDTrapezoidalProfile);

  // Create the list of axes we want to coordinate
  PMDUtils::AxisList axisList;
  axisList.push_back(m_panAxis);
  axisList.push_back(m_tiltAxis);

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  // Get the currently defined (default) motion profiles.
  // PMDAxisControl::Profile m_panProfile,m_tiltProfile;
  m_panAxis->GetProfile(m_panProfile);
  m_tiltAxis->GetProfile(m_tiltProfile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_panProfile.pos = PMDUtils::RadsToRevs(q[0]);
  m_panProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  m_tiltProfile.pos = PMDUtils::RadsToRevs(q[1]);
  m_tiltProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  // Inform the controller of the new desired position.
  m_panAxis->SetProfile(m_panProfile);
  m_tiltAxis->SetProfile(m_tiltProfile);

#else // old API

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_panProfile.pos = PMDUtils::RadsToRevs(q[0]);
  m_panProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  vpDEBUG_TRACE(12, "Speed percent: %lf", vpBiclops::speedLimit * percentVelocity / 100.);

  m_panAxis->ProfileToCounts(m_panProfile, desired_profile);
  vpCDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << std::endl;
  vpCDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << std::endl;

  m_panAxis->SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_tiltProfile.pos = PMDUtils::RadsToRevs(q[1]);
  m_tiltProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  m_tiltAxis->ProfileToCounts(m_tiltProfile, desired_profile);
  vpCDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << std::endl;
  vpCDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << std::endl;

  m_tiltAxis->SetProfile(desired_profile);
#endif

  // Coordinate motion
  PMDUtils::Coordinate(axisList);
  m_biclops.Move(Biclops::PanMask + Biclops::TiltMask /*, 0*/); //
}

void vpRobotBiclops::vpRobotBiclopsController::setVelocity(const vpColVector &q_dot)
{
  if (q_dot.getRows() != vpBiclops::ndof) {
    vpERROR_TRACE("Bad dimension for velocity vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for velocity vector.");
  }

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  // Get the currently defined (default) motion profiles.
  // PMDAxisControl::Profile m_panProfile, m_tiltProfile;
  m_panAxis->GetProfile(m_panProfile);
  m_tiltAxis->GetProfile(m_tiltProfile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_panProfile.vel = PMDUtils::RadsToRevs(q_dot[0]);
  m_tiltProfile.vel = PMDUtils::RadsToRevs(q_dot[1]);

  // Inform the controller of the new desired position.
  m_panAxis->SetProfile(m_panProfile);
  m_tiltAxis->SetProfile(m_tiltProfile);

  m_panAxis->SetProfileMode(PMDVelocityContouringProfile);
  m_tiltAxis->SetProfileMode(PMDVelocityContouringProfile);
#else // old API
  m_panAxis->SetProfileMode(PMDVelocityContouringProfile);
  m_tiltAxis->SetProfileMode(PMDVelocityContouringProfile);

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_panProfile.vel = PMDUtils::RadsToRevs(q_dot[0]);

  m_panAxis->ProfileToCounts(m_panProfile, desired_profile);
  m_panAxis->SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  m_tiltProfile.vel = PMDUtils::RadsToRevs(q_dot[1]);

  m_tiltAxis->ProfileToCounts(m_tiltProfile, desired_profile);
  m_tiltAxis->SetProfile(desired_profile);
#endif
  // Coordinate motion
  m_biclops.Move(Biclops::PanMask + Biclops::TiltMask, 0); //
}

vpColVector vpRobotBiclops::vpRobotBiclopsController::getPosition()
{
  vpDEBUG_TRACE(12, "Start vpRobotBiclopsController::getPosition() ");
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  m_panAxis->GetPosition(panpos);
  m_tiltAxis->GetPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(m_panAxis->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(m_tiltAxis->CountsToUnits(tiltpos));

  vpCDEBUG(11) << "++++++++ Mesure : " << q.t();
  vpDEBUG_TRACE(12, "End vpRobotBiclopsController::getPosition()");

  return q;
}

vpColVector vpRobotBiclops::vpRobotBiclopsController::getActualPosition()
{
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  m_panAxis->GetActualPosition(panpos);
  m_tiltAxis->GetActualPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(m_panAxis->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(m_tiltAxis->CountsToUnits(tiltpos));

  return q;
}

vpColVector vpRobotBiclops::vpRobotBiclopsController::getVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  m_panAxis->GetVelocity(pan_vel);
  m_tiltAxis->GetVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(m_panAxis->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(m_tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

vpColVector vpRobotBiclops::vpRobotBiclopsController::getActualVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  m_panAxis->GetActualVelocity(pan_vel);
  m_tiltAxis->GetActualVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(m_panAxis->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(m_tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

void vpRobotBiclops::vpRobotBiclopsController::writeShm(shmType &shm)
{
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "q_dot[%d]=%f", i, m_shm.q_dot[i]);
  }
  memcpy(&this->m_shm, &shm, sizeof(shmType));
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, m_shm.q_dot[i]);
  }
}

vpRobotBiclops::vpRobotBiclopsController::shmType vpRobotBiclops::vpRobotBiclopsController::readShm()
{
  shmType tmp_shm;

  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, m_shm.q_dot[i]);
  }
  memcpy(&tmp_shm, &this->m_shm, sizeof(shmType));
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "tmp_shm.q_dot[%d]=%f", i, tmp_shm.q_dot[i]);
  }

  return tmp_shm;
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning:
// libvisp_robot.a(vpRobotBiclopsController.cpp.o) has no symbols
void dummy_vpRobotBiclopsController() { };
#endif

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
