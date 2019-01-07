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
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

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
#include <visp3/robot/vpRobotBiclopsController.h>
#include <visp3/robot/vpRobotException.h>

//#define VP_DEBUG        // Activate the debug mode
//#define VP_DEBUG_MODE 20 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!
   Default constructor.
*/
vpRobotBiclopsController::vpRobotBiclopsController()
  : biclops(), axisMask(0), panAxis(NULL), tiltAxis(NULL), vergeAxis(NULL), panProfile(), tiltProfile(), vergeProfile(),
    shm(), stopControllerThread_(false)
{
  axisMask = Biclops::PanMask + Biclops::TiltMask
      /*+ Biclops::VergeMask*/; // add this if you want verge.

  // Set Debug level depending on how much info you want to see about
  // the inner workings of the API. Level 2 is highest with 0 being
  // the default (i.e., no messages).
  biclops.SetDebugLevel(0);

  // initialize the shared data structure
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    shm.status[i] = STOP;
    shm.q_dot[i] = 0.;
    shm.actual_q[i] = 0.;
    shm.jointLimit[i] = false;
    shm.status[i] = STOP;
  }
}

/*!

  Destructor.

*/
vpRobotBiclopsController::~vpRobotBiclopsController() {}

/*!

  Initialize the biclops by homing all axis.

  \param configfile : Biclops configuration file.

  \exception vpRobotException::notInitializedError If the biclops head connot
  be initialized. The initialization can failed,
  - if the head is not powered on,
  - if the head is not connected to your computer throw a serial cable,
  - if you try to open a bad serial port. Check you config file to verify
    which is the used serial port.
*/
void vpRobotBiclopsController::init(const std::string &configfile)
{
  vpDEBUG_TRACE(12, "Initialize biclops.");
  bool binit = false;
  for (int i = 0; i < 1; i++) {
    try {
      std::cout << "Try to initialize biclops head " << std::endl;
      binit = biclops.Initialize(configfile.c_str());
      usleep(100000);
      if (binit) {
        // Initialization completed successfully. Close the config file.
        std::cout << "Initialization succeed...\n";
        break;
      } else {
        std::cout << "Initialization failed...\n";
      }
    } catch (...) {
      std::cout << "Initialization failed..." << std::endl;
    }
  }

  if (!binit) {
    std::cout << "Cannot initialize biclops head. " << std::endl;
    std::cout << "Check if the serial cable is connected." << std::endl;
    std::cout << "Check if the robot is powered on." << std::endl;
    std::cout << "Check if you try to open the good serial port." << std::endl;
    std::cout << "Try to power off/on and restart..." << std::endl;

    throw vpRobotException(vpRobotException::notInitializedError, "Cannot initialize biclops head.");
  }

  vpDEBUG_TRACE(12, "Biclops initialized");

  // Get shortcut references to each axis.
  panAxis = biclops.GetAxis(Biclops::Pan);
  tiltAxis = biclops.GetAxis(Biclops::Tilt);
  if ((axisMask & Biclops::VergeMask) != 0)
    vergeAxis = biclops.GetAxis(Biclops::Verge);

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  if (!panAxis->GetHomedState() || !tiltAxis->GetHomedState()) {
    vpDEBUG_TRACE(12, "Biclops is not homed");
  }
#else // old API
  if (!panAxis->IsAlreadyHomed() || !tiltAxis->IsAlreadyHomed()) {
    vpDEBUG_TRACE(12, "Biclops is not homed");
  }
#endif

  // Execute the homing sequence for all axes.
  vpDEBUG_TRACE(12, "Execute the homing sequence for all axes");
  vpDEBUG_TRACE(12, "Execute the homing sequence for all axes");
  if (biclops.HomeAxes(axisMask))
    vpDEBUG_TRACE(12, "Homing sequence succeed.");
  else {
    vpERROR_TRACE("Homing sequence failed. Program is stopped");
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with biclops");
  }

  // Get the currently defined (default) motion profiles.
  //      PMDAxisControl::Profile panProfile,tiltProfile,vergeProfile;
  panAxis->GetProfile(panProfile);
  tiltAxis->GetProfile(tiltProfile);
  if ((axisMask & Biclops::VergeMask) != 0)
    vergeAxis->GetProfile(vergeProfile);
}

/*!

  Set the biclops axis position. The motion of the axis is synchronized to end
  on the same time.

  \warning Wait the end of the positionning.

  \param q : The position to set for each axis.

  \param percentVelocity : The velocity displacement to reach the new position
  in the range [0: 100.0]. 100 % corresponds to the maximal admissible
  speed. The maximal admissible speed is given by vpBiclops::speedLimit.

*/

void vpRobotBiclopsController::setPosition(const vpColVector &q, const double percentVelocity)
{
  if (q.getRows() != vpBiclops::ndof) {
    vpERROR_TRACE("Bad dimension for positioning vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for positioning vector.");
  }

  panAxis->SetProfileMode(PMDTrapezoidalProfile);
  tiltAxis->SetProfileMode(PMDTrapezoidalProfile);

  // Create the list of axes we want to coordinate
  PMDUtils::AxisList axisList;
  axisList.push_back(panAxis);
  axisList.push_back(tiltAxis);

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  // Get the currently defined (default) motion profiles.
  // PMDAxisControl::Profile panProfile,tiltProfile;
  panAxis->GetProfile(panProfile);
  tiltAxis->GetProfile(tiltProfile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.pos = PMDUtils::RadsToRevs(q[0]);
  panProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  tiltProfile.pos = PMDUtils::RadsToRevs(q[1]);
  tiltProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  // Inform the controller of the new desired position.
  panAxis->SetProfile(panProfile);
  tiltAxis->SetProfile(tiltProfile);

#else // old API

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.pos = PMDUtils::RadsToRevs(q[0]);
  panProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  vpDEBUG_TRACE(12, "Speed percent: %lf", vpBiclops::speedLimit * percentVelocity / 100.);

  panAxis->ProfileToCounts(panProfile, desired_profile);
  vpCDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << std::endl;
  vpCDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << std::endl;

  panAxis->SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  tiltProfile.pos = PMDUtils::RadsToRevs(q[1]);
  tiltProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit * percentVelocity / 100.);

  tiltAxis->ProfileToCounts(tiltProfile, desired_profile);
  vpCDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << std::endl;
  vpCDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << std::endl;

  tiltAxis->SetProfile(desired_profile);
#endif

  // Coordinate motion
  PMDUtils::Coordinate(axisList);
  biclops.Move(Biclops::PanMask + Biclops::TiltMask /*, 0*/); //
}

/*!

  Apply a velocity to each axis of the biclops robot.

  \warning This method is non blocking.

  \param q_dot : Velocity to apply.

*/
void vpRobotBiclopsController::setVelocity(const vpColVector &q_dot)
{
  if (q_dot.getRows() != vpBiclops::ndof) {
    vpERROR_TRACE("Bad dimension for velocity vector.");
    throw vpRobotException(vpRobotException::lowLevelError, "Bad dimension for velocity vector.");
  }

#ifdef VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION // new API
  // Get the currently defined (default) motion profiles.
  // PMDAxisControl::Profile panProfile, tiltProfile;
  panAxis->GetProfile(panProfile);
  tiltAxis->GetProfile(tiltProfile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.vel = PMDUtils::RadsToRevs(q_dot[0]);
  tiltProfile.vel = PMDUtils::RadsToRevs(q_dot[1]);

  // Inform the controller of the new desired position.
  panAxis->SetProfile(panProfile);
  tiltAxis->SetProfile(tiltProfile);

  panAxis->SetProfileMode(PMDVelocityContouringProfile);
  tiltAxis->SetProfileMode(PMDVelocityContouringProfile);
#else // old API
  panAxis->SetProfileMode(PMDVelocityContouringProfile);
  tiltAxis->SetProfileMode(PMDVelocityContouringProfile);

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.vel = PMDUtils::RadsToRevs(q_dot[0]);

  panAxis->ProfileToCounts(panProfile, desired_profile);
  panAxis->SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  tiltProfile.vel = PMDUtils::RadsToRevs(q_dot[1]);

  tiltAxis->ProfileToCounts(tiltProfile, desired_profile);
  tiltAxis->SetProfile(desired_profile);
#endif
  // Coordinate motion
  biclops.Move(Biclops::PanMask + Biclops::TiltMask, 0); //
}

/*!

  Get the biclops articular position.

  \return The axis articular position in radians.

*/
vpColVector vpRobotBiclopsController::getPosition()
{
  vpDEBUG_TRACE(12, "Start vpRobotBiclopsController::getPosition() ");
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  panAxis->GetPosition(panpos);
  tiltAxis->GetPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(panAxis->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tiltpos));

  vpCDEBUG(11) << "++++++++ Mesure : " << q.t();
  vpDEBUG_TRACE(12, "End vpRobotBiclopsController::getPosition()");

  return q;
}

/*!

  Get the biclops actual articular position.

  \return The axis actual articular position in radians.

*/
vpColVector vpRobotBiclopsController::getActualPosition()
{
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  panAxis->GetActualPosition(panpos);
  tiltAxis->GetActualPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(panAxis->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tiltpos));

  return q;
}

/*!

  Get the biclops articular velocity.

  \return The axis articular velocity in rad/s.

*/
vpColVector vpRobotBiclopsController::getVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  panAxis->GetVelocity(pan_vel);
  tiltAxis->GetVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(panAxis->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

/*!

  Get the biclops actual articular velocity.

  \return The axis actual articular velocity in rad/s.

*/
vpColVector vpRobotBiclopsController::getActualVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  panAxis->GetActualVelocity(pan_vel);
  tiltAxis->GetActualVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(panAxis->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

/*!

  Update the shared memory.

  \param shm_ : Content to write in the shared memory.
*/
void vpRobotBiclopsController::writeShm(shmType &shm_)
{
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "q_dot[%d]=%f", i, shm_.q_dot[i]);
  }
  memcpy(&this->shm, &shm_, sizeof(shmType));
  // this->shm = shm_;
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, shm.q_dot[i]);
  }
}

/*!

  Get a copy of the shared memory.

  \return A copy of the shared memory.
*/
vpRobotBiclopsController::shmType vpRobotBiclopsController::readShm()
{
  shmType tmp_shm;

  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, shm.q_dot[i]);
  }
  memcpy(&tmp_shm, &this->shm, sizeof(shmType));
  // tmp_shm = shm;
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    vpDEBUG_TRACE(13, "tmp_shm.q_dot[%d]=%f", i, tmp_shm.q_dot[i]);
  }

  return tmp_shm;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_robot.a(vpRobotBiclopsController.cpp.o) has no symbols
void dummy_vpRobotBiclopsController(){};
#endif

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
