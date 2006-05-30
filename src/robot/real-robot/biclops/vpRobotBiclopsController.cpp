/****************************************************************************
 *
 * $Id: vpRobotBiclopsController.cpp,v 1.3 2006-05-30 08:40:45 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Interface for the Biclops robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <signal.h>
#ifdef UNIX
#  include <unistd.h>
#endif
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

/* Headers */
#include <visp/vpRobotBiclops.h>
#include <visp/vpRobotBiclopsController.h>
#include <visp/vpRobotException.h>

#include <visp/vpDebug.h>



/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!
   Default constructor.
*/
vpRobotBiclopsController::vpRobotBiclopsController()
{
  axisMask = Biclops::PanMask
    + Biclops::TiltMask
    /*+ Biclops::VergeMask*/; // add this if you want verge.

  panAxis = NULL;
  tiltAxis = NULL;
  vergeAxis = NULL;

  // Set Debug level depending on how much info you want to see about
  // the inner workings of the API. Level 2 is highest with 0 being
  // the default (i.e., no messages).
  biclops.SetDebugLevel(0);

  // initialize the shared data structure
  for (int i=0; i < vpBiclops::ndof; i ++) {
    shm.status[i] = STOP;
    shm.q_dot[i] = 0.;
    shm.actual_q[i] = 0.;
    shm.jointLimit[i] = false;
  }

}

/*!

  Destructor.

*/
vpRobotBiclopsController::~vpRobotBiclopsController()
{
}

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
void
vpRobotBiclopsController::init(const char *configfile)
{
  DEBUG_TRACE (12, "Initialize biclops.");
  if (biclops.Initialize(configfile)) {
    DEBUG_TRACE(12, "Biclops initialized");

    // Get shortcut references to each axis.
    panAxis = biclops.GetAxis(Biclops::Pan);
    tiltAxis = biclops.GetAxis(Biclops::Tilt);
    if ((axisMask & Biclops::VergeMask) != 0)
      vergeAxis = biclops.GetAxis(Biclops::Verge);


    if (!panAxis -> IsAlreadyHomed() || !tiltAxis -> IsAlreadyHomed()) {
       DEBUG_TRACE(12, "Biclops is not homed");
    }

    //Execute the homing sequence for all axes.
    DEBUG_TRACE(12, "Execute the homing sequence for all axes");
    if ( biclops.HomeAxes(axisMask))
      DEBUG_TRACE(12, "Homing sequence succeed.");
    else {
      ERROR_TRACE("Homing sequence failed. Program is stopped");
      throw vpRobotException (vpRobotException::constructionError,
			      "Cannot open connexion with biclops");
    }

    // Get the currently defined (default) motion profiles.
    //      PMDAxisControl::Profile panProfile,tiltProfile,vergeProfile;
    panAxis->GetProfile(panProfile);
    tiltAxis->GetProfile(tiltProfile);
    if ((axisMask & Biclops::VergeMask) != 0)
      vergeAxis->GetProfile(vergeProfile);
  }
  else {
    ERROR_TRACE ("Cannot initialize biclops head.");
    ERROR_TRACE ("Check if the robot is powered on.");
    ERROR_TRACE ("Check if the serial cable is connected.");
    ERROR_TRACE ("Check if you try to open the good serial port.");
    throw vpRobotException (vpRobotException::notInitializedError,
			    "Cannot initialize biclops head.");
  }

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

void
vpRobotBiclopsController::setPosition(const vpColVector & q,
				      const double percentVelocity )
{
  if (q.getRows() != vpBiclops::ndof )
  {
    ERROR_TRACE ("Bad dimension for positioning vector.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Bad dimension for positioning vector.");
  }

  panAxis  -> SetProfileMode(PMDTrapezoidalProfile);
  tiltAxis -> SetProfileMode(PMDTrapezoidalProfile);

  // Create the list of axes we want to coordinate
  PMDUtils::AxisList axisList;
  axisList.push_back(panAxis);
  axisList.push_back(tiltAxis);

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.pos = PMDUtils::RadsToRevs(q[0]);
  panProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit
					* percentVelocity / 100.);

  DEBUG_TRACE(12, "Speed percent: %lf",
	      vpBiclops::speedLimit * percentVelocity / 100.);

  panAxis -> ProfileToCounts(panProfile, desired_profile);
  CDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << endl;
  CDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << endl;

  panAxis -> SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  tiltProfile.pos = PMDUtils::RadsToRevs(q[1]);
  tiltProfile.vel = PMDUtils::RadsToRevs(vpBiclops::speedLimit
					* percentVelocity / 100.);

  tiltAxis -> ProfileToCounts(tiltProfile, desired_profile);
  CDEBUG(12) << "desired_profile.pos: " << desired_profile.pos << endl;
  CDEBUG(12) << "desired_profile.vel: " << desired_profile.vel << endl;

  tiltAxis -> SetProfile(desired_profile);

  // Coordinate motion
  PMDUtils::Coordinate(axisList);
  biclops.Move(Biclops::PanMask + Biclops::TiltMask/*, 0*/); //

}

/*!

  Apply a velocity to each axis of the biclops robot.

  \warning This method is non blocking.

  \param q_dot : Velocity to apply.

*/
void
vpRobotBiclopsController::setVelocity(const vpColVector & q_dot)
{
  if (q_dot.getRows() != vpBiclops::ndof )
  {
    ERROR_TRACE ("Bad dimension for velocity vector.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Bad dimension for velocity vector.");
  }

  panAxis  -> SetProfileMode(PMDVelocityContouringProfile);
  tiltAxis -> SetProfileMode(PMDVelocityContouringProfile);

  PMDAxisControl::CountsProfile desired_profile;

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  panProfile.vel = PMDUtils::RadsToRevs(q_dot[0]);

  panAxis -> ProfileToCounts(panProfile, desired_profile);
  panAxis -> SetProfile(desired_profile);

  // Set a position to move to by modifying the respective profiles.
  // NOTE: profile values are in revolutions, so here we convert
  // from degrees (divide by 360) for readability.
  tiltProfile.vel = PMDUtils::RadsToRevs(q_dot[1]);

  tiltAxis -> ProfileToCounts(tiltProfile, desired_profile);
  tiltAxis -> SetProfile(desired_profile);

  // Coordinate motion
  biclops.Move(Biclops::PanMask + Biclops::TiltMask, 0); //
}

/*!

  Get the biclops articular position.

  \return The axis articular position in radians.

*/
vpColVector
vpRobotBiclopsController::getPosition()
{
  DEBUG_TRACE (12, "Start vpRobotBiclopsController::getPosition() ");
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  panAxis  -> GetPosition(panpos);
  tiltAxis -> GetPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(panAxis ->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tiltpos));

  CDEBUG(11) << "++++++++ Mesure : " << q.t();
  DEBUG_TRACE (12, "End vpRobotBiclopsController::getPosition()");

  return q;
}

/*!

  Get the biclops actual articular position.

  \return The axis actual articular position in radians.

*/
vpColVector
vpRobotBiclopsController::getActualPosition()
{
  vpColVector q(vpBiclops::ndof);
  PMDint32 panpos, tiltpos;

  panAxis  -> GetActualPosition(panpos);
  tiltAxis -> GetActualPosition(tiltpos);

  q[0] = PMDUtils::RevsToRads(panAxis ->CountsToUnits(panpos));
  q[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tiltpos));

  return q;
}

/*!

  Get the biclops articular velocity.

  \return The axis articular velocity in rad/s.

*/
vpColVector
vpRobotBiclopsController::getVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  panAxis  -> GetVelocity(pan_vel);
  tiltAxis -> GetVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(panAxis ->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

/*!

  Get the biclops actual articular velocity.

  \return The axis actual articular velocity in rad/s.

*/
vpColVector
vpRobotBiclopsController::getActualVelocity()
{
  vpColVector q_dot(vpBiclops::ndof);
  PMDint32 pan_vel, tilt_vel;

  panAxis  -> GetActualVelocity(pan_vel);
  tiltAxis -> GetActualVelocity(tilt_vel);

  q_dot[0] = PMDUtils::RevsToRads(panAxis ->CountsToUnits(pan_vel));
  q_dot[1] = PMDUtils::RevsToRads(tiltAxis->CountsToUnits(tilt_vel));

  return q_dot;
}

/*!

  Update the shared memory.

  \param shm_ : Content to write in the shared memory.
*/
void
vpRobotBiclopsController::writeShm(shmType &shm_)
{
  for(int i=0; i < vpBiclops::ndof; i ++) {
    DEBUG_TRACE(13, "q_dot[%d]=%f", i, shm_.q_dot[i]);
  }
  memcpy(&this->shm, &shm_, sizeof(shmType));
   //this->shm = shm_;
  for(int i=0; i < vpBiclops::ndof; i ++) {
    DEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, shm.q_dot[i]);
  }
}

/*!

  Get a copy of the shared memory.

  \return A copy of the shared memory.
*/
vpRobotBiclopsController::shmType
vpRobotBiclopsController::readShm()
{
  shmType tmp_shm;

  for(int i=0; i < vpBiclops::ndof; i ++) {
    DEBUG_TRACE(13, "shm.q_dot[%d]=%f", i, shm.q_dot[i]);
  }
  memcpy(&tmp_shm, &this->shm, sizeof(shmType));
  //tmp_shm = shm;
  for(int i=0; i < vpBiclops::ndof; i ++) {
    DEBUG_TRACE(13, "tmp_shm.q_dot[%d]=%f", i, tmp_shm.q_dot[i]);
  }

  return tmp_shm;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

