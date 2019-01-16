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

#include <cmath> // std::fabs
#include <errno.h>
#include <limits> // numeric_limits
#include <signal.h>
#include <string.h>

#include <visp3/core/vpTime.h>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_BICLOPS

#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobotBiclops.h>
#include <visp3/robot/vpRobotException.h>

//#define VP_DEBUG        // Activate the debug mode
//#define VP_DEBUG_MODE 10 // Activate debug level 1 and 2
#include <visp3/core/vpDebug.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotBiclops::robotAlreadyCreated = false;
const double vpRobotBiclops::defaultPositioningVelocity = 10.0;

static pthread_mutex_t vpEndThread_mutex;
static pthread_mutex_t vpShm_mutex;
static pthread_mutex_t vpMeasure_mutex;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  Default constructor.

  Does nothing more than setting the default configuration file
  to /usr/share/BiclopsDefault.cfg.

  As shown in the following example,the turret need to be initialized
  using init() function.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotBiclops.h>

int main()
{
#ifdef VISP_HAVE_BICLOPS
  vpRobotBiclops robot; // Use the default config file in
/usr/share/BiclopsDefault.cfg"

  // Specify the config file location
  robot.setConfigFile("/usr/share/BiclopsDefault.cfg"); // Not mandatory since the file is the default one

  // Initialize the head
  robot.init();

  // Move the robot to a specified pan and tilt
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
  vpColVector q(2);
  q[0] = vpMath::rad(20); // pan
  q[1] = vpMath::rad(40); // tilt
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
#endif
  return 0;
}
  \endcode

*/
vpRobotBiclops::vpRobotBiclops()
  : vpBiclops(), vpRobot(), control_thread(), controller(), positioningVelocity(defaultPositioningVelocity),
    q_previous(), controlThreadCreated(false)
{
  vpDEBUG_TRACE(12, "Begin default constructor.");

  vpRobotBiclops::robotAlreadyCreated = false;
  setConfigFile("/usr/share/BiclopsDefault.cfg");

  // Initialize the mutex dedicated to she shm protection
  pthread_mutex_init(&vpShm_mutex, NULL);
  pthread_mutex_init(&vpEndThread_mutex, NULL);
  pthread_mutex_init(&vpMeasure_mutex, NULL);

  control_thread = 0;
}

/*!

  Default constructor.

  Initialize the biclops pan, tilt head by reading the
  configuration file provided by Traclabs
  and do the homing sequence.

  The following example shows how to use the constructor.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotBiclops.h>

int main()
{
#ifdef VISP_HAVE_BICLOPS
  // Specify the config file location and initialize the turret
  vpRobotBiclops robot("/usr/share/BiclopsDefault.cfg");

  // Move the robot to a specified pan and tilt
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL) ;

  vpColVector q(2);
  q[0] = vpMath::rad(-20); // pan
  q[1] = vpMath::rad(10); // tilt
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
#endif
  return 0;
}
  \endcode

*/
vpRobotBiclops::vpRobotBiclops(const std::string &filename)
  : vpBiclops(), vpRobot(), control_thread(), controller(), positioningVelocity(defaultPositioningVelocity),
    q_previous(), controlThreadCreated(false)
{
  vpDEBUG_TRACE(12, "Begin default constructor.");

  vpRobotBiclops::robotAlreadyCreated = false;
  setConfigFile(filename);

  // Initialize the mutex dedicated to she shm protection
  pthread_mutex_init(&vpShm_mutex, NULL);
  pthread_mutex_init(&vpEndThread_mutex, NULL);
  pthread_mutex_init(&vpMeasure_mutex, NULL);

  init();

  return;
}

/*!

  Destructor.
  Wait the end of the control thread.

*/

vpRobotBiclops::~vpRobotBiclops(void)
{

  vpDEBUG_TRACE(12, "Start vpRobotBiclops::~vpRobotBiclops()");
  setRobotState(vpRobot::STATE_STOP);

  vpDEBUG_TRACE(12, "Unlock mutex vpEndThread_mutex");
  pthread_mutex_unlock(&vpEndThread_mutex);

  /* wait the end of the control thread */
  vpDEBUG_TRACE(12, "Wait end of control thread");

  if (controlThreadCreated == true) {
    int code = pthread_join(control_thread, NULL);
    if (code != 0) {
      vpCERROR << "Cannot terminate the control thread: " << code << " strErr=" << strerror(errno)
               << " strCode=" << strerror(code) << std::endl;
    }
  }

  pthread_mutex_destroy(&vpShm_mutex);
  pthread_mutex_destroy(&vpEndThread_mutex);
  pthread_mutex_destroy(&vpMeasure_mutex);

  vpRobotBiclops::robotAlreadyCreated = false;

  vpDEBUG_TRACE(12, "Stop vpRobotBiclops::~vpRobotBiclops()");
  return;
}

/* -------------------------------------------------------------------------
 */
/* --- INITIALISATION ------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/*!

  Set the Biclops config filename.

*/
void vpRobotBiclops::setConfigFile(const std::string &filename) { this->configfile = filename; }

/*!

  Set the Biclops config filename.
  Check if the config file exists and initialize the head.

  \exception vpRobotException::constructionError If the config file cannot be
  oppened.

*/
void vpRobotBiclops::init()
{
  // test if the config file exists
  FILE *fd = fopen(configfile.c_str(), "r");
  if (fd == NULL) {
    vpCERROR << "Cannot open biclops config file: " << configfile << std::endl;
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with biclops");
  }
  fclose(fd);

  // Initialize the controller
  controller.init(configfile);

  try {
    setRobotState(vpRobot::STATE_STOP);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }

  vpRobotBiclops::robotAlreadyCreated = true;

  // Initialize previous articular position to manage getDisplacement()
  q_previous.resize(vpBiclops::ndof);
  q_previous = 0;

  controlThreadCreated = false;

  return;
}

/*
  Control loop to manage the biclops joint limits in speed control.

  This control loop is running in a seperate thread in order to detect each 5
  ms joint limits during the speed control. If a joint limit is detected the
  axis should be halted.

  \warning Velocity control mode is not exported from the top-level Biclops
  API class provided by Traclabs. That means that there is no protection in
  this mode to prevent an axis from striking its hard limit. In position mode,
  Traclabs put soft limits in that keep any command from driving to a position
  too close to the hard limits. In velocity mode this protection does not
  exist in the current API.

  \warning With the understanding that hitting the hard limits at full
  speed/power can damage the unit, damage due to velocity mode commanding is
  under user responsibility.
*/
void *vpRobotBiclops::vpRobotBiclopsSpeedControlLoop(void *arg)
{
  vpRobotBiclopsController *controller = static_cast<vpRobotBiclopsController *>(arg);

  int iter = 0;
  //   PMDAxisControl *panAxis  = controller->getPanAxis();
  //   PMDAxisControl *tiltAxis = controller->getTiltAxis();
  vpRobotBiclopsController::shmType shm;

  vpDEBUG_TRACE(10, "Start control loop");
  vpColVector mes_q;
  vpColVector mes_q_dot;
  vpColVector softLimit(vpBiclops::ndof);
  vpColVector q_dot(vpBiclops::ndof);
  bool *new_q_dot = new bool[vpBiclops::ndof];
  bool *change_dir = new bool[vpBiclops::ndof];   // change of direction
  bool *force_halt = new bool[vpBiclops::ndof];   // force an axis to halt
  bool *enable_limit = new bool[vpBiclops::ndof]; // enable soft limit
  vpColVector prev_q_dot(vpBiclops::ndof);        // previous desired speed
  double secure = vpMath::rad(2);                 // add a security angle before joint limit

  // Set the soft limits
  softLimit[0] = vpBiclops::panJointLimit - secure;
  softLimit[1] = vpBiclops::tiltJointLimit - secure;
  vpDEBUG_TRACE(12, "soft limit pan: %f tilt: %f", vpMath::deg(softLimit[0]), vpMath::deg(softLimit[1]));

  // Initilisation
  vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
  pthread_mutex_lock(&vpShm_mutex);

  shm = controller->readShm();

  vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
  pthread_mutex_unlock(&vpShm_mutex);

  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    prev_q_dot[i] = shm.q_dot[i];
    new_q_dot[i] = false;
    change_dir[i] = false;
    force_halt[i] = false;
    enable_limit[i] = true;
  }

  // Initialize actual position and velocity
  mes_q = controller->getActualPosition();
  mes_q_dot = controller->getActualVelocity();

  vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
  pthread_mutex_lock(&vpShm_mutex);

  shm = controller->readShm();
  // Updates the shm
  for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
    shm.actual_q[i] = mes_q[i];
    shm.actual_q_dot[i] = mes_q_dot[i];
  }
  // Update the actuals positions
  controller->writeShm(shm);

  vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
  pthread_mutex_unlock(&vpShm_mutex);

  vpDEBUG_TRACE(11, "unlock mutex vpMeasure_mutex");
  pthread_mutex_unlock(&vpMeasure_mutex); // A position is available

  while (!controller->isStopRequested()) {

    // Get actual position and velocity
    mes_q = controller->getActualPosition();
    mes_q_dot = controller->getActualVelocity();

    vpDEBUG_TRACE(11, "Lock mutex vpShm_mutex");
    pthread_mutex_lock(&vpShm_mutex);

    shm = controller->readShm();

    // Updates the shm
    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      shm.actual_q[i] = mes_q[i];
      shm.actual_q_dot[i] = mes_q_dot[i];
    }

    vpDEBUG_TRACE(12, "mes pan: %f tilt: %f", vpMath::deg(mes_q[0]), vpMath::deg(mes_q[1]));
    vpDEBUG_TRACE(13, "mes pan vel: %f tilt vel: %f", vpMath::deg(mes_q_dot[0]), vpMath::deg(mes_q_dot[1]));
    vpDEBUG_TRACE(12, "desired  q_dot : %f %f", vpMath::deg(shm.q_dot[0]), vpMath::deg(shm.q_dot[1]));
    vpDEBUG_TRACE(13, "previous q_dot : %f %f", vpMath::deg(prev_q_dot[0]), vpMath::deg(prev_q_dot[1]));

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      // test if joint limits are reached
      if (mes_q[i] < -softLimit[i]) {
        vpDEBUG_TRACE(12, "Axe %d in low joint limit", i);
        shm.status[i] = vpRobotBiclopsController::STOP;
        shm.jointLimit[i] = true;
      } else if (mes_q[i] > softLimit[i]) {
        vpDEBUG_TRACE(12, "Axe %d in hight joint limit", i);
        shm.status[i] = vpRobotBiclopsController::STOP;
        shm.jointLimit[i] = true;
      } else {
        shm.status[i] = vpRobotBiclopsController::SPEED;
        shm.jointLimit[i] = false;
      }

      // Test if new a speed is demanded
      // if (shm.q_dot[i] != prev_q_dot[i])
      if (std::fabs(shm.q_dot[i] - prev_q_dot[i]) >
          std::fabs(vpMath::maximum(shm.q_dot[i], prev_q_dot[i])) * std::numeric_limits<double>::epsilon())
        new_q_dot[i] = true;
      else
        new_q_dot[i] = false;

      // Test if desired speed change of sign
      if ((shm.q_dot[i] * prev_q_dot[i]) < 0.)
        change_dir[i] = true;
      else
        change_dir[i] = false;
    }
    vpDEBUG_TRACE(13, "status      : %d %d", shm.status[0], shm.status[1]);
    vpDEBUG_TRACE(13, "joint       : %d %d", shm.jointLimit[0], shm.jointLimit[1]);
    vpDEBUG_TRACE(13, "new q_dot   : %d %d", new_q_dot[0], new_q_dot[1]);
    vpDEBUG_TRACE(13, "new dir     : %d %d", change_dir[0], change_dir[1]);
    vpDEBUG_TRACE(13, "force halt  : %d %d", force_halt[0], force_halt[1]);
    vpDEBUG_TRACE(13, "enable limit: %d %d", enable_limit[0], enable_limit[1]);

    bool updateVelocity = false;
    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      // Test if a new desired speed is to apply
      if (new_q_dot[i]) {
        // A new desired speed is to apply
        if (shm.status[i] == vpRobotBiclopsController::STOP) {
          // Axis in joint limit
          if (change_dir[i] == false) {
            // New desired speed without change of direction
            // We go in the joint limit
            if (enable_limit[i] == true) { // limit detection active
              // We have to stop this axis
              // Test if this axis was stopped before
              if (force_halt[i] == false) {
                q_dot[i] = 0.;
                force_halt[i] = true;  // indicate that it will be stopped
                updateVelocity = true; // We have to send this new speed
              }
            } else {
              // We have to apply the desired speed to go away the joint
              // Update the desired speed
              q_dot[i] = shm.q_dot[i];
              shm.status[i] = vpRobotBiclopsController::SPEED;
              force_halt[i] = false;
              updateVelocity = true; // We have to send this new speed
            }
          } else {
            // New desired speed and change of direction.
            if (enable_limit[i] == true) { // limit detection active
              // Update the desired speed to go away the joint limit
              q_dot[i] = shm.q_dot[i];
              shm.status[i] = vpRobotBiclopsController::SPEED;
              force_halt[i] = false;
              enable_limit[i] = false; // Disable joint limit detection
              updateVelocity = true;   // We have to send this new speed
            } else {
              // We have to stop this axis
              // Test if this axis was stopped before
              if (force_halt[i] == false) {
                q_dot[i] = 0.;
                force_halt[i] = true;   // indicate that it will be stopped
                enable_limit[i] = true; // Joint limit detection must be active
                updateVelocity = true;  // We have to send this new speed
              }
            }
          }
        } else {
          // Axis not in joint limit

          // Update the desired speed
          q_dot[i] = shm.q_dot[i];
          shm.status[i] = vpRobotBiclopsController::SPEED;
          enable_limit[i] = true; // Joint limit detection must be active
          updateVelocity = true;  // We have to send this new speed
        }
      } else {
        // No change of the desired speed. We have to stop the robot in case
        // of joint limit
        if (shm.status[i] == vpRobotBiclopsController::STOP) { // axis limit
          if (enable_limit[i] == true) {                       // limit detection active

            // Test if this axis was stopped before
            if (force_halt[i] == false) {
              // We have to stop this axis
              q_dot[i] = 0.;
              force_halt[i] = true;  // indicate that it will be stopped
              updateVelocity = true; // We have to send this new speed
            }
          }
        } else {
          // No need to stop the robot
          enable_limit[i] = true; // Normal situation, activate limit detection
        }
      }
    }
    // Update the actuals positions
    controller->writeShm(shm);

    vpDEBUG_TRACE(11, "unlock mutex vpShm_mutex");
    pthread_mutex_unlock(&vpShm_mutex);

    if (updateVelocity) {
      vpDEBUG_TRACE(12, "apply q_dot : %f %f", vpMath::deg(q_dot[0]), vpMath::deg(q_dot[1]));

      // Apply the velocity
      controller->setVelocity(q_dot);
    }

    // Update the previous speed for next iteration
    for (unsigned int i = 0; i < vpBiclops::ndof; i++)
      prev_q_dot[i] = shm.q_dot[i];

    vpDEBUG_TRACE(12, "iter: %d", iter);

    // wait 5 ms
    vpTime::wait(5.0);

    //    if (pthread_mutex_trylock(&vpEndThread_mutex) == 0) {
    //      vpDEBUG_TRACE (12, "Calling thread will end");
    //      vpDEBUG_TRACE (12, "Unlock mutex vpEndThread_mutex");
    //      std::cout << "Calling thread will end" << std::endl;
    //      std::cout << "Unlock mutex vpEndThread_mutex" << std::endl;
    //
    //      pthread_mutex_unlock(&vpEndThread_mutex);
    //      break;
    //    }

    iter++;
  }
  controller->stopRequest(false);
  // Stop the robot
  vpDEBUG_TRACE(10, "End of the control thread: stop the robot");
  q_dot = 0;
  controller->setVelocity(q_dot);

  delete[] new_q_dot;
  delete[] change_dir;
  delete[] force_halt;
  delete[] enable_limit;
  vpDEBUG_TRACE(11, "unlock vpEndThread_mutex");
  pthread_mutex_unlock(&vpEndThread_mutex);

  vpDEBUG_TRACE(10, "Exit control thread ");
  //  pthread_exit(0);

  return NULL;
}

/*!

  Change the state of the robot either to stop them, or to set position or
  speed control.

*/
vpRobot::vpRobotStateType vpRobotBiclops::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    if (vpRobot::STATE_STOP != getRobotState()) {
      stopMotion();
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      vpDEBUG_TRACE(12, "Speed to position control.");
      stopMotion();
    }

    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {

    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      vpDEBUG_TRACE(12, "Lock mutex vpEndThread_mutex");
      pthread_mutex_lock(&vpEndThread_mutex);

      vpDEBUG_TRACE(12, "Create speed control thread");
      int code;
      code = pthread_create(&control_thread, NULL, &vpRobotBiclops::vpRobotBiclopsSpeedControlLoop, &controller);
      if (code != 0) {
        vpCERROR << "Cannot create speed biclops control thread: " << code << " strErr=" << strerror(errno)
                 << " strCode=" << strerror(code) << std::endl;
      }

      controlThreadCreated = true;

      vpDEBUG_TRACE(12, "Speed control thread created");
    }
    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

/*!

  Halt all the axis.

*/
void vpRobotBiclops::stopMotion(void)
{
  vpColVector q_dot(vpBiclops::ndof);
  q_dot = 0;
  controller.setVelocity(q_dot);
  // std::cout << "Request to stop the velocity controller thread...."<<
  // std::endl;
  controller.stopRequest(true);
}

/*!

  Get the twist matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cVe : Twist transformation between camera and end effector frame to
  expess a velocity skew from end effector frame in camera frame.

*/
void vpRobotBiclops::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  cMe = vpBiclops::get_cMe();

  cVe.buildFrom(cMe);
}

/*!

  Get the homogeneous matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cMe :  Homogeneous matrix between camera and end effector frame.

*/
void vpRobotBiclops::get_cMe(vpHomogeneousMatrix &cMe) const { cMe = vpBiclops::get_cMe(); }

/*!
  Get the robot jacobian expressed in the end-effector frame.

  \warning Re is not the embedded camera frame. It corresponds to the frame
  associated to the tilt axis (see also get_cMe).

  \param _eJe : Jacobian between end effector frame and end effector frame (on
  tilt axis).

*/
void vpRobotBiclops::get_eJe(vpMatrix &_eJe)
{
  vpColVector q(2);
  getPosition(vpRobot::ARTICULAR_FRAME, q);

  try {
    vpBiclops::get_eJe(q, _eJe);
  } catch (...) {
    vpERROR_TRACE("catch exception ");
    throw;
  }
}

/*!
  Get the robot jacobian expressed in the robot reference frame

  \param _fJe : Jacobian between reference frame (or fix frame) and end
  effector frame (on tilt axis).

*/
void vpRobotBiclops::get_fJe(vpMatrix &_fJe)
{
  vpColVector q(2);
  getPosition(vpRobot::ARTICULAR_FRAME, q);

  try {
    vpBiclops::get_fJe(q, _fJe);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Set the velocity used for a position control.

  \param velocity : Velocity in % of the maximum velocity between [0,100]. The
  maximum velocity is given vpBiclops::speedLimit.
*/
void vpRobotBiclops::setPositioningVelocity(const double velocity)
{
  if (velocity < 0 || velocity > 100) {
    vpERROR_TRACE("Bad positionning velocity");
    throw vpRobotException(vpRobotException::constructionError, "Bad positionning velocity");
  }

  positioningVelocity = velocity;
}
/*!
  Get the velocity in % used for a position control.

  \return Positionning velocity in [0, 100.0]. The
  maximum positionning velocity is given vpBiclops::speedLimit.

*/
double vpRobotBiclops::getPositioningVelocity(void) { return positioningVelocity; }

/*!
   Move the robot in position control.

   \warning This method is blocking. That mean that it waits the end of the
   positionning.

   \param frame : Control frame. This biclops head can only be controlled in
   articular.

   \param q : The position to set for each axis in radians.

   \exception vpRobotException::wrongStateError : If a not supported frame
   type is given.

*/
void vpRobotBiclops::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    vpERROR_TRACE("Robot was not in position-based control\n"
                  "Modification of the robot state");
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in camera frame: "
                                                              "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in reference frame: "
                                                              "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in mixt frame: "
                                                              "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break;
  }

  // test if position reachable
  //   if ( (fabs(q[0]) > vpBiclops::panJointLimit) ||
  //        (fabs(q[1]) > vpBiclops::tiltJointLimit) ) {
  //     vpERROR_TRACE ("Positionning error.");
  //     throw vpRobotException (vpRobotException::wrongStateError,
  // 			    "Positionning error.");
  //   }

  vpDEBUG_TRACE(12, "Lock mutex vpEndThread_mutex");
  pthread_mutex_lock(&vpEndThread_mutex);
  controller.setPosition(q, positioningVelocity);
  vpDEBUG_TRACE(12, "Unlock mutex vpEndThread_mutex");
  pthread_mutex_unlock(&vpEndThread_mutex);
  return;
}

/*!
   Move the robot in position control.

   \warning This method is blocking. That mean that it wait the end of the
   positionning.

   \param frame : Control frame. This biclops head can only be controlled in
   articular.

   \param q1 : The pan position to set in radians.
   \param q2 : The tilt position to set in radians.

   \exception vpRobotException::wrongStateError : If a not supported frame
   type is given.

*/
void vpRobotBiclops::setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2)
{
  try {
    vpColVector q(2);
    q[0] = q1;
    q[1] = q2;

    setPosition(frame, q);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Read the content of the position file and moves to head to articular
  position.

  \param filename : Position filename

  \exception vpRobotException::readingParametersError : If the articular
  position cannot be read from file.

  \sa readPositionFile()

*/
void vpRobotBiclops::setPosition(const char *filename)
{
  vpColVector q;
  if (readPositionFile(filename, q) == false) {
    vpERROR_TRACE("Cannot get biclops position from file");
    throw vpRobotException(vpRobotException::readingParametersError, "Cannot get biclops position from file");
  }
  setPosition(vpRobot::ARTICULAR_FRAME, q);
}

/*!

  Return the position of each axis.
  - In positionning control mode, call vpRobotBiclopsController::getPosition()
  - In speed control mode, call vpRobotBiclopsController::getActualPosition()

  \param frame : Control frame. This biclops head can only be controlled in
  articular.

  \param q : The position of the axis in radians.

  \exception vpRobotException::wrongStateError : If a not supported frame type
  is given.

*/
void vpRobotBiclops::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in camera frame: "
                                                              "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in reference frame: "
                                                              "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in mixt frame: "
                                                              "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break;
  }

  vpRobot::vpRobotStateType state;
  state = vpRobot::getRobotState();

  switch (state) {
  case STATE_STOP:
  case STATE_POSITION_CONTROL:
    q = controller.getPosition();

    break;
  case STATE_VELOCITY_CONTROL:
  case STATE_ACCELERATION_CONTROL:
  default:
    q.resize(vpBiclops::ndof);

    vpDEBUG_TRACE(12, "Lock mutex vpMeasure_mutex");
    pthread_mutex_lock(&vpMeasure_mutex); // Wait until a position is available

    vpRobotBiclopsController::shmType shm;

    vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
    pthread_mutex_lock(&vpShm_mutex);

    shm = controller.readShm();

    vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
    pthread_mutex_unlock(&vpShm_mutex);

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      q[i] = shm.actual_q[i];
    }

    vpCDEBUG(11) << "++++++++ Measure actuals: " << q.t();

    vpDEBUG_TRACE(12, "unlock mutex vpMeasure_mutex");
    pthread_mutex_unlock(&vpMeasure_mutex); // A position is available

    break;
  }
}

/*!

  Send a velocity on each axis.

  \param frame : Control frame. This biclops head can only be controlled in
  articular. Be aware, the camera frame (vpRobot::CAMERA_FRAME), the reference
  frame (vpRobot::REFERENCE_FRAME), end-effector frame (vpRobot::END_EFFECTOR_FRAME)
  and the mixt frame (vpRobot::MIXT_FRAME) are not implemented.

  \param q_dot : The desired articular velocity of the axis in rad/s. \f$ \dot
  {r} = [\dot{q}_1, \dot{q}_2]^t \f$ with \f$ \dot{q}_1 \f$ the pan of the
  camera and \f$ \dot{q}_2\f$ the tilt of the camera.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \exception vpRobotException::wrongStateError : If a not supported frame type
  (vpRobot::CAMERA_FRAME, vpRobot::REFERENCE_FRAME, vpRobot::END_EFFECTOR_FRAME
  or vpRobot::MIXT_FRAME) is given.

  \warning Velocities could be saturated if one of them exceed the maximal
  autorized speed (see vpRobot::maxRotationVelocity).

*/
void vpRobotBiclops::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &q_dot)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot "
                           "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "in the camera frame: "
                  "functionality not implemented");
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the camera frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::ARTICULAR_FRAME: {
    if (q_dot.getRows() != 2) {
      vpERROR_TRACE("Bad dimension fo speed vector in articular frame");
      throw vpRobotException(vpRobotException::wrongStateError, "Bad dimension for speed vector "
                                                                "in articular frame");
    }
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the reference frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the mixt frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot "
                                                              "in the end-effector frame:"
                                                              "functionality not implemented");
  }
  default: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot send a velocity to the robot ");
  }
  }

  vpDEBUG_TRACE(12, "Velocity limitation.");
  bool norm = false; // Flag to indicate when velocities need to be nomalized

  // Saturate articular speed
  double max = vpBiclops::speedLimit;
  vpColVector q_dot_sat(vpBiclops::ndof);

  // init q_dot_saturated
  q_dot_sat = q_dot;

  for (unsigned int i = 0; i < vpBiclops::ndof; ++i) // q1 and q2
  {
    if (fabs(q_dot[i]) > max) {
      norm = true;
      max = fabs(q_dot[i]);
      vpERROR_TRACE("Excess velocity: ROTATION "
                    "(axe nr.%d).",
                    i);
    }
  }
  // Rotations velocities normalisation
  if (norm == true) {
    max = vpBiclops::speedLimit / max;
    q_dot_sat = q_dot * max;
  }

  vpCDEBUG(12) << "send velocity: " << q_dot_sat.t() << std::endl;

  vpRobotBiclopsController::shmType shm;

  vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
  pthread_mutex_lock(&vpShm_mutex);

  shm = controller.readShm();

  for (unsigned int i = 0; i < vpBiclops::ndof; i++)
    shm.q_dot[i] = q_dot[i];

  controller.writeShm(shm);

  vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
  pthread_mutex_unlock(&vpShm_mutex);

  return;
}

/* -------------------------------------------------------------------------
 */
/* --- GET -----------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/*!

  Get the articular velocity.

  \param frame : Control frame. This head can only be controlled in articular.

  \param q_dot : The measured articular velocity in rad/s.

  \exception vpRobotException::wrongStateError : If a not supported frame type
  is given.
*/
void vpRobotBiclops::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot)
{
  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in camera frame: "
                                                              "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in reference frame: "
                                                              "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in mixt frame: "
                                                              "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get position in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break;
  }

  vpRobot::vpRobotStateType state;
  state = vpRobot::getRobotState();

  switch (state) {
  case STATE_STOP:
  case STATE_POSITION_CONTROL:
    q_dot = controller.getVelocity();

    break;
  case STATE_VELOCITY_CONTROL:
  case STATE_ACCELERATION_CONTROL:
  default:
    q_dot.resize(vpBiclops::ndof);

    vpDEBUG_TRACE(12, "Lock mutex vpMeasure_mutex");
    pthread_mutex_lock(&vpMeasure_mutex); // Wait until a position is available

    vpRobotBiclopsController::shmType shm;

    vpDEBUG_TRACE(12, "Lock mutex vpShm_mutex");
    pthread_mutex_lock(&vpShm_mutex);

    shm = controller.readShm();

    vpDEBUG_TRACE(12, "unlock mutex vpShm_mutex");
    pthread_mutex_unlock(&vpShm_mutex);

    for (unsigned int i = 0; i < vpBiclops::ndof; i++) {
      q_dot[i] = shm.actual_q_dot[i];
    }

    vpCDEBUG(11) << "++++++++ Velocity actuals: " << q_dot.t();

    vpDEBUG_TRACE(12, "unlock mutex vpMeasure_mutex");
    pthread_mutex_unlock(&vpMeasure_mutex); // A position is available

    break;
  }
}

/*!

  Return the articular velocity.

  \param frame : Control frame. This head can only be controlled in articular.

  \return The measured articular velocity in rad/s.

  \exception vpRobotException::wrongStateError : If a not supported frame type
  is given.
*/
vpColVector vpRobotBiclops::getVelocity(vpRobot::vpControlFrameType frame)
{
  vpColVector q_dot;
  getVelocity(frame, q_dot);

  return q_dot;
}

/*!

  Get an articular position from the position file.

  \param filename : Position file.

  \param q : The articular position read in the file.

  \code
  # Example of biclops position file
  # The axis positions must be preceed by R:
  # First value : pan  articular position in degrees
  # Second value: tilt articular position in degrees
  R: 15.0 5.0
  \endcode

  \return true if a position was found, false otherwise.

*/
bool vpRobotBiclops::readPositionFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#PTU-EVI - Position");
  bool pos_found = false;
  int lineNum = 0;

  q.resize(vpBiclops::ndof);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Biclops position file
        std::cout << "Error: this position file " << filename << " is not for Biclops robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < vpBiclops::ndof + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < vpBiclops::ndof + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < vpBiclops::ndof; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  q.deg2rad();

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Biclops robot in " << filename << std::endl;
    return false;
  }

  return true;
}

/*!

  Get the robot displacement since the last call of this method.

  \warning The first call of this method gives not a good value for the
  displacement.

  \param frame The frame in which the measured displacement is expressed.

  \param d The displacement:

  - In articular, the dimension of q is 2  (the number of axis of the robot)
  with respectively d[0] (pan displacement), d[1] (tilt displacement).

  - In camera frame, the dimension of d is 6 (tx, ty, ty, tux, tuy, tuz).
  Translations are expressed in meters, rotations in radians with the theta U
  representation.

  \exception vpRobotException::wrongStateError If a not supported frame type
  is given.

*/
void vpRobotBiclops::getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &d)
{
  vpColVector q_current; // current position

  getPosition(vpRobot::ARTICULAR_FRAME, q_current);

  switch (frame) {
  case vpRobot::ARTICULAR_FRAME:
    d.resize(vpBiclops::ndof);
    d = q_current - q_previous;
    break;

  case vpRobot::CAMERA_FRAME: {
    d.resize(6);
    vpHomogeneousMatrix fMc_current;
    vpHomogeneousMatrix fMc_previous;
    fMc_current = vpBiclops::get_fMc(q_current);
    fMc_previous = vpBiclops::get_fMc(q_previous);
    vpHomogeneousMatrix c_previousMc_current;
    // fMc_c = fMc_p * c_pMc_c
    // => c_pMc_c = (fMc_p)^-1 * fMc_c
    c_previousMc_current = fMc_previous.inverse() * fMc_current;

    // Compute the instantaneous velocity from this homogeneous matrix.
    d = vpExponentialMap::inverse(c_previousMc_current);
    break;
  }

  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the reference frame:"
                                                              "functionality not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the mixt frame:"
                                                              "functionality not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the end-effector frame:"
                                                              "functionality not implemented");
    break;
  }

  q_previous = q_current; // Update for next call of this method
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotBiclops.cpp.o) has no
// symbols
void dummy_vpRobotBiclops(){};
#endif
