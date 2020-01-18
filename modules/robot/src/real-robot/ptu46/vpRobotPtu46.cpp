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
 * Interface for the ptu-46 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <signal.h>
#include <string.h>

#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_PTU46

/* Headers des fonctions implementees. */
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/robot/vpPtu46.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotPtu46.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */

bool vpRobotPtu46::robotAlreadyCreated = false;
const double vpRobotPtu46::defaultPositioningVelocity = 10.0;

/* ----------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  Default constructor.

  Initialize the ptu-46 pan, tilt head by opening the serial port.

  \sa init()

*/
vpRobotPtu46::vpRobotPtu46(const char *device) : vpRobot()
{
  this->device = new char[FILENAME_MAX];

  sprintf(this->device, "%s", device);

  vpDEBUG_TRACE(12, "Open communication with Ptu-46.");
  try {
    init();
  } catch (...) {
    delete[] this->device;
    vpERROR_TRACE("Error caught");
    throw;
  }

  try {
    setRobotState(vpRobot::STATE_STOP);
  } catch (...) {
    delete[] this->device;
    vpERROR_TRACE("Error caught");
    throw;
  }
  positioningVelocity = defaultPositioningVelocity;
  return;
}

/* ------------------------------------------------------------------------ */
/* --- DESTRUCTOR  -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Destructor.
  Close the serial connection with the head.

*/
vpRobotPtu46::~vpRobotPtu46(void)
{

  setRobotState(vpRobot::STATE_STOP);

  if (0 != ptu.close()) {
    vpERROR_TRACE("Error while closing communications with the robot ptu-46.");
  }

  vpRobotPtu46::robotAlreadyCreated = false;

  delete[] device;

  return;
}

/* --------------------------------------------------------------------------
 */
/* --- INITIALISATION -------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!

  Open the serial port.


  \exception vpRobotException::constructionError : If the device cannot be
  oppened.

*/
void vpRobotPtu46::init()
{

  vpDEBUG_TRACE(12, "Open connection Ptu-46.");
  if (0 != ptu.init(device)) {
    vpERROR_TRACE("Cannot open connection with ptu-46.");
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with ptu-46");
  }

  return;
}

/*!

  Change the state of the robot either to stop them, or to set position or
  speed control.

*/
vpRobot::vpRobotStateType vpRobotPtu46::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    if (vpRobot::STATE_STOP != getRobotState()) {
      ptu.stop();
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      vpDEBUG_TRACE(12, "Passage vitesse -> position.");
      ptu.stop();
    } else {
      vpDEBUG_TRACE(1, "Passage arret -> position.");
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
      vpDEBUG_TRACE(10, "Arret du robot...");
      ptu.stop();
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
void vpRobotPtu46::stopMotion(void)
{
  ptu.stop();
  setRobotState(vpRobot::STATE_STOP);
}

/*!

  Get the twist matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cVe : Twist transformation between camera and end effector frame to
  expess a velocity skew from end effector frame in camera frame.

*/
void vpRobotPtu46::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  vpPtu46::get_cMe(cMe);

  cVe.buildFrom(cMe);
}

/*!

  Get the homogeneous matrix corresponding to the transformation between the
  camera frame and the end effector frame. The end effector frame is located
  on the tilt axis.

  \param cMe :  Homogeneous matrix between camera and end effector frame.

*/
void vpRobotPtu46::get_cMe(vpHomogeneousMatrix &cMe) const { vpPtu46::get_cMe(cMe); }

/*!
  Get the robot jacobian expressed in the end-effector frame.

  \warning Re is not the embedded camera frame. It corresponds to the frame
  associated to the tilt axis (see also get_cMe).

  \param eJe : Jacobian between end effector frame and end effector frame (on
  tilt axis).

*/
void vpRobotPtu46::get_eJe(vpMatrix &eJe)
{
  vpColVector q(2);
  getPosition(vpRobot::ARTICULAR_FRAME, q);

  try {
    vpPtu46::get_eJe(q, eJe);
  } catch (...) {
    vpERROR_TRACE("catch exception ");
    throw;
  }
}

/*!
  Get the robot jacobian expressed in the robot reference frame

  \param fJe : Jacobian between reference frame (or fix frame) and end
  effector frame (on tilt axis).

*/
void vpRobotPtu46::get_fJe(vpMatrix &fJe)
{
  vpColVector q(2);
  getPosition(vpRobot::ARTICULAR_FRAME, q);

  try {
    vpPtu46::get_fJe(q, fJe);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Set the velocity used for a position control.

  \param velocity : Velocity in % of the maximum velocity between [0,100].
*/
void vpRobotPtu46::setPositioningVelocity(const double velocity) { positioningVelocity = velocity; }
/*!
  Get the velocity in % used for a position control.

  \return Positionning velocity in [0, 100]

*/
double vpRobotPtu46::getPositioningVelocity(void) { return positioningVelocity; }

/*!
   Move the robot in position control.

   \warning This method is blocking. That mean that it waits the end of the
   positionning.

   \param frame : Control frame. This head can only be controlled in
   articular.

   \param q : The position to set for each axis.

   \exception vpRobotException::wrongStateError : If a not supported frame
   type is given.

*/

void vpRobotPtu46::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
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

  // Interface for the controller
  double artpos[2];

  artpos[0] = q[0];
  artpos[1] = q[1];

  if (0 != ptu.move(artpos, positioningVelocity, PTU_ABSOLUTE_MODE)) {
    vpERROR_TRACE("Positionning error.");
    throw vpRobotException(vpRobotException::lowLevelError, "Positionning error.");
  }

  return;
}

/*!
   Move the robot in position control.

   \warning This method is blocking. That mean that it wait the end of the
   positionning.

   \param frame : Control frame. This head can only be controlled in
   articular.

   \param q1 : The pan position to set.
   \param q2 : The tilt position to set.

   \exception vpRobotException::wrongStateError : If a not supported frame
   type is given.

*/
void vpRobotPtu46::setPosition(const vpRobot::vpControlFrameType frame, const double &q1, const double &q2)
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
void vpRobotPtu46::setPosition(const char *filename)
{
  vpColVector q;
  if (readPositionFile(filename, q) == false) {
    vpERROR_TRACE("Cannot get ptu-46 position from file");
    throw vpRobotException(vpRobotException::readingParametersError, "Cannot get ptu-46 position from file");
  }
  setPosition(vpRobot::ARTICULAR_FRAME, q);
}

/*!

  Return the position of each axis.

  \param frame : Control frame. This head can only be controlled in
  articular.

  \param q : The position of the axis.

  \exception vpRobotException::wrongStateError : If a not supported frame type
  is given.

*/
void vpRobotPtu46::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  vpDEBUG_TRACE(9, "# Entree.");

  switch (frame) {
  case vpRobot::CAMERA_FRAME:
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position in camera frame: "
                                                            "not implemented");
    break;
  case vpRobot::REFERENCE_FRAME:
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position in reference frame: "
                                                            "not implemented");
    break;
  case vpRobot::MIXT_FRAME:
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position in mixt frame: "
                                                            "not implemented");
    break;
  case vpRobot::END_EFFECTOR_FRAME:
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot move the robot in end-effector frame: "
                                                              "not implemented");
    break;
  case vpRobot::ARTICULAR_FRAME:
    break;
  }

  double artpos[2];

  if (0 != ptu.getCurrentPosition(artpos)) {
    vpERROR_TRACE("Error when calling  recup_posit_Afma4.");
    throw vpRobotException(vpRobotException::lowLevelError, "Error when calling  recup_posit_Afma4.");
  }

  q.resize(vpPtu46::ndof);

  q[0] = artpos[0];
  q[1] = artpos[1];
}

/*!

  Send a velocity on each axis.

  \param frame : Control frame. This head can only be controlled in articular
  and camera frame. Be aware, the reference frame (vpRobot::REFERENCE_FRAME)
  end-effector frame (vpRobot::END_EFFECTOR_FRAME) and the mixt frame (vpRobot::MIXT_FRAME)
  are not implemented.

  \param v : The desired velocity of the axis. The size of this vector is
  always 2. Velocities are expressed in rad/s.

  - In camera frame, \f$ v = [\omega_x, \omega_y]^t \f$ expressed in rad/s.

  - In articular, we control the 2 dof, \f$ v = [\dot{q}_1, \dot{q}_2]^t \f$
  with \f$ \dot{q}_1 \f$ the pan of the camera and \f$ \dot{q}_2\f$ the tilt
  of the camera in rad/s.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \exception vpRobotException::wrongStateError : If a non supported frame type
  (vpRobot::REFERENCE_FRAME, vpRobot::END_EFFECTOR_FRAME, vpRobot::MIXT_FRAME) is given.

  \warning Velocities could be saturated if one of them exceed the maximal
  autorized speed (see vpRobot::maxRotationVelocity).
*/

void vpRobotPtu46::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  TPtuFrame ptuFrameInterface;

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot "
                           "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    ptuFrameInterface = PTU_CAMERA_FRAME;
    if (v.getRows() != 2) {
      vpERROR_TRACE("Bad dimension fo speed vector in camera frame");
      throw vpRobotException(vpRobotException::wrongStateError, "Bad dimension for speed vector "
                                                                "in camera frame");
    }
    break;
  }
  case vpRobot::ARTICULAR_FRAME: {
    ptuFrameInterface = PTU_ARTICULAR_FRAME;
    if (v.getRows() != 2) {
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
  double ptuSpeedInterface[2];

  switch (frame) {
  case vpRobot::ARTICULAR_FRAME:
  case vpRobot::CAMERA_FRAME: {
    double max = this->maxRotationVelocity;
    bool norm = false;                   // Flag to indicate when velocities need to be nomalized
    for (unsigned int i = 0; i < 2; ++i) // rx and ry of the camera
    {
      if (fabs(v[i]) > max) {
        norm = true;
        max = fabs(v[i]);
        vpERROR_TRACE("Excess velocity: ROTATION "
                      "(axe nr.%d).",
                      i);
      }
    }
    // Rotations velocities normalisation
    if (norm == true) {
      max = this->maxRotationVelocity / max;
      for (unsigned int i = 0; i < 2; ++i)
        ptuSpeedInterface[i] = v[i] * max;
    }
    break;
  }
  default:
    // Should never occur
    break;
  }

  vpCDEBUG(12) << "v: " << ptuSpeedInterface[0] << " " << ptuSpeedInterface[1] << std::endl;
  ptu.move(ptuSpeedInterface, ptuFrameInterface);
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
void vpRobotPtu46::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &q_dot)
{

  TPtuFrame ptuFrameInterface = PTU_ARTICULAR_FRAME;

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the camera frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::ARTICULAR_FRAME: {
    ptuFrameInterface = PTU_ARTICULAR_FRAME;
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the reference frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the mixt frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a velocity in the end-effector frame:"
                                                              "functionality not implemented");
  }
  }

  q_dot.resize(vpPtu46::ndof);
  double ptuSpeedInterface[2];

  ptu.getCurrentSpeed(ptuSpeedInterface, ptuFrameInterface);

  q_dot[0] = ptuSpeedInterface[0];
  q_dot[1] = ptuSpeedInterface[1];
}

/*!

  Return the articular velocity.

  \param frame : Control frame. This head can only be controlled in articular.

  \return The measured articular velocity in rad/s.

  \exception vpRobotException::wrongStateError : If a not supported frame type
  is given.
*/
vpColVector vpRobotPtu46::getVelocity(vpRobot::vpControlFrameType frame)
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
  # Example of ptu-46 position file
  # The axis positions must be preceed by R:
  # First value : pan  articular position in degrees
  # Second value: tilt articular position in degrees
  R: 15.0 5.0
  \endcode

  \return true if a position was found, false otherwise.

*/
bool vpRobotPtu46::readPositionFile(const std::string &filename, vpColVector &q)
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

  q.resize(vpPtu46::ndof);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Ptu-46 position file
        std::cout << "Error: this position file " << filename << " is not for Ptu-46 robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < vpPtu46::ndof + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < vpPtu46::ndof + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < vpPtu46::ndof; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  q.deg2rad();

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Ptu-46 robot in " << filename << std::endl;
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
  - In camera frame, the dimension of d is 6 (tx, ty, ty, rx, ry,
  rz). Translations are expressed in meters, rotations in radians.

  \exception vpRobotException::wrongStateError If a not supported frame type
  is given.

*/
void vpRobotPtu46::getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &d)
{
  double d_[6];

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    d.resize(6);
    ptu.measureDpl(d_, PTU_CAMERA_FRAME);
    d[0] = d_[0];
    d[1] = d_[1];
    d[2] = d_[2];
    d[3] = d_[3];
    d[4] = d_[4];
    d[5] = d_[5];
    break;
  }
  case vpRobot::ARTICULAR_FRAME: {
    ptu.measureDpl(d_, PTU_ARTICULAR_FRAME);
    d.resize(vpPtu46::ndof);
    d[0] = d_[0]; // pan
    d[1] = d_[1]; // tilt
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a displacement in the reference frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a displacement in the reference frame:"
                                                              "functionality not implemented");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::wrongStateError, "Cannot get a displacement in the end-effector frame:"
                                                              "functionality not implemented");
  }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotPtu46.cpp.o) has no
// symbols
void dummy_vpRobotPtu46(){};
#endif
