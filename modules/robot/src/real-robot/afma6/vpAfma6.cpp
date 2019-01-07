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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpAfma6.cpp

  Control of Irisa's gantry robot named Afma6.

*/

#include <iostream>
#include <sstream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/robot/vpAfma6.h>
#include <visp3/robot/vpRobotException.h>

/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

static const char *opt_Afma6[] = {"JOINT_MAX", "JOINT_MIN",   "LONG_56",       "COUPL_56",
                                  "CAMERA",    "eMc_ROT_XYZ", "eMc_TRANS_XYZ", NULL};

#ifdef VISP_HAVE_AFMA6_DATA
const std::string vpAfma6::CONST_AFMA6_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_CCMOP_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_ccmop_without_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_CCMOP_WITH_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_ccmop_with_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_GRIPPER_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_gripper_without_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_GRIPPER_WITH_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_gripper_with_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_VACUUM_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_vacuum_without_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_VACUUM_WITH_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_vacuum_with_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_generic_without_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_eMc_generic_with_distortion_Afma6.cnf");

const std::string vpAfma6::CONST_CAMERA_AFMA6_FILENAME =
    std::string(VISP_AFMA6_DATA_PATH) + std::string("/include/const_camera_Afma6.xml");

#endif // VISP_HAVE_AFMA6_DATA

const char *const vpAfma6::CONST_CCMOP_CAMERA_NAME = "Dragonfly2-8mm-ccmop";
const char *const vpAfma6::CONST_GRIPPER_CAMERA_NAME = "Dragonfly2-6mm-gripper";
const char *const vpAfma6::CONST_VACUUM_CAMERA_NAME = "Dragonfly2-6mm-vacuum";
const char *const vpAfma6::CONST_GENERIC_CAMERA_NAME = "Generic-camera";

const vpAfma6::vpAfma6ToolType vpAfma6::defaultTool = TOOL_CCMOP;

const unsigned int vpAfma6::njoint = 6;

/*!

  Default constructor.

*/
vpAfma6::vpAfma6()
  : _coupl_56(0), _long_56(0), _etc(), _erc(), _eMc(), tool_current(vpAfma6::defaultTool),
    projModel(vpCameraParameters::perspectiveProjWithoutDistortion)
{
  // Set the default parameters in case of the config files are not available.

  //
  // Geometric model constant parameters
  //
  // coupling between join 5 and 6
  this->_coupl_56 = 0.009091;
  // distance between join 5 and 6
  this->_long_56 = -0.06924;
  // Camera extrinsic parameters: effector to camera frame
  this->_eMc.eye(); // Default values are initialized ...
  //  ... in init (vpAfma6::vpAfma6ToolType tool,
  //               vpCameraParameters::vpCameraParametersProjType projModel)
  // Maximal value of the joints
  this->_joint_max[0] = 0.7001;
  this->_joint_max[1] = 0.5201;
  this->_joint_max[2] = 0.4601;
  this->_joint_max[3] = 2.7301;
  this->_joint_max[4] = 2.4801;
  this->_joint_max[5] = 1.5901;
  // Minimal value of the joints
  this->_joint_min[0] = -0.6501;
  this->_joint_min[1] = -0.6001;
  this->_joint_min[2] = -0.5001;
  this->_joint_min[3] = -2.7301;
  this->_joint_min[4] = -0.1001;
  this->_joint_min[5] = -1.5901;

  init();
}

/*!

  Initialize the robot with the default tool vpAfma6::defaultTool.
 */
void vpAfma6::init(void)
{
  this->init(vpAfma6::defaultTool);
  return;
}

/*!

  Read files containing the constant parameters related to the robot
  kinematics and to the end-effector to camera transformation.

  \param camera_extrinsic_parameters : Filename containing the constant
  parameters of the robot kinematics \f$^e{\bf M}_c \f$ transformation.

  \param camera_intrinsic_parameters : Filename containing the camera
  extrinsic parameters.

*/
void vpAfma6::init(const std::string &camera_extrinsic_parameters, const std::string &camera_intrinsic_parameters)
{
  this->parseConfigFile(camera_extrinsic_parameters);

  this->parseConfigFile(camera_intrinsic_parameters);
}

/*!

  Set the type of tool attached to the robot and transformation
  between the end-effector and the tool (\f$^e{\bf M}c\f$).
  This last parameter is loaded from a file.

  \param tool : Type of tool in use.

  \param filename : Path of the configuration file containing the
  transformation between the end-effector frame and the tool frame.

  The configuration file should have the form below:

  \code
# Start with any number of consecutive lines
# beginning with the symbol '#'
#
# The 3 following lines contain the name of the camera,
# the rotation parameters of the geometric transformation
# using the Euler angles in degrees with convention XYZ and
# the translation parameters expressed in meters
CAMERA CameraName
eMc_ROT_XYZ 10.0 -90.0 20.0
eMc_TRANS_XYZ  0.05 0.01 0.06
    \endcode

  \sa init(vpAfma6::vpToolType,
vpCameraParameters::vpCameraParametersProjType), init(vpAfma6::vpToolType,
const vpHomogeneousMatrix&)
*/
void vpAfma6::init(vpAfma6::vpAfma6ToolType tool, const std::string &filename)
{
  this->setToolType(tool);
  this->parseConfigFile(filename.c_str());
}

/*!

  Read files containing the constant parameters related to the robot
  tools in order to set the end-effector to tool transformation.

  \param camera_extrinsic_parameters : Filename containing the camera
  extrinsic parameters.

*/
void vpAfma6::init(const std::string &camera_extrinsic_parameters)
{
  this->parseConfigFile(camera_extrinsic_parameters);
}

/*!

  Set the type of tool attached to the robot and the transformation
  between the end-effector and the tool (\f$^e{\bf M}c\f$).

  \param tool : Type of tool in use.

  \param eMc : Homogeneous matrix representation of the transformation
  between the end-effector frame and the tool frame.

  \sa init(vpAfma6::vpAfma6ToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpAfma6::vpAfma6ToolType, const std::string&)

*/
void vpAfma6::init(vpAfma6::vpAfma6ToolType tool, const vpHomogeneousMatrix &eMc)
{
  this->setToolType(tool);
  this->set_eMc(eMc);
}

/*!

  Get the constant parameters related to the robot kinematics and to
  the end-effector to camera transformation (eMc) corresponding to the
  camera extrinsic parameters. These last parameters depend on the
  camera and projection model in use.

  \warning If the macro VISP_HAVE_AFMA6_DATA is defined in vpConfig.h
  this function reads the camera extrinsic parameters from the file
  corresponding to the specified camera type and projection type.
  Otherwise corresponding default parameters are loaded.

  \param tool : Camera in use.

  \param proj_model : Projection model of the camera.

*/
void vpAfma6::init(vpAfma6::vpAfma6ToolType tool, vpCameraParameters::vpCameraParametersProjType proj_model)
{

  this->projModel = proj_model;

#ifdef VISP_HAVE_AFMA6_DATA
  // Read the robot parameters from files
  std::string filename_eMc;
  switch (tool) {
  case vpAfma6::TOOL_CCMOP: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_CCMOP_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_CCMOP_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpAfma6::TOOL_GRIPPER: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_GRIPPER_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_GRIPPER_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpAfma6::TOOL_VACUUM: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_VACUUM_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_VACUUM_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpAfma6::TOOL_GENERIC_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  default: {
    vpERROR_TRACE("This error should not occur!");
    //       vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
    // 		   "que les specs de la classe ont ete modifiee, "
    // 		   "et que le code n'a pas ete mis a jour "
    // 		   "correctement.");
    //       vpERROR_TRACE ("Verifiez les valeurs possibles du type "
    // 		   "vpAfma6::vpAfma6ToolType, et controlez que "
    // 		   "tous les cas ont ete pris en compte dans la "
    // 		   "fonction init(camera).");
    break;
  }
  }

  this->init(vpAfma6::CONST_AFMA6_FILENAME, filename_eMc);

#else  // VISP_HAVE_AFMA6_DATA

  // Use here default values of the robot constant parameters.
  switch (tool) {
  case vpAfma6::TOOL_CCMOP: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      _erc[0] = vpMath::rad(164.35); // rx
      _erc[1] = vpMath::rad(89.64);  // ry
      _erc[2] = vpMath::rad(-73.05); // rz
      _etc[0] = 0.0117;              // tx
      _etc[1] = 0.0033;              // ty
      _etc[2] = 0.2272;              // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      _erc[0] = vpMath::rad(33.54); // rx
      _erc[1] = vpMath::rad(89.34); // ry
      _erc[2] = vpMath::rad(57.83); // rz
      _etc[0] = 0.0373;             // tx
      _etc[1] = 0.0024;             // ty
      _etc[2] = 0.2286;             // tz
      break;
    }
    break;
  }
  case vpAfma6::TOOL_GRIPPER: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      _erc[0] = vpMath::rad(88.33); // rx
      _erc[1] = vpMath::rad(72.07); // ry
      _erc[2] = vpMath::rad(2.53);  // rz
      _etc[0] = 0.0783;             // tx
      _etc[1] = 0.1234;             // ty
      _etc[2] = 0.1638;             // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      _erc[0] = vpMath::rad(86.69); // rx
      _erc[1] = vpMath::rad(71.93); // ry
      _erc[2] = vpMath::rad(4.17);  // rz
      _etc[0] = 0.1034;             // tx
      _etc[1] = 0.1142;             // ty
      _etc[2] = 0.1642;             // tz
      break;
    }
    break;
  }
  case vpAfma6::TOOL_VACUUM: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      _erc[0] = vpMath::rad(90.40); // rx
      _erc[1] = vpMath::rad(75.11); // ry
      _erc[2] = vpMath::rad(0.18);  // rz
      _etc[0] = 0.0038;             // tx
      _etc[1] = 0.1281;             // ty
      _etc[2] = 0.1658;             // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      _erc[0] = vpMath::rad(91.61); // rx
      _erc[1] = vpMath::rad(76.17); // ry
      _erc[2] = vpMath::rad(-0.98); // rz
      _etc[0] = 0.0815;             // tx
      _etc[1] = 0.1162;             // ty
      _etc[2] = 0.1658;             // tz
      break;
    }
    break;
  }
  case vpAfma6::TOOL_CUSTOM:
  case vpAfma6::TOOL_GENERIC_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
    case vpCameraParameters::perspectiveProjWithDistortion:
      // set eMc to identity
      _erc[0] = 0; // rx
      _erc[1] = 0; // ry
      _erc[2] = 0; // rz
      _etc[0] = 0; // tx
      _etc[1] = 0; // ty
      _etc[2] = 0; // tz
      break;
    }
    break;
  }
  }
  vpRotationMatrix eRc(_erc);
  this->_eMc.buildFrom(_etc, eRc);
#endif // VISP_HAVE_AFMA6_DATA

  setToolType(tool);
  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
  all the six joints.

  This method is the same than get_fMc(const vpColVector & q).

  \param q : Articular position of the six joints: q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix vpAfma6::getForwardKinematics(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Compute the inverse kinematics (inverse geometric model).

  By inverse kinematics we mean here the six articular values of the joint
  positions given the position and the orientation of the camera frame
  relative to the base frame.

  \param fMc : Homogeneous matrix describing the transformation from
  base frame to the camera frame.

  \param q : In input, the current articular joint position of the
  robot. In output, the solution of the inverse kinematics. Articular
  position of the six joints: q[0], q[1], q[2] correspond to the first
  3 translations expressed in meter, while q[3], q[4] and q[5]
  correspond to the 3 succesives rotations expressed in radians.

  \param nearest : true to return the nearest solution to q. false to
  return the farest.

  \param verbose : Activates printings when no solution is found.

  \return The number of solutions (1 or 2) of the inverse geometric
  model. O, if no solution can be found.

  The code below shows how to compute the inverse geometric model:

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotAfma6.h>

int main()
{
#ifdef VISP_HAVE_AFMA6
  vpColVector q1(6), q2(6);
  vpHomogeneousMatrix fMc;

  vpRobotAfma6 robot;

  // Get the current articular position of the robot
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q1);

  // Compute the pose of the camera in the reference frame using the
  // direct geometric model
  fMc = robot.getForwardKinematics(q1);
  // this is similar to  fMc = robot.get_fMc(q1);
  // or robot.get_fMc(q1, fMc);

  // Compute the inverse geometric model
  int nbsol; // number of solutions (0, 1 or 2) of the inverse geometric model
  // get the nearest solution to the current articular position
  nbsol = robot.getInverseKinematics(fMc, q1, true);

  if (nbsol == 0)
    std::cout << "No solution of the inverse geometric model " << std::endl;
  else if (nbsol >= 1)
    std::cout << "First solution: " << q1 << std::endl;

  if (nbsol == 2) {
    // Compute the other solution of the inverse geometric model
    q2 = q1;
    robot.getInverseKinematics(fMc, q2, false);
    std::cout << "Second solution: " << q2 << std::endl;
  }
#endif
}
  \endcode

  \sa getForwardKinematics()

*/
int vpAfma6::getInverseKinematics(const vpHomogeneousMatrix &fMc, vpColVector &q, const bool &nearest,
                                  const bool &verbose) const
{
  vpHomogeneousMatrix fMe;
  double q_[2][6], d[2], t;
  int ok[2];
  double cord[6];

  int nbsol = 0;

  if (q.getRows() != njoint)
    q.resize(6);

  //   for(unsigned int i=0;i<3;i++) {
  //     fMe[i][3] = fMc[i][3];
  //     for(int j=0;j<3;j++) {
  //       fMe[i][j] = 0.0;
  //       for (int k=0;k<3;k++) fMe[i][j] += fMc[i][k]*rpi[j][k];
  //       fMe[i][3] -= fMe[i][j]*rpi[j][3];
  //     }
  //   }

  //   std::cout << "\n\nfMc: " << fMc;
  //   std::cout << "\n\neMc: " << _eMc;

  fMe = fMc * this->_eMc.inverse();
  //   std::cout << "\n\nfMe: " << fMe;

  if (fMe[2][2] >= .99999f) {
    vpTRACE("singularity\n");
    q_[0][4] = q_[1][4] = M_PI / 2.f;
    t = atan2(fMe[0][0], fMe[0][1]);
    q_[1][3] = q_[0][3] = q[3];
    q_[1][5] = q_[0][5] = t - q_[0][3];

    while ((q_[1][5] + vpMath::rad(2)) >= this->_joint_max[5])
    /*			-> a cause du couplage 4/5	*/
    {
      q_[1][5] -= vpMath::rad(10);
      q_[1][3] += vpMath::rad(10);
    }
    while (q_[1][5] <= this->_joint_min[5]) {
      q_[1][5] += vpMath::rad(10);
      q_[1][3] -= vpMath::rad(10);
    }
  } else if (fMe[2][2] <= -.99999) {
    vpTRACE("singularity\n");
    q_[0][4] = q_[1][4] = -M_PI / 2;
    t = atan2(fMe[1][1], fMe[1][0]);
    q_[1][3] = q_[0][3] = q[3];
    q_[1][5] = q_[0][5] = q_[0][3] - t;
    while ((q_[1][5] + vpMath::rad(2)) >= this->_joint_max[5])
    /*			-> a cause du couplage 4/5	*/
    {
      q_[1][5] -= vpMath::rad(10);
      q_[1][3] -= vpMath::rad(10);
    }
    while (q_[1][5] <= this->_joint_min[5]) {
      q_[1][5] += vpMath::rad(10);
      q_[1][3] += vpMath::rad(10);
    }
  } else {
    q_[0][3] = atan2(-fMe[0][2], fMe[1][2]);
    if (q_[0][3] >= 0.0)
      q_[1][3] = q_[0][3] - M_PI;
    else
      q_[1][3] = q_[0][3] + M_PI;

    q_[0][4] = asin(fMe[2][2]);
    if (q_[0][4] >= 0.0)
      q_[1][4] = M_PI - q_[0][4];
    else
      q_[1][4] = -M_PI - q_[0][4];

    q_[0][5] = atan2(-fMe[2][1], fMe[2][0]);
    if (q_[0][5] >= 0.0)
      q_[1][5] = q_[0][5] - M_PI;
    else
      q_[1][5] = q_[0][5] + M_PI;
  }
  q_[0][0] = fMe[0][3] - this->_long_56 * cos(q_[0][3]);
  q_[1][0] = fMe[0][3] - this->_long_56 * cos(q_[1][3]);
  q_[0][1] = fMe[1][3] - this->_long_56 * sin(q_[0][3]);
  q_[1][1] = fMe[1][3] - this->_long_56 * sin(q_[1][3]);
  q_[0][2] = q_[1][2] = fMe[2][3];

  /* prise en compte du couplage axes 5/6	*/
  q_[0][5] += this->_coupl_56 * q_[0][4];
  q_[1][5] += this->_coupl_56 * q_[1][4];

  for (int j = 0; j < 2; j++) {
    ok[j] = 1;
    // test is position is reachable
    for (unsigned int i = 0; i < 6; i++) {
      if (q_[j][i] < this->_joint_min[i] || q_[j][i] > this->_joint_max[i]) {
        if (verbose) {
          if (i < 3)
            std::cout << "Joint " << i << " not in limits: " << this->_joint_min[i] << " < " << q_[j][i] << " < "
                      << this->_joint_max[i] << std::endl;
          else
            std::cout << "Joint " << i << " not in limits: " << vpMath::deg(this->_joint_min[i]) << " < "
                      << vpMath::deg(q_[j][i]) << " < " << vpMath::deg(this->_joint_max[i]) << std::endl;
        }
        ok[j] = 0;
      }
    }
  }
  if (ok[0] == 0) {
    if (ok[1] == 0) {
      std::cout << "No solution..." << std::endl;
      nbsol = 0;
      return nbsol;
    } else if (ok[1] == 1) {
      for (unsigned int i = 0; i < 6; i++)
        cord[i] = q_[1][i];
      nbsol = 1;
    }
  } else {
    if (ok[1] == 0) {
      for (unsigned int i = 0; i < 6; i++)
        cord[i] = q_[0][i];
      nbsol = 1;
    } else {
      nbsol = 2;
      // vpTRACE("2 solutions\n");
      for (int j = 0; j < 2; j++) {
        d[j] = 0.0;
        for (unsigned int i = 3; i < 6; i++)
          d[j] += (q_[j][i] - q[i]) * (q_[j][i] - q[i]);
      }
      if (nearest == true) {
        if (d[0] <= d[1])
          for (unsigned int i = 0; i < 6; i++)
            cord[i] = q_[0][i];
        else
          for (unsigned int i = 0; i < 6; i++)
            cord[i] = q_[1][i];
      } else {
        if (d[0] <= d[1])
          for (unsigned int i = 0; i < 6; i++)
            cord[i] = q_[1][i];
        else
          for (unsigned int i = 0; i < 6; i++)
            cord[i] = q_[0][i];
      }
    }
  }
  for (unsigned int i = 0; i < 6; i++)
    q[i] = cord[i];

  return nbsol;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
  all the six joints.

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Articular position of the six joints: q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa getForwardKinematics(const vpColVector & q)
*/
vpHomogeneousMatrix vpAfma6::get_fMc(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of
  all the six joints.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param fMc The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (fMc).

*/
void vpAfma6::get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{

  // Compute the direct geometric model: fMe = transformation between
  // fix and end effector frame.
  vpHomogeneousMatrix fMe;

  get_fMe(q, fMe);

  fMc = fMe * this->_eMc;

  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the end effector with respect to the base frame given the
  articular positions of all the six joints.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param fMe The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  end effector frame (fMe).

*/
void vpAfma6::get_fMe(const vpColVector &q, vpHomogeneousMatrix &fMe) const
{
  double q0 = q[0]; // meter
  double q1 = q[1]; // meter
  double q2 = q[2]; // meter

  /* Decouplage liaisons 2 et 3. */
  double q5 = q[5] - this->_coupl_56 * q[4];

  double c1 = cos(q[3]);
  double s1 = sin(q[3]);
  double c2 = cos(q[4]);
  double s2 = sin(q[4]);
  double c3 = cos(q5);
  double s3 = sin(q5);

  // Compute the direct geometric model: fMe = transformation betwee
  // fix and end effector frame.
  fMe[0][0] = s1 * s2 * c3 + c1 * s3;
  fMe[0][1] = -s1 * s2 * s3 + c1 * c3;
  fMe[0][2] = -s1 * c2;
  fMe[0][3] = q0 + this->_long_56 * c1;

  fMe[1][0] = -c1 * s2 * c3 + s1 * s3;
  fMe[1][1] = c1 * s2 * s3 + s1 * c3;
  fMe[1][2] = c1 * c2;
  fMe[1][3] = q1 + this->_long_56 * s1;

  fMe[2][0] = c2 * c3;
  fMe[2][1] = -c2 * s3;
  fMe[2][2] = s2;
  fMe[2][3] = q2;

  fMe[3][0] = 0;
  fMe[3][1] = 0;
  fMe[3][2] = 0;
  fMe[3][3] = 1;

  //  vpCTRACE << "Effector position fMe: " << std::endl << fMe;

  return;
}

/*!

  Get the geometric transformation between the camera frame and the
  end-effector frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void vpAfma6::get_cMe(vpHomogeneousMatrix &cMe) const { cMe = this->_eMc.inverse(); }
/*!

  Get the geometric transformation between the end-effector frame and the
  camera or tool frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by calibration.

  \return Transformation between the end-effector frame and the
  camera frame.

*/
vpHomogeneousMatrix vpAfma6::get_eMc() const { return (this->_eMc); }

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void vpAfma6::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  get_cMe(cMe);

  cVe.buildFrom(cMe);

  return;
}

/*!

  Get the robot jacobian expressed in the end-effector frame.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param eJe : Robot jacobian expressed in the end-effector frame.

*/
void vpAfma6::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{

  eJe.resize(6, 6);

  double s4, c4, s5, c5, s6, c6;

  s4 = sin(q[3]);
  c4 = cos(q[3]);
  s5 = sin(q[4]);
  c5 = cos(q[4]);
  s6 = sin(q[5] - this->_coupl_56 * q[4]);
  c6 = cos(q[5] - this->_coupl_56 * q[4]);

  eJe = 0;
  eJe[0][0] = s4 * s5 * c6 + c4 * s6;
  eJe[0][1] = -c4 * s5 * c6 + s4 * s6;
  eJe[0][2] = c5 * c6;
  eJe[0][3] = -this->_long_56 * s5 * c6;

  eJe[1][0] = -s4 * s5 * s6 + c4 * c6;
  eJe[1][1] = c4 * s5 * s6 + s4 * c6;
  eJe[1][2] = -c5 * s6;
  eJe[1][3] = this->_long_56 * s5 * s6;

  eJe[2][0] = -s4 * c5;
  eJe[2][1] = c4 * c5;
  eJe[2][2] = s5;
  eJe[2][3] = this->_long_56 * c5;

  eJe[3][3] = c5 * c6;
  eJe[3][4] = s6;

  eJe[4][3] = -c5 * s6;
  eJe[4][4] = c6;

  eJe[5][3] = s5;
  eJe[5][4] = -this->_coupl_56;
  eJe[5][5] = 1;

  return;
}

/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame.

  \f[
  {^f}J_e = \left(\begin{array}{cccccc}
  1  &   0  &   0  & -Ls4 &   0  &   0   \\
  0  &   1  &   0  &  Lc4 &   0  &   0   \\
  0  &   0  &   1  &   0  &   0  &   0   \\
  0  &   0  &   0  &   0  &   c4+\gamma s4c5 & -s4c5 \\
  0  &   0  &   0  &   0  &   s4-\gamma c4c5 &  c4c5 \\
  0  &   0  &   0  &   1  &   -gamma s5  &  s5   \\
  \end{array}
  \right)
  \f]
  where \f$\gamma\f$ is the coupling factor between join 5 and 6.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param fJe : Robot jacobian expressed in the robot reference frame.

*/

void vpAfma6::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{

  fJe.resize(6, 6);

  // block superieur gauche
  fJe[0][0] = fJe[1][1] = fJe[2][2] = 1;

  double s4 = sin(q[3]);
  double c4 = cos(q[3]);

  // block superieur droit
  fJe[0][3] = -this->_long_56 * s4;
  fJe[1][3] = this->_long_56 * c4;

  double s5 = sin(q[4]);
  double c5 = cos(q[4]);
  // block inferieur droit
  fJe[3][4] = c4;
  fJe[3][5] = -s4 * c5;
  fJe[4][4] = s4;
  fJe[4][5] = c4 * c5;
  fJe[5][3] = 1;
  fJe[5][5] = s5;

  // coupling between joint 5 and 6
  fJe[3][4] += this->_coupl_56 * s4 * c5;
  fJe[4][4] += -this->_coupl_56 * c4 * c5;
  fJe[5][4] += -this->_coupl_56 * s5;

  return;
}

/*!
  Get min joint values.

  \return Minimal joint values for the 6 dof
  X,Y,Z,A,B,C. Translation X,Y,Z are expressed in meters. Rotation
  A,B,C in radians.

*/
vpColVector vpAfma6::getJointMin() const
{
  vpColVector qmin(6);
  for (unsigned int i = 0; i < 6; i++)
    qmin[i] = this->_joint_min[i];
  return qmin;
}

/*!
  Get max joint values.

  \return Maximal joint values for the 6 dof
  X,Y,Z,A,B,C. Translation X,Y,Z are expressed in meters. Rotation
  A,B,C in radians.

*/
vpColVector vpAfma6::getJointMax() const
{
  vpColVector qmax(6);
  for (unsigned int i = 0; i < 6; i++)
    qmax[i] = this->_joint_max[i];
  return qmax;
}

/*!

  Return the coupling factor between join 5 and 6.

  \return Coupling factor between join 5 and 6.
*/
double vpAfma6::getCoupl56() const { return _coupl_56; }

/*!

  Return the distance between join 5 and 6.

  \return Distance between join 5 and 6.
*/
double vpAfma6::getLong56() const { return _long_56; }

/*!

  This function gets the robot constant parameters from a file.

  \param filename : File name containing the robot constant
  parameters, like max/min joint values, distance between 5 and 6 axis,
  coupling factor between axis 5 and 6, and the hand-to-eye homogeneous
  matrix.

*/
void vpAfma6::parseConfigFile(const std::string &filename)
{
  vpRxyzVector erc;        // eMc rotation
  vpTranslationVector etc; // eMc translation

  std::ifstream fdconfig(filename.c_str(), std::ios::in);

  if (!fdconfig.is_open()) {
    throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the config file: %s",
                           filename.c_str());
  }

  std::string line;
  int lineNum = 0;
  bool get_erc = false;
  bool get_etc = false;
  int code;

  while (std::getline(fdconfig, line)) {
    lineNum++;
    if ((line.compare(0, 1, "#") == 0) || line.empty()) { // skip comment or empty line
      continue;
    }
    std::istringstream ss(line);
    std::string key;
    ss >> key;

    for (code = 0; NULL != opt_Afma6[code]; ++code) {
      if (key.compare(opt_Afma6[code]) == 0) {
        break;
      }
    }

    switch (code) {
    case 0:
      ss >> this->_joint_max[0] >> this->_joint_max[1] >> this->_joint_max[2] >> this->_joint_max[3] >>
          this->_joint_max[4] >> this->_joint_max[5];
      break;

    case 1:
      ss >> this->_joint_min[0] >> this->_joint_min[1] >> this->_joint_min[2] >> this->_joint_min[3] >>
          this->_joint_min[4] >> this->_joint_min[5];
      break;

    case 2:
      ss >> this->_long_56;
      break;

    case 3:
      ss >> this->_coupl_56;
      break;

    case 4:
      break; // Nothing to do: camera name

    case 5:
      ss >> erc[0] >> erc[1] >> erc[2];

      // Convert rotation from degrees to radians
      erc = erc * M_PI / 180.0;
      get_erc = true;
      break;

    case 6:
      ss >> etc[0] >> etc[1] >> etc[2];
      get_etc = true;
      break;

    default:
      throw(vpRobotException(vpRobotException::readingParametersError, "Bad configuration file %s line #%d",
                             filename.c_str(), lineNum));
    }
  }

  fdconfig.close();

  // Compute the eMc matrix from the translations and rotations
  if (get_etc && get_erc) {
    _erc = erc;
    _etc = etc;

    vpRotationMatrix eRc(_erc);
    this->_eMc.buildFrom(_etc, eRc);
  }
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame (commonly a camera).

  \param eMc : Transformation between the end-effector frame
  and the tool frame.
*/
void vpAfma6::set_eMc(const vpHomogeneousMatrix &eMc)
{
  this->_eMc = eMc;
  this->_etc = eMc.getTranslationVector();

  vpRotationMatrix R(eMc);
  this->_erc.buildFrom(R);
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpAfma6::CONST_CAMERA_AFMA6_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp3/core/vpConfig.h file.

  \warning Thid method needs also an access to the files containing the
  camera parameters in XML format. This access is available if
  VISP_HAVE_AFMA6_DATA macro is defined in include/visp3/core/vpConfig.h file.

  - If VISP_HAVE_AFMA6_DATA and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from const_camera_Afma6.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param image_width : Image width used to compute camera calibration.
  \param image_height : Image height used to compute camera calibration.

  The code below shows how to get the camera parameters of the camera
  attached to the robot.

  \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_AFMA6)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;

  vpRobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  // Camera parameters are read from
  // /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml
  // if VISP_HAVE_AFMA6_DATA and VISP_HAVE_XML2 macros are defined in vpConfig.h file
  try {
    robot.getCameraParameters (cam, I.getWidth(), I.getHeight());
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
#endif
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.
*/

void vpAfma6::getCameraParameters(vpCameraParameters &cam, const unsigned int &image_width,
                                  const unsigned int &image_height) const
{
#if defined(VISP_HAVE_XML2) && defined(VISP_HAVE_AFMA6_DATA)
  vpXmlParserCamera parser;
  switch (getToolType()) {
  case vpAfma6::TOOL_CCMOP: {
    std::cout << "Get camera parameters for camera \"" << vpAfma6::CONST_CCMOP_CAMERA_NAME << "\"" << std::endl
              << "from the XML file: \"" << vpAfma6::CONST_CAMERA_AFMA6_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpAfma6::CONST_CAMERA_AFMA6_FILENAME, vpAfma6::CONST_CCMOP_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_GRIPPER: {
    std::cout << "Get camera parameters for camera \"" << vpAfma6::CONST_GRIPPER_CAMERA_NAME << "\"" << std::endl
              << "from the XML file: \"" << vpAfma6::CONST_CAMERA_AFMA6_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpAfma6::CONST_CAMERA_AFMA6_FILENAME, vpAfma6::CONST_GRIPPER_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_VACUUM: {
    std::cout << "Get camera parameters for camera \"" << vpAfma6::CONST_VACUUM_CAMERA_NAME << "\"" << std::endl
              << "from the XML file: \"" << vpAfma6::CONST_CAMERA_AFMA6_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpAfma6::CONST_CAMERA_AFMA6_FILENAME, vpAfma6::CONST_VACUUM_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_GENERIC_CAMERA: {
    std::cout << "Get camera parameters for camera \"" << vpAfma6::CONST_GENERIC_CAMERA_NAME << "\"" << std::endl
              << "from the XML file: \"" << vpAfma6::CONST_CAMERA_AFMA6_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpAfma6::CONST_CAMERA_AFMA6_FILENAME, vpAfma6::CONST_GENERIC_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  default: {
    vpERROR_TRACE("This error should not occur!");
    //       vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
    //        "que les specs de la classe ont ete modifiee, "
    //        "et que le code n'a pas ete mis a jour "
    //        "correctement.");
    //       vpERROR_TRACE ("Verifiez les valeurs possibles du type "
    //        "vpAfma6::vpAfma6ToolType, et controlez que "
    //        "tous les cas ont ete pris en compte dans la "
    //        "fonction init(camera).");
    throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
  }
  }
#else
  // Set default parameters
  switch (getToolType()) {
  case vpAfma6::TOOL_CCMOP: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpAfma6::CONST_CCMOP_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(1108.0, 1110.0, 314.5, 243.2);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(1090.6, 1090.0, 310.1, 260.8, -0.2114, 0.2217);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_GRIPPER: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpAfma6::CONST_GRIPPER_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(850.9, 853.0, 311.1, 243.6);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(837.0, 837.5, 308.7, 251.6, -0.1455, 0.1511);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_VACUUM: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpAfma6::CONST_VACUUM_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(853.5, 856.0, 307.8, 236.8);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(828.5, 829.0, 322.5, 232.9, -0.1921, 0.2057);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpAfma6::TOOL_GENERIC_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpAfma6::CONST_GENERIC_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(853.5, 856.0, 307.8, 236.8);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(828.5, 829.0, 322.5, 232.9, -0.1921, 0.2057);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  default:
    vpERROR_TRACE("This error should not occur!");
    break;
  }
#endif
  return;
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  Camera parameters are read from
  /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml

  \param cam : In output, camera parameters to fill.
  \param I : A B&W image send by the current camera in use.

  \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_AFMA6)
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;

  vpRobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  try {
    robot.getCameraParameters (cam, I);
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
#endif
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.

*/
void vpAfma6::getCameraParameters(vpCameraParameters &cam, const vpImage<unsigned char> &I) const
{
  getCameraParameters(cam, I.getWidth(), I.getHeight());
}
/*!
  \brief Get the current intrinsic camera parameters obtained by calibration.

  Camera parameters are read from
  /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml

  \param cam : In output, camera parameters to fill.
  \param I : A color image send by the current camera in use.

  \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotAfma6.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#if defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_AFMA6)
  vpImage<vpRGBa> I;
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;

  vpRobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  try {
    robot.getCameraParameters (cam, I);
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
#endif
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.

*/

void vpAfma6::getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I) const
{
  getCameraParameters(cam, I.getWidth(), I.getHeight());
}

/*!

  Print on the output stream \e os the robot parameters (joint
  min/max, distance between axis 5 and 6, coupling factor between axis
  5 and 6, hand-to-eye homogeneous matrix.

  \param os : Output stream.
  \param afma6 : Robot parameters.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAfma6 &afma6)
{
  vpRotationMatrix eRc;
  afma6._eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  os << "Joint Max:" << std::endl
     << "\t" << afma6._joint_max[0] << "\t" << afma6._joint_max[1] << "\t" << afma6._joint_max[2] << "\t"
     << afma6._joint_max[3] << "\t" << afma6._joint_max[4] << "\t" << afma6._joint_max[5] << "\t" << std::endl

     << "Joint Min: " << std::endl
     << "\t" << afma6._joint_min[0] << "\t" << afma6._joint_min[1] << "\t" << afma6._joint_min[2] << "\t"
     << afma6._joint_min[3] << "\t" << afma6._joint_min[4] << "\t" << afma6._joint_min[5] << "\t" << std::endl

     << "Long 5-6: " << std::endl
     << "\t" << afma6._long_56 << "\t" << std::endl

     << "Coupling 5-6:" << std::endl
     << "\t" << afma6._coupl_56 << "\t" << std::endl

     << "eMc: " << std::endl
     << "\tTranslation (m): " << afma6._eMc[0][3] << " " << afma6._eMc[1][3] << " " << afma6._eMc[2][3] << "\t"
     << std::endl
     << "\tRotation Rxyz (rad) : " << rxyz[0] << " " << rxyz[1] << " " << rxyz[2] << "\t" << std::endl
     << "\tRotation Rxyz (deg) : " << vpMath::deg(rxyz[0]) << " " << vpMath::deg(rxyz[1]) << " " << vpMath::deg(rxyz[2])
     << "\t" << std::endl;

  return os;
}
