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
 * Interface for the ADEPT Viper 850 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpViper850.cpp

  Modelisation of the ADEPT Viper 850 robot.

*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/robot/vpViper850.h>

static const char *opt_viper850[] = {"CAMERA", "eMc_ROT_XYZ", "eMc_TRANS_XYZ", NULL};

#ifdef VISP_HAVE_VIPER850_DATA
const std::string vpViper850::CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) +
    std::string("/include/const_eMc_MarlinF033C_without_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_eMc_MarlinF033C_with_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) +
    std::string("/include/const_eMc_PTGreyFlea2_without_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_eMc_PTGreyFlea2_with_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_SCHUNK_GRIPPER_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/"
                                                       "const_eMc_schunk_gripper_without_distortion_Viper850."
                                                       "cnf");

const std::string vpViper850::CONST_EMC_SCHUNK_GRIPPER_WITH_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) +
    std::string("/include/const_eMc_schunk_gripper_with_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_eMc_generic_without_distortion_Viper850.cnf");

const std::string vpViper850::CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_eMc_generic_with_distortion_Viper850.cnf");

const std::string vpViper850::CONST_CAMERA_FILENAME =
    std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_camera_Viper850.xml");

#endif // VISP_HAVE_VIPER850_DATA

const char *const vpViper850::CONST_MARLIN_F033C_CAMERA_NAME = "Marlin-F033C-12mm";
const char *const vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME = "PTGrey-Flea2-6mm";
const char *const vpViper850::CONST_SCHUNK_GRIPPER_CAMERA_NAME = "Schunk-Gripper-PTGrey-Flea2-6mm";
const char *const vpViper850::CONST_GENERIC_CAMERA_NAME = "Generic-camera";

const vpViper850::vpToolType vpViper850::defaultTool = vpViper850::TOOL_PTGREY_FLEA2_CAMERA;

/*!

  Default constructor.
  Sets the specific parameters like the Denavit Hartenberg parameters.

*/
vpViper850::vpViper850()
  : tool_current(vpViper850::defaultTool), projModel(vpCameraParameters::perspectiveProjWithoutDistortion)

{
  // Denavit Hartenberg parameters
  a1 = 0.075;
  a2 = 0.365;
  a3 = 0.090;
  d1 = 0.335;
  d4 = 0.405;
  d6 = 0.080 + 0.1016; // To take into account the offset to go to the tool changer
  c56 = -341.33 / 9102.22;

  // Software joint limits in radians
  joint_min[0] = vpMath::rad(-170);
  joint_min[1] = vpMath::rad(-190);
  joint_min[2] = vpMath::rad(-29);
  joint_min[3] = vpMath::rad(-190);
  joint_min[4] = vpMath::rad(-120);
  joint_min[5] = vpMath::rad(-360);
  joint_max[0] = vpMath::rad(170);
  joint_max[1] = vpMath::rad(45);
  joint_max[2] = vpMath::rad(256);
  joint_max[3] = vpMath::rad(190);
  joint_max[4] = vpMath::rad(120);
  joint_max[5] = vpMath::rad(360);

  init(); // Set the default tool
}

/*!

  Initialize the robot with the default tool vpViper850::defaultTool.
 */
void vpViper850::init(void)
{
  this->init(vpViper850::defaultTool);
  return;
}

/*!

  Read files containing the constant parameters related to the robot
  tools in order to set the end-effector to tool transformation.

  \param camera_extrinsic_parameters : Filename containing the camera
  extrinsic parameters.

*/
void vpViper850::init(const std::string &camera_extrinsic_parameters)
{
  // vpTRACE ("Parse camera file \""%s\"".", camera_filename);
  this->parseConfigFile(camera_extrinsic_parameters);

  return;
}

/*!

  Set the constant parameters related to the robot kinematics and to
  the end-effector to camera transformation (\f$^e{\bf M}c\f$)
  corresponding to the camera extrinsic parameters. These last
  parameters depend on the camera and projection model in use and are
  loaded from predefined files or parameters.

  \warning If the macro VISP_HAVE_VIPER850_DATA is defined in vpConfig.h
  this function reads the camera extrinsic parameters from the file
  corresponding to the specified camera type and projection type.
  Otherwise corresponding default parameters are loaded.

  \param tool : Camera in use.

  \param proj_model : Projection model of the camera.

  \sa init(vpViper850::vpToolType, const std::string&),
  init(vpViper850::vpToolType, const vpHomogeneousMatrix&)

*/
void vpViper850::init(vpViper850::vpToolType tool, vpCameraParameters::vpCameraParametersProjType proj_model)
{

  this->projModel = proj_model;

#ifdef VISP_HAVE_VIPER850_DATA
  // Read the robot parameters from files
  std::string filename_eMc;
  switch (tool) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpViper850::TOOL_SCHUNK_GRIPPER_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      filename_eMc = CONST_EMC_SCHUNK_GRIPPER_WITHOUT_DISTORTION_FILENAME;
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      filename_eMc = CONST_EMC_SCHUNK_GRIPPER_WITH_DISTORTION_FILENAME;
      break;
    }
    break;
  }
  case vpViper850::TOOL_GENERIC_CAMERA: {
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
  case vpViper850::TOOL_CUSTOM: {
    throw vpRobotException(vpRobotException::badValue,
                           "No predefined file available for a custom tool"
                           "You should use init(vpViper850::vpToolType, const std::string&) or"
                           "init(vpViper850::vpToolType, const vpHomogeneousMatrix&) instead");
  }
  default: {
    vpERROR_TRACE("This error should not occur!");
    //       vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
    // 		   "que les specs de la classe ont ete modifiee, "
    // 		   "et que le code n'a pas ete mis a jour "
    // 		   "correctement.");
    //       vpERROR_TRACE ("Verifiez les valeurs possibles du type "
    // 		   "vpViper850::vpViper850ToolType, et controlez que "
    // 		   "tous les cas ont ete pris en compte dans la "
    // 		   "fonction init(camera).");
    break;
  }
  }

  this->init(filename_eMc);

#else  // VISP_HAVE_VIPER850_DATA

  // Use here default values of the robot constant parameters.
  switch (tool) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      erc[0] = vpMath::rad(0.07);   // rx
      erc[1] = vpMath::rad(2.76);   // ry
      erc[2] = vpMath::rad(-91.50); // rz
      etc[0] = -0.0453;             // tx
      etc[1] = 0.0005;              // ty
      etc[2] = 0.0728;              // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      erc[0] = vpMath::rad(0.26);   // rx
      erc[1] = vpMath::rad(2.12);   // ry
      erc[2] = vpMath::rad(-91.31); // rz
      etc[0] = -0.0444;             // tx
      etc[1] = -0.0005;             // ty
      etc[2] = 0.1022;              // tz
      break;
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA:
  case vpViper850::TOOL_SCHUNK_GRIPPER_CAMERA: {
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
      erc[0] = vpMath::rad(0.15);  // rx
      erc[1] = vpMath::rad(1.28);  // ry
      erc[2] = vpMath::rad(-90.8); // rz
      etc[0] = -0.0456;            // tx
      etc[1] = -0.0013;            // ty
      etc[2] = 0.001;              // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion:
      erc[0] = vpMath::rad(0.72);  // rx
      erc[1] = vpMath::rad(2.10);  // ry
      erc[2] = vpMath::rad(-90.5); // rz
      etc[0] = -0.0444;            // tx
      etc[1] = -0.0012;            // ty
      etc[2] = 0.078;              // tz
      break;
    }
    break;
  }
  case vpViper850::TOOL_GENERIC_CAMERA: {
    // Set eMc to identity
    switch (projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion:
    case vpCameraParameters::perspectiveProjWithDistortion:
      erc[0] = 0; // rx
      erc[1] = 0; // ry
      erc[2] = 0; // rz
      etc[0] = 0; // tx
      etc[1] = 0; // ty
      etc[2] = 0; // tz
      break;
    }
    break;
  }
  case vpViper850::TOOL_CUSTOM: {
    throw vpRobotException(vpRobotException::badValue,
                           "No predefined parameters available for a custom tool"
                           "You should use init(vpViper850::vpToolType, const std::string&) or"
                           "init(vpViper850::vpToolType, const vpHomogeneousMatrix&) instead");
  }
  }
  vpRotationMatrix eRc(erc);
  this->eMc.buildFrom(etc, eRc);
#endif // VISP_HAVE_VIPER850_DATA

  setToolType(tool);
  return;
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

  \sa init(vpViper850::vpToolType,
vpCameraParameters::vpCameraParametersProjType), init(vpViper850::vpToolType,
const vpHomogeneousMatrix&)
*/
void vpViper850::init(vpViper850::vpToolType tool, const std::string &filename)
{
  this->setToolType(tool);
  this->parseConfigFile(filename.c_str());
}

/*!

  Set the type of tool attached to the robot and the transformation
  between the end-effector and the tool (\f$^e{\bf M}c\f$).

  \param tool : Type of tool in use.

  \param eMc_ : Homogeneous matrix representation of the transformation
  between the end-effector frame and the tool frame.

  \sa init(vpViper850::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpViper850::vpToolType, const std::string&)

*/
void vpViper850::init(vpViper850::vpToolType tool, const vpHomogeneousMatrix &eMc_)
{
  this->setToolType(tool);
  this->set_eMc(eMc_);
}

/*!

  This function gets the robot constant parameters from a file.

  \param filename : File name containing the robot constant
  parameters, like the hand-to-eye transformation.

*/
void vpViper850::parseConfigFile(const std::string &filename)
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

    for (code = 0; NULL != opt_viper850[code]; ++code) {
      if (key.compare(opt_viper850[code]) == 0) {
        break;
      }
    }

    switch (code) {
    case 0:
      break; // Nothing to do: camera name

    case 1: {
      ss >> erc[0] >> erc[1] >> erc[2];

      // Convert rotation from degrees to radians
      erc = erc * M_PI / 180.0;
      get_erc = true;
      break;
    }

    case 2: {
      ss >> etc[0] >> etc[1] >> etc[2];
      get_etc = true;
      break;
    }

    default:
      throw(vpRobotException(vpRobotException::readingParametersError, "Bad configuration file %s line #%d",
                             filename.c_str(), lineNum));
    }
  }

  fdconfig.close();

  // Compute the eMc matrix from the translations and rotations
  if (get_etc && get_erc) {
    this->set_eMc(etc, erc);
  } else {
    throw vpRobotException(vpRobotException::readingParametersError,
                           "Could not read translation and rotation "
                           "parameters from config file %s",
                           filename.c_str());
  }
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp3/core/vpConfig.h file.

  \warning Thid method needs also an access to the files containing the
  camera parameters in XML format. This access is available if
  VISP_HAVE_VIPER850_DATA macro is defined in include/visp3/core/vpConfig.h
file.

  - If VISP_HAVE_VIPER850_DATA and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param image_width : Image width used to compute camera calibration.
  \param image_height : Image height used to compute camera calibration.

  The code below shows how to get the camera parameters of the camera
  attached to the robot.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/robot/vpViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
  vpImage<unsigned char> I(480, 640);

#ifdef VISP_HAVE_DC1394
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;
#endif

#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
#else
  vpViper850 robot;
#endif

  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  // Camera parameters are read from
  // /udd/fspindle/robot/Viper850/current/include/const_camera_Viper850.xml
  // if VISP_HAVE_VIPER850_DATA and VISP_HAVE_XML2 macros are defined
  // in vpConfig.h file
  try {
    robot.getCameraParameters (cam, I.getWidth(), I.getHeight());
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.

*/

void vpViper850::getCameraParameters(vpCameraParameters &cam, const unsigned int &image_width,
                                     const unsigned int &image_height) const
{
#if defined(VISP_HAVE_XML2) && defined(VISP_HAVE_VIPER850_DATA)
  vpXmlParserCamera parser;
  switch (getToolType()) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    std::cout << "Get camera parameters for camera \"" << vpViper850::CONST_MARLIN_F033C_CAMERA_NAME << "\""
              << std::endl
              << "from the XML file: \"" << vpViper850::CONST_CAMERA_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpViper850::CONST_CAMERA_FILENAME, vpViper850::CONST_MARLIN_F033C_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    std::cout << "Get camera parameters for camera \"" << vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME << "\""
              << std::endl
              << "from the XML file: \"" << vpViper850::CONST_CAMERA_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpViper850::CONST_CAMERA_FILENAME, vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_SCHUNK_GRIPPER_CAMERA: {
    std::cout << "Get camera parameters for camera \"" << vpViper850::CONST_SCHUNK_GRIPPER_CAMERA_NAME << "\""
              << std::endl
              << "from the XML file: \"" << vpViper850::CONST_CAMERA_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpViper850::CONST_CAMERA_FILENAME, vpViper850::CONST_SCHUNK_GRIPPER_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_GENERIC_CAMERA: {
    std::cout << "Get camera parameters for camera \"" << vpViper850::CONST_GENERIC_CAMERA_NAME << "\"" << std::endl
              << "from the XML file: \"" << vpViper850::CONST_CAMERA_FILENAME << "\"" << std::endl;
    if (parser.parse(cam, vpViper850::CONST_CAMERA_FILENAME, vpViper850::CONST_GENERIC_CAMERA_NAME, projModel,
                     image_width, image_height) != vpXmlParserCamera::SEQUENCE_OK) {
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_CUSTOM: {
    throw vpRobotException(vpRobotException::badValue, "No intrinsic parameters available for a custom tool");
  }
  default: {
    vpERROR_TRACE("This error should not occur!");
    //       vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
    //        "que les specs de la classe ont ete modifiee, "
    //        "et que le code n'a pas ete mis a jour "
    //        "correctement.");
    //       vpERROR_TRACE ("Verifiez les valeurs possibles du type "
    //        "vpViper850::vpViper850ToolType, et controlez que "
    //        "tous les cas ont ete pris en compte dans la "
    //        "fonction init(camera).");
    throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
  }
  }
#else
  // Set default parameters
  switch (getToolType()) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpViper850::CONST_MARLIN_F033C_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(1232.0, 1233.0, 317.7, 253.9);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(1214.0, 1213.0, 323.1, 240.0, -0.1824, 0.1881);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA:
  case vpViper850::TOOL_SCHUNK_GRIPPER_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(868.0, 869.0, 314.8, 254.1);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(831.3, 831.6, 322.7, 265.8, -0.1955, 0.2047);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_GENERIC_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" << vpViper850::CONST_GENERIC_CAMERA_NAME << "\""
                << std::endl;
      switch (this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion:
        cam.initPersProjWithoutDistortion(868.0, 869.0, 314.8, 254.1);
        break;
      case vpCameraParameters::perspectiveProjWithDistortion:
        cam.initPersProjWithDistortion(831.3, 831.6, 322.7, 265.8, -0.1955, 0.2047);
        break;
      }
    } else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image "
              "resolution");
      throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
    }
    break;
  }
  case vpViper850::TOOL_CUSTOM: {
    throw vpRobotException(vpRobotException::badValue, "No intrinsic parameters available for a custom tool");
  }
  default:
    vpERROR_TRACE("This error should not occur!");
    throw vpRobotException(vpRobotException::readingParametersError, "Impossible to read the camera parameters.");
  }
#endif
  return;
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp3/core/vpConfig.h file.

  \warning Thid method needs also an access to the files containing the
  camera parameters in XML format. This access is available if
  VISP_HAVE_VIPER850_DATA macro is defined in include/visp3/core/vpConfig.h
file.

  - If VISP_HAVE_VIPER850_DATA and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param I : A B&W image send by the current camera in use.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/robot/vpViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
  vpImage<unsigned char> I(480, 640);

#ifdef VISP_HAVE_DC1394
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;
#endif

#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
#else
  vpViper850 robot;
#endif

  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  try {
    robot.getCameraParameters (cam, I);
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.

*/
void vpViper850::getCameraParameters(vpCameraParameters &cam, const vpImage<unsigned char> &I) const
{
  getCameraParameters(cam, I.getWidth(), I.getHeight());
}
/*!
  \brief Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp3/core/vpConfig.h file.

  \warning Thid method needs also an access to the files containing the camera
  parameters in XML format. This access is available if
VISP_HAVE_VIPER850_DATA macro is defined in include/visp3/core/vpConfig.h
file.

  - If VISP_HAVE_VIPER850_DATA and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param I : A color image send by the current camera in use.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/robot/vpViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
  vpImage<vpRGBa> I(480, 640);

#ifdef VISP_HAVE_DC1394
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;
#endif

#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
#else
  vpViper850 robot;
#endif

  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  try {
    robot.getCameraParameters (cam, I);
  }
  catch(...) {
    std::cout << "Cannot get camera parameters for image: " << I.getWidth() << " x " << I.getHeight() << std::endl;
  }
  std::cout << "Camera parameters: " << cam << std::endl;
}
  \endcode

  \exception vpRobotException::readingParametersError : If the camera
parameters are not found.

*/

void vpViper850::getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I) const
{
  getCameraParameters(cam, I.getWidth(), I.getHeight());
}
