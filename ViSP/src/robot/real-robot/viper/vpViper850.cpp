/****************************************************************************
 *
 * $Id: vpAfma6.cpp 2158 2009-05-07 07:24:51Z fspindle $
 *
 * Copyright (C) 1998-2009 Inria. All rights reserved.
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


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpViper850.h>
#include <visp/vpMath.h>
#include <visp/vpXmlParserCamera.h>

#ifdef VISP_HAVE_ACCESS_TO_NAS
static const char *opt_viper850[] = {"CAMERA", "eMc_ROT_XYZ","eMc_TRANS_XYZ",
				     NULL};

const char * const vpViper850::CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME
#ifdef WIN32
= "Z:/robot/Viper850/current/include/const_eMc_MarlinF033C_without_distortion_Viper850.cnf";
#else
= "/udd/fspindle/robot/Viper850/current/include/const_eMc_MarlinF033C_without_distortion_Viper850.cnf";
#endif

const char * const vpViper850::CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME
#ifdef WIN32 
= "Z:/robot/Viper850/current/include/const_eMc_MarlinF033C_with_distortion_Viper850.cnf";
#else
= "/udd/fspindle/robot/Viper850/current/include/const_eMc_MarlinF033C_with_distortion_Viper850.cnf";
#endif

const char * const vpViper850::CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME
#ifdef WIN32
= "Z:/robot/Viper850/current/include/const_eMc_PTGreyFlea2_without_distortion_Viper850.cnf";
#else
= "/udd/fspindle/robot/Viper850/current/include/const_eMc_PTGreyFlea2_without_distortion_Viper850.cnf";
#endif

const char * const vpViper850::CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME
#ifdef WIN32 
= "Z:/robot/Viper850/current/include/const_eMc_PTGreyFlea2_with_distortion_Viper850.cnf";
#else
= "/udd/fspindle/robot/Viper850/current/include/const_eMc_PTGreyFlea2_with_distortion_Viper850.cnf";
#endif


const char * const vpViper850::CONST_CAMERA_FILENAME
#ifdef WIN32 
= "Z:/robot/Viper850/current/include/const_camera_Viper850.xml";
#else
= "/udd/fspindle/robot/Viper850/current/include/const_camera_Viper850.xml";
#endif


#endif // VISP_HAVE_ACCESS_TO_NAS

const char * const vpViper850::CONST_MARLIN_F033C_CAMERA_NAME = "Marlin-F033C-12mm";
const char * const vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME = "PTGrey-Flea2-6mm";

const vpViper850::vpToolType vpViper850::defaultTool = vpViper850::TOOL_PTGREY_FLEA2_CAMERA;



/*!

  Default constructor.
  Sets the specific parameters like the Denavit Hartenberg parameters.

*/
vpViper850::vpViper850()
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
void
vpViper850::init (void)
{
  this->init(vpViper850::defaultTool);
  return;
}

/*!

  Read files containing the constant parameters related to the robot
  tools in order to set the end-effector to camera transformation.

  \warning This function is only available if the macro
  VISP_HAVE_ACCESS_TO_NAS is defined in vpConfig.h.

  \param camera_extrinsic_parameters : Filename containing the camera
  extrinsic parameters.

*/
#ifdef VISP_HAVE_ACCESS_TO_NAS
void
vpViper850::init (const char *camera_extrinsic_parameters)
{
  //vpTRACE ("Parse camera file \""%s\"".", camera_filename);
  this->parseConfigFile (camera_extrinsic_parameters);

  return ;
}
#endif

/*!

  Set the constant parameters related to the robot kinematics and to
  the end-effector to camera transformation (\f$^e{\bf M}c\f$)
  correponding to the camera extrinsic parameters. These last
  parameters depend on the camera and projection model in use.

  \param tool : Camera in use.

  \param projModel : Projection model of the camera.

*/
void
vpViper850::init (vpViper850::vpToolType tool,
		  vpCameraParameters::vpCameraParametersProjType projModel)
{
  
  this->projModel = projModel;
  
#ifdef VISP_HAVE_ACCESS_TO_NAS
  // Read the robot parameters from files
  char filename_eMc [FILENAME_MAX];
  switch (tool) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    switch(projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion :
#ifdef UNIX
      snprintf(filename_eMc, FILENAME_MAX, "%s",
	       CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME);
#else // WIN32
      _snprintf(filename_eMc, FILENAME_MAX, "%s",
		CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME);
#endif
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
#ifdef UNIX
      snprintf(filename_eMc, FILENAME_MAX, "%s",
	       CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME);
#else // WIN32
      _snprintf(filename_eMc, FILENAME_MAX, "%s",
		CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME);
#endif
      break;
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    switch(projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion :
#ifdef UNIX
      snprintf(filename_eMc, FILENAME_MAX, "%s",
	       CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME);
#else // WIN32
      _snprintf(filename_eMc, FILENAME_MAX, "%s",
		CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME);
#endif
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
#ifdef UNIX
      snprintf(filename_eMc, FILENAME_MAX, "%s",
	       CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME);
#else // WIN32
      _snprintf(filename_eMc, FILENAME_MAX, "%s",
		CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME);
#endif
      break;
    }
    break;
  }
  default: {
    vpERROR_TRACE ("This error should not occur!");
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
  
  this->init (filename_eMc);

#else // VISP_HAVE_ACCESS_TO_NAS

  // Use here default values of the robot constant parameters.
  switch (tool) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    switch(projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      erc[0] = vpMath::rad(0.07); // rx
      erc[1] = vpMath::rad(2.76); // ry
      erc[2] = vpMath::rad(-91.50); // rz
      etc[0] = -0.0453; // tx
      etc[1] =  0.0005; // ty
      etc[2] =  0.0728; // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      erc[0] = vpMath::rad(0.26); // rx
      erc[1] = vpMath::rad(2.12); // ry
      erc[2] = vpMath::rad(-91.31); // rz
      etc[0] = -0.0444; // tx
      etc[1] = -0.0005; // ty
      etc[2] =  0.1022; // tz
      break;
    }
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    switch(projModel) {
    case vpCameraParameters::perspectiveProjWithoutDistortion :
      erc[0] = vpMath::rad(0.07); // rx
      erc[1] = vpMath::rad(2.76); // ry
      erc[2] = vpMath::rad(-91.50); // rz
      etc[0] = -0.0453; // tx
      etc[1] =  0.0005; // ty
      etc[2] =  0.0728; // tz
      break;
    case vpCameraParameters::perspectiveProjWithDistortion :
      erc[0] = vpMath::rad(0.26); // rx
      erc[1] = vpMath::rad(2.12); // ry
      erc[2] = vpMath::rad(-91.31); // rz
      etc[0] = -0.0444; // tx
      etc[1] = -0.0005; // ty
      etc[2] =  0.1022; // tz
      break;
    }
  }
  }
  vpRotationMatrix eRc(erc);
  this->eMc.buildFrom(etc, eRc);
#endif // VISP_HAVE_ACCESS_TO_NAS

  setToolType(tool);
  return ;
}

/*!

  This function gets the robot constant parameters from a file.

  \warning This function is only available if the macro
  VISP_HAVE_ACCESS_TO_NAS is defined in vpConfig.h.

  \param filename : File name containing the robot constant
  parameters, like the hand-to-eye transformation.

*/
#ifdef VISP_HAVE_ACCESS_TO_NAS
void
vpViper850::parseConfigFile (const char * filename)
{
  int               dim;
  int               code;
  char              Ligne[FILENAME_MAX];
  char              namoption[100];
  FILE *            fdtask;
  int               numLn = 0;
  double rot_eMc[3]; // rotation
  double trans_eMc[3]; // translation
  bool get_rot_eMc = false;
  bool get_trans_eMc = false;

  //vpTRACE("Read the config file for constant parameters %s.", filename);
  if ((fdtask = fopen(filename, "r" )) == NULL)
  {
    vpERROR_TRACE ("Impossible to read the config file %s.",
		 filename);
    fclose(fdtask);
    throw vpRobotException (vpRobotException::readingParametersError,
			    "Impossible to read the config file.");
  }

  while (fgets(Ligne, FILENAME_MAX, fdtask) != NULL) {
    numLn ++;
    if ('#' == Ligne[0]) { continue; }
    sscanf(Ligne, "%s", namoption);
    dim = strlen(namoption);

    for (code = 0;
	 NULL != opt_viper850[code];
	 ++ code)
    {
      if (strncmp(opt_viper850[code], namoption, dim) == 0)
      {
	break;
      }
    }

    switch(code) {
    case 0:
      break; // Nothing to do: camera name

    case 1:
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &rot_eMc[0],
	     &rot_eMc[1],
	     &rot_eMc[2]);

      // Convert rotation from degrees to radians
      rot_eMc[0] *= M_PI / 180.0;
      rot_eMc[1] *= M_PI / 180.0;
      rot_eMc[2] *= M_PI / 180.0;
      get_rot_eMc = true;
      break;

    case 2:
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &trans_eMc[0],
	     &trans_eMc[1],
	     &trans_eMc[2]);
      get_trans_eMc = true;
      break;

    default:
      vpERROR_TRACE ("Bad configuration file %s  "
		     "ligne #%d.", filename, numLn);
    } /* SWITCH */
  } /* WHILE */

  fclose (fdtask);

  // Compute the eMc matrix from the translations and rotations
  if (get_rot_eMc && get_trans_eMc) {
    for (int i=0; i < 3; i ++) {
      erc[i] = rot_eMc[i];
      etc[i] = trans_eMc[i];
    }

    vpRotationMatrix eRc(erc);
    this->eMc.buildFrom(etc, eRc);
  }

  return;
}
#endif


/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp/vpConfig.h file.

  \warning Thid method needs also an access to the file located on
  Inria's NAS server and containing the camera parameters in XML
  format. This access is available if VISP_HAVE_ACCESS_TO_NAS macro is
  defined in include/visp/vpConfig.h file.

  - If VISP_HAVE_ACCESS_TO_NAS and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from
  /udd/fspindle/robot/Viper850/current/include/const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param image_width : Image width used to compute camera calibration.
  \param image_height : Image height used to compute camera calibration.

  The code below shows how to get the camera parameters of the camera
  attached to the robot.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpViper850.h>
#include <visp/vpRobotViper850.h>

int main()
{
  vpImage<unsigned char> I(480, 640);

#ifdef VISP_HAVE_DC1394_2
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
  // if VISP_HAVE_ACCESS_TO_NAS and VISP_HAVE_XML2 macros are defined
  // in vpConfig.h file
  robot.getCameraParameters (cam, I.getWidth(), I.getHeight());
}
  \endcode
*/

void
vpViper850::getCameraParameters (vpCameraParameters &cam,
				 const unsigned int &image_width,
				 const unsigned int &image_height)
{
#if defined(VISP_HAVE_XML2) && defined (VISP_HAVE_ACCESS_TO_NAS)
  vpXmlParserCamera parser;
  switch (getToolType()) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    std::cout << "Get camera parameters for camera \"" 
	      << vpViper850::CONST_MARLIN_F033C_CAMERA_NAME << "\"" << std::endl
	      << "from the XML file: \"" 
	      << vpViper850::CONST_CAMERA_FILENAME << "\""<< std::endl;
    parser.parse(cam,
		 vpViper850::CONST_CAMERA_FILENAME,
		 vpViper850::CONST_MARLIN_F033C_CAMERA_NAME,
		 projModel,
		 image_width, image_height);
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    std::cout << "Get camera parameters for camera \"" 
	      << vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME << "\"" << std::endl
	      << "from the XML file: \"" 
	      << vpViper850::CONST_CAMERA_FILENAME << "\""<< std::endl;
    parser.parse(cam,
		 vpViper850::CONST_CAMERA_FILENAME,
		 vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME,
		 projModel,
		 image_width, image_height);
    break;
  }
  default: {
    vpERROR_TRACE ("This error should not occur!");
//       vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
//        "que les specs de la classe ont ete modifiee, "
//        "et que le code n'a pas ete mis a jour "
//        "correctement.");
//       vpERROR_TRACE ("Verifiez les valeurs possibles du type "
//        "vpViper850::vpViper850ToolType, et controlez que "
//        "tous les cas ont ete pris en compte dans la "
//        "fonction init(camera).");
      break;
    }
  }
#else
  // Set default parameters
  switch (getToolType()) {
  case vpViper850::TOOL_MARLIN_F033C_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" 
		<< vpViper850::CONST_MARLIN_F033C_CAMERA_NAME << "\"" << std::endl;
      switch(this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion :
	cam.initPersProjWithoutDistortion(1232.0, 1233.0, 317.7, 253.9);
	break;
      case vpCameraParameters::perspectiveProjWithDistortion :
	cam.initPersProjWithDistortion(1214.0, 1213.0, 323.1, 240.0, -0.1824, 0.1881);
	break;
      }
    }
    else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image resolution");
    }
    break;
  }
  case vpViper850::TOOL_PTGREY_FLEA2_CAMERA: {
    // Set default intrinsic camera parameters for 640x480 images
    if (image_width == 640 && image_height == 480) {
      std::cout << "Get default camera parameters for camera \"" 
		<< vpViper850::CONST_PTGREY_FLEA2_CAMERA_NAME << "\"" << std::endl;
      switch(this->projModel) {
      case vpCameraParameters::perspectiveProjWithoutDistortion :
	cam.initPersProjWithoutDistortion(1232.0, 1233.0, 317.7, 253.9);
	break;
      case vpCameraParameters::perspectiveProjWithDistortion :
	cam.initPersProjWithDistortion(1214.0, 1213.0, 323.1, 240.0, -0.1824, 0.1881);
	break;
      }
    }
    else {
      vpTRACE("Cannot get default intrinsic camera parameters for this image resolution");
    }
    break;
  }
  default: 
    vpERROR_TRACE ("This error should not occur!");
    break;
  }
#endif
  return;
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp/vpConfig.h file.

  \warning Thid method needs also an access to the file located on
  Inria's NAS server and containing the camera parameters in XML
  format. This access is available if VISP_HAVE_ACCESS_TO_NAS macro is
  defined in include/visp/vpConfig.h file.

  - If VISP_HAVE_ACCESS_TO_NAS and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from
  /udd/fspindle/robot/Viper850/current/include/const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param I : A B&W image send by the current camera in use.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpViper850.h>
#include <visp/vpRobotViper850.h>

int main()
{
  vpImage<unsigned char> I(480, 640);

#ifdef VISP_HAVE_DC1394_2
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
  robot.getCameraParameters (cam, I);
}
  \endcode

*/
void
vpViper850::getCameraParameters (vpCameraParameters &cam,
			      const vpImage<unsigned char> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}
/*!
  \brief Get the current intrinsic camera parameters obtained by calibration.

    \warning This method needs XML library to parse the file defined in
  vpViper850::CONST_CAMERA_FILENAME and containing the camera
  parameters. If XML is detected by ViSP, VISP_HAVE_XML2 macro is
  defined in include/visp/vpConfig.h file.

  \warning Thid method needs also an access to the file located on
  Inria's NAS server and containing the camera parameters in XML
  format. This access is available if VISP_HAVE_ACCESS_TO_NAS macro is
  defined in include/visp/vpConfig.h file.

  - If VISP_HAVE_ACCESS_TO_NAS and VISP_HAVE_XML2 macros are defined,
  this method gets the camera parameters from
  /udd/fspindle/robot/Viper850/current/include/const_camera_Viper850.xml
  config file.

  - If these two macros are not defined, this method set the camera parameters
  to default one.

  \param cam : In output, camera parameters to fill.
  \param I : A color image send by the current camera in use.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpViper850.h>
#include <visp/vpRobotViper850.h>

int main()
{
  vpImage<vpRGBa> I(480, 640);

#ifdef VISP_HAVE_DC1394_2
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
  robot.getCameraParameters (cam, I);
}
  \endcode
*/

void
vpViper850::getCameraParameters (vpCameraParameters &cam,
				 const vpImage<vpRGBa> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
