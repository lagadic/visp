/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Interface for the ADEPT Viper 650 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpViper650_h
#define vpViper650_h

/*!

  \file vpViper650.h

  Modelisation of the ADEPT Viper 650 robot.

*/

/*!

  \class vpViper650

  \ingroup group_robot_real_arm

  \brief Modelisation of the ADEPT Viper 650 robot.

*/

#include <visp3/robot/vpViper.h>


class VISP_EXPORT vpViper650: public vpViper
{
 public:
#ifdef VISP_HAVE_ACCESS_TO_NAS
  //! Files where constant tranformation between end-effector and camera frame
  //! are stored.
  static const char * const CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME;
  static const char * const CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME;
  static const char * const CONST_EMC_SCHUNK_GRIPPER_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_SCHUNK_GRIPPER_WITH_DISTORTION_FILENAME;
  static const char * const CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME;
  static const char * const CONST_CAMERA_FILENAME;
#endif
  /*!
    Name of the camera attached to the end-effector.
  */
  static const char * const CONST_MARLIN_F033C_CAMERA_NAME;
  static const char * const CONST_PTGREY_FLEA2_CAMERA_NAME;
  static const char * const CONST_SCHUNK_GRIPPER_CAMERA_NAME;
  static const char * const CONST_GENERIC_CAMERA_NAME;

  //! List of possible tools that can be attached to the robot end-effector.
  typedef enum {
    TOOL_MARLIN_F033C_CAMERA,   /*!< Marlin F033C camera. */
    TOOL_PTGREY_FLEA2_CAMERA,   /*!< Point Grey Flea 2 camera. */
    TOOL_SCHUNK_GRIPPER_CAMERA, /*!< Camera attached to the Schunk gripper. */
    TOOL_GENERIC_CAMERA,        /*!< A generic camera. */
    TOOL_CUSTOM                 /*!< A user defined tool. */
  } vpToolType;

  //! Default tool attached to the robot end effector
  static const vpToolType defaultTool;

  vpViper650();
  virtual ~vpViper650() {};

  void init (void);
#ifdef VISP_HAVE_ACCESS_TO_NAS
  void init(const char *camera_extrinsic_parameters);
#endif
  void init (vpViper650::vpToolType tool,
	     vpCameraParameters::vpCameraParametersProjType projModel =
	     vpCameraParameters::perspectiveProjWithoutDistortion);
  void init (vpViper650::vpToolType tool,
         const std::string &filename);
  void init (vpViper650::vpToolType tool,
         const vpHomogeneousMatrix &eMc_);


  //! Get the current camera model projection type
  vpCameraParameters::vpCameraParametersProjType getCameraParametersProjType() const {
    return projModel;
  };

  void getCameraParameters(vpCameraParameters &cam,
			   const unsigned int &image_width,
               const unsigned int &image_height) const;
  void getCameraParameters(vpCameraParameters &cam,
               const vpImage<unsigned char> &I) const;
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I) const;

  //! Get the current tool type
  vpToolType getToolType() const{
    return tool_current;
  };

  void parseConfigFile (const char * filename);

 protected:
  //! Set the current tool type
  void setToolType(vpViper650::vpToolType tool){
    tool_current = tool;
  };

 protected:
  //! Current tool in use
  vpToolType tool_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;

};

#endif

