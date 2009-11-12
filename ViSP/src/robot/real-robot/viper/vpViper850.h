/****************************************************************************
 *
 * $Id: vpAfma6.h 2158 2009-05-07 07:24:51Z fspindle $
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
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

#ifndef vpViper850_h
#define vpViper850_h

/*!

  \file vpViper850.h

  Modelisation of the ADEPT Viper 850 robot.

*/

/*!

  \class vpViper850

  \ingroup Viper

  \brief Modelisation of the ADEPT Viper 850 robot. 

*/

#include <visp/vpConfig.h>
#include <visp/vpViper.h>


class VISP_EXPORT vpViper850: public vpViper
{
 public:
#ifdef VISP_HAVE_ACCESS_TO_NAS
  //! Files where constant tranformation between end-effector and camera frame
  //! are stored.
  static const char * const CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME;
  static const char * const CONST_CAMERA_FILENAME;
#endif
  /*!
    Name of the camera attached to the end-effector.
  */
  static const char * const CONST_MARLIN_F033C_CAMERA_NAME;

  //! List of possible tools that can be attached to the robot end-effector.
  typedef enum {
    TOOL_MARLIN_F033C_CAMERA   /*!< Marlin F033C camera. */
  } vpToolType;

  //! Default tool attached to the robot end effector
  static const vpToolType defaultTool;

  vpViper850();
  virtual ~vpViper850() {};

  void init ();
#ifdef VISP_HAVE_ACCESS_TO_NAS
  void init(const char *camera_extrinsic_parameters);
#endif
  void init (vpViper850::vpToolType tool,
	     vpCameraParameters::vpCameraParametersProjType projModel =
	     vpCameraParameters::perspectiveProjWithoutDistortion);


  //! Get the current camera model projection type
  vpCameraParameters::vpCameraParametersProjType getCameraParametersProjType(){
    return projModel;
  };

  void getCameraParameters(vpCameraParameters &cam,
			   const unsigned int &image_width,
			   const unsigned int &image_height);
  void getCameraParameters(vpCameraParameters &cam,
			   const vpImage<unsigned char> &I);
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I);

  //! Get the current tool type
  vpToolType getToolType(){
    return tool_current;
  };

#ifdef VISP_HAVE_ACCESS_TO_NAS
  void parseConfigFile (const char * filename);
#endif

 protected:
  //! Set the current tool type
  void setToolType(vpViper850::vpToolType tool){
    tool_current = tool;
  };

 protected:
  //! Current tool in use
  vpToolType tool_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;

};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

