/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpAfma6_h
#define __vpAfma6_h

/*!

  \file vpAfma6.h

  Modelisation of Irisa's gantry robot named Afma6.

*/

/*!

  \class vpAfma6

  \ingroup Afma6

  \brief Modelisation of Irisa's gantry robot named Afma6.

*/

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpVelocityTwistMatrix.h>


class VISP_EXPORT vpAfma6
{
 public:
#ifdef VISP_HAVE_ACCESS_TO_NAS
  //! File where constant parameters in relation with the robot are stored:
  //! joint max, min, coupling factor between 4 ant 5 joint, distance between 5
  //! and 6 joint, tranformation eMc between end-effector and camera frame.
  static const char * const CONST_AFMA6_FILENAME;
  static const char * const CONST_EMC_CCMOP_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_CCMOP_WITH_DISTORTION_FILENAME;
  static const char * const CONST_EMC_GRIPPER_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_GRIPPER_WITH_DISTORTION_FILENAME;
  static const char * const CONST_EMC_VACUUM_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_VACUUM_WITH_DISTORTION_FILENAME;
  static const char * const CONST_CAMERA_AFMA6_FILENAME;
#endif
  /*!
    Name of the camera attached to the CCMOP tool (vpAfma6ToolType::TOOL_CCMOP).
  */
  static const char * const CONST_CCMOP_CAMERA_NAME;
  /*!
    Name of the camera attached to the 2 fingers gripper tool
    (vpAfma6ToolType::TOOL_GRIPPER).
  */
  static const char * const CONST_GRIPPER_CAMERA_NAME;
  /*!
    Name of the camera attached to the vacuum gripper tool
    (vpAfma6ToolType::TOOL_VACUUM).
  */
  static const char * const CONST_VACUUM_CAMERA_NAME;

  //! List of possible tools that can be attached to the robot end-effector.
  typedef enum
    {
      TOOL_CCMOP,   /*!< Pneumatic CCMOP gripper. */
      TOOL_GRIPPER, /*!< Pneumatic gripper with 2 fingers. */
      TOOL_VACUUM   /*!< Pneumatic vaccum gripper. */
    } vpAfma6ToolType;

  //! Default tool attached to the robot end effector
  static const vpAfma6ToolType defaultTool;

 public:
  vpAfma6();

  void init (void);
#ifdef VISP_HAVE_ACCESS_TO_NAS
  void init (const char * paramAfma6, const char * paramCamera);
#endif
  void init (vpAfma6::vpAfma6ToolType tool,
	     vpCameraParameters::vpCameraParametersProjType projModel =
	     vpCameraParameters::perspectiveProjWithoutDistortion);

  vpHomogeneousMatrix getForwardKinematics(const vpColVector & q);
  int getInverseKinematics(const vpHomogeneousMatrix & fMc,
			   vpColVector & q, const bool &nearest=true);
  vpHomogeneousMatrix get_fMc (const vpColVector & q);
  void get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe);
  void get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc);

  void get_cMe(vpHomogeneousMatrix &cMe) ;
  void get_cVe(vpVelocityTwistMatrix &cVe) ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe)  ;
  void get_fJe(const vpColVector &q, vpMatrix &fJe)  ;

#ifdef VISP_HAVE_ACCESS_TO_NAS
  void parseConfigFile (const char * filename);
#endif

  //! Get the current tool type
  vpAfma6ToolType getToolType(){
    return tool_current;
  };
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

  friend VISP_EXPORT std::ostream & operator << (std::ostream & os,
						 const vpAfma6 & afma6);

  vpColVector getJointMin();
  vpColVector getJointMax();
  double getCoupl56();
  double getLong56();

 protected:
  //! Set the current tool type
  void setToolType(vpAfma6::vpAfma6ToolType tool){
    tool_current = tool;
  };

 public:

  static const unsigned int njoint; ///< Number of joint.


 protected:
  double _coupl_56; // coupling between join 5 and 6
  double _long_56;  // distance between join 5 and 6
  double _joint_max[6]; // Maximal value of the joints
  double _joint_min[6]; // Minimal value of the joints
  // Minimal representation of _eMc
  vpTranslationVector _etc; // meters
  vpRxyzVector        _erc; // radian

  vpHomogeneousMatrix _eMc; // Camera extrinsic parameters: effector to camera

 protected:
  //! Current tool in use
  vpAfma6ToolType tool_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;

};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

