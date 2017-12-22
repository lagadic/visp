/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef __vpAfma6_h
#define __vpAfma6_h

/*!

  \file vpAfma6.h

  Modelisation of Irisa's gantry robot named Afma6.

*/

/*!

  \class vpAfma6

  \ingroup group_robot_real_gantry group_robot_simu_gantry

  \brief Modelisation of Irisa's gantry robot named Afma6.

  In this modelisation, different frames have to be considered.

  - \f$ {\cal F}_f \f$: the reference frame, also called world frame

  - \f$ {\cal F}_e \f$: the end-effector frame located at the intersection of
  the 3 rotations.

  - \f$ {\cal F}_c \f$: the camera or tool frame, with \f$^f{\bf M}_c = ^f{\bf
    M}_e \; ^e{\bf M}_c \f$ where \f$ ^e{\bf M}_c \f$ is the result of
    a calibration stage. We can also consider a custom tool TOOL_CUSTOM and
  set this tool during robot initialisation or using set_eMc().

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

class VISP_EXPORT vpAfma6
{
public:
#ifdef VISP_HAVE_AFMA6_DATA
  //! File where constant parameters in relation with the robot are stored:
  //! joint max, min, coupling factor between 4 ant 5 joint, distance between
  //! 5 and 6 joint, tranformation eMc between end-effector and camera frame.
  static const std::string CONST_AFMA6_FILENAME;
  static const std::string CONST_EMC_CCMOP_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_CCMOP_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GRIPPER_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GRIPPER_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_VACUUM_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_VACUUM_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME;
  static const std::string CONST_CAMERA_AFMA6_FILENAME;
#endif
  /*!
    Name of the camera attached to the CCMOP tool
    (vpAfma6ToolType::TOOL_CCMOP).
  */
  static const char *const CONST_CCMOP_CAMERA_NAME;
  /*!
    Name of the camera attached to the 2 fingers gripper tool
    (vpAfma6ToolType::TOOL_GRIPPER).
  */
  static const char *const CONST_GRIPPER_CAMERA_NAME;
  /*!
    Name of the camera attached to the vacuum gripper tool
    (vpAfma6ToolType::TOOL_VACUUM).
  */
  static const char *const CONST_VACUUM_CAMERA_NAME;
  /*!
    Name of the generic camera attached to the robot hand
    (vpAfma6ToolType::TOOL_GENERIC_CAMERA).
  */
  static const char *const CONST_GENERIC_CAMERA_NAME;

  //! List of possible tools that can be attached to the robot end-effector.
  typedef enum {
    TOOL_CCMOP,          /*!< Pneumatic CCMOP gripper. */
    TOOL_GRIPPER,        /*!< Pneumatic gripper with 2 fingers. */
    TOOL_VACUUM,         /*!< Pneumatic vaccum gripper. */
    TOOL_GENERIC_CAMERA, /*!< A generic camera. */
    TOOL_CUSTOM          /*!< A user defined tool. */
  } vpAfma6ToolType;

  //! Default tool attached to the robot end effector
  static const vpAfma6ToolType defaultTool;

public:
  vpAfma6();
  /*! Destructor that does nothing. */
  virtual ~vpAfma6(){};

  /** @name Inherited functionalities from vpAfma6 */
  //@{
  void init(void);
  void init(const std::string &camera_extrinsic_parameters);
  void init(const std::string &camera_extrinsic_parameters, const std::string &camera_intrinsic_parameters);
  void init(vpAfma6::vpAfma6ToolType tool, const std::string &filename);
  void init(vpAfma6::vpAfma6ToolType tool, const vpHomogeneousMatrix &eMc_);
  void
  init(vpAfma6::vpAfma6ToolType tool,
       vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);

  vpHomogeneousMatrix getForwardKinematics(const vpColVector &q) const;
  int getInverseKinematics(const vpHomogeneousMatrix &fMc, vpColVector &q, const bool &nearest = true,
                           const bool &verbose = false) const;

  vpHomogeneousMatrix get_eMc() const;
  vpHomogeneousMatrix get_fMc(const vpColVector &q) const;
  void get_fMe(const vpColVector &q, vpHomogeneousMatrix &fMe) const;
  void get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const;

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_eJe(const vpColVector &q, vpMatrix &eJe) const;
  void get_fJe(const vpColVector &q, vpMatrix &fJe) const;

  //! Get the current tool type
  vpAfma6ToolType getToolType() const { return tool_current; };
  //! Get the current camera model projection type
  vpCameraParameters::vpCameraParametersProjType getCameraParametersProjType() const { return projModel; };

  void getCameraParameters(vpCameraParameters &cam, const unsigned int &image_width,
                           const unsigned int &image_height) const;
  void getCameraParameters(vpCameraParameters &cam, const vpImage<unsigned char> &I) const;
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I) const;

  vpColVector getJointMin() const;
  vpColVector getJointMax() const;
  double getCoupl56() const;
  double getLong56() const;

  void parseConfigFile(const std::string &filename);

  virtual void set_eMc(const vpHomogeneousMatrix &eMc);
  //@}

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAfma6 &afma6);

protected:
  /** @name Protected Member Functions Inherited from vpAfma6 */
  //@{
  //! Set the current tool type
  void setToolType(vpAfma6::vpAfma6ToolType tool) { tool_current = tool; };
  //@}

public:
  static const unsigned int njoint; ///< Number of joint.

protected:
  double _coupl_56;     // coupling between join 5 and 6
  double _long_56;      // distance between join 5 and 6
  double _joint_max[6]; // Maximal value of the joints
  double _joint_min[6]; // Minimal value of the joints
  // Minimal representation of _eMc
  vpTranslationVector _etc; // meters
  vpRxyzVector _erc;        // radian

  vpHomogeneousMatrix _eMc; // Camera extrinsic parameters: effector to camera

protected:
  //! Current tool in use
  vpAfma6ToolType tool_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;
};

#endif
