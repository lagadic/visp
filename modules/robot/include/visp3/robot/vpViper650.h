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

  The model of the robot is the following:
  \image html model-viper.png Model of the Viper 650 robot.

  The non modified Denavit-Hartenberg representation of the robot is
  given in the table below, where \f$q_1^*, \ldots, q_6^*\f$
  are the variable joint positions.

  \f[
  \begin{tabular}{|c|c|c|c|c|}
  \hline
  Joint & $a_i$ & $d_i$ & $\alpha_i$ & $\theta_i$ \\
  \hline
  1 & $a_1$ & $d_1$ & $-\pi/2$ & $q_1^*$ \\
  2 & $a_2$ & 0     & 0        & $q_2^*$ \\
  3 & $a_3$ & 0     & $-\pi/2$ & $q_3^* - \pi$ \\
  4 & 0     & $d_4$ & $\pi/2$  & $q_4^*$ \\
  5 & 0     & 0     & $-\pi/2$ & $q_5^*$ \\
  6 & 0     & 0     & 0        & $q_6^*-\pi$ \\
  7 & 0     & $d_6$ & 0        & 0 \\
  \hline
  \end{tabular}
  \f]

  In this modelisation, different frames have to be considered.

  - \f$ {\cal F}_f \f$: the reference frame, also called world frame

  - \f$ {\cal F}_w \f$: the wrist frame located at the intersection of
    the last three rotations, with \f$ ^f{\bf M}_w = ^0{\bf M}_6 \f$

  - \f$ {\cal F}_e \f$: the end-effector frame located at the interface of the
    two tool changers, with \f$^f{\bf M}_e = 0{\bf M}_7 \f$

  - \f$ {\cal F}_c \f$: the camera or tool frame, with \f$^f{\bf M}_c = ^f{\bf
    M}_e \; ^e{\bf M}_c \f$ where \f$ ^e{\bf M}_c \f$ is the result of
    a calibration stage. We can also consider a custom tool TOOL_CUSTOM and
  set this tool during robot initialisation or using set_eMc().

  - \f$ {\cal F}_s \f$: the force/torque sensor frame, with \f$d7=0.0666\f$.

*/

#include <visp3/robot/vpViper.h>

class VISP_EXPORT vpViper650 : public vpViper
{
public:
#ifdef VISP_HAVE_VIPER650_DATA
  //! Files where constant tranformation between end-effector and camera frame
  //! are stored.
  static const std::string CONST_EMC_MARLIN_F033C_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_MARLIN_F033C_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_PTGREY_FLEA2_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_PTGREY_FLEA2_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_SCHUNK_GRIPPER_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_SCHUNK_GRIPPER_WITH_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GENERIC_WITHOUT_DISTORTION_FILENAME;
  static const std::string CONST_EMC_GENERIC_WITH_DISTORTION_FILENAME;
  static const std::string CONST_CAMERA_FILENAME;
#endif
  /*!
    Name of the camera attached to the end-effector.
  */
  static const char *const CONST_MARLIN_F033C_CAMERA_NAME;
  static const char *const CONST_PTGREY_FLEA2_CAMERA_NAME;
  static const char *const CONST_SCHUNK_GRIPPER_CAMERA_NAME;
  static const char *const CONST_GENERIC_CAMERA_NAME;

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
  virtual ~vpViper650(){};

  /** @name Inherited functionalities from vpViper650 */
  //@{
  void init(void);
  void init(const std::string &camera_extrinsic_parameters);
  void
  init(vpViper650::vpToolType tool,
       vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);
  void init(vpViper650::vpToolType tool, const std::string &filename);
  void init(vpViper650::vpToolType tool, const vpHomogeneousMatrix &eMc_);

  //! Get the current camera model projection type
  vpCameraParameters::vpCameraParametersProjType getCameraParametersProjType() const { return projModel; };

  void getCameraParameters(vpCameraParameters &cam, const unsigned int &image_width,
                           const unsigned int &image_height) const;
  void getCameraParameters(vpCameraParameters &cam, const vpImage<unsigned char> &I) const;
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I) const;

  //! Get the current tool type
  vpToolType getToolType() const { return tool_current; };

  void parseConfigFile(const std::string &filename);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpViper650 */
  //@{
  //! Set the current tool type
  void setToolType(vpViper650::vpToolType tool) { tool_current = tool; };
  //@}

protected:
  //! Current tool in use
  vpToolType tool_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;
};

#endif
