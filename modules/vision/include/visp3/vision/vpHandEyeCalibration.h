/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Hand-eye calibration.
 */

/*!
 * \file vpHandEyeCalibration.h
 * \brief Tools for hand-eye calibration.
 *
 * \sa The example in calibrate-hand-eye.cpp
 */
#ifndef VP_HAND_EYE_CALIBRATION_H
#define VP_HAND_EYE_CALIBRATION_H

#include <vector>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpHandEyeCalibration
 *
 * \ingroup group_vision_calib
 *
 * \brief Tool for hand-eye calibration.
 * This class is able to consider eye-in-hand and eye-to-hand configurations.
*/
class VISP_EXPORT vpHandEyeCalibration
{
public:
  /*!
   * Perform hand-eye calibration:
   * - For the eye-in hand configuration, compute the constant transformations
   *   from the end effector to the camera frames (eMc), and from the robot
   *   reference to the object frames (rMo).
   * - For the eye-to hand configuration, compute the constant transformations
   *   from the end effector to the object frames (eMo), and from the robot
   *   reference to the camera frames (rMo).
   *
   * \param[in] cMo : Vector of homogeneous matrices representing the transformation
   * between the camera and the object for the eye-in-hand configuration and
   * the inverse transformation for the eye-to-hand configuration (oMc).
   * \param[in] rMe : Vector of homogeneous matrices representing the
   * corresponding transformation between the end effector and robot reference
   * frame. Must be the same size as cMo.
   *
   * \param[out] eMc : Homogeneous matrix representing the transformation
   * between the effector and the camera in the eye-in-hand configuration and
   * between the effector and the object in the eye-to-hand configuration.
   *
   * \param[out] rMo : Homogeneous matrix representing the transformation
   * between the robot reference and the object in the eye-in-hand configuration
   * and between the robot reference and the camera in the eye-to-hand configuration.
   *
   * \return 0 if calibration succeed, -1 if the system is not full rank, 1 if the algorithm doesn't converge.
   */
  static int calibrate(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                       vpHomogeneousMatrix &eMc, vpHomogeneousMatrix &rMo);
  /*!
   * Perform hand-eye calibration:
   * - For the eye-in hand configuration, compute the constant transformations
   *   from the end effector to the camera frames (eMc), and from the robot
   *   reference to the object frames (rMo).
   * - For the eye-to hand configuration, compute the constant transformations
   *   from the end effector to the object frames (eMo), and from the robot
   *   reference to the camera frames (rMo).
   *
   * \param[in] cMo : Vector of homogeneous matrices representing the transformation
   * between the camera and the object for the eye-in-hand configuration and
   * the inverse transformation for the eye-to-hand configuration (oMc).
   * \param[in] rMe : Vector of homogeneous matrices representing the
   * corresponding transformation between the end effector and robot reference
   * frame. Must be the same size as cMo.
   *
   * \param[out] eMc : Homogeneous matrix representing the transformation
   * between the effector and the camera in the eye-in-hand configuration and
   * between the effector and the object in the eye-to-hand configuration.
   *
   * \return 0 if calibration succeed, -1 if the system is not full rank, 1 if the algorithm doesn't converge.
   */
  static int calibrate(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                       vpHomogeneousMatrix &eMc);

private:
  static void calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                                  const vpHomogeneousMatrix &eMc, vpHomogeneousMatrix &rMo);
  static void calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo,
                                  const std::vector<vpHomogeneousMatrix> &rMe, const vpHomogeneousMatrix &eMc);
  static int calibrationRotationTsai(const std::vector<vpHomogeneousMatrix> &cMo,
                                     const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationRotationTsaiOld(const std::vector<vpHomogeneousMatrix> &cMo,
                                        const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationRotationProcrustes(const std::vector<vpHomogeneousMatrix> &cMo,
                                           const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationTranslation(const std::vector<vpHomogeneousMatrix> &cMo,
                                    const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc,
                                    vpTranslationVector &eTc);
  static int calibrationTranslationOld(const std::vector<vpHomogeneousMatrix> &cMo,
                                       const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc,
                                       vpTranslationVector &eTc);
  static double calibrationErrVVS(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                                  const vpHomogeneousMatrix &eMc, vpColVector &errVVS);
  static int calibrationVVS(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                            vpHomogeneousMatrix &eMc);
};
END_VISP_NAMESPACE
#endif
