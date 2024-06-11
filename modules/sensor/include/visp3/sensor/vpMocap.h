/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Generic motion capture.
 *
*****************************************************************************/
#ifndef vpMocap_h
#define vpMocap_h

#include <map>
#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMocap
 * \ingroup group_sensor_mocap
 * Generic motion capture wrapper.
*/
class VISP_EXPORT vpMocap
{
public:
  /*!
   * Default constructor that turns off the verbose mode.
   */
  vpMocap() : m_verbose(false), m_serverAddr() { };
  /*!
   * Destructor.
   */
  virtual ~vpMocap() { };

  /*!
   * Close connexion with the motion capture device.
   */
  virtual void close() = 0;

  /*!
   * Connect to the motion capture device which address is specified using setServerAddress().
   *
   * \sa close(), setServerAddress()
   */
  virtual bool connect() = 0;

  /*!
   * Get the bodies pose.
   * \param[out] bodies_pose : Map of bodies pose.
   * \param[in] all_bodies : When true consider all the bodies, even those that are not visible and that may contain nan
   * in their pose. \return true when success, false otherwise.
   */
  virtual bool getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies = false) = 0;

  /*!
   * Get the pose of a specific body.
   * \param[in] body_name : Name of the body to consider.
   * \param[out] body_pose : Body pose.
   * \return true when success, false otherwise.
   */
  virtual bool getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose) = 0;

  /*!
   * Set mocap server address.
   * \param[in] serverAddr : Server address.
   *
   * \sa connect()
   */
  virtual inline void setServerAddress(const std::string &serverAddr) { m_serverAddr = serverAddr; }

  /*!
   * Enable or disable verbose mode.
   * \param[in] verbose : When true enable verbose mode, otherwise disable verbose mode.
   */
  virtual inline void setVerbose(bool verbose) { m_verbose = verbose; }

protected:
  bool m_verbose;
  std::string m_serverAddr;
};
END_VISP_NAMESPACE
#endif // vpMocap_h
