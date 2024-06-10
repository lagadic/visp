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
 * Motion capture using Qualisys device.
 *
*****************************************************************************/
#ifndef vpMocapQualisys_h
#define vpMocapQualisys_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_QUALISYS

#include <visp3/sensor/vpMocap.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMocapQualisys
 * \ingroup group_sensor_mocap
 * Qualisys motion capture wrapper.
*/
class VISP_EXPORT vpMocapQualisys : public vpMocap
{
public:
  vpMocapQualisys();
  virtual ~vpMocapQualisys();

  void close();
  bool connect();

  bool getBodiesPose(std::map<std::string, vpHomogeneousMatrix> &bodies_pose, bool all_bodies = false);
  bool getSpecificBodyPose(const std::string &body_name, vpHomogeneousMatrix &body_pose);
  void setServerAddress(const std::string &serverAddr);
  void setVerbose(bool verbose);

private:
  vpMocapQualisys(const vpMocapQualisys &);            // noncopyable
  vpMocapQualisys &operator=(const vpMocapQualisys &); //

  class vpMocapQualisysImpl;
  vpMocapQualisysImpl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif // vpMocapQualisys_h
