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
 * Description: Factory class used to create vpPylonGrabber instances.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

/*!
  \file vpPylonFactory.cpp
  \brief Implementation file for factory class used to create
  vpPylonGrabber instances.
*/

#include <visp3/sensor/vpPylonFactory.h>

#ifdef VISP_HAVE_PYLON

#include "vpPylonGrabberGigE.h"
#include "vpPylonGrabberUsb.h"

/*!
  \brief Get the vpPylonFactory singleton.
 */
vpPylonFactory &vpPylonFactory::instance()
{
  static vpPylonFactory instance;

  return instance;
}

/*!
  \brief Create an object of vpPylonGrabber.

  \param  dev_class The device class. See vpPylonFactory::DeviceClass
  for valid values.
  \return The pointer towards the vpPylonGrabber object. It's the
  caller's responsibility to destroy the object. NULL pointer will be
  returned if requested object can't be properly created.
 */
vpPylonGrabber *vpPylonFactory::createPylonGrabber(DeviceClass dev_class)
{
  switch (dev_class) {
  case BASLER_GIGE:
    return new vpPylonGrabberGigE();
    break;
  case BASLER_USB:
    return new vpPylonGrabberUsb();
    break;
  default:
    return NULL;
    break;
  }
}

#else
// Work arround to avoid warning:
// libvisp_pylon.a(vpPylonFactory.cpp.o) has no symbols
void dummy_vpPylonFactory(){};
#endif // #ifdef VISP_HAVE_PYLON
