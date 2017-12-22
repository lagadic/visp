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
 * Description: Factory class used to create vpPylonGrabber instances.
 *
 * Authors:
 * Wenfeng CAI
 *
 *****************************************************************************/

#ifndef __vpPylonFactory_h_
#define __vpPylonFactory_h_

#include <visp3/core/vpConfig.h>

#include <visp3/sensor/vpPylonGrabber.h>

#ifdef VISP_HAVE_PYLON

/*!
  \file vpPylonFactory.h
  \brief Description: Factory class used to create vpPylonGrabber
  instances.
*/

/*!
  \brief Factory singleton class to create vpPylonGrabber subclass
  instances.

  \ingroup group_sensor_camera

  Use vpPylonFactory::instance() to get the singleton instance. This
  class can also help to initialize and terminate pylon runtime system.

  Example code.
  \code
  vpPylonFactory &factory = vpPylonFactory::instance();
  vpPylonGrabber *g = factory.createPylonGrabber(vpPylonFactory::BASLER_GIGE);
  \endcode
 */
class VISP_EXPORT vpPylonFactory
{
public:
  static vpPylonFactory &instance();

  /*! Device class of cameras.
   */
  enum DeviceClass {
    BASLER_GIGE, //!< Basler GigE camera.
    BASLER_USB   //!< Basler USB camera.
  };

  vpPylonGrabber *createPylonGrabber(DeviceClass dev_class);

private:
  //! Default constructor.
  vpPylonFactory(){};
  vpPylonFactory(vpPylonFactory const &);
  void operator=(vpPylonFactory const &);

  Pylon::PylonAutoInitTerm m_autoInitTerm; //!< Auto initialize and terminate object for pylon SDK.
};

#endif // #ifdef VISP_HAVE_PYLON
#endif // #ifndef __vpPylonFactory_h_
