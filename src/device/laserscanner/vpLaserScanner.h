/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Generic laser scanner.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpLaserScanner_h
#define vpLaserScanner_h


#include "visp/vpConfig.h"

/*!

  \file vpLaserScanner.h

  \brief Implements a generic laser scanner.
 */

/*!

  \class vpLaserScanner

  \brief Class that defines a generic laser scanner.
 */
class VISP_EXPORT vpLaserScanner
{
 public:
  /*! Default constructor that initialize all the internal variable to zero. */
  vpLaserScanner() {
    ip = "null";
    port = 0;
  };
  /*! Copy constructor. */
  vpLaserScanner(const vpLaserScanner &scanner) {
    ip = scanner.ip;
    port = scanner.port;
  };
  /*! Default destructor that does nothing. */
  virtual ~vpLaserScanner() {};

  /*! Set the Ethernet address of the laser. */
  void setIpAddress(std::string ip) {
    this->ip = ip;
  };
  
  /*! Set the communication port. */
  void setPort(int port) {
    this->port = port;
  };
  
 protected:
  std::string ip;
  int port;
};

#endif
