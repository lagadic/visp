/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
