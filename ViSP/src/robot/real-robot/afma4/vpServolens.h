/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Interface for the Servolens lens attached to the camera fixed on the 
 * Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpServolens_h
#define __vpServolens_h

#ifdef UNIX

/*!

  \file vpServolens.h

  Interface for the Servolens lens attached to the camera fixed on the 
  Afma4 robot.

*/

#include <visp/vpConfig.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImage.h>


/*!

  \class vpServolens

  \ingroup Afma4 RobotDriver

  \brief Interface for the Servolens lens attached to the camera fixed on the 
  Afma4 robot.

  The code below shows how to manipulate this class to get and modify
  the position of the focal lens.

  \code
#include <iostream>
#include <visp/vpServolens.h>

int main()
{
  // Open the serial device to communicate with the Servolens lens
  vpServolens servolens("/dev/ttyS0"); 

  // Get the current zoom position
  unsigned zoom;
  servolens.getPosition(vpServolens::ZOOM, zoom);
  std::cout << "Actual zoom value: " << zoom << std::endl;

  // Set a new zoom value
  servolens.setPosition(vpServolens::ZOOM, zoom+1000);  
}
  \endcode

*/

class VISP_EXPORT vpServolens
{
 public:
  typedef enum {
    ZOOM = 1,
    FOCUS= 2,
    IRIS = 3
  } vpServoType;
  typedef enum {
    ZOOM_MAX  = 10000,	// Valeur maxi zoom (mm/100)
    ZOOM_MIN  = 1000,	// Valeur mini zoom (mm/100)
    FOCUS_MAX = 1500,	// Valeur maxi focus (metres/100)
    FOCUS_MIN = 100,	// Valeur mini focus (metres/100)
    IRIS_MAX  = 1000,	// Valeur maxi diaph (ouverture/100)
    IRIS_MIN  = 160	// Valeur mini disph (ouverture/100)
  } vpLimitsType;
  typedef enum {
    AUTO       = 1,
    CONTROLLED = 2,
    RELEASED   = 3    
  } vpControllerType;

  vpServolens();
  vpServolens(const char *port);
  ~vpServolens();

  void open(const char *port="/dev/ttyS0");
  void close();
  void reset();

  void setController(vpControllerType controller);
  void setAutoIris(bool enable);
  void setPosition(vpServoType servo, unsigned position);
  bool getPosition(vpServoType servo, unsigned &position);
  vpCameraParameters getCameraParameters(vpImage<unsigned char> &I);

  void enablePrompt(bool active);

 private:

  void init();

  void enableCmdComplete(vpServoType servo, bool active);

  char wait();
  void wait(vpServoType servo);

  bool read(char *c, long timeout_s);
  void write(const char *s);

  bool clean(const char *in, char *out);

  int remfd; // file pointer of the host's tty
  bool isinit;
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
#endif
#endif

