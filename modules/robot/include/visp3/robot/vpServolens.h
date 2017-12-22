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
 * Interface for the Servolens lens attached to the camera fixed on the
 * Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpServolens_h
#define __vpServolens_h

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

/*!

  \file vpServolens.h

  Interface for the Servolens lens attached to the camera fixed on the
  Afma4 robot.

*/

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImage.h>

/*!

  \class vpServolens

  \ingroup group_robot_real_cylindrical

  \brief Interface for the Servolens lens attached to the camera fixed on the
  Afma4 robot.

  The code below shows how to manipulate this class to get and modify
  the position of the focal lens.

  \code
#include <iostream>
#include <visp3/vs/vpServolens.h>

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
  typedef enum { ZOOM = 1, FOCUS = 2, IRIS = 3 } vpServoType;
  typedef enum {
    ZOOM_MAX = 10000, // Valeur maxi zoom (mm/100)
    ZOOM_MIN = 1000,  // Valeur mini zoom (mm/100)
    FOCUS_MAX = 1500, // Valeur maxi focus (metres/100)
    FOCUS_MIN = 100,  // Valeur mini focus (metres/100)
    IRIS_MAX = 1000,  // Valeur maxi diaph (ouverture/100)
    IRIS_MIN = 160    // Valeur mini disph (ouverture/100)
  } vpLimitsType;
  typedef enum { AUTO = 1, CONTROLLED = 2, RELEASED = 3 } vpControllerType;

  vpServolens();
  explicit vpServolens(const char *port);
  ~vpServolens();

  void open(const char *port = "/dev/ttyS0");
  void close();
  void reset() const;

  void setController(vpControllerType controller) const;
  void setAutoIris(bool enable) const;
  void setPosition(vpServoType servo, unsigned int position) const;
  bool getPosition(vpServoType servo, unsigned int &position) const;
  vpCameraParameters getCameraParameters(vpImage<unsigned char> &I) const;

  void enablePrompt(bool active) const;

private:
  void init() const;

  char wait() const;
  void wait(vpServoType servo) const;

  bool read(char *c, long timeout_s) const;
  void write(const char *s) const;

  bool clean(const char *in, char *out) const;

  int remfd; // file pointer of the host's tty
  bool isinit;
};

#endif
#endif
