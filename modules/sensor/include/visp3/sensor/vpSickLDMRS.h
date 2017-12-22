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
 * Sick LD-MRS laser driver.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef vpSickLDMRS_h
#define vpSickLDMRS_h

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))

#include <arpa/inet.h>
#include <iostream>
#include <string.h>
#include <vector>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/sensor/vpLaserScan.h>
#include <visp3/sensor/vpLaserScanner.h>
#include <visp3/sensor/vpScanPoint.h>

/*!

  \file vpSickLDMRS.h

  \brief Driver for the Sick LD-MRS laser scanner.
*/

/*!

  \class vpSickLDMRS

  \ingroup group_sensor_laserscanner

  \brief Driver for the Sick LD-MRS laser scanner.

  \warning For the moment, this driver works only on UNIX platform.

  The code below shows how the four laser scan provided by the Sick
  LD-MRS could be acquired.

  \code
#include "visp3/sensor/vpSickLDMRS.h"

int main()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) ||
(defined(__APPLE__) && defined(__MACH__))) // UNIX std::string ip =
"131.254.12.119";

  vpSickLDMRS laser;
  laser.setIpAddress(ip);
  laser.setup();

  vpLaserScan laserscan[4];
  for ( ; ; ) {
    // Get the measured points in the four layers
    laser.measure(laserscan);

    // Prints all the measured points
    for (int layer=0; layer<4; layer++) {
      std::vector<vpScanPoint> pointsInLayer = laserscan[layer].getScanPoints(); vpScanPoint p;

      for (unsigned int i=0; i < pointsInLayer.size(); i++) {
        std::cout << pointsInLayer[i] << std::endl;
      }
    }
  }
#endif
}
  \endcode
*/
class VISP_EXPORT vpSickLDMRS : public vpLaserScanner
{
public:
  enum MagicWord {
    MagicWordC2 = 0xAFFEC0C2 ///< The magic word that allows to identify the
                             ///< messages that are sent by the Sick LD-MRS.
  };
  enum DataType {
    MeasuredData = 0x2202 ///< Flag to indicate that the body of a message
                          ///< contains measured data.
  };
  vpSickLDMRS();
  /*! Copy constructor. */
  vpSickLDMRS(const vpSickLDMRS &sick)
    : vpLaserScanner(sick), socket_fd(-1), body(NULL), vAngle(), time_offset(0), isFirstMeasure(true),
      maxlen_body(104000)
  {
    *this = sick;
  };
  virtual ~vpSickLDMRS();
  /*! Copy constructor. */
  vpSickLDMRS &operator=(const vpSickLDMRS &sick)
  {
    if (this != &sick) {
      socket_fd = sick.socket_fd;
      vAngle = sick.vAngle;
      time_offset = sick.time_offset;
      isFirstMeasure = sick.isFirstMeasure;
      maxlen_body = sick.maxlen_body;
      if (body)
        delete[] body;
      body = new unsigned char[104000];
      memcpy(body, sick.body, maxlen_body);
    }
    return (*this);
  };

  bool setup(const std::string &ip, int port);
  bool setup();
  bool measure(vpLaserScan laserscan[4]);

protected:
#if defined(_WIN32)
  SOCKET socket_fd;
#else
  int socket_fd;
#endif
  unsigned char *body;
  vpColVector vAngle; // constant vertical angle for each layer
  double time_offset;
  bool isFirstMeasure;
  size_t maxlen_body;
};

#endif

#endif
