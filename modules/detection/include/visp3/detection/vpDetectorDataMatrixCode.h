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
 * Description:
 * Base class for bar code detection.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef _vpDetectorDataMatrixCode_h_
#define _vpDetectorDataMatrixCode_h_

#include <string>
#include <utility>
#include <vector>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_DMTX

#include <visp3/core/vpImage.h>
#include <visp3/detection/vpDetectorBase.h>

/*!
  \class vpDetectorDataMatrixCode
  \ingroup group_detection_barcode
  Base class for bar code detector. This class is a wrapper over libdmtx
  available from http://www.libdmtx.org. Installation instructions are
  provided here https://visp.inria.fr/3rd_dmtx.

  The detect() function allows to detect multiple QR codes in an image. Once
  detected, for each QR code it is possible to retrieve the location of the
  corners using getPolygon(), the encoded message using getMessage(), the
  bounding box using getBBox() and the center of gravity using getCog().

  The following sample code shows how to use this class to detect QR codes in
  an image.
\code
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_DMTX
  vpImage<unsigned char> I;
  vpImageIo::read(I, "bar-code.pgm");

  vpDetectorDataMatrixCode detector;

  bool status = detector.detect(I);
  if (status) {
    for(size_t i=0; i < detector.getNbObjects(); i++) {
      std::cout << "Bar code " << i << ":" << std::endl;
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      for(size_t j=0; j < p.size(); j++)
        std::cout << "  Point " << j << ": " << p[j] << std::endl;
      std::cout << "  Message: \"" << detector.getMessage(i) << "\"" << std::endl;
    }
  }
#endif
}
  \endcode

  The previous example may produce results like:
  \code
Bar code 0:
  Point 0: 273.21, 78.9799
  Point 1: 390.016, 85.1014
  Point 2: 388.024, 199.185
  Point 3: 269.23, 192.96
  Message: "datamatrix 1"
Bar code 1:
  Point 0: 262.23, 396.404
  Point 1: 381.041, 402.631
  Point 2: 378.92, 524.188
  Point 3: 257.916, 519.962
  Message: "datamatrix 2"
  \endcode

  Other examples are also provided in tutorial-barcode-detector.cpp and
  tutorial-barcode-detector-live.cpp

 */
class VISP_EXPORT vpDetectorDataMatrixCode : public vpDetectorBase
{
public:
  vpDetectorDataMatrixCode();
  virtual ~vpDetectorDataMatrixCode(){};
  bool detect(const vpImage<unsigned char> &I);
};

#endif
#endif
