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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_ZBAR

#include <visp3/detection/vpDetectorQRCode.h>

/*!
   Default constructor.
 */
vpDetectorQRCode::vpDetectorQRCode() : m_scanner()
{
  // configure the reader
  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

/*!
  Detect QR codes in the image. Return true if a code is detected, false
  otherwise.

  \param I : Input image.
 */
bool vpDetectorQRCode::detect(const vpImage<unsigned char> &I)
{
  bool detected = false;
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  // wrap image data
  zbar::Image img(width, height, "Y800", I.bitmap, (unsigned long)(width * height));

  // scan the image for barcodes
  m_nb_objects = (size_t)m_scanner.scan(img);

  // extract results
  for (zbar::Image::SymbolIterator symbol = img.symbol_begin(); symbol != img.symbol_end(); ++symbol) {
    m_message.push_back(symbol->get_data());
    detected = true;

    std::vector<vpImagePoint> polygon;
    for (unsigned int i = 0; i < (unsigned int)symbol->get_location_size(); i++) {
      polygon.push_back(vpImagePoint(symbol->get_location_y(i), symbol->get_location_x(i)));
    }
    m_polygon.push_back(polygon);
  }

  // clean up
  img.set_data(NULL, 0);

  return detected;
}
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorQRCode.cpp.o) has
// no symbols
void dummy_vpDetectorQRCode(){};
#endif
