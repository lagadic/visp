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

#include <assert.h>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_DMTX

#include <dmtx.h>

#include <visp3/detection/vpDetectorDataMatrixCode.h>

/*!
   Default constructor that does nothing.
 */
vpDetectorDataMatrixCode::vpDetectorDataMatrixCode() {}

/*!
  Detect datamatrix bar codes in the image. Return true if a bar code is
  detected, false otherwise.

  \param I : Input image.
 */
bool vpDetectorDataMatrixCode::detect(const vpImage<unsigned char> &I)
{
  bool detected = false;
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;
  DmtxRegion *reg;
  DmtxDecode *dec;
  DmtxImage *img;
  DmtxMessage *msg;

  img = dmtxImageCreate(I.bitmap, (int)I.getWidth(), (int)I.getHeight(), DmtxPack8bppK);
  assert(img != NULL);

  dec = dmtxDecodeCreate(img, 1);
  assert(dec != NULL);

  bool end = false;
  do {
    reg = dmtxRegionFindNext(dec, 0);

    if (reg != NULL) {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if (msg != NULL) {

        std::vector<vpImagePoint> polygon;

        DmtxVector2 p00, p10, p11, p01;

        p00.X = p00.Y = p10.Y = p01.X = 0.0;
        p10.X = p01.Y = p11.X = p11.Y = 1.0;
        dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);

        polygon.push_back(vpImagePoint(I.getHeight() - p00.Y, p00.X));
        polygon.push_back(vpImagePoint(I.getHeight() - p10.Y, p10.X));
        polygon.push_back(vpImagePoint(I.getHeight() - p11.Y, p11.X));
        polygon.push_back(vpImagePoint(I.getHeight() - p01.Y, p01.X));

        m_polygon.push_back(polygon);
        detected = true;
        m_message.push_back((const char *)msg->output);

        m_nb_objects++;
      } else {
        end = true;
      }
      dmtxMessageDestroy(&msg);
    } else {
      end = true;
    }
    dmtxRegionDestroy(&reg);

  } while (!end);

  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&img);
  return detected;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning:
// libvisp_core.a(vpDetectorDataMatrixCode.cpp.o) has no symbols
void dummy_vpDetectorDataMatrixCode(){};
#endif
