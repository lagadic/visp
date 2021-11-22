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
<<<<<<< HEAD
 * Read/write images.
 *
 * Authors:
 * Eric Marchand
=======
 * Simd backend for JPEG and PNG image I/O operations.
>>>>>>> 557f1beda01f36ca886ec039d0a1a80a7446ca59
 *
 *****************************************************************************/

/*!
  \file vpImageIo.cpp
  \brief Simd backend for JPEG and PNG image I/O operations.
*/

#include "vpImageIoBackend.h"
#include <Simd/SimdLib.h>


//TODO:
void readSimdlib(vpImage<unsigned char> &I, const std::string &filename)
{
  size_t stride = 0, width = 0, height = 0;
  SimdPixelFormatType format = SimdPixelFormatGray8;
  uint8_t* data = SimdImageLoadFromFile(filename.c_str(), &stride, &width, &height, &format);
  const bool copyData = true;
  I.init(data, (unsigned int)height, (unsigned int)width, copyData);
  SimdFree(data);
}

void readSimdlib(vpImage<vpRGBa> &I, const std::string &filename)
{
  size_t stride = 0, width = 0, height = 0;
  SimdPixelFormatType format = SimdPixelFormatRgba32;
  uint8_t* data = SimdImageLoadFromFile(filename.c_str(), &stride, &width, &height, &format);
  const bool copyData = true;
  I.init((vpRGBa *)data, (unsigned int)height, (unsigned int)width, copyData);
  SimdFree(data);
}

void writeJPEGSimdlib(const vpImage<unsigned char> &I, const std::string &filename, int quality)
{
  SimdImageSaveToFile((const uint8_t *)I.bitmap, I.getWidth()*4, I.getWidth(), I.getHeight(), SimdPixelFormatGray8, SimdImageFileJpeg, quality, filename.c_str());
}

void writeJPEGSimdlib(const vpImage<vpRGBa> &I, const std::string &filename, int quality)
{
  SimdImageSaveToFile((const uint8_t *)I.bitmap, I.getWidth()*4, I.getWidth(), I.getHeight(), SimdPixelFormatRgba32, SimdImageFileJpeg, quality, filename.c_str());
}

void writePNGSimdlib(const vpImage<unsigned char> &I, const std::string &filename)
{
  SimdImageSaveToFile((const uint8_t *)I.bitmap, I.getWidth()*4, I.getWidth(), I.getHeight(), SimdPixelFormatGray8, SimdImageFilePng, 90, filename.c_str());
}

void writePNGSimdlib(const vpImage<vpRGBa> &I, const std::string &filename)
{
  SimdImageSaveToFile((const uint8_t *)I.bitmap, I.getWidth()*4, I.getWidth(), I.getHeight(), SimdPixelFormatRgba32, SimdImageFilePng, 90, filename.c_str());
}
