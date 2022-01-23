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
 * stb backend for JPEG and PNG image I/O operations.
>>>>>>> 557f1beda01f36ca886ec039d0a1a80a7446ca59
 *
 *****************************************************************************/

/*!
  \file vpImageIo.cpp
  \brief stb backend for JPEG and PNG image I/O operations.
*/

#include "vpImageIoBackend.h"

//TODO:
#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#  define VISP_HAVE_SSE2 1
#endif

#ifndef VISP_HAVE_SSE2
#  define STBI_NO_SIMD
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>


//TODO:
void readStb(vpImage<unsigned char> &I, const std::string &filename)
{
  int width = 0, height = 0, channels = 0;
  unsigned char *image = stbi_load(filename.c_str(), &width, &height, &channels, STBI_grey);
  if (image == NULL) {
    throw(vpImageException(vpImageException::ioError, "Can't read the image: %s", filename.c_str()));
  }
  I.init(image, static_cast<unsigned int>(height), static_cast<unsigned int>(width), true);
  stbi_image_free(image);
}

void readStb(vpImage<vpRGBa> &I, const std::string &filename)
{
  int width = 0, height = 0, channels = 0;
  unsigned char *image = stbi_load(filename.c_str(), &width, &height, &channels, STBI_rgb_alpha);
  if (image == NULL) {
    throw(vpImageException(vpImageException::ioError, "Can't read the image: %s", filename.c_str()));
  }
  I.init(reinterpret_cast<vpRGBa*>(image), static_cast<unsigned int>(height), static_cast<unsigned int>(width), true);
  stbi_image_free(image);
}

void writeJPEGStb(const vpImage<unsigned char> &I, const std::string &filename, int quality)
{
  int res = stbi_write_jpg(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_grey,
                           reinterpret_cast<void*>(I.bitmap), quality);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "JEPG write error"));
  }
}

void writeJPEGStb(const vpImage<vpRGBa> &I, const std::string &filename, int quality)
{
  int res = stbi_write_jpg(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_rgb_alpha,
                           reinterpret_cast<void*>(I.bitmap), quality);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "JEPG write error"));
  }
}

void writePNGStb(const vpImage<unsigned char> &I, const std::string &filename)
{
  const int stride_in_bytes = static_cast<int>(I.getWidth());
  int res = stbi_write_png(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_grey,
                           reinterpret_cast<void*>(I.bitmap), stride_in_bytes);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "PNG write error: %s", filename.c_str()));
  }
}

void writePNGStb(const vpImage<vpRGBa> &I, const std::string &filename)
{
  const int stride_in_bytes = static_cast<int>(4 * I.getWidth());
  int res = stbi_write_png(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_rgb_alpha,
                           reinterpret_cast<void*>(I.bitmap), stride_in_bytes);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "PNG write error: %s", filename.c_str()));
  }
}
