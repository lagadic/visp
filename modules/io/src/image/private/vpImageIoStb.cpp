/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * stb backend for JPEG and PNG image I/O operations.
 */

/*!
  \file vpImageIo.cpp
  \brief stb backend for JPEG and PNG image I/O operations.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_STBIMAGE)

#include "vpImageIoBackend.h"
#include <visp3/core/vpImageConvert.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#define VISP_HAVE_SSE2 1
#endif

#ifndef VISP_HAVE_SSE2
#define STBI_NO_SIMD
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

BEGIN_VISP_NAMESPACE

void readStb(vpImage<unsigned char> &I, const std::string &filename)
{
  int width = 0, height = 0, channels = 0;
  unsigned char *image = stbi_load(filename.c_str(), &width, &height, &channels, STBI_grey);
  if (image == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Can't read the image: %s", filename.c_str()));
  }
  I.init(image, static_cast<unsigned int>(height), static_cast<unsigned int>(width), true);
  stbi_image_free(image);
}

void readStb(vpImage<vpRGBa> &I, const std::string &filename)
{
  int width = 0, height = 0, channels = 0;
  unsigned char *image = stbi_load(filename.c_str(), &width, &height, &channels, STBI_rgb_alpha);
  if (image == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Can't read the image: %s", filename.c_str()));
  }
  I.init(reinterpret_cast<vpRGBa *>(image), static_cast<unsigned int>(height), static_cast<unsigned int>(width), true);
  stbi_image_free(image);
}

void writeJPEGStb(const vpImage<unsigned char> &I, const std::string &filename, int quality)
{
  int res = stbi_write_jpg(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_grey,
                           reinterpret_cast<void *>(I.bitmap), quality);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "JEPG write error"));
  }
}

void writeJPEGStb(const vpImage<vpRGBa> &I, const std::string &filename, int quality)
{
  int res = stbi_write_jpg(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()),
                           STBI_rgb_alpha, reinterpret_cast<void *>(I.bitmap), quality);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "JEPG write error"));
  }
}

void writePNGStb(const vpImage<unsigned char> &I, const std::string &filename)
{
  const int stride_in_bytes = static_cast<int>(I.getWidth());
  int res = stbi_write_png(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()), STBI_grey,
                           reinterpret_cast<void *>(I.bitmap), stride_in_bytes);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "PNG write error: %s", filename.c_str()));
  }
}

void writePNGStb(const vpImage<vpRGBa> &I, const std::string &filename)
{
  const int stride_in_bytes = static_cast<int>(4 * I.getWidth());
  int res = stbi_write_png(filename.c_str(), static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight()),
                           STBI_rgb_alpha, reinterpret_cast<void *>(I.bitmap), stride_in_bytes);
  if (res == 0) {
    throw(vpImageException(vpImageException::ioError, "PNG write error: %s", filename.c_str()));
  }
}

namespace
{
typedef struct
{
  int last_pos;
  void *context;
} custom_stbi_mem_context;

// custom write function
static void custom_stbi_write_mem(void *context, void *data, int size)
{
  custom_stbi_mem_context *c = (custom_stbi_mem_context *)context;
  char *dst = (char *)c->context;
  char *src = (char *)data;
  int cur_pos = c->last_pos;
  for (int i = 0; i < size; ++i) {
    dst[cur_pos++] = src[i];
  }
  c->last_pos = cur_pos;
}
}

/*!
  Read the content of the grayscale image bitmap stored in memory and encoded using the PNG format.

  \param[in] buffer : Grayscale image buffer encoded in PNG as 1-D unsigned char vector.
  \param[out] I : Output decoded grayscale image.
*/
void readPNGfromMemStb(const std::vector<unsigned char> &buffer, vpImage<unsigned char> &I)
{
  int x = 0, y = 0, comp = 0;
  const int req_channels = 1;
  unsigned char *buffer_read = stbi_load_from_memory(buffer.data(), static_cast<int>(buffer.size()), &x, &y, &comp, req_channels);
  assert(comp == req_channels);

  I.init(buffer_read, y, x, true);
  STBI_FREE(buffer_read);
}

/*!
  Read the content of the color image bitmap stored in memory and encoded using the PNG format.

  \param[in] buffer : Color image buffer encoded in PNG as 1-D unsigned char vector.
  \param[out] I_color : Output decoded color image.
*/
void readPNGfromMemStb(const std::vector<unsigned char> &buffer, vpImage<vpRGBa> &I_color)
{
  int x = 0, y = 0, comp = 0;
  unsigned char *buffer_read = stbi_load_from_memory(buffer.data(), static_cast<int>(buffer.size()), &x, &y, &comp, 0);

  if (comp == 4) {
    const bool copyData = true;
    I_color.init(reinterpret_cast<vpRGBa *>(buffer_read), y, x, copyData);
  }
  else if (comp == 3) {
    I_color.init(y, x);
    const bool flip = false;
    vpImageConvert::RGBToRGBa(buffer_read, reinterpret_cast<unsigned char *>(I_color.bitmap), x, y, flip);
  }
  else {
    STBI_FREE(buffer_read);

#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
    std::string message = "Wrong number of channels for the input buffer: " + std::to_string(comp);
    throw(vpImageException(vpImageException::ioError, message));
#else
    throw(vpImageException(vpImageException::ioError, "Wrong number of channels for the input buffer: %d", comp));
#endif
  }

  STBI_FREE(buffer_read);
}

/*!
  In-memory PNG encoding of the grayscale image.

  \param[in] I : Input grayscale image.
  \param[out] buffer : Encoded image as 1-D unsigned char vector using the PNG format.
*/
void writePNGtoMemStb(const vpImage<unsigned char> &I, std::vector<unsigned char> &buffer)
{
  const int height = I.getRows();
  const int width = I.getCols();
  const int channels = 1;

  custom_stbi_mem_context context;
  context.last_pos = 0;
  buffer.resize(I.getHeight() * I.getWidth());
  context.context = (void *)buffer.data();

  const int stride_bytes = 0;
  int result = stbi_write_png_to_func(custom_stbi_write_mem, &context, width, height, channels, I.bitmap, stride_bytes);

  if (result) {
    buffer.resize(context.last_pos);
  }
  else {
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
    std::string message = "Cannot write png to memory, result: " + std::to_string(result);
    throw(vpImageException(vpImageException::ioError, message));
#else
    throw(vpImageException(vpImageException::ioError, "Cannot write png to memory, result: %d", result));
#endif
  }
}

/*!
  In-memory PNG encoding of the color image.

  \param[in] I_color : Input color image.
  \param[out] buffer : Encoded image as 1-D unsigned char vector using the PNG format.
  \param[in] saveAlpha : If true, alpha channel is also used for encoding.
*/
void writePNGtoMemStb(const vpImage<vpRGBa> &I_color, std::vector<unsigned char> &buffer, bool saveAlpha)
{
  const int height = I_color.getRows();
  const int width = I_color.getCols();
  const int channels = saveAlpha ? 4 : 3;

  custom_stbi_mem_context context;
  context.last_pos = 0;
  buffer.resize(height * width * channels);
  context.context = (void *)buffer.data();

  const int stride_bytes = 0;
  int result = 0;
  if (saveAlpha) {
    result = stbi_write_png_to_func(custom_stbi_write_mem, &context, width, height, channels,
      reinterpret_cast<unsigned char *>(I_color.bitmap), stride_bytes);
  }
  else {
    unsigned char *bitmap = new unsigned char[height * width * channels];
    vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(I_color.bitmap), bitmap, height*width);
    result = stbi_write_png_to_func(custom_stbi_write_mem, &context, width, height, channels, bitmap, stride_bytes);
    delete[] bitmap;
  }

  if (result) {
    buffer.resize(context.last_pos);
  }
  else {
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
    std::string message = "Cannot write png to memory, result: " + std::to_string(result);
    throw(vpImageException(vpImageException::ioError, message));
#else
    throw(vpImageException(vpImageException::ioError, "Cannot write png to memory, result: %d", result));
#endif
  }
}

END_VISP_NAMESPACE

#endif
