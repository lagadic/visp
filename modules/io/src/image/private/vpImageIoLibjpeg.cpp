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
 * Libjpeg backend for JPEG image I/O operations.
 */

/*!
  \file vpImageIoLibjpeg.cpp
  \brief Libjpeg backend for JPEG image I/O operations.
*/

#include "vpImageIoBackend.h"
#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_JPEG)
#include <jerror.h>
#include <jpeglib.h>
#endif

//--------------------------------------------------------------------------
// JPEG
//--------------------------------------------------------------------------

#if defined(VISP_HAVE_JPEG)

BEGIN_VISP_NAMESPACE
/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
  \param quality : JPEG quality for compression.
*/
void writeJPEGLibjpeg(const vpImage<unsigned char> &I, const std::string &filename, int quality)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file \"%s\"", filename.c_str()));
  }

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  jpeg_stdio_dest(&cinfo, file);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 1;
  cinfo.in_color_space = JCS_GRAYSCALE;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  jpeg_start_compress(&cinfo, TRUE);

  unsigned char *line;
  line = new unsigned char[width];
  unsigned char *input = (unsigned char *)I.bitmap;
  while (cinfo.next_scanline < cinfo.image_height) {
    for (unsigned int i = 0; i < width; ++i) {
      line[i] = *(input);
      input++;
    }
    jpeg_write_scanlines(&cinfo, &line, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete[] line;
  fclose(file);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
  \param quality : JPEG quality for compression.
*/
void writeJPEGLibjpeg(const vpImage<vpRGBa> &I, const std::string &filename, int quality)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create JPEG file \"%s\"", filename.c_str()));
  }

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  jpeg_stdio_dest(&cinfo, file);

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  jpeg_start_compress(&cinfo, TRUE);

  unsigned char *line;
  line = new unsigned char[3 * width];
  unsigned char *input = (unsigned char *)I.bitmap;
  while (cinfo.next_scanline < cinfo.image_height) {
    for (unsigned int i = 0; i < width; ++i) {
      line[i * 3] = *(input);
      input++;
      line[i * 3 + 1] = *(input);
      input++;
      line[i * 3 + 2] = *(input);
      input++;
      input++;
    }
    jpeg_write_scanlines(&cinfo, &line, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete[] line;
  fclose(file);
}

/*!
  Read the contents of the JPEG file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap with the gray level data. That means that
  the image \e I is a "black and white" rendering of the original image in \e
  filename, as in a black and white photograph. If necessary, the quantization
  formula used is \f$0,299 r + 0,587 g + 0,114 b\f$.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void readJPEGLibjpeg(vpImage<unsigned char> &I, const std::string &filename)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG file \"%s\"", filename.c_str()));
  }

  jpeg_stdio_src(&cinfo, file);
  jpeg_read_header(&cinfo, TRUE);

  unsigned int width = cinfo.image_width;
  unsigned int height = cinfo.image_height;

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  jpeg_start_decompress(&cinfo);

  unsigned int rowbytes = cinfo.output_width * (unsigned int)(cinfo.output_components);
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, rowbytes, 1);

  if (cinfo.out_color_space == JCS_RGB) {
    vpImage<vpRGBa> Ic(height, width);
    unsigned char *output = (unsigned char *)Ic.bitmap;
    while (cinfo.output_scanline < cinfo.output_height) {
      jpeg_read_scanlines(&cinfo, buffer, 1);
      for (unsigned int i = 0; i < width; ++i) {
        *(output++) = buffer[0][i * 3];
        *(output++) = buffer[0][i * 3 + 1];
        *(output++) = buffer[0][i * 3 + 2];
        *(output++) = vpRGBa::alpha_default;
      }
    }
    vpImageConvert::convert(Ic, I);
  }

  else if (cinfo.out_color_space == JCS_GRAYSCALE) {
    while (cinfo.output_scanline < cinfo.output_height) {
      unsigned int row = cinfo.output_scanline;
      jpeg_read_scanlines(&cinfo, buffer, 1);
      memcpy(I[row], buffer[0], rowbytes);
    }
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(file);
}

/*!
  Read a JPEG file and initialize a scalar image.

  Read the contents of the JPEG file, allocate
  memory for the corresponding image, and set
  the bitmap with the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a gray scaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void readJPEGLibjpeg(vpImage<vpRGBa> &I, const std::string &filename)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  FILE *file;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot read JPEG file \"%s\"", filename.c_str()));
  }

  jpeg_stdio_src(&cinfo, file);

  jpeg_read_header(&cinfo, TRUE);

  unsigned int width = cinfo.image_width;
  unsigned int height = cinfo.image_height;

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  jpeg_start_decompress(&cinfo);

  unsigned int rowbytes = cinfo.output_width * (unsigned int)(cinfo.output_components);
  JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, rowbytes, 1);

  if (cinfo.out_color_space == JCS_RGB) {
    unsigned char *output = (unsigned char *)I.bitmap;
    while (cinfo.output_scanline < cinfo.output_height) {
      jpeg_read_scanlines(&cinfo, buffer, 1);
      for (unsigned int i = 0; i < width; ++i) {
        *(output++) = buffer[0][i * 3];
        *(output++) = buffer[0][i * 3 + 1];
        *(output++) = buffer[0][i * 3 + 2];
        *(output++) = vpRGBa::alpha_default;
      }
    }
  }

  else if (cinfo.out_color_space == JCS_GRAYSCALE) {
    vpImage<unsigned char> Ig(height, width);

    while (cinfo.output_scanline < cinfo.output_height) {
      unsigned int row = cinfo.output_scanline;
      jpeg_read_scanlines(&cinfo, buffer, 1);
      memcpy(Ig[row], buffer[0], rowbytes);
    }

    vpImageConvert::convert(Ig, I);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(file);
}

END_VISP_NAMESPACE

#endif
