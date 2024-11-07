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
 * Libpng backend for PNG image I/O operations.
 */

/*!
  \file vpImageIoLibpng.cpp
  \brief Libpng backend for PNG image I/O operations.
*/

#include "vpImageIoBackend.h"
#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_PNG)
#include <png.h>
#endif

//--------------------------------------------------------------------------
// PNG
//--------------------------------------------------------------------------

#if defined(VISP_HAVE_PNG)

BEGIN_VISP_NAMESPACE
/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void writePNGLibpng(const vpImage<unsigned char> &I, const std::string &filename)
{
  FILE *file;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file \"%s\"", filename.c_str()));
  }

  /* create a png info struct */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occurred
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* setup libpng for using standard C fwrite() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  int bit_depth = 8;
  int color_type = PNG_COLOR_TYPE_GRAY;
  /* set some useful information from header */

  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  png_bytep *row_ptrs = new png_bytep[height];
  for (unsigned int i = 0; i < height; ++i)
    row_ptrs[i] = new png_byte[width];

  unsigned char *input = (unsigned char *)I.bitmap;

  for (unsigned int i = 0; i < height; ++i) {
    png_byte *row = row_ptrs[i];
    for (unsigned int j = 0; j < width; ++j) {
      row[j] = *(input);
      input++;
    }
  }

  png_write_image(png_ptr, row_ptrs);

  png_write_end(png_ptr, nullptr);

  for (unsigned int j = 0; j < height; ++j)
    delete[] row_ptrs[j];

  delete[] row_ptrs;

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(file);
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a PNG file.

  \param I : Image to save as a PNG file.
  \param filename : Name of the file containing the image.
*/
void writePNGLibpng(const vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *file;

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file: filename empty"));
  }

  file = fopen(filename.c_str(), "wb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot create PNG file \"%s\"", filename.c_str()));
  }

  /* create a png info struct */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occurred
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  /* setup libpng for using standard C fwrite() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  int bit_depth = 8;
  int color_type = PNG_COLOR_TYPE_RGB;
  /* set some useful information from header */

  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw(vpImageException(vpImageException::ioError, "PNG write error"));
  }

  png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_BASE,
               PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  png_bytep *row_ptrs = new png_bytep[height];
  for (unsigned int i = 0; i < height; ++i)
    row_ptrs[i] = new png_byte[3 * width];

  unsigned char *input = (unsigned char *)I.bitmap;

  for (unsigned int i = 0; i < height; ++i) {
    png_byte *row = row_ptrs[i];
    for (unsigned int j = 0; j < width; ++j) {
      row[3 * j] = *(input);
      input++;
      row[3 * j + 1] = *(input);
      input++;
      row[3 * j + 2] = *(input);
      input++;
      input++;
    }
  }

  png_write_image(png_ptr, row_ptrs);

  png_write_end(png_ptr, nullptr);

  for (unsigned int j = 0; j < height; ++j)
    delete[] row_ptrs[j];

  delete[] row_ptrs;

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(file);
}

/*!
  Read the contents of the PNG file, allocate memory
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
void readPNGLibpng(vpImage<unsigned char> &I, const std::string &filename)
{
  FILE *file;
  png_byte magic[8];
  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot read file \"%s\"", filename.c_str()));
  }

  /* read magic number */
  if (fread(magic, 1, sizeof(magic), file) != sizeof(magic)) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read magic number in file \"%s\"", filename.c_str()));
  }

  /* check for valid magic number */
  if (png_sig_cmp(magic, 0, sizeof(magic))) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG file: \"%s\" is not a valid PNG image",
                           filename.c_str()));
  }

  /* create a png read struct */
  // printf("version %s\n", PNG_LIBPNG_VER_STRING);
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (png_ptr == nullptr) {
    fprintf(stderr, "error: can't create a png read structure!\n");
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "error reading png file"));
  }

  /* create a png info struct */
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (info_ptr == nullptr) {
    fprintf(stderr, "error: can't create a png info structure!\n");
    fclose(file);
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "error reading png file"));
  }

  /* initialize the setjmp for returning properly after a libpng error occurred
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* setup libpng for using standard C fread() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  /* tell libpng that we have already read the magic number */
  png_set_sig_bytes(png_ptr, sizeof(magic));

  /* read png info */
  png_read_info(png_ptr, info_ptr);

  unsigned int width = png_get_image_width(png_ptr, info_ptr);
  unsigned int height = png_get_image_height(png_ptr, info_ptr);

  unsigned int bit_depth, channels, color_type;
  /* get some useful information from header */
  bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  channels = png_get_channels(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);

  /* convert index color images to RGB images */
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);

  /* convert 1-2-4 bits grayscale images to 8 bits grayscale. */
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);

  //  if (png_get_valid (png_ptr, info_ptr, PNG_INFO_tRNS))
  //    png_set_tRNS_to_alpha (png_ptr);

  if (color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_strip_alpha(png_ptr);

  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  else if (bit_depth < 8)
    png_set_packing(png_ptr);

  /* update info structure to apply transformations */
  png_read_update_info(png_ptr, info_ptr);

  channels = png_get_channels(png_ptr, info_ptr);

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  png_bytep *rowPtrs = new png_bytep[height];

  unsigned int stride = png_get_rowbytes(png_ptr, info_ptr);
  unsigned char *data = new unsigned char[stride * height];

  for (unsigned int i = 0; i < height; ++i)
    rowPtrs[i] = (png_bytep)data + (i * stride);

  png_read_image(png_ptr, rowPtrs);

  vpImage<vpRGBa> Ic(height, width);
  unsigned char *output;

  switch (channels) {
  case 1:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i];
    }
    break;

  case 2:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 2];
    }
    break;

  case 3:
    output = (unsigned char *)Ic.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 3];
      *(output++) = data[i * 3 + 1];
      *(output++) = data[i * 3 + 2];
      *(output++) = vpRGBa::alpha_default;
    }
    vpImageConvert::convert(Ic, I);
    break;

  case 4:
    output = (unsigned char *)Ic.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 4];
      *(output++) = data[i * 4 + 1];
      *(output++) = data[i * 4 + 2];
      *(output++) = data[i * 4 + 3];
    }
    vpImageConvert::convert(Ic, I);
    break;
  }

  delete[](png_bytep) rowPtrs;
  delete[] data;
  png_read_end(png_ptr, nullptr);
  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
  fclose(file);
}

/*!
  Read a PNG file and initialize a scalar image.

  Read the contents of the PNG file, allocate
  memory for the corresponding image, and set
  the bitmap with the content of
  the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void readPNGLibpng(vpImage<vpRGBa> &I, const std::string &filename)
{
  FILE *file;
  png_byte magic[8];

  // Test the filename
  if (filename.empty()) {
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG image: filename empty"));
  }

  file = fopen(filename.c_str(), "rb");

  if (file == nullptr) {
    throw(vpImageException(vpImageException::ioError, "Cannot read file \"%s\"", filename.c_str()));
  }

  /* read magic number */
  if (fread(magic, 1, sizeof(magic), file) != sizeof(magic)) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read magic number in file \"%s\"", filename.c_str()));
  }

  /* check for valid magic number */
  if (png_sig_cmp(magic, 0, sizeof(magic))) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "Cannot read PNG file: \"%s\" is not a valid PNG image",
                           filename.c_str()));
  }

  /* create a png read struct */
  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr) {
    fclose(file);
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* create a png info struct */
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, nullptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* initialize the setjmp for returning properly after a libpng error occurred
   */
  if (setjmp(png_jmpbuf(png_ptr))) {
    fclose(file);
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    throw(vpImageException(vpImageException::ioError, "PNG read error"));
  }

  /* setup libpng for using standard C fread() function with our FILE pointer
   */
  png_init_io(png_ptr, file);

  /* tell libpng that we have already read the magic number */
  png_set_sig_bytes(png_ptr, sizeof(magic));

  /* read png info */
  png_read_info(png_ptr, info_ptr);

  unsigned int width = png_get_image_width(png_ptr, info_ptr);
  unsigned int height = png_get_image_height(png_ptr, info_ptr);

  unsigned int bit_depth, channels, color_type;
  /* get some useful information from header */
  bit_depth = png_get_bit_depth(png_ptr, info_ptr);
  channels = png_get_channels(png_ptr, info_ptr);
  color_type = png_get_color_type(png_ptr, info_ptr);

  /* convert index color images to RGB images */
  if (color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_palette_to_rgb(png_ptr);

  /* convert 1-2-4 bits grayscale images to 8 bits grayscale. */
  if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
    png_set_expand(png_ptr);

  //  if (png_get_valid (png_ptr, info_ptr, PNG_INFO_tRNS))
  //    png_set_tRNS_to_alpha (png_ptr);

  if (color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_strip_alpha(png_ptr);

  if (bit_depth == 16)
    png_set_strip_16(png_ptr);
  else if (bit_depth < 8)
    png_set_packing(png_ptr);

  /* update info structure to apply transformations */
  png_read_update_info(png_ptr, info_ptr);

  channels = png_get_channels(png_ptr, info_ptr);

  if ((width != I.getWidth()) || (height != I.getHeight()))
    I.resize(height, width);

  png_bytep *rowPtrs = new png_bytep[height];

  unsigned int stride = png_get_rowbytes(png_ptr, info_ptr);
  unsigned char *data = new unsigned char[stride * height];

  for (unsigned int i = 0; i < height; ++i)
    rowPtrs[i] = (png_bytep)data + (i * stride);

  png_read_image(png_ptr, rowPtrs);

  vpImage<unsigned char> Ig(height, width);
  unsigned char *output;

  switch (channels) {
  case 1:
    output = (unsigned char *)Ig.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i];
    }
    vpImageConvert::convert(Ig, I);
    break;

  case 2:
    output = (unsigned char *)Ig.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 2];
    }
    vpImageConvert::convert(Ig, I);
    break;

  case 3:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 3];
      *(output++) = data[i * 3 + 1];
      *(output++) = data[i * 3 + 2];
      *(output++) = vpRGBa::alpha_default;
    }
    break;

  case 4:
    output = (unsigned char *)I.bitmap;
    for (unsigned int i = 0; i < width * height; ++i) {
      *(output++) = data[i * 4];
      *(output++) = data[i * 4 + 1];
      *(output++) = data[i * 4 + 2];
      *(output++) = data[i * 4 + 3];
    }
    break;
  }

  delete[](png_bytep) rowPtrs;
  delete[] data;
  png_read_end(png_ptr, nullptr);
  png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
  fclose(file);
}

END_VISP_NAMESPACE

#endif
