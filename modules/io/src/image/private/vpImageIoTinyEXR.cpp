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
 * TinyEXR backend for EXR image I/O operations.
 */

/*!
  \file vpImageIoTinyEXR.cpp
  \brief TinyEXR backend for EXR image I/O operations.
*/


#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_STBIMAGE) && defined(VISP_HAVE_TINYEXR)

#include "vpImageIoBackend.h"

#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 1
#include <stb_image.h>
#include <stb_image_write.h>

#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

BEGIN_VISP_NAMESPACE

void readEXRTiny(vpImage<float> &I, const std::string &filename)
{
  EXRVersion exr_version;

  int ret = ParseEXRVersionFromFile(&exr_version, filename.c_str());
  if (ret != 0) {
    throw(vpImageException(vpImageException::ioError, "Error: Invalid EXR file %s", filename.c_str()));
  }

  if (exr_version.multipart) {
    // must be multipart flag is false.
    throw(vpImageException(vpImageException::ioError, "Error: Multipart EXR images are not supported."));
  }

  EXRHeader exr_header;
  InitEXRHeader(&exr_header);

  const char *err = nullptr; // or `nullptr` in C++11 or later.
  ret = ParseEXRHeaderFromFile(&exr_header, &exr_version, filename.c_str(), &err);
  if (ret != 0) {
    std::string err_msg(err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    throw(vpImageException(vpImageException::ioError, "Error: Unable to parse EXR header from %s : %s", filename.c_str(), err_msg.c_str()));
  }

  // Read HALF channel as FLOAT.
  for (int i = 0; i < exr_header.num_channels; ++i) {
    if (exr_header.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
      exr_header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
    }
  }

  EXRImage exr_image;
  InitEXRImage(&exr_image);

  ret = LoadEXRImageFromFile(&exr_image, &exr_header, filename.c_str(), &err);

  if (ret != 0) {
    std::string err_msg(err);
    FreeEXRHeader(&exr_header);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    throw(vpImageException(vpImageException::ioError, "Error: Unable to load EXR image from %s : %s", filename.c_str(), err_msg.c_str()));
  }

  // `exr_image.images` will be filled when EXR is scanline format.
  // `exr_image.tiled` will be filled when EXR is tiled format.
  if (exr_image.images) {
    I.resize(exr_image.height, exr_image.width);
    memcpy(I.bitmap, exr_image.images[0], exr_image.height*exr_image.width*sizeof(float));
  }
  else if (exr_image.tiles) {
    I.resize(exr_image.height, exr_image.width);
    size_t data_width = static_cast<size_t>(exr_header.data_window.max_x - exr_header.data_window.min_x + 1);

    for (int tile_idx = 0; tile_idx < exr_image.num_tiles; ++tile_idx) {
      int sx = exr_image.tiles[tile_idx].offset_x * exr_header.tile_size_x;
      int sy = exr_image.tiles[tile_idx].offset_y * exr_header.tile_size_y;
      int ex = exr_image.tiles[tile_idx].offset_x * exr_header.tile_size_x + exr_image.tiles[tile_idx].width;
      int ey = exr_image.tiles[tile_idx].offset_y * exr_header.tile_size_y + exr_image.tiles[tile_idx].height;

      for (unsigned int y = 0; y < static_cast<unsigned int>(ey - sy); ++y) {
        for (unsigned int x = 0; x < static_cast<unsigned int>(ex - sx); ++x) {
          const float *src_image = reinterpret_cast<const float *>(exr_image.tiles[tile_idx].images[0]);
          I.bitmap[(y + sy) * data_width + (x + sx)] = src_image[y * exr_header.tile_size_x + x];
        }
      }
    }
  }

  FreeEXRImage(&exr_image);
  FreeEXRHeader(&exr_header);
}

void readEXRTiny(vpImage<vpRGBf> &I, const std::string &filename)
{
  EXRVersion exr_version;

  int ret = ParseEXRVersionFromFile(&exr_version, filename.c_str());
  if (ret != 0) {
    throw(vpImageException(vpImageException::ioError, "Error: Invalid EXR file %s", filename.c_str()));
  }

  if (exr_version.multipart) {
    // must be multipart flag is false.
    throw(vpImageException(vpImageException::ioError, "Error: Multipart EXR images are not supported."));
  }

  EXRHeader exr_header;
  InitEXRHeader(&exr_header);

  const char *err = nullptr; // or `nullptr` in C++11 or later.
  ret = ParseEXRHeaderFromFile(&exr_header, &exr_version, filename.c_str(), &err);
  if (ret != 0) {
    std::string err_msg(err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    throw(vpImageException(vpImageException::ioError, "Error: Unable to parse EXR header from %s : %s", filename.c_str(), err_msg.c_str()));
  }

  // Read HALF channel as FLOAT.
  for (int i = 0; i < exr_header.num_channels; ++i) {
    if (exr_header.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
      exr_header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
    }
  }

  EXRImage exr_image;
  InitEXRImage(&exr_image);

  ret = LoadEXRImageFromFile(&exr_image, &exr_header, filename.c_str(), &err);

  if (ret != 0) {
    std::string err_msg(err);
    FreeEXRHeader(&exr_header);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    throw(vpImageException(vpImageException::ioError, "Error: Unable to load EXR image from %s : %s", filename.c_str(), err_msg.c_str()));
  }

  // `exr_image.images` will be filled when EXR is scanline format.
  // `exr_image.tiled` will be filled when EXR is tiled format.
  if (exr_image.images) {
    I.resize(exr_image.height, exr_image.width);
    for (int i = 0; i < exr_image.height; ++i) {
      for (int j = 0; j < exr_image.width; ++j) {
        I[i][j].R = reinterpret_cast<float **>(exr_image.images)[2][i * exr_image.width + j];
        I[i][j].G = reinterpret_cast<float **>(exr_image.images)[1][i * exr_image.width + j];
        I[i][j].B = reinterpret_cast<float **>(exr_image.images)[0][i * exr_image.width + j];
      }
    }
  }
  else if (exr_image.tiles) {
    I.resize(exr_image.height, exr_image.width);
    size_t data_width = static_cast<size_t>(exr_header.data_window.max_x - exr_header.data_window.min_x + 1);

    for (int tile_idx = 0; tile_idx < exr_image.num_tiles; ++tile_idx) {
      int sx = exr_image.tiles[tile_idx].offset_x * exr_header.tile_size_x;
      int sy = exr_image.tiles[tile_idx].offset_y * exr_header.tile_size_y;
      int ex = exr_image.tiles[tile_idx].offset_x * exr_header.tile_size_x + exr_image.tiles[tile_idx].width;
      int ey = exr_image.tiles[tile_idx].offset_y * exr_header.tile_size_y + exr_image.tiles[tile_idx].height;

      //for (size_t c = 0; c < static_cast<size_t>(exr_header.num_channels); ++c) {
      //  const float *src_image = reinterpret_cast<const float *>(exr_image.tiles[tile_idx].images[c]);
      //  for (size_t y = 0; y < static_cast<size_t>(ey - sy); ++y) {
      //    for (size_t x = 0; x < static_cast<size_t>(ex - sx); ++x) {
      //       reinterpret_cast<float *>(I.bitmap)[(y + sy) * data_width * 3 + (x + sx) * 3 + c] = src_image[y * exr_header.tile_size_x + x];
      //    }
      //  }
      //}

      for (unsigned int y = 0; y < static_cast<unsigned int>(ey - sy); ++y) {
        for (unsigned int x = 0; x < static_cast<unsigned int>(ex - sx); ++x) {
          for (unsigned int c = 0; c < 3; ++c) {
            const float *src_image = reinterpret_cast<const float *>(exr_image.tiles[tile_idx].images[c]);
            reinterpret_cast<float *>(I.bitmap)[(y + sy) * data_width * 3 + (x + sx) * 3 + c] = src_image[y * exr_header.tile_size_x + x];
          }
        }
      }
    }
  }

  FreeEXRImage(&exr_image);
  FreeEXRHeader(&exr_header);
}

void writeEXRTiny(const vpImage<float> &I, const std::string &filename)
{
  EXRHeader header;
  InitEXRHeader(&header);

  EXRImage image;
  InitEXRImage(&image);

  image.num_channels = 1;

  image.images = (unsigned char **)&I.bitmap;
  image.width = I.getWidth();
  image.height = I.getHeight();

  header.num_channels = 1;
  header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
  // Must be (A)BGR order, since most of EXR viewers expect this channel order.
  strncpy(header.channels[0].name, "Y", 255); header.channels[0].name[strlen("Y")] = '\0';

  header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  header.compression_type = TINYEXR_COMPRESSIONTYPE_ZIP;
  for (int i = 0; i < header.num_channels; ++i) {
    header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;          // pixel type of input image
    header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of output image to be stored in .EXR
  }

  const char *err = nullptr; // or nullptr in C++11 or later.
  int ret = SaveEXRImageToFile(&image, &header, filename.c_str(), &err);
  if (ret != TINYEXR_SUCCESS) {
    std::string err_msg(err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    free(header.channels);
    free(header.requested_pixel_types);
    free(header.pixel_types);
    throw(vpImageException(vpImageException::ioError, "Error: Unable to save EXR image to %s : %s", filename.c_str(), err_msg.c_str()));
  }

  free(header.channels);
  free(header.requested_pixel_types);
  free(header.pixel_types);
}

void writeEXRTiny(const vpImage<vpRGBf> &I, const std::string &filename)
{
  EXRHeader header;
  InitEXRHeader(&header);

  EXRImage image;
  InitEXRImage(&image);

  image.num_channels = 3;

  std::vector<float> images[3];
  images[0].resize(I.getSize());
  images[1].resize(I.getSize());
  images[2].resize(I.getSize());

  // Split RGBRGBRGB... into R, G and B layer
  for (unsigned int i = 0; i < I.getSize(); ++i) {
    images[0][i] = I.bitmap[i].R;
    images[1][i] = I.bitmap[i].G;
    images[2][i] = I.bitmap[i].B;
  }

  float *image_ptr[3];
  image_ptr[0] = &(images[2].at(0)); // B
  image_ptr[1] = &(images[1].at(0)); // G
  image_ptr[2] = &(images[0].at(0)); // R

  image.images = (unsigned char **)image_ptr;
  image.width = I.getWidth();
  image.height = I.getHeight();

  header.num_channels = 3;
  header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
  // Must be (A)BGR order, since most of EXR viewers expect this channel order.
  strncpy(header.channels[0].name, "B", 255); header.channels[0].name[strlen("B")] = '\0';
  strncpy(header.channels[1].name, "G", 255); header.channels[1].name[strlen("G")] = '\0';
  strncpy(header.channels[2].name, "R", 255); header.channels[2].name[strlen("R")] = '\0';

  header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  header.compression_type = TINYEXR_COMPRESSIONTYPE_ZIP;
  for (int i = 0; i < header.num_channels; ++i) {
    header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;            // pixel type of input image
    header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;  // pixel type of output image to be stored in .EXR
  }

  const char *err = nullptr; // or nullptr in C++11 or later.
  int ret = SaveEXRImageToFile(&image, &header, filename.c_str(), &err);
  if (ret != TINYEXR_SUCCESS) {
    std::string err_msg(err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    free(header.channels);
    free(header.requested_pixel_types);
    free(header.pixel_types);
    throw(vpImageException(vpImageException::ioError, "Error: Unable to save EXR image to %s : %s", filename.c_str(), err_msg.c_str()));
  }

  free(header.channels);
  free(header.requested_pixel_types);
  free(header.pixel_types);
}

END_VISP_NAMESPACE

#endif
