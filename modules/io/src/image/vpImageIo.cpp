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
 * Read/write images.
 */

/*!
  \file vpImageIo.cpp
  \brief Read/write images
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#include "private/vpImageIoBackend.h"

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

vpImageIo::vpImageFormatType vpImageIo::getFormat(const std::string &filename)
{
  std::string ext = vpImageIo::getExtension(filename);

  if (ext.compare(".PGM") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".pgm") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".PPM") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".ppm") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".JPG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".JPEG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpeg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".PNG") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".png") == 0)
    return FORMAT_PNG;
  // Formats supported by opencv
  else if (ext.compare(".TIFF") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".tiff") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".TIF") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".tif") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".BMP") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".bmp") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".DIB") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".dib") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".PBM") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".pbm") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".SR") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".sr") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".RAS") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".ras") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".JP2") == 0)
    return FORMAT_JPEG2000;
  else if (ext.compare(".jp2") == 0)
    return FORMAT_JPEG2000;
  else
    return FORMAT_UNKNOWN;
}

// return the extension of the file including the dot
std::string vpImageIo::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size() - 1);
  return ext;
}

/*!
  Read the contents of the image filename, allocate memory for the
  corresponding grayscale image, update its content, and return a reference to
  the image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  Supported formats are:
  - portable gray map: `*.pgm` file
  - portable pix map: `*.ppm` file
  - portable float map: `*.pfm` file
  - jpeg: `*.jpg`, `*.jpeg` files
  - png: `*.png` file

  If ViSP is build with OpenCV support, additional formats are considered:
  - `*.jp2`, `*.rs`, `*.ras`, `*.tiff`, `*.tif`, `*.png`, `*.bmp`, `*.pbm` files.

  If EXIF information is embedded in the image file, the EXIF orientation is ignored.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
  \param backend : Library backend type (see vpImageIo::vpImageIoBackendType) for image reading.
  This parameter is only used when the image need to be loaded in jpeg or png format. To know
  which is the default backend see respectively
  void vpImageIo::readJPEG(const vpImage<unsigned char> &, const std::string &, int) and
  void vpImageIo::readPNG(const vpImage<unsigned char> &, const std::string &, int).
 */
void vpImageIo::read(vpImage<unsigned char> &I, const std::string &filename, int backend)
{
  bool exist = vpIoTools::checkFilename(filename);
  if (!exist) {
    const std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
    throw(vpImageException(vpImageException::ioError, message));
  }

  // Allows to use ~ symbol or env variables in path
  std::string final_filename = vpIoTools::path(filename);

  bool try_opencv_reader = false;

  switch (getFormat(final_filename)) {
  case FORMAT_PGM:
    readPGM(I, final_filename);
    break;
  case FORMAT_PPM:
    readPPM(I, final_filename);
    break;
  case FORMAT_JPEG:
    readJPEG(I, final_filename, backend);
    break;
  case FORMAT_PNG:
    readPNG(I, final_filename, backend);
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_reader = true;
    break;
  }

  if (try_opencv_reader) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    const std::string message = "Cannot read file \"" + filename + "\": No backend able to support this image format";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Read the contents of the image filename, allocate memory for the
  corresponding grayscale image, update its content, and return a reference to
  the image.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  Supported formats are:
  - portable gray map: `*.pgm` file
  - portable pix map: `*.ppm` file
  - portable float map: `*.pfm` file
  - jpeg: `*.jpg`, `*.jpeg` files
  - png: `*.png` file

  If ViSP is build with OpenCV support, additional formats are considered:
  - `*.jp2`, `*.rs`, `*.ras`, `*.tiff`, `*.tif`, `*.png`, `*.bmp`, `*.pbm` files.

  If EXIF information is embedded in the image file, the EXIF orientation is ignored.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.
  \param backend : Library backend type (see vpImageIo::vpImageIoBackendType) for image reading.
  This parameter is only used when the image need to be loaded in jpeg or png format. To know
  which is the default backend see respectively
  void vpImageIo::readJPEG(const vpImage<unsigned char> &, const std::string &, int) and
  void vpImageIo::readPNG(const vpImage<unsigned char> &, const std::string &, int).
 */
void vpImageIo::read(vpImage<vpRGBa> &I, const std::string &filename, int backend)
{
  bool exist = vpIoTools::checkFilename(filename);
  if (!exist) {
    const std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
    throw(vpImageException(vpImageException::ioError, message));
  }
  // Allows to use ~ symbol or env variables in path
  std::string final_filename = vpIoTools::path(filename);

  bool try_opencv_reader = false;

  switch (getFormat(final_filename)) {
  case FORMAT_PGM:
    readPGM(I, final_filename);
    break;
  case FORMAT_PPM:
    readPPM(I, final_filename);
    break;
  case FORMAT_JPEG:
    readJPEG(I, final_filename, backend);
    break;
  case FORMAT_PNG:
    readPNG(I, final_filename, backend);
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_reader = true;
    break;
  }

  if (try_opencv_reader) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    const std::string message = "Cannot read file \"" + filename + "\": No backend able to support this image format";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Write the content of the image in the file which name is given by \e
  filename.

  Supported formats are:
  - portable gray map: `*.pgm` file
  - portable pix map: `*.ppm` file
  - portable float map: `*.pfm` file
  - jpeg: `*.jpg`, `*.jpeg` files
  - png: `*.png` file

  If ViSP is build with OpenCV support, additional formats are considered:
  - `*.jp2`, `*.rs`, `*.ras`, `*.tiff`, `*.tif`, `*.png`, `*.bmp`, `*.pbm` files.

  \param I : Image to write.
  \param filename : Name of the file containing the image.
  \param backend : Library backend type (see vpImageIo::vpImageIoBackendType) for image writing.
  This parameter is only used when the image need to be saved in jpeg or png format. To know
  which is the default backend see respectively
  void vpImageIo::writeJPEG(const vpImage<unsigned char> &, const std::string &, int, int) and
  void vpImageIo::writePNG(const vpImage<unsigned char> &, const std::string &, int).
*/
void vpImageIo::write(const vpImage<unsigned char> &I, const std::string &filename, int backend)
{
  bool try_opencv_writer = false;

  switch (getFormat(filename)) {
  case FORMAT_PGM:
    writePGM(I, filename);
    break;
  case FORMAT_PPM:
    writePPM(I, filename);
    break;
  case FORMAT_JPEG:
    writeJPEG(I, filename, backend);
    break;
  case FORMAT_PNG:
    writePNG(I, filename, backend);
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_writer = true;
    break;
  }

  if (try_opencv_writer) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, 90);
#else
    const std::string message = "Cannot write file \"" + filename + "\": No backend able to support this image format";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Write the content of the image in the file which name is given by \e
  filename.

  Supported formats are:
  - portable gray map: `*.pgm` file
  - portable pix map: `*.ppm` file
  - portable float map: `*.pfm` file
  - jpeg: `*.jpg`, `*.jpeg` files
  - png: `*.png` file

  If ViSP is build with OpenCV support, additional formats are considered:
  - `*.jp2`, `*.rs`, `*.ras`, `*.tiff`, `*.tif`, `*.png`, `*.bmp`, `*.pbm` files.

  \param I : Image to write.
  \param filename : Name of the file containing the image.
  \param backend : Library backend type (see vpImageIo::vpImageIoBackendType) for image writing.
  This parameter is only used when the image need to be saved in jpeg or png format. To know
  which is the default backend see respectively
  void vpImageIo::writeJPEG(const vpImage<vpRGBa> &, const std::string &, int, int) and
  void vpImageIo::writePNG(const vpImage<vpRGBa> &, const std::string &, int).
 */
void vpImageIo::write(const vpImage<vpRGBa> &I, const std::string &filename, int backend)
{
  bool try_opencv_writer = false;

  switch (getFormat(filename)) {
  case FORMAT_PGM:
    writePGM(I, filename);
    break;
  case FORMAT_PPM:
    writePPM(I, filename);
    break;
  case FORMAT_JPEG:
    writeJPEG(I, filename, backend);
    break;
  case FORMAT_PNG:
    writePNG(I, filename, backend);
    break;
  case FORMAT_TIFF:
  case FORMAT_BMP:
  case FORMAT_DIB:
  case FORMAT_PBM:
  case FORMAT_RASTER:
  case FORMAT_JPEG2000:
  case FORMAT_UNKNOWN:
    try_opencv_writer = true;
    break;
  }

  if (try_opencv_writer) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, 90);
#else
    const std::string message = "Cannot write file \"" + filename + "\": No backend able to support this image format";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load a jpeg image. If it is a color image it is converted in gray.
  \param[out] I : Gray level image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_STB_IMAGE_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. opencv, 2. system, 3. stb_image
void vpImageIo::readJPEG(vpImage<unsigned char> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_JPEG)
    // Libjpeg backend is not available to read file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    readJPEGLibjpeg(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": jpeg library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    readStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    readSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load a jpeg image. If it is a gray image it is converted in color.
  \param[out] I : Color image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_STB_IMAGE_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. opencv, 2. system, 3. stb_image
void vpImageIo::readJPEG(vpImage<vpRGBa> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_JPEG)
    // Libjpeg backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    readJPEGLibjpeg(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": jpeg library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    readStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    readSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load an image in png format. If it is a color image it is converted in gray.
  \param[out] I : Gray level image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_STB_IMAGE_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. system, 2. opencv, 3. stb_image
void vpImageIo::readPNG(vpImage<unsigned char> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_PNG)
    // Libpng backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_PNG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    readPNGLibpng(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": png library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    readStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    readSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load an image in png format. If it is a gray level image it is converted in color.
  \param[out] I : Color image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_STB_IMAGE_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. opencv, 2. stb_image
void vpImageIo::readPNG(vpImage<vpRGBa> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_PNG)
    // Libpng backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    readPNGLibpng(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": png library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    readStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    readSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load an image in EXR format.
  \param[out] I : Floating-point single channel image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Only OpenCV and the Tiny OpenEXR image libraries can currently read EXR image.
  The default backend vpImageIo::IO_DEFAULT_BACKEND is the Tiny OpenEXR image library.
 */
void vpImageIo::readEXR(vpImage<float> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND || backend == IO_STB_IMAGE_BACKEND) {
    // This backend cannot read file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if !defined(VISP_HAVE_TINYEXR)
    // TinyEXR backend is not available to read file \"" + filename + "\": switch to the OpenCV backend
    backend = IO_OPENCV_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_TINYEXR)
    readEXRTiny(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": Default TinyEXR backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Load an image in EXR format.
  \param[out] I : Floating-point three channels image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Only OpenCV and the Tiny OpenEXR image libraries can currently read EXR image.
  The default backend vpImageIo::IO_DEFAULT_BACKEND is the Tiny OpenEXR image library.
 */
void vpImageIo::readEXR(vpImage<vpRGBf> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND || backend == IO_STB_IMAGE_BACKEND) {
    // This backend cannot read file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to read file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if !defined(VISP_HAVE_TINYEXR)
    // TinyEXR backend is not available to read file \"" + filename + "\": switch to the OpenCV backend
    backend = IO_OPENCV_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_TINYEXR)
    readEXRTiny(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot read file \"" + filename + "\": TinyEXR backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in jpeg format.
  \param[in] I : Gray level image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SIMDLIB_BACKEND.
  \param[in] quality : Image quality percentage in range 0-100.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. system, 2. opencv, 3. simd
void vpImageIo::writeJPEG(const vpImage<unsigned char> &I, const std::string &filename, int backend, int quality)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_JPEG)
#if defined(VISP_HAVE_SIMDLIB)
    // Libjpeg backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // Libjpeg backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
#if defined(VISP_HAVE_SIMDLIB)
    // OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // OpenCV backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": no available backend";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    writeJPEGLibjpeg(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": jpeg backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) \
    && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    writeJPEGSimdlib(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writeJPEGStb(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in jpeg format.
  \param[in] I : Color image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SIMDLIB_BACKEND.
  \param[in] quality : Image quality percentage in range 0-100.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. system, 2. opencv, , 3. simd
void vpImageIo::writeJPEG(const vpImage<vpRGBa> &I, const std::string &filename, int backend, int quality)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_JPEG)
#if defined(VISP_HAVE_SIMDLIB)
    // Libjpeg backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // Libjpeg backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
#if defined(VISP_HAVE_SIMDLIB)
    // OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // OpenCV backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    writeJPEGLibjpeg(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": jpeg library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    writeJPEGSimdlib(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writeJPEGStb(I, filename, quality);
#else
    (void)I;
    (void)filename;
    (void)quality;
    const std::string message = "Cannot save file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in png format.
  \param[in] I : Gray level image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SIMDLIB_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. opencv, 2. simd
void vpImageIo::writePNG(const vpImage<unsigned char> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_PNG)
#if defined(VISP_HAVE_SIMDLIB)
    // Libpng backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // Libpng backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
#if defined(VISP_HAVE_SIMDLIB)
    // OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // OpenCV backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, 90);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    writePNGSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writePNGStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    writePNGLibpng(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": png library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in png format.
  \param[in] I : Color image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Depending on its availability, the default backend vpImageIo::IO_DEFAULT_BACKEND is chosen in the following order:
  vpImageIo::IO_OPENCV_BACKEND, vpImageIo::IO_SYSTEM_LIB_BACKEND, vpImageIo::IO_SIMDLIB_BACKEND.
 */
// Strategy based on benchmark: see https://github.com/lagadic/visp/pull/1004
// Default: 1. opencv, 2. system, 3. simd
void vpImageIo::writePNG(const vpImage<vpRGBa> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if !defined(VISP_HAVE_PNG)
#if defined(VISP_HAVE_SIMDLIB)
    // Libpng backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // Libpng backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
#if defined(VISP_HAVE_SIMDLIB)
    // OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend
    backend = IO_SIMDLIB_BACKEND;
#else
    // OpenCV backend is not available to save file \"" + filename + "\": switch to stb_image backend
    backend = IO_STB_IMAGE_BACKEND;
#endif
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_SIMDLIB)
    backend = IO_SIMDLIB_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": no backend available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename, 90);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SIMDLIB_BACKEND) {
#if defined(VISP_HAVE_SIMDLIB)
    writePNGSimdlib(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": Simd library backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writePNGStb(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": stb_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    writePNGLibpng(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": libpng backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in EXR format.
  \param[in] I : Floating-point single channel image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Only OpenCV and the Tiny OpenEXR image libraries can currently save EXR image.
  The default backend vpImageIo::IO_DEFAULT_BACKEND is the Tiny OpenEXR image library.
 */
void vpImageIo::writeEXR(const vpImage<float> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND || backend == IO_STB_IMAGE_BACKEND) {
    // This backend cannot save file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    (void)I;
    // OpenCV backend is not available to save file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if !defined(VISP_HAVE_TINYEXR)
    // TinyEXR backend is not available to save file \"" + filename + "\": switch to the OpenCV backend
    backend = IO_OPENCV_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_TINYEXR)
    writeEXRTiny(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": TinyEXR backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in EXR format.
  \param[in] I : Floating-point three channels image.
  \param[in] filename : Image location.
  \param[in] backend : Supported backends are described in vpImageIo::vpImageIoBackendType.
  Only OpenCV and the Tiny OpenEXR image libraries can currently save EXR image.
  The default backend vpImageIo::IO_DEFAULT_BACKEND is the Tiny OpenEXR image library.
 */
void vpImageIo::writeEXR(const vpImage<vpRGBf> &I, const std::string &filename, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND || backend == IO_STB_IMAGE_BACKEND) {
    // This backend cannot save file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    // OpenCV backend is not available to save file \"" + filename + "\": switch to the default TinyEXR backend
    backend = IO_DEFAULT_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if !defined(VISP_HAVE_TINYEXR)
    // TinyEXR backend is not available to save file \"" + filename + "\": switch to the OpenCV backend
    backend = IO_OPENCV_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writeOpenCV(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_TINYEXR)
    writeEXRTiny(I, filename);
#else
    (void)I;
    (void)filename;
    const std::string message = "Cannot save file \"" + filename + "\": TinyEXR backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Save an image in portable float map format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePFM(const vpImage<float> &I, const std::string &filename) { vp_writePFM(I, filename); }

/*!
  Save a high-dynamic range (not restricted to the [0-255] intensity range) floating-point image
  in portable float map format.
  \param[in] I : Grayscale floating-point image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePFM_HDR(const vpImage<float> &I, const std::string &filename) { vp_writePFM_HDR(I, filename); }

/*!
  Save a RGB high-dynamic range (not restricted to the [0-255] intensity range) floating-point image
  in portable float map format.
  \param[in] I : RGB floating-point image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePFM_HDR(const vpImage<vpRGBf> &I, const std::string &filename) { vp_writePFM_HDR(I, filename); }

/*!
  Save an image in portable gray map format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePGM(const vpImage<unsigned char> &I, const std::string &filename) { vp_writePGM(I, filename); }

/*!
  Save a gray level image in portable gray map format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePGM(const vpImage<short> &I, const std::string &filename) { vp_writePGM(I, filename); }

/*!
  Save a color image in portable gray map format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePGM(const vpImage<vpRGBa> &I, const std::string &filename) { vp_writePGM(I, filename); }

/*!
  Load an image in portable float map format.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPFM(vpImage<float> &I, const std::string &filename) { vp_readPFM(I, filename); }

/*!
  Load an image in portable float map format and not restricted to the [0, 255] dynamic range.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPFM_HDR(vpImage<float> &I, const std::string &filename) { vp_readPFM_HDR(I, filename); }

/*!
  Load an image in portable float map format and not restricted to the [0, 255] dynamic range.
  \param[out] I : Image read from filename and with three channels.
  \param[in] filename : Image location.
 */
void vpImageIo::readPFM_HDR(vpImage<vpRGBf> &I, const std::string &filename) { vp_readPFM_HDR(I, filename); }

/*!
  Load an image in portable gray map format. If the image is in color, it is converted in gray level.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPGM(vpImage<unsigned char> &I, const std::string &filename) { vp_readPGM(I, filename); }

/*!
  Load an image in portable float map format. If the image is in gray, it is converted in color.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPGM(vpImage<vpRGBa> &I, const std::string &filename) { vp_readPGM(I, filename); }

/*!
  Load an image in portable pixmap format. If the image is in color, it is converted in gray level.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPPM(vpImage<unsigned char> &I, const std::string &filename) { vp_readPPM(I, filename); }

/*!
  Load an image in portable pixmap format. If the image is in gray, it is converted in color.
  \param[out] I : Image read from filename.
  \param[in] filename : Image location.
 */
void vpImageIo::readPPM(vpImage<vpRGBa> &I, const std::string &filename) { vp_readPPM(I, filename); }

/*!
  Save a gray level image in portable pixmap format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePPM(const vpImage<unsigned char> &I, const std::string &filename) { vp_writePPM(I, filename); }

/*!
  Save a color level image in portable pixmap format.
  \param[in] I : Image to save.
  \param[in] filename : Image location.
 */
void vpImageIo::writePPM(const vpImage<vpRGBa> &I, const std::string &filename) { vp_writePPM(I, filename); }

/*!
  Read the content of the grayscale image bitmap stored in memory and encoded using the PNG format.
  \param[in] buffer : Grayscale image buffer encoded in PNG as 1-D unsigned char vector.
  \param[out] I : Output decoded grayscale image.
  \param[in] backend : Supported backends are IO_OPENCV_BACKEND and IO_STB_IMAGE_BACKEND, default IO_DEFAULT_BACKEND
  will choose IO_OPENCV_BACKEND if available or IO_STB_IMAGE_BACKEND otherwise.
 */
void vpImageIo::readPNGfromMem(const std::vector<unsigned char> &buffer, vpImage<unsigned char> &I, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND) {
    backend = IO_STB_IMAGE_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png read: OpenCV or std_image backend are not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) \
    && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readPNGfromMemOpenCV(buffer, I);
    (void)backend;
#else
    (void)buffer;
    (void)I;
    (void)backend;
    const std::string message = "Cannot in-memory png read: OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else {
#if defined(VISP_HAVE_STBIMAGE)
    readPNGfromMemStb(buffer, I);
    (void)backend;
#else
    (void)buffer;
    (void)I;
    (void)backend;
    const std::string message = "Cannot in-memory png read: std_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Read the content of the grayscale image bitmap stored in memory and encoded using the PNG format.
  \param[in] buffer : Color image buffer encoded in PNG as 1-D unsigned char vector.
  \param[out] I : Output decoded color image.
  \param[in] backend : Supported backends are IO_OPENCV_BACKEND and IO_STB_IMAGE_BACKEND, default IO_DEFAULT_BACKEND
  will choose IO_OPENCV_BACKEND if available or IO_STB_IMAGE_BACKEND otherwise.
 */
void vpImageIo::readPNGfromMem(const std::vector<unsigned char> &buffer, vpImage<vpRGBa> &I, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND) {
    backend = IO_STB_IMAGE_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png read: OpenCV or std_image backend are not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) \
    && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    readPNGfromMemOpenCV(buffer, I);
    (void)backend;
#else
    (void)buffer;
    (void)I;
    (void)backend;
    const std::string message = "Cannot in-memory png read: OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else {
#if defined(VISP_HAVE_STBIMAGE)
    readPNGfromMemStb(buffer, I);
    (void)backend;
#else
    (void)buffer;
    (void)I;
    (void)backend;
    const std::string message = "Cannot in-memory png read: std_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  In-memory PNG encoding of the grayscale image.

  \param[in] I : Input grayscale image.
  \param[out] buffer : Encoded image as 1-D unsigned char vector using the PNG format.
  \param[in] backend : Supported backends are IO_OPENCV_BACKEND and IO_STB_IMAGE_BACKEND, default IO_DEFAULT_BACKEND
  will choose IO_OPENCV_BACKEND if available or IO_STB_IMAGE_BACKEND otherwise.
*/
void vpImageIo::writePNGtoMem(const vpImage<unsigned char> &I, std::vector<unsigned char> &buffer, int backend)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND) {
    backend = IO_STB_IMAGE_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: OpenCV or std_image backend are not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) \
    && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writePNGtoMemOpenCV(I, buffer);
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writePNGtoMemStb(I, buffer);
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: std_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else {
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
    const std::string message = "The " + std::to_string(backend) + " backend is not available.";
    throw(vpImageException(vpImageException::ioError, message));
#else
    throw(vpImageException(vpImageException::ioError, "The %d backend is not available", backend));
#endif
  }
}

/*!
  In-memory PNG encoding of the color image.

  \param[in] I : Input color image.
  \param[out] buffer : Encoded image as 1-D unsigned char vector using the PNG format.
  \param[in] backend : Supported backends are IO_OPENCV_BACKEND and IO_STB_IMAGE_BACKEND, default IO_DEFAULT_BACKEND
  will choose IO_OPENCV_BACKEND if available or IO_STB_IMAGE_BACKEND otherwise.
  \param[in] saveAlpha : If true, alpha channel is also used for encoding.
*/
void vpImageIo::writePNGtoMem(const vpImage<vpRGBa> &I, std::vector<unsigned char> &buffer, int backend, bool saveAlpha)
{
  if (backend == IO_SYSTEM_LIB_BACKEND || backend == IO_SIMDLIB_BACKEND) {
    backend = IO_STB_IMAGE_BACKEND;
  }
  else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS))
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }
  else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS)
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_STBIMAGE)
    backend = IO_STB_IMAGE_BACKEND;
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: OpenCV or std_image backend are not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_IMGCODECS)) || ((VISP_HAVE_OPENCV_VERSION < 0x030000) \
    && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC))
    writePNGtoMemOpenCV(I, buffer, saveAlpha);
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: OpenCV backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else if (backend == IO_STB_IMAGE_BACKEND) {
#if defined(VISP_HAVE_STBIMAGE)
    writePNGtoMemStb(I, buffer, saveAlpha);
#else
    (void)I;
    (void)buffer;
    const std::string message = "Cannot in-memory png write: std_image backend is not available";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
  else {
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
    const std::string message = "The " + std::to_string(backend) + " backend is not available.";
    throw(vpImageException(vpImageException::ioError, message));
#else
    throw(vpImageException(vpImageException::ioError, "The %d backend is not available", backend));
#endif
  }
}
