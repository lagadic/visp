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
 * Read/write images.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpImageIo.cpp
  \brief Read/write images
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>

#include "private/vpImageIoBackend.h"

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
  corresponding greyscale image, update its content, and return a reference to
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
    std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
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
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#else
    std::string message = "Cannot read file \"" + filename + "\": No backend able to support this image format";
    throw(vpImageException(vpImageException::ioError, message));
#endif
  }
}

/*!
  Read the contents of the image filename, allocate memory for the
  corresponding greyscale image, update its content, and return a reference to
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
    std::string message = "Cannot read file: \"" + std::string(filename) + "\" doesn't exist";
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
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#else
    std::string message = "Cannot read file \"" + filename + "\": No backend able to support this image format";
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
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, 90);
#else
    std::string message = "Cannot write file \"" + filename + "\": No backend able to support this image format";
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
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, 90);
#else
    std::string message = "Cannot write file \"" + filename + "\": No backend able to support this image format";
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
    std::string message =
        "Libjpeg backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#else
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    readJPEGLibjpeg(I, filename);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    readStb(I, filename);
  } else if (backend == IO_SIMDLIB_BACKEND) {
    readSimdlib(I, filename);
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
    std::string message =
        "Libjpeg backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#elif defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#else
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    readJPEGLibjpeg(I, filename);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    readStb(I, filename);
  } else if (backend == IO_SIMDLIB_BACKEND) {
    readSimdlib(I, filename);
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
    std::string message =
        "Libpng backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_PNG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    readPNGLibpng(I, filename);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    readStb(I, filename);
  } else if (backend == IO_SIMDLIB_BACKEND) {
    readSimdlib(I, filename);
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
    std::string message =
        "Libpng backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to stb_image backend";
    backend = IO_STB_IMAGE_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_STB_IMAGE_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    readPNGLibpng(I, filename);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    readStb(I, filename);
  } else if (backend == IO_SIMDLIB_BACKEND) {
    readSimdlib(I, filename);
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
    std::string message =
        "This backend cannot read file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    readEXRTiny(I, filename);
#else
    (void)I;
    std::string message =
        "TinyEXR backend is not available to read file \"" + filename + "\": cxx standard should be greater or equal to cxx11";
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
    std::string message =
        "This backend cannot read file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message =
        "OpenCV backend is not available to read file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    readOpenCV(I, filename);
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    readEXRTiny(I, filename);
#else
    (void)I;
    std::string message =
        "TinyEXR backend is not available to read file \"" + filename + "\": cxx standard should be greater or equal to cxx11";
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
    std::string message = "Libjpeg backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message = "OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_SIMDLIB_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    writeJPEGLibjpeg(I, filename, quality);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, quality);
#endif
  } else if (backend == IO_SIMDLIB_BACKEND) {
    writeJPEGSimdlib(I, filename, quality);
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    writeJPEGStb(I, filename, quality);
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
    std::string message = "Libjpeg backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message = "OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    backend = IO_SYSTEM_LIB_BACKEND;
#elif defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_SIMDLIB_BACKEND;
#endif
  }

  if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_JPEG)
    writeJPEGLibjpeg(I, filename, quality);
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, quality);
#endif
  } else if (backend == IO_SIMDLIB_BACKEND) {
    writeJPEGSimdlib(I, filename, quality);
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    writeJPEGStb(I, filename, quality);
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
    std::string message = "Libpng backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message = "OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_SIMDLIB_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, 90);
#endif
  } else if (backend == IO_SIMDLIB_BACKEND) {
    writePNGSimdlib(I, filename);
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    writePNGStb(I, filename);
  } else if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    writePNGLibpng(I, filename);
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
    std::string message = "Libpng backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    std::string message = "OpenCV backend is not available to save file \"" + filename + "\": switch to simd backend";
    backend = IO_SIMDLIB_BACKEND;
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    backend = IO_OPENCV_BACKEND;
#else
    backend = IO_SIMDLIB_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename, 90);
#endif
  } else if (backend == IO_SIMDLIB_BACKEND) {
    writePNGSimdlib(I, filename);
  } else if (backend == IO_STB_IMAGE_BACKEND) {
    writePNGStb(I, filename);
  } else if (backend == IO_SYSTEM_LIB_BACKEND) {
#if defined(VISP_HAVE_PNG)
    writePNGLibpng(I, filename);
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
    std::string message =
        "This backend cannot save file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    (void)I;
    std::string message =
        "OpenCV backend is not available to save file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename);
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    writeEXRTiny(I, filename);
#else
    std::string message =
        "TinyEXR backend is not available to save file \"" + filename + "\": cxx standard should be greater or equal to cxx11";
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
    std::string message =
        "This backend cannot save file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
  } else if (backend == IO_OPENCV_BACKEND) {
#if !(defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100)
    (void)I;
    std::string message =
        "OpenCV backend is not available to save file \"" + filename + "\": switch to the default TinyEXR backend";
    backend = IO_DEFAULT_BACKEND;
#endif
  }

  if (backend == IO_OPENCV_BACKEND) {
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
    writeOpenCV(I, filename);
#endif
  } else if (backend == IO_DEFAULT_BACKEND) {
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    writeEXRTiny(I, filename);
#else
    std::string message =
        "TinyEXR backend is not available to save file \"" + filename + "\": cxx standard should be greater or equal to cxx11";
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
