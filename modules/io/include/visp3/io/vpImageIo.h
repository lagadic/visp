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
  \file vpImageIo.h
  \brief Read/write images
*/

#ifndef VP_IMAGE_IO_H
#define VP_IMAGE_IO_H

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>

#include <iostream>
#include <stdio.h>

BEGIN_VISP_NAMESPACE

/*!
  \class vpImageIo

  \ingroup group_io_image

  \brief Read/write images with various image format.

  This class has its own implementation of PGM and PPM images read/write.

  This class may benefit from optional 3rd parties:
  - libpng: If installed this optional 3rd party is used to read/write PNG
    images. Installation instructions are provided here
    https://visp.inria.fr/3rd_png.
  - libjpeg: If installed this optional 3rd party is used to read/write JPEG
    images. Installation instructions are provided here
    https://visp.inria.fr/3rd_jpeg.
  - OpenCV: If installed this optional 3rd party is used to read/write other
    image formats TIFF, BMP, DIB, PBM, RASTER, JPEG2000. If libpng or libjpeg is
    not installed OpenCV is also used to consider these image formats.
    Installation instructions are provided here https://visp.inria.fr/3rd_opencv.

  The code below shows how to convert an PPM P6 image file format into
  a PGM P5 image file format. The extension of the filename is here
  used in read() and write() functions to set the image file format
  (".pgm" for PGM P5 and ".ppm" for PPM P6).

  \code
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<unsigned char> I;
  #if defined(_WIN32)
    std::string filename("C:/Temp/visp-images/Klimt/Klimt.ppm");
  #else // UNIX
    std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.ppm");
  #endif

    vpImageIo::read(I, filename); // Convert the color image in a gray level image
    vpImageIo::write(I, "Klimt.pgm"); // Write the image in a PGM P5 image file format
  }
  \endcode

  This other example available in tutorial-image-reader.cpp shows how to
  read/write jpeg images. It supposes that `libjpeg` is installed.

  \include tutorial-image-reader.cpp
*/

class VISP_EXPORT vpImageIo
{
private:
  typedef enum
  {
    FORMAT_PGM,
    FORMAT_PPM,
    FORMAT_JPEG,
    FORMAT_PNG,
    // Formats supported by opencv
    FORMAT_TIFF,
    FORMAT_BMP,
    FORMAT_DIB,
    FORMAT_PBM,
    FORMAT_RASTER,
    FORMAT_JPEG2000,
    FORMAT_UNKNOWN
  } vpImageFormatType;

  static vpImageFormatType getFormat(const std::string &filename);
  static std::string getExtension(const std::string &filename);

public:
  //! Image IO backend for only jpeg and png formats image loading and saving
  enum vpImageIoBackendType
  {
    IO_DEFAULT_BACKEND,    //!< Default backend
    IO_SYSTEM_LIB_BACKEND, //!< Use system libraries like libpng or libjpeg-turbo
    IO_OPENCV_BACKEND,     //!< Use OpenCV imgcodecs module
    IO_SIMDLIB_BACKEND,    //!< Use embedded simd library
    IO_STB_IMAGE_BACKEND   //!< Use embedded stb_image library
  };

  static void read(vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void read(vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void write(const vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void write(const vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void readPFM(vpImage<float> &I, const std::string &filename);
  static void readPFM_HDR(vpImage<float> &I, const std::string &filename);
  static void readPFM_HDR(vpImage<vpRGBf> &I, const std::string &filename);

  static void readPGM(vpImage<unsigned char> &I, const std::string &filename);
  static void readPGM(vpImage<vpRGBa> &I, const std::string &filename);

  static void readPPM(vpImage<unsigned char> &I, const std::string &filename);
  static void readPPM(vpImage<vpRGBa> &I, const std::string &filename);

  static void readJPEG(vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void readJPEG(vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void readPNG(vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void readPNG(vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void readEXR(vpImage<float> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void readEXR(vpImage<vpRGBf> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void writePFM(const vpImage<float> &I, const std::string &filename);
  static void writePFM_HDR(const vpImage<float> &I, const std::string &filename);
  static void writePFM_HDR(const vpImage<vpRGBf> &I, const std::string &filename);

  static void writePGM(const vpImage<unsigned char> &I, const std::string &filename);
  static void writePGM(const vpImage<short> &I, const std::string &filename);
  static void writePGM(const vpImage<vpRGBa> &I, const std::string &filename);

  static void writePPM(const vpImage<unsigned char> &I, const std::string &filename);
  static void writePPM(const vpImage<vpRGBa> &I, const std::string &filename);

  static void writeJPEG(const vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND,
                        int quality = 90);
  static void writeJPEG(const vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND,
                        int quality = 90);

  static void writePNG(const vpImage<unsigned char> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void writePNG(const vpImage<vpRGBa> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void writeEXR(const vpImage<float> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);
  static void writeEXR(const vpImage<vpRGBf> &I, const std::string &filename, int backend = IO_DEFAULT_BACKEND);

  static void readPNGfromMem(const std::vector<unsigned char> &buffer, vpImage<unsigned char> &I,
      int backend = IO_DEFAULT_BACKEND);
  static void readPNGfromMem(const std::vector<unsigned char> &buffer, vpImage<vpRGBa> &I,
      int backend = IO_DEFAULT_BACKEND);

  static void writePNGtoMem(const vpImage<unsigned char> &I, std::vector<unsigned char> &buffer,
      int backend = IO_DEFAULT_BACKEND);
  static void writePNGtoMem(const vpImage<vpRGBa> &I, std::vector<unsigned char> &buffer,
      int backend = IO_DEFAULT_BACKEND, bool saveAlpha = false);
};

END_VISP_NAMESPACE

#endif
