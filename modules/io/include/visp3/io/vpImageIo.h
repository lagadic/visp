/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
  \file vpImageIo.h
  \brief Read/write images
*/

#ifndef vpIMAGEIO_H
#define vpIMAGEIO_H

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>

#include <iostream>
#include <stdio.h>

#if defined(_WIN32)
// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <windows.h>
#endif

#if defined(VISP_HAVE_JPEG)
#include <jerror.h>
#include <jpeglib.h>
#endif

#if defined(VISP_HAVE_PNG)
#include <png.h>
#endif

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
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  vpImage<unsigned char> I;
#if defined(_WIN32)
  std::string filename("C:/temp/ViSP-images/Klimt/Klimt.ppm");
#else // UNIX
  std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.ppm");
#endif

  vpImageIo::read(I, filename); // Convert the color image in a gray level image
  vpImageIo::write(I, "Klimt.pgm"); // Write the image in a PGM P5 image file format
}
  \endcode

  This other example available in tutorial-image-reader.cpp shows how to
read/write jpeg images. It supposes that \c libjpeg is installed. \include
tutorial-image-reader.cpp
*/

class VISP_EXPORT vpImageIo
{

private:
  typedef enum {
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
  static void read(vpImage<unsigned char> &I, const std::string &filename);
  static void read(vpImage<vpRGBa> &I, const std::string &filename);

  static void write(const vpImage<unsigned char> &I, const std::string &filename);
  static void write(const vpImage<vpRGBa> &I, const std::string &filename);

  static void readPFM(vpImage<float> &I, const std::string &filename);

  static void readPGM(vpImage<unsigned char> &I, const std::string &filename);
  static void readPGM(vpImage<vpRGBa> &I, const std::string &filename);

  static void readPPM(vpImage<unsigned char> &I, const std::string &filename);
  static void readPPM(vpImage<vpRGBa> &I, const std::string &filename);

#if (defined(VISP_HAVE_JPEG) || defined(VISP_HAVE_OPENCV))
  static void readJPEG(vpImage<unsigned char> &I, const std::string &filename);
  static void readJPEG(vpImage<vpRGBa> &I, const std::string &filename);
#endif

#if (defined(VISP_HAVE_PNG) || defined(VISP_HAVE_OPENCV))
  static void readPNG(vpImage<unsigned char> &I, const std::string &filename);
  static void readPNG(vpImage<vpRGBa> &I, const std::string &filename);
#endif

  static void writePFM(const vpImage<float> &I, const std::string &filename);

  static void writePGM(const vpImage<unsigned char> &I, const std::string &filename);
  static void writePGM(const vpImage<short> &I, const std::string &filename);
  static void writePGM(const vpImage<vpRGBa> &I, const std::string &filename);

  static void writePPM(const vpImage<unsigned char> &I, const std::string &filename);
  static void writePPM(const vpImage<vpRGBa> &I, const std::string &filename);

#if (defined(VISP_HAVE_JPEG) || defined(VISP_HAVE_OPENCV))
  static void writeJPEG(const vpImage<unsigned char> &I, const std::string &filename);
  static void writeJPEG(const vpImage<vpRGBa> &I, const std::string &filename);
#endif

#if (defined(VISP_HAVE_PNG) || defined(VISP_HAVE_OPENCV))
  static void writePNG(const vpImage<unsigned char> &I, const std::string &filename);
  static void writePNG(const vpImage<vpRGBa> &I, const std::string &filename);
#endif
};
#endif
