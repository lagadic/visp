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
 * Convert image types.
 */

/*!
  \file vpImageConvert_yarp.cpp
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_YARP

#include <visp3/core/vpImageConvert.h>

BEGIN_VISP_NAMESPACE
/*!
  Convert a vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[out] dest : Destination image in YARP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    vpImage<unsigned char> I; // A monochrome image
    // Read an image on a disk
    vpImageIo::read(I, "image.pgm");

    yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
    // Convert the vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>
    vpImageConvert::convert(I, Iyarp);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, yarp::sig::ImageOf<yarp::sig::PixelMono> *dest,
                             bool copyData)
{
  if (copyData) {
    dest->resize(src.getWidth(), src.getHeight());
    for (unsigned int i = 0; i < src.getHeight(); ++i) {
      for (unsigned int j = 0; j < src.getWidth(); ++j) {
        dest->pixel(j, i) = src[i][j];
      }
    }
  }
  else {
    dest->setExternal(src.bitmap, static_cast<int>(src.getCols()), static_cast<int>(src.getRows()));
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelMono\> to a vpImage\<unsigned
  char\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  #if defined(VISP_HAVE_YARP)
    #include <yarp/sig/ImageFile.h>
  #endif

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelMono >();
    // Read an image on a disk
    yarp::sig::file::read(*Iyarp, "image.pgm");

    // Convert the yarp::sig::ImageOf<yarp::sig::PixelMono> to a vpImage<unsigned char>
    vpImage<unsigned char> I;
    vpImageConvert::convert(Iyarp, I);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelMono> *src, vpImage<unsigned char> &dest,
                             bool copyData)
{
  dest.resize(src->height(), src->width());
  (void)(copyData);
  for (unsigned int i = 0; i < dest.getHeight(); ++i) {
    for (unsigned int j = 0; j < dest.getWidth(); ++j) {
      dest[i][j] = src->pixel(j, i);
    }
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgba>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[in] dest : Destination image in YARP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/core/vpRGBa.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    vpImage<vpRGBa> I; // A color image
    // Read an image on a disk
    vpImageIo::read(I,"image.jpg");

    yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
    // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgba>
    vpImageConvert::convert(I,Iyarp);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgba> *dest, bool copyData)
{
  if (copyData) {
    dest->resize(src.getWidth(), src.getHeight());
    memcpy(dest->getRawImage(), src.bitmap, src.getHeight() * src.getWidth() * sizeof(vpRGBa));
  }
  else {
    dest->setExternal(src.bitmap, static_cast<int>(src.getCols()), static_cast<int>(src.getRows()));
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgba> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] copyData : Set to true to copy all the image content. If false we
  only update the image pointer.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/core/vpRGBa.h>
  #include <visp3/io/vpImageIo.h>

  #if defined(VISP_HAVE_YARP)
    #include <yarp/sig/ImageFile.h>
  #endif

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgba >();
    // Read an image on a disk
    yarp::sig::file::read(*Iyarp,"image.pgm");

    // Convert the yarp::sig::ImageOf<yarp::sig::PixelRgba> to a vpImage<vpRGBa>
    vpImage<vpRGBa> I;
    vpImageConvert::convert(Iyarp,I);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelRgba> *src, vpImage<vpRGBa> &dest, bool copyData)
{
  dest.resize(src->height(), src->width());
  (void)(copyData);
  for (unsigned int i = 0; i < dest.getHeight(); ++i) {
    for (unsigned int j = 0; j < dest.getWidth(); ++j) {
      dest[i][j].R = src->pixel(j, i).r;
      dest[i][j].G = src->pixel(j, i).g;
      dest[i][j].B = src->pixel(j, i).b;
      dest[i][j].A = src->pixel(j, i).a;
    }
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgb>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  \param[in] src : Source image in ViSP format.
  \param[out] dest : Destination image in YARP format.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/core/vpRGBa.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    vpImage<vpRGBa> I; // A color image
    // Read an image on a disk
    vpImageIo::read(I,"image.jpg");

    yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgb >();
    // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgb>
    vpImageConvert::convert(I,Iyarp);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, yarp::sig::ImageOf<yarp::sig::PixelRgb> *dest)
{
  const unsigned int srcRows = src.getRows(), srcWidth = src.getWidth();
  dest->resize(src.getWidth(), src.getHeight());
  for (unsigned int i = 0; i < srcRows; ++i) {
    for (unsigned int j = 0; j < srcWidth; ++j) {
      dest->pixel(j, i).r = src[i][j].R;
      dest->pixel(j, i).g = src[i][j].G;
      dest->pixel(j, i).b = src[i][j].B;
    }
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgb> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See
  http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for the
  YARP image class documentation.

  The alpha component of the resulting image is set to vpRGBa::alpha_default.

  \param[in] src : Source image in YARP format.
  \param[out] dest : Destination image in ViSP format.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/core/vpRGBa.h>
  #include <visp3/io/vpImageIo.h>

  #if defined(VISP_HAVE_YARP)
    #include <yarp/sig/ImageFile.h>
  #endif

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_YARP)
    yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf<yarp::sig::PixelRgb >();
    // Read an image on a disk
    yarp::sig::file::read(*Iyarp,"image.pgm");

    // Convert the yarp::sig::ImageOf<yarp::sig::PixelRgb> to a vpImage<vpRGBa>
    vpImage<vpRGBa> I;
    vpImageConvert::convert(Iyarp,I);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const yarp::sig::ImageOf<yarp::sig::PixelRgb> *src, vpImage<vpRGBa> &dest)
{
  const int srcHeight = src->height(), srcWidth = src->width();
  dest.resize(srcHeight, srcWidth);
  for (int i = 0; i < srcHeight; ++i) {
    for (int j = 0; j < srcWidth; ++j) {
      dest[i][j].R = src->pixel(j, i).r;
      dest[i][j].G = src->pixel(j, i).g;
      dest[i][j].B = src->pixel(j, i).b;
      dest[i][j].A = vpRGBa::alpha_default;
    }
  }
}

END_VISP_NAMESPACE

#endif
