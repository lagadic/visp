/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * OpenCV backend for image I/O operations.
 *
 *****************************************************************************/

/*!
  \file vpImageIoOpenCV.cpp
  \brief OpenCV backend for image I/O operations.
*/

#include "vpImageIoBackend.h"

#ifdef VISP_HAVE_OPENCV
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000) // Require opencv >= 3.0.0
#include <opencv2/imgcodecs.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020408) // Require opencv >= 2.4.8
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/legacy/legacy.hpp>
#else
#include <highgui.h>
#endif
#endif

#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_OPENCV)

/*!
  Read the contents of the image file, allocate memory
  for the corresponding gray level image, if necessary convert the data in
  gray level, and set the bitmap with the gray level data.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If EXIF information is embedded in the image file, the EXIF orientation is ignored.

  \param I : Image to set with the \e filename content.
  \param filename : Name of the file containing the image.

*/
void readOpenCV(vpImage<unsigned char> &I, const std::string &filename)
{
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
#if VISP_HAVE_OPENCV_VERSION >= 0x030200
  int flags = cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION;
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
  int flags = cv::IMREAD_GRAYSCALE;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  int flags = CV_LOAD_IMAGE_GRAYSCALE;
#endif
  cv::Mat Ip = cv::imread(filename.c_str(), flags);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

/*!
  Read the contents of an image file, allocate
  memory for the corresponding color image, and set
  the bitmap whith the content of the file.

  If the image has been already initialized, memory allocation is done
  only if the new image size is different, else we re-use the same
  memory space.

  If the file corresponds to a grayscaled image, a conversion is done to deal
  with \e I which is a color image.

  If EXIF information is embedded in the image file, the EXIF orientation is ignored.

  \param I : Color image to set with the \e filename content.
  \param filename : Name of the file containing the image.
*/
void readOpenCV(vpImage<vpRGBa> &I, const std::string &filename)
{
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
#if VISP_HAVE_OPENCV_VERSION >= 0x030200
  int flags = cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION;
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
  int flags = cv::IMREAD_COLOR;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  int flags = CV_LOAD_IMAGE_COLOR;
#endif
  cv::Mat Ip = cv::imread(filename.c_str(), flags);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  IplImage *Ip = NULL;
  Ip = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_COLOR);
  if (Ip != NULL)
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
  cvReleaseImage(&Ip);
#endif
}

void readOpenCV(vpImage<float> &I, const std::string &filename)
{
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
#if VISP_HAVE_OPENCV_VERSION >= 0x030200
  int flags = cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION;
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
  int flags = cv::IMREAD_COLOR;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  int flags = CV_LOAD_IMAGE_COLOR;
#endif
  cv::Mat Ip = cv::imread(filename.c_str(), flags);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#endif
}

void readOpenCV(vpImage<vpRGBf> &I, const std::string &filename)
{
#if defined(VISP_HAVE_OPENCV) && VISP_HAVE_OPENCV_VERSION >= 0x020100
#if VISP_HAVE_OPENCV_VERSION >= 0x030200
  int flags = cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION;
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
  int flags = cv::IMREAD_COLOR;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  int flags = CV_LOAD_IMAGE_COLOR;
#endif
  cv::Mat Ip = cv::imread(filename.c_str(), flags);
  if (!Ip.empty())
    vpImageConvert::convert(Ip, I);
  else
    throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#else
  throw(vpImageException(vpImageException::ioError, "Can't read the image"));
#endif
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void writeOpenCV(const vpImage<unsigned char> &I, const std::string &filename, int quality)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(quality);
  cv::imwrite(filename.c_str(), Ip, compression_params);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
  (void)quality;
#endif
}

/*!
  Write the content of the image bitmap in the file which name is given by \e
  filename. This function writes a JPEG file.

  \param I : Image to save as a JPEG file.
  \param filename : Name of the file containing the image.
*/
void writeOpenCV(const vpImage<vpRGBa> &I, const std::string &filename, int quality)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);

  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(quality);
  cv::imwrite(filename.c_str(), Ip, compression_params);
#else
  IplImage *Ip = NULL;
  vpImageConvert::convert(I, Ip);

  cvSaveImage(filename.c_str(), Ip);

  cvReleaseImage(&Ip);
  (void)quality;
#endif
}

void writeOpenCV(const vpImage<float> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);

  cv::imwrite(filename.c_str(), Ip);
#else
  (void)I;
  (void)filename;
  throw(vpImageException(vpImageException::ioError, "Not supported image type with the current OpenCV library version"));
#endif
}

void writeOpenCV(const vpImage<vpRGBf> &I, const std::string &filename)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  cv::Mat Ip;
  vpImageConvert::convert(I, Ip);

  cv::imwrite(filename.c_str(), Ip);
#else
  (void)I;
  (void)filename;
  throw(vpImageException(vpImageException::ioError, "Not supported image type with the current OpenCV library version"));
#endif
}

#endif
