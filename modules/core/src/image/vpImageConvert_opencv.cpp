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
 *
*****************************************************************************/

/*!
  \file vpImageConvert_opencv.cpp
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)

BEGIN_VISP_NAMESPACE
/*!
  Convert a cv::Mat to a vpImage\<vpRGBa\>.

  A cv::Mat is an OpenCV image class.

  If the input image is of type CV_8UC1 or CV_8UC3, the alpha channel is set
  to vpRGBa::alpha_default, or 0 in certain case (see the warning below).

  \warning This function is only available if OpenCV (version 2.1.0 or
  greater) was detected during the configuration step.

  \warning If ViSP is built with SSSE3 flag and the CPU supports this intrinsics set,
  alpha channel will be set to 0, otherwise it will be set to vpRGBa::alpha_default (255).

  \param[in] src : Source image in OpenCV format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] flip : Set to true to vertically flip the converted image.

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
  #if defined(VISP_HAVE_OPENCV)
    vpImage<vpRGBa> Ic; // A color image
    cv::Mat Ip;

    // Read an image on a disk with openCV library
    Ip = cv::imread("image.pgm", cv::IMREAD_COLOR); // Second parameter for a BGR encoding.
    // Convert the grayscale cv::Mat into vpImage<vpRGBa>
    vpImageConvert::convert(Ip, Ic);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const cv::Mat &src, vpImage<vpRGBa> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  if (src.type() == CV_8UC4) {
    vpRGBa rgbaVal;
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        cv::Vec4b tmp = src.at<cv::Vec4b>(static_cast<int>(i), static_cast<int>(j));
        rgbaVal.R = tmp[index_2];
        rgbaVal.G = tmp[index_1];
        rgbaVal.B = tmp[index_0];
        rgbaVal.A = tmp[index_3];
        if (flip) {
          dest[destRows - i - 1][j] = rgbaVal;
        }
        else {
          dest[i][j] = rgbaVal;
        }
      }
  }
  else if (src.type() == CV_8UC3) {
#if defined(VISP_HAVE_SIMDLIB)
    if (src.isContinuous() && (!flip)) {
      SimdRgbToBgra(src.data, src.cols, src.rows, src.step[0], reinterpret_cast<uint8_t *>(dest.bitmap),
                    dest.getWidth() * sizeof(vpRGBa), vpRGBa::alpha_default);
    }
    else {
#endif
      vpRGBa rgbaVal;
      rgbaVal.A = vpRGBa::alpha_default;
      for (unsigned int i = 0; i < destRows; ++i) {
        for (unsigned int j = 0; j < destCols; ++j) {
          cv::Vec3b tmp = src.at<cv::Vec3b>(static_cast<int>(i), static_cast<int>(j));
          rgbaVal.R = tmp[index_2];
          rgbaVal.G = tmp[index_1];
          rgbaVal.B = tmp[index_0];
          if (flip) {
            dest[destRows - i - 1][j] = rgbaVal;
          }
          else {
            dest[i][j] = rgbaVal;
          }
        }
      }
#if defined(VISP_HAVE_SIMDLIB)
    }
#endif
  }
  else if (src.type() == CV_8UC1) {
#if defined(VISP_HAVE_SIMDLIB)
    if (src.isContinuous() && (!flip)) {
      SimdGrayToBgra(src.data, src.cols, src.rows, src.step[0], reinterpret_cast<uint8_t *>(dest.bitmap),
                     dest.getWidth() * sizeof(vpRGBa), vpRGBa::alpha_default);
    }
    else {
#endif
      vpRGBa rgbaVal;
      for (unsigned int i = 0; i < destRows; ++i) {
        for (unsigned int j = 0; j < destCols; ++j) {
          rgbaVal = src.at<unsigned char>(static_cast<int>(i), static_cast<int>(j));
          rgbaVal.A = vpRGBa::alpha_default;
          if (flip) {
            dest[destRows - i - 1][j] = rgbaVal;
          }
          else {
            dest[i][j] = rgbaVal;
          }
        }
      }
#if defined(VISP_HAVE_SIMDLIB)
    }
#endif
  }
}

/*!
  Convert a cv::Mat to a vpImage\<unsigned char\>.

  A cv::Mat is an OpenCV image class.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param[in] src : Source image in OpenCV format.
  \param[out] dest : Destination image in ViSP format.
  \param[in] flip : Set to true to vertically flip the converted image.
  \param[in] nThreads : number of threads to use if OpenMP is available. If 0 is passed,
  OpenMP will choose the number of threads.

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_OPENCV)
    vpImage<unsigned char> Ig; // A grayscale image
    cv::Mat Ip;

    // Read an image on a disk with openCV library
    Ip = cv::imread("image.pgm", cv::IMREAD_GRAYSCALE); // Second parameter for a gray level.
    // Convert the grayscale cv::Mat into vpImage<unsigned char>
    vpImageConvert::convert(Ip, Ig);

    // ...
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const cv::Mat &src, vpImage<unsigned char> &dest, bool flip, unsigned int nThreads)
{
  if (src.type() == CV_8UC1) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous() && (!flip)) {
      memcpy(dest.bitmap, src.data, static_cast<size_t>(src.rows * src.cols));
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + ((destRows - i - 1) * src.step1()), static_cast<size_t>(src.step));
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + (i * src.step1()), static_cast<size_t>(src.step));
        }
      }
    }
  }
  else if (src.type() == CV_8UC3) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous()) {
      BGRToGrey((unsigned char *)src.data, (unsigned char *)dest.bitmap, static_cast<unsigned int>(src.cols), static_cast<unsigned int>(src.rows),
                flip, nThreads);
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRToGrey((unsigned char *)src.data + (i * src.step1()),
                    (unsigned char *)dest.bitmap + ((destRows - i - 1) * destCols),
                    static_cast<unsigned int>(destCols), 1, false);
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRToGrey((unsigned char *)src.data + (i * src.step1()), (unsigned char *)dest.bitmap + (i * destCols),
                    static_cast<unsigned int>(destCols), 1, false);
        }
      }
    }
  }
  else if (src.type() == CV_8UC4) {
    dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
    unsigned int destRows = dest.getRows();
    unsigned int destCols = dest.getCols();
    if (src.isContinuous()) {
      BGRaToGrey((unsigned char *)src.data, (unsigned char *)dest.bitmap, static_cast<unsigned int>(src.cols),
                 static_cast<unsigned int>(src.rows), flip, nThreads);
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRaToGrey((unsigned char *)src.data + (i * src.step1()),
                     (unsigned char *)dest.bitmap + ((destRows - i - 1) * destCols),
                     static_cast<unsigned int>(destCols), 1, false);
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          BGRaToGrey((unsigned char *)src.data + (i * src.step1()), (unsigned char *)dest.bitmap + (i * destCols),
                     static_cast<unsigned int>(destCols), 1, false);
        }
      }
    }
  }
}

/*!
 * Converts cv::Mat CV_32FC1 format to ViSP vpImage<float>.
 *
 * \param[in] src : OpenCV image in CV_32FC1 format.
 * \param[out] dest : ViSP image in float format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<float> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();

  if (src.type() == CV_32FC1) {
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        if (flip) {
          dest[dest.getRows() - i - 1][j] = src.at<float>(static_cast<int>(i), static_cast<int>(j));
        }
        else {
          dest[i][j] = src.at<float>(static_cast<int>(i), static_cast<int>(j));
        }
      }
  }
  else {
    throw vpException(vpException::badValue, "cv::Mat type is not supported!");
  }
}

/*!
 * Converts cv::Mat CV_32FC1 format to ViSP vpImage<double>.
 *
 * \param[in] src : OpenCV image in CV_32FC1 format.
 * \param[out] dest : ViSP image in double format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<double> &dest, bool flip)
{
  vpImage<float> I_float;
  convert(src, I_float, flip);
  unsigned int nbRows = static_cast<unsigned int>(src.rows);
  unsigned int nbCols = static_cast<unsigned int>(src.cols);
  dest.resize(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      dest[i][j] = I_float[i][j];
    }
  }
}

/*!
 * Converts cv::Mat CV_16UC1 format to ViSP vpImage<uint16_t>.
 *
 * \param[in] src : OpenCV image in CV_16UC1 format.
 * \param[out] dest : ViSP image in uint16_t format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<uint16_t> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();

  if (src.type() == CV_16UC1) {
    if (src.isContinuous()) {
      memcpy(dest.bitmap, src.data, static_cast<size_t>(src.rows * src.cols) * sizeof(uint16_t));
    }
    else {
      if (flip) {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + ((destRows - i - 1) * src.step1() * sizeof(uint16_t)), static_cast<size_t>(src.step));
        }
      }
      else {
        for (unsigned int i = 0; i < destRows; ++i) {
          memcpy(dest.bitmap + (i * destCols), src.data + (i * src.step1() * sizeof(uint16_t)), static_cast<size_t>(src.step));
        }
      }
    }
  }
  else {
    throw(vpException(vpException::fatalError, "cv:Mat format not supported for conversion into vpImage<uint16_t>"));
  }
}

/*!
 * Converts cv::Mat CV_32FC3 format to ViSP vpImage<vpRGBf>.
 *
 * \param[in] src : OpenCV image in CV_32FC3 format.
 * \param[out] dest : ViSP image in vpRGBf format.
 * \param[in] flip : When true during conversion flip image vertically.
 */
void vpImageConvert::convert(const cv::Mat &src, vpImage<vpRGBf> &dest, bool flip)
{
  dest.resize(static_cast<unsigned int>(src.rows), static_cast<unsigned int>(src.cols));
  unsigned int destRows = dest.getRows();
  unsigned int destCols = dest.getCols();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  if (src.type() == CV_32FC3) {
    vpRGBf rgbVal;
    for (unsigned int i = 0; i < destRows; ++i)
      for (unsigned int j = 0; j < destCols; ++j) {
        cv::Vec3f tmp = src.at<cv::Vec3f>(static_cast<int>(i), static_cast<int>(j));
        rgbVal.R = tmp[index_2];
        rgbVal.G = tmp[index_1];
        rgbVal.B = tmp[index_0];
        if (flip) {
          dest[destRows - i - 1][j] = rgbVal;
        }
        else {
          dest[i][j] = rgbVal;
        }
      }
  }
  else {
    throw vpException(vpException::badValue, "cv::Mat type is not supported!");
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a cv::Mat color image.

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param[in] src : Source image (vpRGBa format).
  \param[out] dest : Destination image (BGR format).

  \code
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_OPENCV)
    vpImage<vpRGBa> I; // A color image
    cv::Mat Icv;

    // Read an image on a disk
    vpImageIo::read(I, "image.ppm");
    // Convert the image into color cv::Mat.
    vpImageConvert::convert(I, Icv);
    // Treatments on cv::Mat Icv
    //...
    // Save the cv::Mat on the disk
    cv::imwrite("image-cv.ppm", Icv);
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> &src, cv::Mat &dest)
{
  cv::Mat vpToMat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC4, (void *)src.bitmap);
  cv::cvtColor(vpToMat, dest, cv::COLOR_RGBA2BGR);
}

/*!
  Convert a vpImage\<unsigned char\> to a cv::Mat grey level image.

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param[in] src : Source image.
  \param[out] dest : Destination image.
  \param[in] copyData : If true, the image is copied and modification in one
  object will not modified the other.

  \code
  #include <visp3/core/vpImageConvert.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_IMGCODECS)
    vpImage<unsigned char> Ig; // A grayscale image
    cv::Mat Ip;

    // Read an image on a disk
    vpImageIo::read(Ig, "image.pgm");
    // Convert the vpImage<unsigned char> in to grayscale cv::Mat
    vpImageConvert::convert(Ig, Ip);
    // Treatments on cv::Mat Ip
    //...
    // Save the cv::Mat on the disk
    cv::imwrite("image-cv.pgm", Ip);
  #endif
  }
  \endcode
*/
void vpImageConvert::convert(const vpImage<unsigned char> &src, cv::Mat &dest, bool copyData)
{
  if (copyData) {
    cv::Mat tmpMap(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC1, (void *)src.bitmap);
    dest = tmpMap.clone();
  }
  else {
    dest = cv::Mat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_8UC1, (void *)src.bitmap);
  }
}

void vpImageConvert::convert(const vpImage<float> &src, cv::Mat &dest, bool copyData)
{
  if (copyData) {
    cv::Mat tmpMap(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC1, (void *)src.bitmap);
    dest = tmpMap.clone();
  }
  else {
    dest = cv::Mat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC1, (void *)src.bitmap);
  }
}

void vpImageConvert::convert(const vpImage<double> &src, cv::Mat &dest, bool copyData)
{
  unsigned int nbRows = src.getRows();
  unsigned int nbCols = src.getCols();
  vpImage<float> I_float(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      I_float[i][j] = static_cast<float>(src[i][j]);
    }
  }
  convert(I_float, dest, copyData);
}

void vpImageConvert::convert(const vpImage<vpRGBf> &src, cv::Mat &dest)
{
  cv::Mat vpToMat(static_cast<int>(src.getRows()), static_cast<int>(src.getCols()), CV_32FC3, (void *)src.bitmap);
  cv::cvtColor(vpToMat, dest, cv::COLOR_RGB2BGR);
}

END_VISP_NAMESPACE

#endif
