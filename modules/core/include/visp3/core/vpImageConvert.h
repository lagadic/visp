/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Convert image types.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpImageConvert.h
  \brief Convert image types
*/

#ifndef vpIMAGECONVERT_H
#define vpIMAGECONVERT_H

#include <stdint.h>

// image
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpDebug.h>
// color
#include <visp3/core/vpRGBa.h>

#ifdef VISP_HAVE_OPENCV
#  if (VISP_HAVE_OPENCV_VERSION >= 0x030000) // Require opencv >= 3.0.0
#    include <opencv2/core/core.hpp>
#    include <opencv2/highgui/highgui.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#  elif (VISP_HAVE_OPENCV_VERSION >= 0x020408) // Require opencv >= 2.4.8
#    include <opencv2/core/core.hpp>
#    include <opencv2/highgui/highgui.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#  elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#    include <opencv2/core/core.hpp>
#    include <opencv2/legacy/legacy.hpp>
#    include <opencv2/highgui/highgui.hpp>
#    include <opencv2/highgui/highgui_c.h>
#  else
#    include <highgui.h>
#  endif
#endif

#ifdef VISP_HAVE_YARP
#  include <yarp/sig/Image.h>
#endif

#if defined(_WIN32)
#  include <windows.h>
#endif

/*!
  \class vpImageConvert

  \ingroup group_core_image

  Convert image types.

  The following example available in tutorial-image-converter.cpp shows how to
  convert an OpenCV cv::Mat image into a vpImage:

  \include tutorial-image-converter.cpp

*/
class VISP_EXPORT vpImageConvert
{

public:
  static void createDepthHistogram(const vpImage<uint16_t> &src_depth, vpImage<vpRGBa> &dest_rgba);
  static void convert(const vpImage<unsigned char> &src, vpImage<vpRGBa> & dest) ;
  static void convert(const vpImage<vpRGBa> &src, vpImage<unsigned char> & dest, const bool fastConversion=true) ;
          
  static void convert(const vpImage<float> &src, vpImage<unsigned char> &dest);
  static void convert(const vpImage<unsigned char> &src, vpImage<float> &dest);
  
  static void convert(const vpImage<double> &src, vpImage<unsigned char> &dest);
  static void convert(const vpImage<unsigned char> &src, vpImage<double> &dest);

  static void convert(const vpImage<uint16_t> &src, vpImage<unsigned char> &dest);
  static void convert(const vpImage<unsigned char> &src, vpImage<uint16_t> &dest);

#ifdef VISP_HAVE_OPENCV
  // Deprecated: will be removed with OpenCV transcient from C to C++ api
  static void convert(const IplImage* src, vpImage<vpRGBa> & dest, bool flip = false) ;
  static void convert(const IplImage* src, vpImage<unsigned char> & dest, bool flip = false) ;
  static void convert(const vpImage<vpRGBa> & src, IplImage *&dest) ;
  static void convert(const vpImage<unsigned char> & src, IplImage* &dest) ;
#  if VISP_HAVE_OPENCV_VERSION >= 0x020100
  static void convert(const cv::Mat& src, vpImage<vpRGBa>& dest, const bool flip = false);
  static void convert(const cv::Mat& src, vpImage<unsigned char>& dest, const bool flip = false, const bool fastConversion=true);
  static void convert(const vpImage<vpRGBa> & src, cv::Mat& dest) ;
  static void convert(const vpImage<unsigned char> & src, cv::Mat& dest, const bool copyData = true) ;
#  endif
#endif
    
#ifdef VISP_HAVE_YARP
  static void convert(const vpImage<unsigned char> & src,
          yarp::sig::ImageOf< yarp::sig::PixelMono > *dest, const bool copyData = true) ;
  static void convert(const yarp::sig::ImageOf< yarp::sig::PixelMono > *src,
    vpImage<unsigned char> & dest,const bool copyData = true ) ;
    
    
  static void convert(const vpImage<vpRGBa> & src,
          yarp::sig::ImageOf< yarp::sig::PixelRgba > *dest, const bool copyData = true) ;
  static void convert(const yarp::sig::ImageOf< yarp::sig::PixelRgba > *src,
    vpImage<vpRGBa> & dest,const bool copyData = true) ;
    
  static void convert(const vpImage<vpRGBa> & src,
          yarp::sig::ImageOf< yarp::sig::PixelRgb > *dest) ;
  static void convert(const yarp::sig::ImageOf< yarp::sig::PixelRgb > *src,
    vpImage<vpRGBa> & dest) ;
#endif
    
  static void split(const vpImage<vpRGBa> &src,
                    vpImage<unsigned char>* pR,
                    vpImage<unsigned char>* pG,
                    vpImage<unsigned char>* pB,
                    vpImage<unsigned char>* pa = NULL);

  static void merge(const vpImage<unsigned char> *R,
                    const vpImage<unsigned char> *G,
                    const vpImage<unsigned char> *B,
                    const vpImage<unsigned char> *a,
                    vpImage<vpRGBa> &RGBa);

  /*!
    Converts a yuv pixel value in rgb format.

    \param y Y component of a pixel.
    \param u U component of a pixel.
    \param v V component of a pixel.
    \param r Red component from the YUV coding format. This value is computed
    using:
    \f[ r = 0.9999695*y - 0.0009508*(u-128) + 1.1359061*(v-128) \f]
    \param g Green component from the YUV coding format. This value is computed
    using:
    \f[g = 0.9999695*y - 0.3959609*(u-128) - 0.5782955*(v-128) \f]
    \param b Blue component from the YUV coding format. This value is computed
    using:
    \f[b = 0.9999695*y + 2.04112*(u-128) - 0.0016314*(v-128) \f]

  */
  static inline void YUVToRGB(unsigned char y,
            unsigned char u,
            unsigned char v,
            unsigned char &r,
            unsigned char &g,
            unsigned char &b)
    {
      double dr, dg, db;
      dr = floor(0.9999695*y - 0.0009508*(u-128) + 1.1359061*(v-128));
      dg = floor(0.9999695*y - 0.3959609*(u-128) - 0.5782955*(v-128));
      db = floor(0.9999695*y + 2.04112*(u-128) - 0.0016314*(v-128));

      dr = dr < 0. ? 0. : dr;
      dg = dg < 0. ? 0. : dg;
      db = db < 0. ? 0. : db;
      dr = dr > 255. ? 255. : dr;
      dg = dg > 255. ? 255. : dg;
      db = db > 255. ? 255. : db;

      r = (unsigned char) dr;
      g = (unsigned char) dg;
      b = (unsigned char) db;
    }
  static void YUYVToRGBa(unsigned char* yuyv, unsigned char* rgba,
      unsigned int width, unsigned int height);
  static void YUYVToRGB(unsigned char* yuyv, unsigned char* rgb,
      unsigned int width, unsigned int height);
  static void YUYVToGrey(unsigned char* yuyv, unsigned char* grey,
      unsigned int size);
  static void YUV411ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int size);
  static void YUV411ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int size);
  static void YUV411ToGrey(unsigned char* yuv,
        unsigned char* grey, unsigned int size);
  static void YUV422ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int size);
  static void YUV422ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int size);
  static void YUV422ToGrey(unsigned char* yuv,
        unsigned char* grey, unsigned int size);
  static void YUV420ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int width, unsigned int height);
  static void YUV420ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int width, unsigned int height);
  static void YUV420ToGrey(unsigned char* yuv,
        unsigned char* grey, unsigned int size);

  static void YUV444ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int size);
  static void YUV444ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int size);
  static void YUV444ToGrey(unsigned char* yuv,
        unsigned char* grey, unsigned int size);

  static void YV12ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int width, unsigned int height);
  static void YV12ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int width, unsigned int height);
  static void YVU9ToRGBa(unsigned char* yuv,
        unsigned char* rgba, unsigned int width, unsigned int height);
  static void YVU9ToRGB(unsigned char* yuv,
        unsigned char* rgb, unsigned int width, unsigned int height);
  static void RGBToRGBa(unsigned char* rgb,
      unsigned char* rgba, unsigned int size);
  static void RGBaToRGB(unsigned char* rgba,
      unsigned char* rgb, unsigned int size);

  static void RGBToGrey(unsigned char* rgb,
      unsigned char* grey, unsigned int size,
      const bool fastConversion=true);
  static void RGBaToGrey(unsigned char* rgba,
      unsigned char* grey, unsigned int size,
      const bool fastConversion=true
  );

  static void RGBToRGBa(unsigned char * rgb, unsigned char * rgba,
      unsigned int width, unsigned int height, bool flip = false);
  static void RGBToGrey(unsigned char * rgb, unsigned char * grey,
      unsigned int width, unsigned int height, bool flip = false,
      const bool fastConversion=true);

  static void GreyToRGBa(unsigned char* grey,
      unsigned char* rgba, unsigned int size);
  static void GreyToRGB(unsigned char* grey,
      unsigned char* rgb, unsigned int size);

  static void BGRToRGBa(unsigned char * bgr, unsigned char * rgba,
      unsigned int width, unsigned int height, bool flip=false);

  static void BGRToGrey(unsigned char * bgr, unsigned char * grey,
      unsigned int width, unsigned int height, bool flip=false,
      const bool fastConversion=true);

  static void YCbCrToRGB(unsigned char *ycbcr, unsigned char *rgb,
      unsigned int size);
  static void YCbCrToRGBa (unsigned char *ycbcr, unsigned char *rgb,
        unsigned int size);
  static void YCrCbToRGB(unsigned char *ycbcr, unsigned char *rgb,
      unsigned int size);
  static void YCrCbToRGBa(unsigned char *ycbcr, unsigned char *rgb,
        unsigned int size);
  static void YCbCrToGrey(unsigned char *ycbcr, unsigned char *grey,
      unsigned int size);
  static void MONO16ToGrey(unsigned char *grey16, unsigned char *grey,
        unsigned int size);
  static void MONO16ToRGBa(unsigned char *grey16, unsigned char *rgba,
        unsigned int size);
  
  static void HSVToRGBa(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
        const unsigned int size);
  static void HSVToRGBa(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
        unsigned char *rgba, const unsigned int size);
  static void RGBaToHSV(const unsigned char *rgba, double *hue, double *saturation, double *value,
        const unsigned int size);
  static void RGBaToHSV(const unsigned char *rgba, unsigned char *hue, unsigned char *saturation, unsigned char *value,
        const unsigned int size);

  static void HSVToRGB(const double *hue, const double *saturation, const double *value, unsigned char *rgb,
        const unsigned int size);
  static void HSVToRGB(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
        unsigned char *rgb, const unsigned int size);
  static void RGBToHSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
        const unsigned int size);
  static void RGBToHSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation, unsigned char *value,
        const unsigned int size);

private:
  static void computeYCbCrLUT();

  static void HSV2RGB(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
        const unsigned int size, const unsigned int step);
  static void RGB2HSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
        const unsigned int size, const unsigned int step);

private:
  static bool YCbCrLUTcomputed;
  static int vpCrr[256];
  static int vpCgb[256];
  static int vpCgr[256];
  static int vpCbb[256];

} ;

#endif


/*
* Local variables:
* c-basic-offset: 2
* End:
*/
