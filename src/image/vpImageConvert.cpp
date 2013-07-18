/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  \file vpImageConvert.cpp
  \brief Convert image types
*/


#include <sstream>

// image
#include <visp/vpImageConvert.h>

bool vpImageConvert::YCbCrLUTcomputed = false;
int vpImageConvert::vpCrr[256];
int vpImageConvert::vpCgb[256];
int vpImageConvert::vpCgr[256];
int vpImageConvert::vpCbb[256];


/*!
Convert a vpImage\<vpRGBa\> to a vpImage\<unsigned char\>
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<unsigned char> &src,
      vpImage<vpRGBa> & dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;

  GreyToRGBa(src.bitmap, (unsigned char *)dest.bitmap,
       src.getHeight() * src.getWidth() );
}

/*!
Convert a vpImage\<unsigned char\> to a vpImage\<vpRGBa\>
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<vpRGBa> &src,
      vpImage<unsigned char> & dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;

  RGBaToGrey((unsigned char *)src.bitmap, dest.bitmap,
       src.getHeight() * src.getWidth() );
}


/*!
Convert a vpImage\<float\> to a vpImage\<unsigend char\> by renormalizing between 0 and 255.
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<float> &src,
          vpImage<unsigned char> &dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;
  unsigned int max_xy = src.getWidth()*src.getHeight();
  float min, max;

  src.getMinMaxValue(min,max);
  
  for (unsigned int i = 0; i < max_xy; i++) {
    float val = 255.f * (src.bitmap[i] - min) / (max - min);
    if(val < 0)
      dest.bitmap[i] = 0;
    else if(val > 255)
      dest.bitmap[i] = 255;
    else
      dest.bitmap[i] = (int)val;
  }
}

/*!
Convert a vpImage\<unsigned char\> to a vpImage\<float\> by basic casting.
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<unsigned char> &src,
          vpImage<float> &dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;
  for (unsigned int i = 0; i < src.getHeight()*src.getWidth(); i++)
    dest.bitmap[i] = (float)src.bitmap[i];
}

/*!
Convert a vpImage\<double\> to a vpImage\<unsigend char\> by renormalizing between 0 and 255.
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<double> &src,
          vpImage<unsigned char> &dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;
  unsigned int max_xy = src.getWidth()*src.getHeight();
  double min, max;

  src.getMinMaxValue(min,max);
  
  for (unsigned int i = 0; i < max_xy; i++) {
    double val = 255. * (src.bitmap[i] - min) / (max - min);
    if(val < 0)
      dest.bitmap[i] = 0;
    else if(val > 255)
      dest.bitmap[i] = 255;
    else
      dest.bitmap[i] = (int)val;
  }
}

/*!
Convert a vpImage\<unsigned char\> to a vpImage\<double\> by basic casting.
\param src : source image
\param dest : destination image
*/
void
vpImageConvert::convert(const vpImage<unsigned char> &src,
          vpImage<double> &dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;
  for (unsigned int i = 0; i < src.getHeight()*src.getWidth(); i++)
    dest.bitmap[i] = (double)src.bitmap[i];
}

#ifdef VISP_HAVE_OPENCV
/*!
  Convert a IplImage to a vpImage\<vpRGBa\>

  An IplImage is an OpenCV (Intel's Open source Computer Vision Library)
  image structure. See http://opencvlibrary.sourceforge.net/ for general
  OpenCV documentation, or http://opencvlibrary.sourceforge.net/CxCore
  for the specific IplImage structure documentation.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param src : Source image in OpenCV format.
  \param dest : Destination image in ViSP format.
  \param flip : Set to true to vertically flip the converted image.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  vpImage<vpRGBa> Ic; // A color image
  IplImage* Ip;

  // Read an image on a disk with openCV library
  Ip = cvLoadImage("image.ppm", CV_LOAD_IMAGE_COLOR);
  // Convert the grayscale IplImage into vpImage<vpRGBa>
  vpImageConvert::convert(Ip, Ic);

  // ...

  // Release Ip header and data
  cvReleaseImage(&Ip);
#endif
}
  \endcode
*/
void
vpImageConvert::convert(const IplImage* src, vpImage<vpRGBa> & dest, bool flip)
{
  int nChannel = src->nChannels;
  int depth = src->depth;
  int height = src->height;
  int width = src->width;
  int widthStep = src->widthStep;
  int lineStep = (flip) ? 1 : 0;

  if(nChannel == 3 && depth == 8){
    dest.resize((unsigned int)height, (unsigned int)width);

    //starting source address
    unsigned char* input = (unsigned char*)src->imageData;
    unsigned char* line;
    unsigned char* beginOutput = (unsigned char*)dest.bitmap;
    unsigned char* output = NULL;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) + (1-lineStep) * 4 * width * i;
      for(int j=0 ; j < width ; j++)
        {
          *(output++) = *(line+2);
          *(output++) = *(line+1);
          *(output++) = *(line);
          *(output++) = 0;

          line+=3;
        }
      //go to the next line
      input+=widthStep;
    }
  }
  else if(nChannel == 1 && depth == 8 ){
    dest.resize((unsigned int)height, (unsigned int)width);
    //starting source address
    unsigned char * input = (unsigned char*)src->imageData;
    unsigned char * line;
    unsigned char* beginOutput = (unsigned char*)dest.bitmap;
    unsigned char* output = NULL;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) + (1-lineStep) * 4 * width * i;
      for(int j=0 ; j < width ; j++)
        {
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);;

          line++;
        }
      //go to the next line
      input+=widthStep;
    }
  }
}

/*!
  Convert a IplImage to a vpImage\<unsigned char\>

  An IplImage is an OpenCV (Intel's Open source Computer Vision Library)
  image structure. See http://opencvlibrary.sourceforge.net/ for general
  OpenCV documentation, or http://opencvlibrary.sourceforge.net/CxCore
  for the specific IplImage structure documentation.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param src : Source image in OpenCV format.
  \param dest : Destination image in ViSP format.
  \param flip : Set to true to vertically flip the converted image.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  vpImage<unsigned char> Ig; // A grayscale image
  IplImage* Ip;

  // Read an image on a disk with openCV library
  Ip = cvLoadImage("image.pgm", CV_LOAD_IMAGE_GRAYSCALE);
  // Convert the grayscale IplImage into vpImage<unsigned char>
  vpImageConvert::convert(Ip, Ig);

  // ...

  // Release Ip header and data
  cvReleaseImage(&Ip);
#endif
}
  \endcode
*/
void
vpImageConvert::convert(const IplImage* src,
      vpImage<unsigned char> &dest, bool flip)
{
  int nChannel = src->nChannels;
  int depth = src->depth;
  int height = src->height;
  int width = src->width;
  int widthStep = src->widthStep;
  int lineStep = (flip) ? 1 : 0;

  if (flip == false)
  {
    if(widthStep == width){
      if(nChannel == 1 && depth == 8){
        dest.resize((unsigned int)height, (unsigned int)width) ;
        memcpy(dest.bitmap, src->imageData,
                (size_t)(height*width));
      }
      if(nChannel == 3 && depth == 8){
        dest.resize((unsigned int)height, (unsigned int)width) ;
        BGRToGrey((unsigned char*)src->imageData,dest.bitmap, (unsigned int)width, (unsigned int)height,false);
      }
    }
    else{
      if(nChannel == 1 && depth == 8){
        dest.resize((unsigned int)height, (unsigned int)width) ;
        for (int i =0  ; i < height ; i++){
          memcpy(dest.bitmap+i*width, src->imageData + i*widthStep,
                (size_t)width);
        }
      }
      if(nChannel == 3 && depth == 8){
        dest.resize((unsigned int)height, (unsigned int)width) ;
        for (int i = 0  ; i < height ; i++){
          BGRToGrey((unsigned char*)src->imageData + i*widthStep,
                      dest.bitmap + i*width, (unsigned int)width, 1, false);
        }
      }
    }
  }
  else
  {
      if(nChannel == 1 && depth == 8){
      unsigned char* beginOutput = (unsigned char*)dest.bitmap;
        dest.resize((unsigned int)height, (unsigned int)width) ;
        for (int i =0  ; i < height ; i++){
          memcpy(beginOutput + lineStep * ( 4 * width * ( height - 1 - i ) ) , src->imageData + i*widthStep,
                (size_t)width);
        }
      }
      if(nChannel == 3 && depth == 8){
        dest.resize((unsigned int)height, (unsigned int)width) ;
        //for (int i = 0  ; i < height ; i++){
          BGRToGrey((unsigned char*)src->imageData /*+ i*widthStep*/,
                      dest.bitmap /*+ i*width*/, (unsigned int)width, (unsigned int)height/*1*/, true);
        //}
      }
  }
}

/*!
  Convert a vpImage\<vpRGBa\> to a IplImage

  An IplImage is an OpenCV (Intel's Open source Computer Vision Library)
  image structure. See http://opencvlibrary.sourceforge.net/ for general
  OpenCV documentation, or http://opencvlibrary.sourceforge.net/CxCore
  for the specific IplImage structure documentation.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param src : source image
  \param dest : destination image

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  vpImage<vpRGBa> Ic; // A color image
  IplImage* Ip = NULL;

  // Read an image on a disk
  vpImageIo::read(Ic, "image.ppm");
  // Convert the vpImage<vpRGBa> in to color IplImage
  vpImageConvert::convert(Ic, Ip);
  // Treatments on IplImage
  //...
  // Save the IplImage on the disk
  cvSaveImage("Ipl.ppm", Ip);

  //Release Ip header and data
  cvReleaseImage(&Ip);
#endif
}
  \endcode
*/
void
vpImageConvert::convert(const vpImage<vpRGBa> & src, IplImage *&dest)
{
  int height = (int)src.getHeight();
  int width  = (int)src.getWidth();
  CvSize size = cvSize(width, height);
  int depth = 8;
  int channels = 3;
  if (dest != NULL){
    if(dest->nChannels != channels || dest->depth != depth
       || dest->height != height || dest->width != width){
      if(dest->nChannels != 0) cvReleaseImage(&dest);
      dest = cvCreateImage( size, depth, channels );
    }
  }
  else dest = cvCreateImage( size, depth, channels );


  //starting source address
  unsigned char * input = (unsigned char*)src.bitmap;//rgba image
  unsigned char * line;
  unsigned char * output = (unsigned char*)dest->imageData;//bgr image

  int j=0;
  int i=0;
  int widthStep = dest->widthStep;

  for(i=0 ; i < height ; i++)
  {
    output = (unsigned char*)dest->imageData + i*widthStep;
    line = input;
    for( j=0 ; j < width ; j++)
      {
        *output++ = *(line+2);  //B
        *output++ = *(line+1);  //G
        *output++ = *(line);  //R

        line+=4;
      }
    //go to the next line
    input+=4*width;
  }
}

/*!
  Convert a vpImage\<unsigned char\> to a IplImage

  An IplImage is an OpenCV (Intel's Open source Computer Vision Library)
  image structure. See http://opencvlibrary.sourceforge.net/ for general
  OpenCV documentation, or http://opencvlibrary.sourceforge.net/CxCore
  for the specific IplImage structure documentation.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param src : source image
  \param dest : destination image

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  vpImage<unsigned char> Ig; // A greyscale image
  IplImage* Ip = NULL;

  // Read an image on a disk
  vpImageIo::read(Ig, "image.pgm");
  // Convert the vpImage<unsigned char> in to greyscale IplImage
  vpImageConvert::convert(Ig, Ip);
  // Treatments on IplImage Ip
  //...
  // Save the IplImage on the disk
  cvSaveImage("Ipl.pgm", Ip);

  //Release Ip header and data
  cvReleaseImage(&Ip);
#endif
}
  \endcode
*/
void
vpImageConvert::convert(const vpImage<unsigned char> & src,
      IplImage* &dest)
{
  unsigned int height = src.getHeight();
  unsigned int width  = src.getWidth();
  CvSize size = cvSize((int)width, (int)height);
  int depth = 8;
  int channels = 1;
  if (dest != NULL){
    if(dest->nChannels != channels || dest->depth != depth
       || dest->height != (int) height || dest->width != (int) width){
      if(dest->nChannels != 0) cvReleaseImage(&dest);
      dest = cvCreateImage( size, depth, channels );
    }
  }
  else dest = cvCreateImage( size, depth, channels );

  unsigned int widthStep = (unsigned int)dest->widthStep;

  if ( width == widthStep){
    memcpy(dest->imageData,src.bitmap, width*height);
  }
  else{
    //copying each line taking account of the widthStep
    for (unsigned int i =0  ; i < height ; i++){
          memcpy(dest->imageData + i*widthStep, src.bitmap + i*width,
                width);
    }
  }
}

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
/*!
  Convert a cv::Mat to a vpImage\<vpRGBa\>

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  Similarily to the convert(const IplImage* src, vpImage<vpRGBa> & dest, bool flip)
  method, only Mat with a depth equal to 8 and a channel between 1 and 3 are
  converted.

  \warning This function is only available if OpenCV (version 2.1.0 or greater)
  was detected during the configuration step.

  \param src : Source image in OpenCV format.
  \param dest : Destination image in ViSP format.
  \param flip : Set to true to vertically flip the converted image.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  vpImage<vpRGBa> Ic; // A color image
  cv::Mat Ip;

  // Read an image on a disk with openCV library
  Ip = cv::imread("image.pgm", 1);// second parameter > 0 for a RGB encoding.
  // Convert the grayscale cv::Mat into vpImage<vpRGBa>
  vpImageConvert::convert(Ip, Ic);

  // ...
#endif
}
  \endcode
*/
void
vpImageConvert::convert(const cv::Mat& src,
          vpImage<vpRGBa>& dest, const bool flip)
{
  if(src.type() == CV_8UC4){
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);
    vpRGBa rgbaVal;
    for(unsigned int i=0; i<dest.getRows(); ++i)
      for(unsigned int j=0; j<dest.getCols(); ++j){
        cv::Vec4b tmp = src.at<cv::Vec4b>((int)i, (int)j);
        rgbaVal.R = tmp[2];
        rgbaVal.G = tmp[1];
        rgbaVal.B = tmp[0];
	rgbaVal.A = tmp[3];
        if(flip)
          dest[dest.getRows()-i-1][j] = rgbaVal;
        else
          dest[i][j] = rgbaVal;
      }
  }else if(src.type() == CV_8UC3){
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);
    vpRGBa rgbaVal;
    rgbaVal.A = 0;
    for(unsigned int i=0; i<dest.getRows(); ++i){
      for(unsigned int j=0; j<dest.getCols(); ++j){
        cv::Vec3b tmp = src.at<cv::Vec3b>((int)i, (int)j);
        rgbaVal.R = tmp[2];
        rgbaVal.G = tmp[1];
        rgbaVal.B = tmp[0];
        if(flip){
          dest[dest.getRows()-i-1][j] = rgbaVal;
        }else{
          dest[i][j] = rgbaVal;
        }
      }
    }
  }else if(src.type() == CV_8UC1){
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);
    vpRGBa rgbaVal;
    for(unsigned int i=0; i<dest.getRows(); ++i){
      for(unsigned int j=0; j<dest.getCols(); ++j){
        rgbaVal = src.at<unsigned char>((int)i, (int)j);
        if(flip){
          dest[dest.getRows()-i-1][j] = rgbaVal;
        }else{
          dest[i][j] = rgbaVal;
        }
      }
    }
  }
}

/*!
  Convert a cv::Mat to a vpImage\<unsigned char\>

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  Similarily to the convert(const IplImage* src, vpImage<vpRGBa> & dest, bool flip)
  method, only Mat with a depth equal to 8 and a channel between 1 and 3 are
  converted.

  \warning This function is only available if OpenCV was detected during
  the configuration step.

  \param src : Source image in OpenCV format.
  \param dest : Destination image in ViSP format.
  \param flip : Set to true to vertically flip the converted image.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  vpImage<unsigned char> Ig; // A grayscale image
  cv::Mat Ip;

  // Read an image on a disk with openCV library
  Ip = cv::imread("image.pgm", 0);// second parameter = 0 for a gray level.
  // Convert the grayscale cv::Mat into vpImage<unsigned char>
  vpImageConvert::convert(Ip, Ig);

  // ...
#endif
}

  \endcode
*/
void
vpImageConvert::convert(const cv::Mat& src,
          vpImage<unsigned char>& dest, const bool flip)
{
  if(src.type() == CV_8UC1){
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);
    if(src.isContinuous() && !flip){
      memcpy(dest.bitmap, src.data, (size_t)(src.rows*src.cols));
    }
    else{
      if(flip){
        for(unsigned int i=0; i<dest.getRows(); ++i){
          memcpy(dest.bitmap+i*dest.getCols(), src.data+(dest.getRows()-i-1)*src.step1(), (size_t)src.step);
        }
      }else{
        for(unsigned int i=0; i<dest.getRows(); ++i){
          memcpy(dest.bitmap+i*dest.getCols(), src.data+i*src.step1(), (size_t)src.step);
        }
      }
    }
  }else if(src.type() == CV_8UC3){
    dest.resize((unsigned int)src.rows, (unsigned int)src.cols);
    if(src.isContinuous() && !flip){
      BGRToGrey((unsigned char*)src.data, (unsigned char*)dest.bitmap, (unsigned int)src.cols, (unsigned int)src.rows, flip);
    }
    else{
      if(flip){
        for(unsigned int i=0; i<dest.getRows(); ++i){
          BGRToGrey((unsigned char*)src.data+i*src.step1(),
                    (unsigned char*)dest.bitmap+(dest.getRows()-i-1)*dest.getCols(),
                    (unsigned int)dest.getCols(), 1, false);
        }
      }else{
        for(unsigned int i=0; i<dest.getRows(); ++i){
          BGRToGrey((unsigned char*)src.data+i*src.step1(),
                    (unsigned char*)dest.bitmap+i*dest.getCols(),
                    (unsigned int)dest.getCols(), 1, false);
        }
      }
    }
  }
}


/*!
  Convert a vpImage\<unsigned char\> to a cv::Mat

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param src : source image (vpRGBa format)
  \param dest : destination image (BGR format)

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  vpImage<unsigned char> Ig; // A greyscale image
  cv::Mat Ip;

  // Read an image on a disk
  vpImageIo::read(Ig, "image.pgm");
  // Convert the vpImage<unsigned char> in to color cv::Mat.
  vpImageConvert::convert(Ig, Ip);
  // Treatments on cv::Mat Ip
  //...
  // Save the cv::Mat on the disk
  cv::imwrite("image.pgm", Ip);
#endif
}

  \endcode
*/
void
vpImageConvert::convert(const vpImage<vpRGBa> & src,
          cv::Mat& dest)
{
  cv::Mat vpToMat((int)src.getRows(), (int)src.getCols(), CV_8UC4, (void*)src.bitmap);

  dest = cv::Mat((int)src.getRows(), (int)src.getCols(), CV_8UC3);
  cv::Mat alpha((int)src.getRows(), (int)src.getCols(), CV_8UC1);

  cv::Mat out[] = {dest, alpha};
  int from_to[] = { 0,2,  1,1,  2,0,  3,3 };
  cv::mixChannels(&vpToMat, 1, out, 2, from_to, 4);
}

/*!
  Convert a vpImage\<unsigned char\> to a cv::Mat

  A cv::Mat is an OpenCV image class. See http://opencv.willowgarage.com for
  the general OpenCV documentation, or
  http://opencv.willowgarage.com/documentation/cpp/core_basic_structures.html
  for the specific Mat structure documentation.

  \warning This function is only available if OpenCV version 2.1.0 or greater
  was detected during the configuration step.

  \param src : source image
  \param dest : destination image
  \param copyData : if true, the image is copied and modification in one object
  will not modified the other.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  vpImage<unsigned char> Ig; // A greyscale image
  cv::Mat Ip;

  // Read an image on a disk
  vpImageIo::read(Ig, "image.pgm");
  // Convert the vpImage<unsigned char> in to greyscale cv::Mat
  vpImageConvert::convert(Ig, Ip);
  // Treatments on cv::MatIp
  //...
  // Save the cv::Mat on the disk
  cv::imwrite("image.pgm", Ip);
#endif
}

  \endcode
*/
void
vpImageConvert::convert(const vpImage<unsigned char> & src,
          cv::Mat& dest, const bool copyData)
{
  if(copyData){
    cv::Mat tmpMap((int)src.getRows(), (int)src.getCols(), CV_8UC1, (void*)src.bitmap);
    dest = tmpMap.clone();
  }else{
    dest = cv::Mat((int)src.getRows(), (int)src.getCols(), CV_8UC1, (void*)src.bitmap);
  }
}

#endif
#endif

#ifdef VISP_HAVE_YARP
/*!
  Convert a vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in ViSP format.
  \param dest : Destination image in YARP format.
  \param copyData : Set to true to copy all the image content. If false we only update the image pointer.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<unsigned char> I; // A mocochrome image
  // Read an image on a disk
  vpImageIo::read(I, "image.pgm");
  
  yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelMono >();
  // Convert the vpImage\<unsigned char\> to a yarp::sig::ImageOf\<yarp::sig::PixelMono\>
  vpImageConvert::convert(I, Iyarp);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<unsigned char> & src,
	yarp::sig::ImageOf< yarp::sig::PixelMono > *dest, const bool copyData)
{
  if(copyData)
  {
    dest->resize(src.getWidth(),src.getHeight());
    memcpy(dest->getRawImage(), src.bitmap, src.getHeight()*src.getWidth());
  }
  else
    dest->setExternal(src.bitmap, (int)src.getCols(), (int)src.getRows());
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelMono\> to a vpImage\<unsigned char\>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in YARP format.
  \param dest : Destination image in ViSP format.
  \param copyData : Set to true to copy all the image content. If false we only update the image pointer.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelMono > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelMono >();
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
void vpImageConvert::convert(const yarp::sig::ImageOf< yarp::sig::PixelMono > *src,
	vpImage<unsigned char> & dest,const bool copyData )
{
  dest.resize(src->height(),src->width());
  if(copyData)
    memcpy(dest.bitmap, src->getRawImage(), src->height()*src->width()*sizeof(yarp::sig::PixelMono));
  else
    dest.bitmap = src->getRawImage();
}
	
/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgba>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in ViSP format.
  \param dest : Destination image in YARP format.
  \param copyData : Set to true to copy all the image content. If false we only update the image pointer.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<vpRGBa> I; // A color image
  // Read an image on a disk
  vpImageIo::read(I,"image.jpg");
  
  yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelRgba >();
  // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgba>
  vpImageConvert::convert(I,Iyarp);

  // ...
#endif
}
  \endcode
*/	
void vpImageConvert::convert(const vpImage<vpRGBa> & src,
	yarp::sig::ImageOf< yarp::sig::PixelRgba > *dest, const bool copyData)
{
  if(copyData){
    dest->resize(src.getWidth(),src.getHeight());
    memcpy(dest->getRawImage(), src.bitmap, src.getHeight()*src.getWidth()*sizeof(vpRGBa));
  }
  else
    dest->setExternal(src.bitmap, (int)src.getCols(), (int)src.getRows());
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgba> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in YARP format.
  \param dest : Destination image in ViSP format.
  \param copyData : Set to true to copy all the image content. If false we only update the image pointer.
  
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelRgba > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelRgba >();
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
void vpImageConvert::convert(const yarp::sig::ImageOf< yarp::sig::PixelRgba > *src,
	vpImage<vpRGBa> & dest,const bool copyData)
{
  dest.resize(src->height(),src->width());
  if(copyData)
    memcpy(dest.bitmap, src->getRawImage(),src->height()*src->width()*sizeof(yarp::sig::PixelRgba));
  else
    dest.bitmap = (vpRGBa*)src->getRawImage();
}

/*!
  Convert a vpImage\<vpRGBa\> to a yarp::sig::ImageOf\<yarp::sig::PixelRgb>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in ViSP format.
  \param dest : Destination image in YARP format.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>

int main()
{
#if defined(VISP_HAVE_YARP)
  vpImage<vpRGBa> I; // A color image
  // Read an image on a disk
  vpImageIo::read(I,"image.jpg");
  
  yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelRgb >();
  // Convert the vpImage<vpRGBa> to a yarp::sig::ImageOf<yarp::sig::PixelRgb>
  vpImageConvert::convert(I,Iyarp);

  // ...
#endif
}
  \endcode
*/
void vpImageConvert::convert(const vpImage<vpRGBa> & src,
	yarp::sig::ImageOf< yarp::sig::PixelRgb > *dest)
{
  dest->resize(src.getWidth(),src.getHeight());
  for(unsigned int i = 0 ; i < src.getRows() ; i++){
    for(unsigned int j = 0 ; j < src.getWidth() ; j++){
	dest->pixel(j,i).r = src[i][j].R;
	dest->pixel(j,i).g = src[i][j].G;
	dest->pixel(j,i).b = src[i][j].B;
    }
  }
}

/*!
  Convert a yarp::sig::ImageOf\<yarp::sig::PixelRgb> to a vpImage\<vpRGBa\>

  A yarp::sig::Image is a YARP image class. See http://eris.liralab.it/yarpdoc/df/d15/classyarp_1_1sig_1_1Image.html for
  the YARP image class documentation.

  \param src : Source image in YARP format.
  \param dest : Destination image in ViSP format.

  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>

#if defined(VISP_HAVE_YARP)
  #include <yarp/sig/ImageFile.h>
#endif

int main()
{
#if defined(VISP_HAVE_YARP)
  yarp::sig::ImageOf< yarp::sig::PixelRgb > *Iyarp = new yarp::sig::ImageOf< yarp::sig::PixelRgb >();
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
void vpImageConvert::convert(const yarp::sig::ImageOf< yarp::sig::PixelRgb > *src,
	vpImage<vpRGBa> & dest)
{
  dest.resize(src->height(),src->width());
  for(int i = 0 ; i < src->height() ; i++){
    for(int j = 0 ; j < src->width() ; j++){
	dest[i][j].R = src->pixel(j,i).r;
	dest[i][j].G = src->pixel(j,i).g;
	dest[i][j].B = src->pixel(j,i).b;
	dest[i][j].A = 0;
    }
  }
}

#endif

#if defined(VISP_HAVE_LIBJPEG)
#if JPEG_LIB_VERSION > 70
/*!
  Convert a vpImage\<unsigned char> to a JPEG compressed buffer

  \param src : Source image in ViSP format.
  \param dest : Destination buffer in JPEG format.
  \param destSize : Size of the destination buffer.
  \param quality : purcentage of the quality of the compressed image.
*/
void vpImageConvert::convertToJPEGBuffer(const vpImage<unsigned char> &src, 
                                         unsigned char **dest, long unsigned int &destSize, int quality)
{
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);
  
  *dest = NULL;
  destSize = 0;
  
  jpeg_mem_dest(&cinfo, dest, &destSize);

  unsigned int width = src.getWidth();
  unsigned int height = src.getHeight();

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 1;
  cinfo.in_color_space = JCS_GRAYSCALE;
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);

  jpeg_start_compress(&cinfo,TRUE);

  unsigned char *line;
  line = new unsigned char[width];
  unsigned char* input = (unsigned char*)src.bitmap;
  while (cinfo.next_scanline < cinfo.image_height)
  {
    for (unsigned int i = 0; i < width; i++)
    {
      line[i] = *(input);
    input++;
    }
  jpeg_write_scanlines(&cinfo, &line, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  delete [] line;
}
  
/*!
  Decompress a JPEG buffer in a vpImage\<unsigned char>

  \param src : Source buffer in JPEG format.
  \param srcSize : Size of the source buffer.
  \param dest : Destination image in ViSP format.
*/
void vpImageConvert::convertToJPEGBuffer(unsigned char *src, long unsigned int srcSize, 
                                         vpImage<unsigned char> &dest)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, src, srcSize);
  jpeg_read_header(&cinfo, TRUE);

  unsigned int width = cinfo.image_width;
  unsigned int height = cinfo.image_height;

  if ( (width != dest.getWidth()) || (height != dest.getHeight()) )
    dest.resize(height,width);

  jpeg_start_decompress(&cinfo);

  unsigned int rowbytes = cinfo.output_width * (unsigned int)(cinfo.output_components);
  JSAMPARRAY buf = (*cinfo.mem->alloc_sarray) ((j_common_ptr) &cinfo, JPOOL_IMAGE, rowbytes, 1);

  if (cinfo.out_color_space == JCS_GRAYSCALE)
  {
    unsigned int row;
    while (cinfo.output_scanline<cinfo.output_height)
    {
      row = cinfo.output_scanline;
      jpeg_read_scanlines(&cinfo,buf,1);
      memcpy(dest[row], buf[0], rowbytes);
    }
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
}
#endif
#endif // defined(VISP_HAVE_LIBJPEG)


#define vpSAT(c) \
        if (c & (~255)) { if (c < 0) c = 0; else c = 255; }
/*!
  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...) to RGB32.
  Destination rgba memory area has to be allocated before.

  \sa YUV422ToRGBa()
*/
void vpImageConvert::YUYVToRGBa(unsigned char* yuyv, unsigned char* rgba,
        unsigned int width, unsigned int height)
{
  unsigned char *s;
  unsigned char *d;
  int w, h, c;
  int r, g, b, cr, cg, cb, y1, y2;

  h = (int)height;
  w = (int)width;
  s = yuyv;
  d = rgba;
  while (h--) {
    c = w >> 1;
    while (c--) {
      y1 = *s++;
      cb = ((*s - 128) * 454) >> 8;
      cg = (*s++ - 128) * 88;
      y2 = *s++;
      cr = ((*s - 128) * 359) >> 8;
      cg = (cg + (*s++ - 128) * 183) >> 8;

      r = y1 + cr;
      b = y1 + cb;
      g = y1 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
      *d++ = 0;

      r = y2 + cr;
      b = y2 + cb;
      g = y2 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
      *d++ = 0;

    }
  }
}
/*!

  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...)
  to RGB24. Destination rgb memory area has to be allocated before.

  \sa YUV422ToRGB()
*/
void vpImageConvert::YUYVToRGB(unsigned char* yuyv, unsigned char* rgb,
             unsigned int width, unsigned int height)
{
  unsigned char *s;
  unsigned char *d;
  int h, w, c;
  int r, g, b, cr, cg, cb, y1, y2;

  h = (int)height;
  w = (int)width;
  s = yuyv;
  d = rgb;
  while (h--) {
    c = w >> 1;
    while (c--) {
      y1 = *s++;
      cb = ((*s - 128) * 454) >> 8;
      cg = (*s++ - 128) * 88;
      y2 = *s++;
      cr = ((*s - 128) * 359) >> 8;
      cg = (cg + (*s++ - 128) * 183) >> 8;

      r = y1 + cr;
      b = y1 + cb;
      g = y1 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);

      r = y2 + cr;
      b = y2 + cb;
      g = y2 - cg;
      vpSAT(r);
      vpSAT(g);
      vpSAT(b);

      *d++ = static_cast<unsigned char>(r);
      *d++ = static_cast<unsigned char>(g);
      *d++ = static_cast<unsigned char>(b);
    }
  }
}
/*!

  Convert an image from YUYV 4:2:2 (y0 u01 y1 v01 y2 u23 y3 v23 ...)
  to grey. Destination rgb memory area has to be allocated before.

  \sa YUV422ToGrey()
*/
void vpImageConvert::YUYVToGrey(unsigned char* yuyv, unsigned char* grey,
        unsigned int size)
{
  unsigned int i=0,j=0;

  while( j < size*2)
    {
      grey[i++] = yuyv[j];
      grey[i++] = yuyv[j+2];
      j+=4;
    }
}


/*!

Convert YUV411 into RGB32
yuv411 : u y1 y2 v y3 y4

*/
void vpImageConvert::YUV411ToRGBa(unsigned char* yuv,
          unsigned char* rgba,
          unsigned int size)
{
#if 1
  //  std::cout << "call optimized ConvertYUV411ToRGBa()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  for(unsigned int i = size / 4; i; i--) {
    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y0  = *yuv++;
    Y1  = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    Y2  = *yuv++;
    Y3  = *yuv++;
    UV  = - U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;
  }
#else
  // tres tres lent ....
  unsigned int i=0,j=0;
  unsigned char r, g, b;
  while( j < numpixels*3/2)
  {

    YUVToRGB (yuv[j+1], yuv[j], yuv[j+3], r, g, b);
    rgba[i]   = r;
    rgba[i+1] = g;
    rgba[i+2] = b;
    rgba[i+3] = 0;
    i+=4;

    YUVToRGB (yuv[j+2], yuv[j], yuv[j+3], r, g, b);
    rgba[i]   = r;
    rgba[i+1] = g;
    rgba[i+2] = b;
    rgba[i+3] = 0;
    i+=4;

    YUVToRGB (yuv[j+4], yuv[j], yuv[j+3], r, g, b);
    rgba[i]   = r;
    rgba[i+1] = g;
    rgba[i+2] = b;
    rgba[i+3] = 0;
    i+=4;

    YUVToRGB (yuv[j+5], yuv[j], yuv[j+3], r, g, b);
    rgba[i]   = r;
    rgba[i+1] = g;
    rgba[i+2] = b;
    rgba[i+3] = 0;
    i+=4;

    j+=6;
  }
#endif

}

/*!
  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into RGB32 images.
  Destination rgba memory area has to be allocated before.

  \sa YUYVToRGBa()
*/
void vpImageConvert::YUV422ToRGBa(unsigned char* yuv,
          unsigned char* rgba,
          unsigned int size)
{

#if 1
  //  std::cout << "call optimized convertYUV422ToRGBa()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1;
  for( unsigned int i = size / 2; i; i-- ) {
    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y0  = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    Y1  = *yuv++;
    UV  = - U - V;

    //---
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    rgba++;
  }

#else
  // tres tres lent ....
 unsigned int i=0,j=0;
 unsigned char r, g, b;

 while( j < size*2)
 {

   YUVToRGB (yuv[j+1], yuv[j], yuv[j+2], r, g, b);
   rgba[i]   = r;
   rgba[i+1] = g;
   rgba[i+2] = b;
   rgba[i+3] = 0;
   i+=4;

   YUVToRGB (yuv[j+3], yuv[j], yuv[j+2], r, g, b);
   rgba[i]   = r;
   rgba[i+1] = g;
   rgba[i+2] = b;
   rgba[i+3] = 0;
   i+=4;
   j+=4;

 }
#endif
}

/*!

Convert YUV411 into Grey
yuv411 : u y1 y2 v y3 y4

*/
void vpImageConvert::YUV411ToGrey(unsigned char* yuv,
          unsigned char* grey,
          unsigned int size)
{


  unsigned int i=0,j=0;
  while( j < size*3/2)
  {

    grey[i  ] = yuv[j+1];
    grey[i+1] = yuv[j+2];
    grey[i+2] = yuv[j+4];
    grey[i+3] = yuv[j+5];

    i+=4;

    j+=6;
  }
}

/*!

  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into RGB images.
  Destination rgb memory area has to be allocated before.

  \sa YUYVToRGB()

*/
void vpImageConvert::YUV422ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int size)
{
#if 1
  //  std::cout << "call optimized convertYUV422ToRGB()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1;
  for( unsigned int i = size / 2; i; i-- ) {
    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y0  = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    Y1  = *yuv++;
    UV  = - U - V;

    //---
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

  }

#else
  // tres tres lent ....
 unsigned int i=0,j=0;
 unsigned char r, g, b;

 while( j < size*2)
 {

   YUVToRGB (yuv[j+1], yuv[j], yuv[j+2], r, g, b);
   rgb[i]   = r;
   rgb[i+1] = g;
   rgb[i+2] = b;
   i+=3;

   YUVToRGB (yuv[j+3], yuv[j], yuv[j+2], r, g, b);
   rgb[i]   = r;
   rgb[i+1] = g;
   rgb[i+2] = b;
   i+=3;
   j+=4;

 }
#endif
}

/*!

  Convert YUV 4:2:2 (u01 y0 v01 y1 u23 y2 v23 y3 ...) images into Grey.
  Destination grey memory area has to be allocated before.

  \sa YUYVToGrey()

*/
void vpImageConvert::YUV422ToGrey(unsigned char* yuv,
          unsigned char* grey,
          unsigned int size)
{
 unsigned int i=0,j=0;

 while( j < size*2)
 {
   grey[i++] = yuv[j+1];
   grey[i++] = yuv[j+3];
   j+=4;
 }
}

/*!

Convert YUV411 into RGB
yuv411 : u y1 y2 v y3 y4

*/
void vpImageConvert::YUV411ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int size)
{
#if 1
  //  std::cout << "call optimized ConvertYUV411ToRGB()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  for(unsigned int i = size / 4; i; i--) {
    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y0  = *yuv++;
    Y1  = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    Y2  = *yuv++;
    Y3  = *yuv++;
    UV  = - U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;
  }
#else
  // tres tres lent ....

  unsigned int i=0,j=0;
  unsigned char r, g, b;

  while( j < size*3/2)
  {
    YUVToRGB (yuv[j+1], yuv[j], yuv[j+3], r, g, b);
    rgb[i]   = r;
    rgb[i+1] = g;
    rgb[i+2] = b;
    i+=3;

    YUVToRGB (yuv[j+2], yuv[j], yuv[j+3], r, g, b);
    rgb[i]   = r;
    rgb[i+1] = g;
    rgb[i+2] = b;
    i+=3;

    YUVToRGB (yuv[j+4], yuv[j], yuv[j+3], r, g, b);
    rgb[i]   = r;
    rgb[i+1] = g;
    rgb[i+2] = b;
    i+=3;

    YUVToRGB (yuv[j+5], yuv[j], yuv[j+3], r, g, b);
    rgb[i]   = r;
    rgb[i+1] = g;
    rgb[i+2] = b;
    i+=3;
    //TRACE("r= %d g=%d b=%d", r, g, b);

    j+=6;
  }
#endif

}



/*!

  Convert YUV420 into RGBa
  yuv420 : Y(NxM), U(N/2xM/2), V(N/2xM/2)

*/
void vpImageConvert::YUV420ToRGBa(unsigned char* yuv,
         unsigned char* rgba,
         unsigned int width, unsigned int height)
{
  //  std::cout << "call optimized ConvertYUV420ToRGBa()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  unsigned int size = width*height;
  unsigned char* iU = yuv + size;
  unsigned char* iV = yuv + 5*size/4;
  for(unsigned int i = 0; i<height/2; i++)
  {
  for(unsigned int j = 0; j < width/2 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv;
    yuv = yuv+width-1;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv-width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba + 4*width-7;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba -4*width+1;
    }
  yuv+=width;
  rgba+=4*width;
  }
}
/*!

  Convert YUV420 into RGB
  yuv420 : Y(NxM), U(N/2xM/2), V(N/2xM/2)

*/
void vpImageConvert::YUV420ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int width, unsigned int height)
{
  //  std::cout << "call optimized ConvertYUV420ToRGB()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  unsigned int size = width*height;
  unsigned char* iU = yuv + size;
  unsigned char* iV = yuv + 5*size/4;
  for(unsigned int i = 0; i<height/2; i++)
  {
  for(unsigned int j = 0; j < width/2 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv;
    yuv = yuv+width-1;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv-width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb + 3*width-5;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb -3*width+1;
    }
  yuv+=width;
  rgb+=3*width;
  }
}

/*!

  Convert YUV420 into Grey
  yuv420 : Y(NxM), U(N/2xM/2), V(N/2xM/2)

*/
void vpImageConvert::YUV420ToGrey(unsigned char* yuv,
         unsigned char* grey,
         unsigned int size)
{
  for(unsigned int i=0 ; i < size ; i++)
  {
    *grey++ = *yuv++;
  }

}
/*!

  Convert YUV444 into RGBa
  yuv444 :  u y v

*/
void vpImageConvert::YUV444ToRGBa(unsigned char* yuv,
         unsigned char* rgba,
         unsigned int size)
{
  register int U, V, R, G, B, V2, U5, UV;
  register int Y;
  for(unsigned int i = 0; i<size; i++)
  {
    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y   = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;


    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;
  }
}
/*!

  Convert YUV444 into RGB
  yuv444 : u y v

*/
void vpImageConvert::YUV444ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int size)
{
  register int U, V, R, G, B, V2, U5, UV;
  register int Y;
  for(unsigned int i = 0; i<size; i++)
  {

    U   = (int)((*yuv++ - 128) * 0.354);
    U5  = 5*U;
    Y   = *yuv++;
    V   = (int)((*yuv++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;
  }
}

/*!

  Convert YUV444 into Grey
  yuv444 : u y v

*/
void vpImageConvert::YUV444ToGrey(unsigned char* yuv,
         unsigned char* grey,
         unsigned int size)
{
  yuv++;
  for(unsigned int i=0 ; i < size ; i++)
  {
    *grey++ = *yuv;
    yuv = yuv + 3;
  }
}

/*!

  Convert YV12 into RGBa
  yuv420 : Y(NxM), V(N/2xM/2), U(N/2xM/2)

*/
void vpImageConvert::YV12ToRGBa(unsigned char* yuv,
         unsigned char* rgba,
         unsigned int width, unsigned int height)
{
  //  std::cout << "call optimized ConvertYV12ToRGBa()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  unsigned int size = width*height;
  unsigned char* iV = yuv + size;
  unsigned char* iU = yuv + 5*size/4;
  for(unsigned int i = 0; i<height/2; i++)
  {
  for(unsigned int j = 0; j < width/2 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv;
    yuv = yuv+width-1;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv-width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba + 4*width-7;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba -4*width+1;
    }
  yuv+=width;
  rgba+=4*width;
  }
}
/*!

  Convert YV12 into RGB
  yuv420 : Y(NxM),  V(N/2xM/2), U(N/2xM/2)

*/
void vpImageConvert::YV12ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int height, unsigned int width)
{
  //  std::cout << "call optimized ConvertYV12ToRGB()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  unsigned int size = width*height;
  unsigned char* iV = yuv + size;
  unsigned char* iU = yuv + 5*size/4;
  for(unsigned int i = 0; i<height/2; i++)
  {
  for(unsigned int j = 0; j < width/2 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv;
    yuv = yuv+width-1;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv-width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb + 3*width-5;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb -3*width+1;
    }
  yuv+=width;
  rgb+=3*width;
  }
}

/*!

  Convert YVU9 into RGBa
  yuv420 : Y(NxM), V(N/4xM/4), U(N/4xM/4)

*/
void vpImageConvert::YVU9ToRGBa(unsigned char* yuv,
         unsigned char* rgba,
         unsigned int width, unsigned int height)
{
  //  std::cout << "call optimized ConvertYVU9ToRGBa()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3,Y4, Y5, Y6, Y7,Y8, Y9, Y10, Y11,Y12, Y13, Y14, Y15;
  unsigned int size = width*height;
  unsigned char* iV = yuv + size;
  unsigned char* iU = yuv + 17*size/16;
  for(unsigned int i = 0; i<height/4; i++)
  {
  for(unsigned int j = 0; j < width/4 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv++;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv+width-3;
    Y4  = *yuv++;
    Y5  = *yuv++;
    Y6  = *yuv++;
    Y7  = *yuv;
    yuv = yuv+width-3;
    Y8  = *yuv++;
    Y9  = *yuv++;
    Y10  = *yuv++;
    Y11  = *yuv;
    yuv = yuv+width-3;
    Y12  = *yuv++;
    Y13  = *yuv++;
    Y14  = *yuv++;
    Y15  = *yuv;
    yuv = yuv-3*width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba + 4*width-15;

    R = Y4 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y4 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y4 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y5 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y5 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y5 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y6 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y6 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y6 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y7 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y7 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y7 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba + 4*width-15;

    R = Y8 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y8 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y8 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y9 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y9 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y9 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y10 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y10 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y10 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y11 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y11 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y11 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba + 4*width-15;

    R = Y12 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y12 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y12 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y13 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y13 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y13 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y14 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y14 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y14 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba++ = 0;

    //---
    R = Y15 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y15 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y15 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgba++ = (unsigned char)R;
    *rgba++ = (unsigned char)G;
    *rgba++ = (unsigned char)B;
    *rgba = 0;
    rgba = rgba -12*width+1;
    }
  yuv+=3*width;
  rgba+=12*width;
  }
}
/*!

  Convert YV12 into RGB
  yuv420 : Y(NxM),  V(N/4xM/4), U(N/4xM/4)

*/
void vpImageConvert::YVU9ToRGB(unsigned char* yuv,
         unsigned char* rgb,
         unsigned int height, unsigned int width)
{
  //  std::cout << "call optimized ConvertYVU9ToRGB()" << std::endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10, Y11, Y12, Y13, Y14, Y15;
  unsigned int size = width*height;
  unsigned char* iV = yuv + size;
  unsigned char* iU = yuv + 17*size/16;
  for(unsigned int i = 0; i<height/4; i++)
  {
  for(unsigned int j = 0; j < width/4 ; j++)
    {
    U   = (int)((*iU++ - 128) * 0.354);
    U5  = 5*U;
    V   = (int)((*iV++ - 128) * 0.707);
    V2  = 2*V;
    UV  = - U - V;
    Y0  = *yuv++;
    Y1  = *yuv++;
    Y2  = *yuv++;
    Y3  = *yuv;
    yuv = yuv+width-3;
    Y4  = *yuv++;
    Y5  = *yuv++;
    Y6  = *yuv++;
    Y7  = *yuv;
    yuv = yuv+width-3;
    Y8  = *yuv++;
    Y9  = *yuv++;
    Y10  = *yuv++;
    Y11  = *yuv;
    yuv = yuv+width-3;
    Y12  = *yuv++;
    Y13  = *yuv++;
    Y14  = *yuv++;
    Y15  = *yuv;
    yuv = yuv-3*width+1;

    // Original equations
    // R = Y           + 1.402 V
    // G = Y - 0.344 U - 0.714 V
    // B = Y + 1.772 U
    R = Y0 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y0 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y0 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y1 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y1 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y1 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y2 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y2 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y2 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y3 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y3 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y3 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb + 3*width-11;

    R = Y4 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y4 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y4 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y5 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y5 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y5 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y6 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y6 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y6 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y7 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y7 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y7 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb + 3*width-11;

    R = Y8 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y8 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y8 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y9 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y9 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y9 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y10 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y10 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y10 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y11 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y11 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y11 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb = (unsigned char)B;
    rgb = rgb + 3*width-11;

    R = Y12 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y12 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y12 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y13 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y13 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y13 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y14 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y14 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y14 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;

    //---
    R = Y15 + V2;
    if ((R >> 8) > 0) R = 255; else if (R < 0) R = 0;

    G = Y15 + UV;
    if ((G >> 8) > 0) G = 255; else if (G < 0) G = 0;

    B = Y15 + U5;
    if ((B >> 8) > 0) B = 255; else if (B < 0) B = 0;

    *rgb++ = (unsigned char)R;
    *rgb++ = (unsigned char)G;
    *rgb++ = (unsigned char)B;
    rgb = rgb -9*width+1;
    }
  yuv+=3*width;
  rgb+=9*width;
  }
}

/*!

  Convert RGB into RGBa

*/
void vpImageConvert::RGBToRGBa(unsigned char* rgb, unsigned char* rgba,
             unsigned int size)
{
  unsigned char *pt_input = rgb;
  unsigned char *pt_end = rgb + 3*size;
  unsigned char *pt_output = rgba;

  while(pt_input != pt_end) {
    *(pt_output++) = *(pt_input++) ; // R
    *(pt_output++) = *(pt_input++) ; // G
    *(pt_output++) = *(pt_input++) ; // B
    *(pt_output++) = 0 ; // A
  }
}

/*!

  Convert RGB into RGBa

*/
void vpImageConvert::RGBaToRGB(unsigned char* rgba, unsigned char* rgb,
             unsigned int size)
{
  unsigned char *pt_input = rgba;
  unsigned char *pt_end = rgba + 4*size;
  unsigned char *pt_output = rgb;

  while(pt_input != pt_end) {
    *(pt_output++) = *(pt_input++) ; // R
    *(pt_output++) = *(pt_input++) ; // G
    *(pt_output++) = *(pt_input++) ; // B
    pt_input++ ;
  }
}
/*!
  Weights convert from linear RGB to CIE luminance assuming a
  modern monitor. See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

*/
void vpImageConvert::RGBToGrey(unsigned char* rgb, unsigned char* grey,
             unsigned int size)
{
  unsigned char *pt_input = rgb;
  unsigned char* pt_end = rgb + size*3;
  unsigned char *pt_output = grey;
  while(pt_input != pt_end) {
    *pt_output = (unsigned char) (0.2126 * (*pt_input)
      + 0.7152 * (*(pt_input + 1))
      + 0.0722 * (*(pt_input + 2)) );
    pt_input += 3;
    pt_output ++;
  }
}
/*!

  Weights convert from linear RGBa to CIE luminance assuming a
  modern monitor. See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

*/
void vpImageConvert::RGBaToGrey(unsigned char* rgba, unsigned char* grey,
        unsigned int size)
{
  unsigned char *pt_input = rgba;
  unsigned char* pt_end = rgba + size*4;
  unsigned char *pt_output = grey;

  while(pt_input != pt_end) {
    *pt_output = (unsigned char) (0.2126 * (*pt_input)
      + 0.7152 * (*(pt_input + 1))
      + 0.0722 * (*(pt_input + 2)) );
    pt_input += 4;
    pt_output ++;
  }
}

/*!
  Convert from grey to linear RGBa.

*/
void
vpImageConvert::GreyToRGBa(unsigned char* grey,
         unsigned char* rgba, unsigned int size)
{
  unsigned char *pt_input = grey;
  unsigned char *pt_end = grey + size;
  unsigned char *pt_output = rgba;

  while(pt_input != pt_end) {
    unsigned char p =  *pt_input ;
    *(pt_output     ) = p ; // R
    *(pt_output  + 1) = p ; // G
    *(pt_output  + 2) = p ; // B
    *(pt_output  + 3) = p ; // A

    pt_input ++;
    pt_output += 4;
  }
}

/*!
  Convert from grey to linear RGBa.

*/
void
vpImageConvert::GreyToRGB(unsigned char* grey,
        unsigned char* rgb, unsigned int size)
{
  unsigned char *pt_input = grey;
  unsigned char* pt_end = grey + size;
  unsigned char *pt_output = rgb;

  while(pt_input != pt_end) {
    unsigned char p =  *pt_input ;
    *(pt_output     ) = p ; // R
    *(pt_output  + 1) = p ; // G
    *(pt_output  + 2) = p ; // B

    pt_input ++;
    pt_output += 3;
  }
}


/*!
  Converts a BGR image to RGBa
  Flips the image verticaly if needed
  assumes that rgba is already resized
*/
void
vpImageConvert::BGRToRGBa(unsigned char * bgr, unsigned char * rgba,
        unsigned int width, unsigned int height, bool flip)
{
  //if we have to flip the image, we start from the end last scanline so the
  //step is negative
  int lineStep = (flip) ? -(int)(width*3) : (int)(width*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? (bgr+(width*height*3)+lineStep) : bgr;
  unsigned char * line;

  unsigned int j=0;
  unsigned int i=0;

  for(i=0 ; i < height ; i++)
  {
    line = src;
    for( j=0 ; j < width ; j++)
    {
      *rgba++ = *(line+2);
      *rgba++ = *(line+1);
      *rgba++ = *(line+0);
      *rgba++ = 0;

      line+=3;
    }
    //go to the next line
    src+=lineStep;
  }

}

/*!
  Converts a BGR image to greyscale
  Flips the image verticaly if needed
  assumes that grey is already resized
*/
void
vpImageConvert::BGRToGrey(unsigned char * bgr, unsigned char * grey,
        unsigned int width, unsigned int height, bool flip)
{
  //if we have to flip the image, we start from the end last scanline so the
  //step is negative
  int lineStep = (flip) ? -(int)(width*3) : (int)(width*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? bgr+(width*height*3)+lineStep : bgr;
  unsigned char * line;

  unsigned int j=0;
  unsigned int i=0;

  for(i=0 ; i < height ; i++)
  {
    line = src;
    for( j=0 ; j < width ; j++)
    {
      *grey++ = (unsigned char)( 0.2126 * *(line+2)
         + 0.7152 * *(line+1)
         + 0.0722 * *(line+0)) ;
      line+=3;
    }

    //go to the next line
    src+=lineStep;
  }
}
/*!
  Converts a RGB image to RGBa
  Flips the image verticaly if needed
  assumes that rgba is already resized
*/
void
vpImageConvert::RGBToRGBa(unsigned char * rgb, unsigned char * rgba,
        unsigned int width, unsigned int height, bool flip)
{
  //if we have to flip the image, we start from the end last scanline so the
  //step is negative
  int lineStep = (flip) ? -(int)(width*3) : (int)(width*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? (rgb+(width*height*3)+lineStep) : rgb;
  unsigned char * line;

  unsigned int j=0;
  unsigned int i=0;

  for(i=0 ; i < height ; i++)
  {
    line = src;
    for( j=0 ; j < width ; j++)
    {
      *rgba++ = *(line++);
      *rgba++ = *(line++);
      *rgba++ = *(line++);
      *rgba++ = 0;
    }
    //go to the next line
    src+=lineStep;
  }
}

/*!
  Converts a RGB image to greyscale
  Flips the image verticaly if needed
  assumes that grey is already resized
*/
void
vpImageConvert::RGBToGrey(unsigned char * rgb, unsigned char * grey,
        unsigned int width, unsigned int height, bool flip)
{
  //if we have to flip the image, we start from the end last scanline so the
  //step is negative
  int lineStep = (flip) ? -(int)(width*3) : (int)(width*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? rgb+(width*height*3)+lineStep : rgb;
  unsigned char * line;

  unsigned int j=0;
  unsigned int i=0;

  unsigned r,g,b;

  for(i=0 ; i < height ; i++)
  {
    line = src;
    for( j=0 ; j < width ; j++)
    {
      r = *(line++);
      g = *(line++);
      b = *(line++);
      *grey++ = (unsigned char)( 0.2126 * r + 0.7152 * g + 0.0722 * b) ;
    }

    //go to the next line
    src+=lineStep;
  }
}

/*!

  Compute the look up table useful for YCbCr conversions.

*/
void vpImageConvert::computeYCbCrLUT()
{
  if (YCbCrLUTcomputed == false) {
    int index = 256, aux;

    while (index-- ) {

      aux = index - 128;
      vpImageConvert::vpCrr[index] = (int)( 364.6610 * aux) >> 8;
      vpImageConvert::vpCgb[index] = (int)( -89.8779 * aux) >> 8;
      vpImageConvert::vpCgr[index] = (int)(-185.8154 * aux) >> 8;
      vpImageConvert::vpCbb[index] = (int)( 460.5724 * aux) >> 8;
    }

    YCbCrLUTcomputed = true;
  }
}


/*!

  Convert an image from YCbCr 4:2:2 (Y0 Cb01 Y1 Cr01 Y2 Cb23 Y3 ...) to RGB
  format. Destination rgb memory area has to be allocated before.

  - In YCbCr (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGB format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue


*/
void vpImageConvert::YCbCrToRGB(unsigned char *ycbcr, unsigned char *rgb,
        unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycbcr;
  unsigned char *pt_rgb = rgb;
  cbv = pt_ycbcr + 1;
  crv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

	while (size--) {
		register int val_r, val_g, val_b;
		if (!(col++ % 2)) {
			cbv = pt_ycbcr + 1;
			crv = pt_ycbcr + 3;
		}

		val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgb++ = (val_r < 0) ? 0u :
      ((val_r > 255) ? 255u : (unsigned char)val_r); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0u :
      ((val_g > 255) ? 255u : (unsigned char)val_g); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0u :
      ((val_b > 255) ? 255u : (unsigned char)val_b); // Blue component.

    pt_ycbcr += 2;
	}
}

/*!

  Convert an image from YCbCr 4:2:2 (Y0 Cb01 Y1 Cr01 Y2 Cb23 Y3...) to
  RGBa format. Destination rgba memory area has to be allocated
  before.

  - In YCbCr (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGBa format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue
    Byte 3: -


*/
void vpImageConvert::YCbCrToRGBa(unsigned char *ycbcr, unsigned char *rgba,
         unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycbcr;
  unsigned char *pt_rgba = rgba;
  cbv = pt_ycbcr + 1;
  crv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    register int val_r, val_g, val_b;
    if (!(col++ % 2)) {
      cbv = pt_ycbcr + 1;
      crv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgba++ = (val_r < 0) ? 0u :
      ((val_r > 255) ? 255u : (unsigned char)val_r); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0u :
      ((val_g > 255) ? 255u : (unsigned char)val_g); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0u :
      ((val_b > 255) ? 255u : (unsigned char)val_b); // Blue component.
    *pt_rgba++ = 0;

    pt_ycbcr += 2;
  }
}


/*!

  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to grey
  format. Destination grey image memory area has to be allocated
  before.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In grey format, each pixel is coded using 8 bytes.

*/
void vpImageConvert::YCbCrToGrey(unsigned char* yuv,
          unsigned char* grey,
          unsigned int size)
{
 unsigned int i=0,j=0;

 while( j < size*2)
 {
   grey[i++] = yuv[j];
   grey[i++] = yuv[j+2];
   j+=4;
 }
}

/*!

  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to RGB
  format. Destination rgb memory area has to be allocated before.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGB format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue

*/
void vpImageConvert::YCrCbToRGB(unsigned char *ycrcb, unsigned char *rgb,
        unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycrcb;
  unsigned char *pt_rgb = rgb;
  crv = pt_ycbcr + 1;
  cbv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    register int val_r, val_g, val_b;
    if (!(col++ % 2)) {
      crv = pt_ycbcr + 1;
      cbv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgb++ = (val_r < 0) ? 0u :
      ((val_r > 255) ? 255u : (unsigned char)val_r); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0u :
      ((val_g > 255) ? 255u : (unsigned char)val_g); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0u :
      ((val_b > 255) ? 255u : (unsigned char)val_b); // Blue component.

    pt_ycbcr += 2;
  }
}
/*!

  Convert an image from YCrCb 4:2:2 (Y0 Cr01 Y1 Cb01 Y2 Cr23 Y3 ...) to RGBa
  format. Destination rgba memory area has to be allocated before.

  - In YCrCb (4:2:2) format  each pixel is coded using 16 bytes.
    Byte 0: YO (Luma for Pixel 0)
    Byte 1: Chroma Red Cr (Red Chroma for Pixel 0 and 1)
    Byte 2: Y1 (Luma for Pixel 1)
    Byte 3: Chroma blue Cb (Blue Chroma for Pixel 0 and 1)
    Byte 4: Y2 (Luma for Pixel 2)

  - In RGBa format, each pixel is coded using 24 bytes.
    Byte 0: Red
    Byte 1: Green
    Byte 2: Blue
    Byte 3: -


*/
void vpImageConvert::YCrCbToRGBa(unsigned char *ycrcb, unsigned char *rgba,
         unsigned int size)
{
  unsigned char *cbv;
  unsigned char *crv;
  unsigned char *pt_ycbcr = ycrcb;
  unsigned char *pt_rgba = rgba;
  crv = pt_ycbcr + 1;
  cbv = pt_ycbcr + 3;

  vpImageConvert::computeYCbCrLUT();

  int col = 0;

  while (size--) {
    register int val_r, val_g, val_b;
    if (!(col++ % 2)) {
      crv = pt_ycbcr + 1;
      cbv = pt_ycbcr + 3;
    }

    val_r = *pt_ycbcr + vpImageConvert::vpCrr[*crv];
    val_g = *pt_ycbcr + vpImageConvert::vpCgb[*cbv] + vpImageConvert::vpCgr[*crv];
    val_b = *pt_ycbcr + vpImageConvert::vpCbb[*cbv];

    vpDEBUG_TRACE(5, "[%d] R: %d G: %d B: %d\n", size, val_r, val_g, val_b);

    *pt_rgba++ = (val_r < 0) ? 0u :
      ((val_r > 255) ? 255u : (unsigned char)val_r); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0u :
      ((val_g > 255) ? 255u : (unsigned char)val_g); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0u :
      ((val_b > 255) ? 255u : (unsigned char)val_b); // Blue component.
    *pt_rgba++ = 0;

    pt_ycbcr += 2;
  }
}

/*!

  Split an image from vpRGBa format to monochrome channels.
  \param src : source image.
  \param pR : red channel. Set as NULL if not needed.
  \param pG : green channel. Set as NULL if not needed.
  \param pB : blue channel. Set as NULL if not needed.
  \param pa : alpha channel. Set as NULL if not needed.

  Example code using split :

  \code
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
  vpImage<vpRGBa> Ic; // A color image

  // Load a color image from the disk
  vpImageIo::read(Ic,"image.ppm");

  // Only R and B Channels are desired.
  vpImage<unsigned char> R, B;

  // Split Ic color image
  // R and B will be resized in split function if needed
  vpImageConvert::split(Ic, &R, NULL, &B, NULL);

  // Save the the R Channel.
  vpImageIo::write(R, "RChannel.pgm");
}
  \endcode
*/
void vpImageConvert::split(const vpImage<vpRGBa> &src,
                    vpImage<unsigned char>* pR,
                    vpImage<unsigned char>* pG,
                    vpImage<unsigned char>* pB,
                    vpImage<unsigned char>* pa)
{
  register size_t n = src.getNumberOfPixel();
  unsigned int height = src.getHeight();
  unsigned int width  = src.getWidth();
  unsigned char* input;
  unsigned char* dst ;

  vpImage<unsigned char>* tabChannel[4];

/*  incrsrc[0] = 0; //init
  incrsrc[1] = 0; //step after the first used channel
  incrsrc[2] = 0; //step after the second used channel
  incrsrc[3] = 0;
  incrsrc[4] = 0;
 */
  tabChannel[0] = pR;
  tabChannel[1] = pG;
  tabChannel[2] = pB;
  tabChannel[3] = pa;

  register size_t    i;    /* ordre    */
  for(unsigned int j = 0;j < 4;j++){
    if(tabChannel[j]!=NULL){
      if(tabChannel[j]->getHeight() != height ||
         tabChannel[j]->getWidth() != width){
        tabChannel[j]->resize(height,width);
      }
      dst = (unsigned char*)tabChannel[j]->bitmap;

      input = (unsigned char*)src.bitmap+j;
      i = 0;
#if 1 //optimization
      if (n >= 4) {    /* boucle deroulee lsize fois    */
        n -= 3;
        for (; i < n; i += 4) {
          *dst = *input; input += 4; dst++;
          *dst = *input; input += 4; dst++;
          *dst = *input; input += 4; dst++;
          *dst = *input; input += 4; dst++;
        }
        n += 3;
      }
#endif
      for (; i < n; i++) {
        *dst = *input; input += 4; dst ++;
      }
    }
  }
}

/*!

  Converts a MONO16 grey scale image (each pixel is coded by two bytes) into a
  grey image where each pixels are coded on one byte.

  \param grey16 : Input image to convert (two bytes per pixel).
  \param grey   : Output image (one byte per pixel)
  \param size : The image size or the number of pixels.

*/
void vpImageConvert::MONO16ToGrey(unsigned char *grey16, unsigned char *grey,
          unsigned int size)
{
  register int i = (((int)size)<<1)-1;
  register int j = (int)size-1;
  register int y;

  while (i >= 0) {
    y = grey16[i--];
    grey[j--] = static_cast<unsigned char>( (y+(grey16[i--]<<8))>>8 );
  }
}

/*!

  Converts a MONO16 grey scale image (each pixel is coded by two bytes) into a
  grey image where each pixels are coded on one byte.

  \param grey16 : Input image to convert (two bytes per pixel).
  \param rgba   : Output image.
  \param size : The image size or the number of pixels.

*/
void vpImageConvert::MONO16ToRGBa(unsigned char *grey16, unsigned char *rgba,
          unsigned int size)
{
  register int i = (((int)size)<<1)-1;
  register int j = (int)(size*4-1);
  register int y;
  register unsigned char v;

  while (i >= 0) {
    y = grey16[i--];
    v = static_cast<unsigned char>( (y+(grey16[i--]<<8))>>8 );
    rgba[j--] = 0;
    rgba[j--] = v;
    rgba[j--] = v;
    rgba[j--] = v;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
