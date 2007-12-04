/****************************************************************************
 *
 * $Id: vpImageConvert.cpp,v 1.23 2007-12-04 16:26:36 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#ifdef VISP_HAVE_OPENCV
/*!
Convert a IplImage to a vpImage\<vpRGBa\>

An IplImage is an OpenCV (Intel's Open source Computer Vision Library)
image structure. See http://opencvlibrary.sourceforge.net/ for general
OpenCV documentation, or http://opencvlibrary.sourceforge.net/CxCore
for the specific IplImage structure documentation.

\warning This function is only available if OpenCV was detected during
the configuration step.

\param src : source image
\param dest : destination image

\code
    #include <visp/vpImage.h>
    #include <visp/vpImageIo.h>
    #include <visp/vpImageConvert.h>

    vpImage<vpRGBa> Ic; // A color image
    IplImage* Ip;

    //Read an image on a disk with openCV library
    Ip = cvLoadImage("image.ppm",CV_LOAD_IMAGE_COLOR);
    //Convert the grayscale IplImage into vpImage<vpRGBa>
    vpImageConvert::convert(Ip,Ic);

    //...

    //Release Ip header and data
    cvReleaseImage(&Ip);
\endcode
*/
void
vpImageConvert::convert(const IplImage* src,
      vpImage<vpRGBa> & dest)
{
  int nChannel = src->nChannels;
  int depth = src->depth;
  int height = src->height;
  int width = src->width;
  int widthStep = src->widthStep;

  if(nChannel == 3 && depth == 8){
    dest.resize(height,width);

    //starting source address
    unsigned char* input = (unsigned char*)src->imageData;
    unsigned char* line;
    unsigned char* output = (unsigned char*)dest.bitmap;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
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
    dest.resize(height,width);
    //starting source address
    unsigned char * input = (unsigned char*)src->imageData;
    unsigned char * line;
    unsigned char * output = (unsigned char*)dest.bitmap;

    for(int i=0 ; i < height ; i++)
    {
      line = input;
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

\param src : source image
\param dest : destination image

\code
    #include <visp/vpImage.h>
    #include <visp/vpImageIo.h>
    #include <visp/vpImageConvert.h>

    vpImage<unsigned char> Ig; // A grayscale image
    IplImage* Ip;

    //Read an image on a disk with openCV library
    Ip = cvLoadImage("image.pgm",CV_LOAD_IMAGE_GRAYSCALE);
    //Convert the grayscale IplImage into vpImage<unsigned char>
    vpImageConvert::convert(Ip,Ig);

    //...

    //Release Ip header and data
    cvReleaseImage(&Ip);
\endcode
*/
void
vpImageConvert::convert(const IplImage* src,
      vpImage<unsigned char> &dest)
{
  int nChannel = src->nChannels;
  int depth = src->depth;
  int height = src->height;
  int width = src->width;
  int widthStep = src->widthStep;

  if(widthStep == width){
    if(nChannel == 1 && depth == 8){
      dest.resize(height,width) ;
      memcpy(dest.bitmap, src->imageData,
              height*width);
    }
    if(nChannel == 3 && depth == 8){
      dest.resize(height,width) ;
      BGRToGrey((unsigned char*)src->imageData,dest.bitmap,width,height,false);
    }
  }
  else{
    if(nChannel == 1 && depth == 8){
      dest.resize(height,width) ;
      for (int i =0  ; i < height ; i++){
        memcpy(dest.bitmap, src->imageData + i*widthStep,
              width);
      }
    }
    if(nChannel == 3 && depth == 8){
      dest.resize(height,width) ;
      for (int i = 0  ; i < height ; i++){
        BGRToGrey((unsigned char*)src->imageData + i*widthStep,
                    dest.bitmap + i*width,width,1,false);
      }
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
    #include <visp/vpImage.h>
    #include <visp/vpImageIo.h>
    #include <visp/vpImageConvert.h>

    vpImage<vpRGBa> Ic; // A color image
    IplImage* Ip;

    //Read an image on a disk
    vpImageIo::readPPM(Ic, "image.ppm");
    //Convert the vpImage<vpRGBa> in to color IplImage
    vpImageConvert::convert(Ic,Ip);
    //Treatments on IplImage
    //...

    //Release Ip header and data
    cvReleaseImage(&Ip);
\endcode
*/
void
vpImageConvert::convert(const vpImage<vpRGBa> & src,
       IplImage* &dest)
{
  unsigned int height = src.getHeight();
  unsigned int width  = src.getWidth();
  CvSize size = cvSize(width,height);
  int depth = 8;
  int channels = 3;
  if (dest != NULL){
    if(dest->nChannels != channels || dest->depth != depth
       || dest->height != (int) height || dest->width != (int) width){
      cvReleaseImage(&dest);
      dest = cvCreateImage( size, depth, channels );
    }
  }
  else dest = cvCreateImage( size, depth, channels );
    
  
  //starting source address
  unsigned char * input = (unsigned char*)src.bitmap;//rgba image
  unsigned char * line;
  unsigned char * output = (unsigned char*)dest->imageData;//bgr image

  unsigned int j=0;
  unsigned int i=0;
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
    #include <visp/vpImage.h>
    #include <visp/vpImageIo.h>
    #include <visp/vpImageConvert.h>

    vpImage<unsigned char> Ig; // A greyscale image
    IplImage* Ip;

    //Read an image on a disk
    vpImageIo::readPGM(Ig, "image.pgm");
    //Convert the vpImage<unsigned char> in to greyscale IplImage
    vpImageConvert::convert(Ig,Ip);
    //Treatments on IplImage Ip
    //...

    //Release Ip header and data
    cvReleaseImage(&Ip);
\endcode
*/
void
vpImageConvert::convert(const vpImage<unsigned char> & src,
      IplImage* &dest)
{
  unsigned int height = src.getHeight();
  unsigned int width  = src.getWidth();
  CvSize size = cvSize(width,height);
  int depth = 8;
  int channels = 1;
  if (dest != NULL){
    if(dest->nChannels != channels || dest->depth != depth
       || dest->height != (int) height || dest->width != (int) width){
      cvReleaseImage(&dest);
      dest = cvCreateImage( size, depth, channels );
    }
  }
  else dest = cvCreateImage( size, depth, channels );
  
  int widthStep = dest->widthStep;

  if ((int) width == widthStep){
    memcpy(dest->imageData,src.bitmap, width*height);
  }
  else{
    //copying each line taking account of the widthStep
    for (unsigned int i =0  ; i < height ; i++){
          memcpy(dest->imageData + i*widthStep,src.bitmap + i*width,
                width);
    }
  }
}

#endif
/*!

Convert YUV411 into RGBa
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

Convert YUV422 into RGBa
yuv422 : u y1 v y2 u y3 v y4

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

Convert YUV422 into RGB
yuv422 : u y1 v y2 u y3 v y4

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

Convert YUV422 into Grey
yuv422 : u y1 v y2 u y3 v y4

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
				 unsigned int height, unsigned int width)
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
	//if we have to flip the image, we start from the end last scanline so the step is negative
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
  //if we have to flip the image, we start from the end last scanline so the step is negative
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

  Convert an image from YCbCr to RGB format. Destination rgb memory area has to
  be allocated before.

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

    *pt_rgb++ = (val_r < 0) ? 0 :
      ((val_r > 255) ? 255 : (unsigned char)val_r); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0 :
      ((val_g > 255) ? 255 : (unsigned char)val_g); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0 :
      ((val_b > 255) ? 255 : (unsigned char)val_b); // Blue component.

    pt_ycbcr += 2;
  }
}

/*!

  Convert an image from YCbCr to RGBa format. Destination rgba memory area has
  to be allocated before.

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

    *pt_rgba++ = (val_r < 0) ? 0 :
      ((val_r > 255) ? 255 : (unsigned char)val_r); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0 :
      ((val_g > 255) ? 255 : (unsigned char)val_g); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0 :
      ((val_b > 255) ? 255 : (unsigned char)val_b); // Blue component.
    *pt_rgba++ = 0;

    pt_ycbcr += 2;
  }
}


/*!

Convert YUV422 into Grey
yuv422 : y1 u1 y2 v1 y3 u2 y4 v2

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

  Convert an image from YCrCb to RGB format. Destination rgb memory area has to
  be allocated before.

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

    *pt_rgb++ = (val_r < 0) ? 0 :
      ((val_r > 255) ? 255 : (unsigned char)val_r); // Red component.
    *pt_rgb++ = (val_g < 0) ? 0 :
      ((val_g > 255) ? 255 : (unsigned char)val_g); // Green component.
    *pt_rgb++ = (val_b < 0) ? 0 :
      ((val_b > 255) ? 255 : (unsigned char)val_b); // Blue component.

    pt_ycbcr += 2;
  }
}
/*!

  Convert an image from YCrCb to RGBa format. Destination rgba memory area has
  to be allocated before.

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

    *pt_rgba++ = (val_r < 0) ? 0 :
      ((val_r > 255) ? 255 : (unsigned char)val_r); // Red component.
    *pt_rgba++ = (val_g < 0) ? 0 :
      ((val_g > 255) ? 255 : (unsigned char)val_g); // Green component.
    *pt_rgba++ = (val_b < 0) ? 0 :
      ((val_b > 255) ? 255 : (unsigned char)val_b); // Blue component.
    *pt_rgba++ = 0;

    pt_ycbcr += 2;
  }
}

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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
