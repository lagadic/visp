/****************************************************************************
 *
 * $Id: vpImageConvert.h,v 1.19 2008-09-26 15:20:54 fspindle Exp $
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
  \file vpImageConvert.h
  \brief Convert image types
*/

#ifndef vpIMAGECONVERT_H
#define vpIMAGECONVERT_H



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
// image
#include <visp/vpImage.h>
// color
#include <visp/vpRGBa.h>

#ifdef VISP_HAVE_OPENCV
#include <highgui.h> // for opencv
#endif


/*!
  \class vpImageConvert

  \ingroup ImageConversion

  \brief Convert image types.

*/
class VISP_EXPORT vpImageConvert
{

public:
  static void convert(const vpImage<unsigned char> &src,
		      vpImage<vpRGBa> & dest) ;
  static void convert(const vpImage<vpRGBa> &src,
		      vpImage<unsigned char> & dest) ;
#ifdef VISP_HAVE_OPENCV
  static void convert(const IplImage* src,
          vpImage<vpRGBa> & dest) ;
  static void convert(const IplImage* src,
          vpImage<unsigned char> & dest) ;
  static void convert(const vpImage<vpRGBa> & src,
          IplImage* &dest) ;
  static void convert(const vpImage<unsigned char> & src,
          IplImage* &dest) ;
#endif

  static void split(const vpImage<vpRGBa> &src,
                    vpImage<unsigned char>* pR,
                    vpImage<unsigned char>* pG,
                    vpImage<unsigned char>* pB,
                    vpImage<unsigned char>* pa = NULL) ;

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
    };
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
			unsigned char* grey, unsigned int size);
  static void RGBaToGrey(unsigned char* rgba,
			 unsigned char* grey, unsigned int size);

  static void RGBToRGBa(unsigned char * bgr, unsigned char * rgba,
			unsigned int width, unsigned int height, bool flip = false);
  static void RGBToGrey(unsigned char * bgr, unsigned char * grey,
			unsigned int width, unsigned int height, bool flip = false);

  static void GreyToRGBa(unsigned char* grey,
			 unsigned char* rgba, unsigned int size);
  static void GreyToRGB(unsigned char* grey,
			unsigned char* rgb, unsigned int size);

  static void BGRToRGBa(unsigned char * bgr, unsigned char * rgba,
			unsigned int width, unsigned int height, bool flip);

  static void BGRToGrey(unsigned char * bgr, unsigned char * grey,
			unsigned int width, unsigned int height, bool flip);

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

private:
  static void computeYCbCrLUT();

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
