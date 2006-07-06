/****************************************************************************
 *
 * $Id: vpImageConvert.h,v 1.5 2006-07-06 12:34:03 brenier Exp $
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
 *
 *****************************************************************************/

/*!
  \file vpImageConvert.h
  \brief Convert image types
*/

#ifndef vpIMAGECONVERT_H
#define vpIMAGECONVERT_H


// image
#include <visp/vpConfig.h>
#include <visp/vpImage.h>

// color
#include <visp/vpRGBa.h>

/*!
  \class vpImageConvert.h
  \brief Convert image types
*/
class VISP_EXPORT vpImageConvert
{

public:
  static void convert(const vpImage<unsigned char> &src,
		      vpImage<vpRGBa> & dest) ;
  static void convert(const vpImage<vpRGBa> &src,
		      vpImage<unsigned char> & dest) ;
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
			   unsigned char* rgba, int size);
  static void YUV411ToRGB(unsigned char* yuv, 
			  unsigned char* rgb, int size);
  static void YUV411ToGrey(unsigned char* yuv, 
			  unsigned char* grey, int size);
  static void YUV422ToRGBa(unsigned char* yuv, 
			   unsigned char* rgba, int size);
  static void YUV422ToRGB(unsigned char* yuv, 
			  unsigned char* rgb, int size);
  static void YUV422ToGrey(unsigned char* yuv, 
			   unsigned char* grey, int size);
  static void RGBToRGBa(unsigned char* rgb, 
			unsigned char* rgba, int size);
  static void RGBToGrey(unsigned char* rgb, 
			unsigned char* grey, int size);
  static void RGBaToGrey(unsigned char* rgba, 
			 unsigned char* grey, int size);
  static void GreyToRGBa(unsigned char* grey, 
			 unsigned char* rgba, int size);
  static void GreyToRGB(unsigned char* grey, 
			unsigned char* rgb, int size);

  static void BGRaToRGBa(const unsigned char * bgra, unsigned char * rgba, int size);

  static void BGRaToGrey(const unsigned char * bgra, unsigned char * grey, int size);
} ;

#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
