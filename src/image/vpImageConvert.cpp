/****************************************************************************
 *
 * $Id: vpImageConvert.cpp,v 1.8 2007-02-26 16:39:17 fspindle Exp $
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
 *
 *****************************************************************************/

/*!
  \file vpImageConvert.cpp
  \brief Convert image types
*/



// image
#include <visp/vpImageConvert.h>

bool vpImageConvert::YCbCrLUTcomputed = false;
static int vpCrr[256];
static int vpCgb[256];
static int vpCgr[256];
static int vpCbb[256];

void
vpImageConvert::convert(const vpImage<unsigned char> &src,
			vpImage<vpRGBa> & dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;

  GreyToRGBa(src.bitmap, (unsigned char *)dest.bitmap,
	     src.getHeight() * src.getWidth() );
}

void
vpImageConvert::convert(const vpImage<vpRGBa> &src,
			vpImage<unsigned char> & dest)
{
  dest.resize(src.getHeight(), src.getWidth()) ;

  RGBaToGrey((unsigned char *)src.bitmap, dest.bitmap,
	     src.getHeight() * src.getWidth() );
}


/*!

Convert YUV411 into RGBa
yuv411 : u y1 y2 v y3 y4

*/
void vpImageConvert::YUV411ToRGBa(unsigned char* yuv,
				  unsigned char* rgba,
				  int size)
{
#if 1
  //  cout << "call optimized ConvertYUV411ToRGBa()" << endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1, Y2, Y3;
  for(int i = size / 4; i; i--) {
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
  int i=0,j=0;
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
				  int size)
{

#if 1
  //  cout << "call optimized convertYUV422ToRGBa()" << endl;
  register int U, V, R, G, B, V2, U5, UV;
  register int Y0, Y1;
  for( int i = size / 2; i; i-- ) {
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
 int i=0,j=0;
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
				  int size)
{

  int i=0,j=0;
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
				 int size)
{
 int i=0,j=0;
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
}

/*!

Convert YUV422 into Grey
yuv422 : u y1 v y2 u y3 v y4

*/
void vpImageConvert::YUV422ToGrey(unsigned char* yuv,
				  unsigned char* grey,
				  int size)
{
 int i=0,j=0;

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
				 int size)
{

  int i=0,j=0;
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

}


/*!

Convert YUV422 into RGBa
yuv422 : u y1 v y2 u y3 v y4

*/
void vpImageConvert::RGBToRGBa(unsigned char* rgb, unsigned char* rgba,
			       int size)
{
 int i=0, j=0;

 while( i < size*3)
 {

   rgba[j]   = (unsigned char)rgb[i];
   rgba[j+1] = (unsigned char)rgb[i+1];
   rgba[j+2] = (unsigned char)rgb[i+2];
   rgba[j+3] = 0;
   i+=3;
   j+=4;
 }
}

/*!

  Weights convert from linear RGB to CIE luminance assuming a
  modern monitor. See Charles Pontyon's Colour FAQ
  http://www.poynton.com/notes/colour_and_gamma/ColorFAQ.html

*/
void vpImageConvert::RGBToGrey(unsigned char* rgb, unsigned char* grey,
			       int size)
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
				int size)
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
			   unsigned char* rgba, int size)
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
			  unsigned char* rgb, int size)
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
vpImageConvert::BGRToRGBa(unsigned char * bgr, unsigned char * rgba, int cols, int rows, bool flip)
{
	//if we have to flip the image, we start from the end last scanline so the step is negative
	int lineStep = (flip) ? -(cols*3) : (cols*3);

	//starting source address = last line if we need to flip the image
	unsigned char * src = (flip) ? (bgr+(cols*rows*3)+lineStep) : bgr;
	unsigned char * line;

	int j=0;
	int i=0;

	for(i=0 ; i < rows ; i++)
	{
	  line = src;
	  for( j=0 ; j < cols ; j++)
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
vpImageConvert::BGRToGrey(unsigned char * bgr, unsigned char * grey, int cols, int rows, bool flip)
{
  //if we have to flip the image, we start from the end last scanline so the step is negative
  int lineStep = (flip) ? -(cols*3) : (cols*3);

  //starting source address = last line if we need to flip the image
  unsigned char * src = (flip) ? bgr+(cols*rows*3)+lineStep : bgr;
  unsigned char * line;

  int j=0;
  int i=0;

  for(i=0 ; i < rows ; i++)
    {
      line = src;
      for( j=0 ; j < cols ; j++)
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
      vpCrr[index] = (int)( 364.6610 * aux) >> 8;
      vpCgb[index] = (int)( -89.8779 * aux) >> 8;
      vpCgr[index] = (int)(-185.8154 * aux) >> 8;
      vpCbb[index] = (int)( 460.5724 * aux) >> 8;
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
				int size)
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

    val_r = *pt_ycbcr + vpCrr[*crv];
    val_g = *pt_ycbcr + vpCgb[*cbv] + vpCgr[*crv];
    val_b = *pt_ycbcr + vpCbb[*cbv];

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
				 int size)
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

    val_r = *pt_ycbcr + vpCrr[*crv];
    val_g = *pt_ycbcr + vpCgb[*cbv] + vpCgr[*crv];
    val_b = *pt_ycbcr + vpCbb[*cbv];

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
				int size)
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

    val_r = *pt_ycbcr + vpCrr[*crv];
    val_g = *pt_ycbcr + vpCgb[*cbv] + vpCgr[*crv];
    val_b = *pt_ycbcr + vpCbb[*cbv];

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
				 int size)
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

    val_r = *pt_ycbcr + vpCrr[*crv];
    val_g = *pt_ycbcr + vpCgb[*cbv] + vpCgr[*crv];
    val_b = *pt_ycbcr + vpCbb[*cbv];

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


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
