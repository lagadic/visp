/****************************************************************************
 *
 * $Id: vpImageTools.h,v 1.11 2007-12-04 16:17:17 asaunier Exp $
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
 * Image tools.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpImageTools_H
#define vpImageTools_H

/*!
  \file vpImageTools.h

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMath.h>
#include <visp/vpRect.h>
#include <visp/vpCameraParameters.h>

/*!
  \class vpImageTools

  \brief Various image tools; sub-image extraction, modification of
  the look up table, binarisation...

*/
class VISP_EXPORT vpImageTools
{

public:
  template<class Type>
  static void createSubImage(const vpImage<Type> &I,
			     unsigned int i_sub, unsigned int j_sub,
			     unsigned int nrow_sub, unsigned int ncol_sub,
			     vpImage<Type> &S);

  template<class Type>
  static void createSubImage(const vpImage<Type> &I,
			     const vpRect &rect,
			     vpImage<Type> &S);
  template<class Type>
  static void binarise(vpImage<Type> &I,
		       Type threshold1, Type threshold2, 
		       Type value1, Type value2, Type value3);
  static void changeLUT(vpImage<unsigned char>& I,
			unsigned char A,
			unsigned char newA,
			unsigned char B,
			unsigned char newB);
  template<class Type>
  static void undistort(const vpImage<Type> &I,
                        const vpCameraParameters &cam,
                        vpImage<Type> &newI);
} ;

/*!
  Extract a sub part of an image

  \param I : Input image from which a sub image will be extracted.
  \param i_sub, j_sub : coordinates of the upper left point of the sub image
  \param nrow_sub, ncol_sub : number of row, column of the sub image
  \param S : Sub-image.
*/
template<class Type>
void vpImageTools::createSubImage(const vpImage<Type> &I,
				  unsigned int i_sub, unsigned int j_sub,
				  unsigned int nrow_sub, unsigned int ncol_sub,
				  vpImage<Type> &S)
{
  unsigned int i,j ;
  unsigned int imax = i_sub + nrow_sub ;
  unsigned int jmax = j_sub + ncol_sub ;

  if (imax > I.getHeight())
  {
    imax = I.getHeight() -1 ;
    nrow_sub = imax-i_sub ;
  }
  if (jmax > I.getWidth())
  {
    jmax = I.getWidth() -1 ;
    ncol_sub = jmax -j_sub ;
  }

  S.resize(nrow_sub, ncol_sub) ;
  for (i=i_sub ; i < imax ; i++)
    for (j=j_sub ; j < jmax ; j++)
    {
      S[i-i_sub][j-j_sub] = I[i][j] ;
    }
}
/*!
  Extract a sub part of an image

  \param I : Input image from which a sub image will be extracted.

  \param rect : Rectangle area in the image \e I corresponding to the
  sub part of the image to extract.

  \param S : Sub-image.
*/
template<class Type>
void vpImageTools::createSubImage(const vpImage<Type> &I,
				  const vpRect &rect,
				  vpImage<Type> &S)
{
  double dleft   = rect.getLeft();
  double dtop    = rect.getTop();
  double dright  = ceil( rect.getRight() );
  double dbottom = ceil( rect.getBottom() );

  if (dleft < 0.0)                   dleft = 0.0;
  else if (dleft >= I.getWidth())    dleft = I.getWidth() - 1;

  if (dright < 0.0)                  dright = 0.0;
  else if (dright >= I.getWidth())   dright = I.getWidth() - 1;

  if (dtop < 0.0)                    dtop = 0.0;
  else if (dtop >= I.getHeight())    dtop = I.getHeight() - 1;

  if (dbottom < 0.0)                 dbottom = 0.0;
  else if (dbottom >= I.getHeight()) dbottom = I.getHeight() - 1;

  // Convert the double-precision rectangle coordinates into integer positions
  unsigned int left   = (unsigned int) dleft;
  unsigned int top    = (unsigned int) dtop;
  unsigned int bottom = (unsigned int) dbottom;
  unsigned int right  = (unsigned int) dright;

  unsigned int width  = right - left + 1;;
  unsigned int height = bottom - top + 1;

  S.resize(height, width) ;
  for (unsigned int i=top ; i <= bottom ; i++) {
    for (unsigned int j=left ; j <= right ; j++) {
      S[i-top][j-left] = I[i][j] ;
    }
  }
}
/*!

  Binarise an image.

  - Pixels whose walues are less than \e threshold1 are set to \e value1

  - Pixels whose walues are greater then or equal to \e threshold1 and
    less then or equal to \e threshold2 are set to \e value2

  - Pixels whose walues are greater than \e threshold2 are set to \e value3
  
*/
template<class Type>
void vpImageTools::binarise(vpImage<Type> &I,
			    Type threshold1, Type threshold2, 
			    Type value1, Type value2, Type value3)
{
  unsigned char v;
  unsigned char *p = I.bitmap;
  unsigned char *pend = I.bitmap + I.getWidth()*I.getHeight();
  for (; p < pend; p ++) {
    v = *p;
    if (v < threshold1) *p = value1;
    else if (v > threshold2) *p = value3;
    else *p = value2;
  }

}

/*!
  Undistort an image
  \param I : Input image to undistort.
  \param cam : Parameters of the camera causing distortion.
  \param newI : Undistorted output image.

  \warning This function works only with Types authorizing "+,-,
   multiplication by a scalar" operators.

  \warning This function is time consuming :
    - On "Rhea"(Intel Core 2 Extreme X6800 2.93GHz, 2Go RAM)
      or "Charon"(Intel Xeon 3 GHz, 2Go RAM) : ~16 ms for a 640x480 image.
*/
template<class Type>
void vpImageTools::undistort(const vpImage<Type> &I,
                             const vpCameraParameters &cam,
                             vpImage<Type> &newI)
{
#if 0 //not optimized version
   
  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();
  newI.resize(height,width);
  double u0 = cam.get_u0_mp();
  double v0 = cam.get_v0_mp();
  double px = cam.get_px_mp();
  double py = cam.get_py_mp();
  double kd = cam.get_kd_mp();
  for(unsigned int i = 0;i < I.getHeight(); i++){
    for(unsigned int j = 0 ; j < I.getWidth(); j++){
      double r2 = vpMath::sqr(((double)j - u0)/px) +
      vpMath::sqr(((double)i-v0)/py);
      double u = ((double)j - u0)*(1.0+kd*r2) + u0;
      double v = ((double)i - v0)*(1.0+kd*r2) + v0;
      newI[i][j] = I.getPixelBI((float)u,(float)v);
    }
  }
  
#else //optimized version
   unsigned int width = I.getWidth();
   unsigned int height = I.getHeight();
   if (width != newI.getWidth() || height != newI.getHeight())
     newI.resize(height, width);
   double u0 = cam.get_u0_mp();
   double v0 = cam.get_v0_mp();
   double px = cam.get_px_mp();
   double py = cam.get_py_mp();
   double kd = cam.get_kd_mp();
   Type *dst = newI.bitmap;
   for (unsigned int i = 0;i < height ; i++) {
     for (unsigned int j = 0 ; j < width ; j++) {
       //computation of u,v : corresponding pixel coordinates in I.
       double  deltau  = (double) (j) - u0;
       double  deltav  = (double) (i) - v0;
       double invpx = 1.0/px;
       double invpy = 1.0/py;
       double fr2 = 1.0 + kd * (vpMath::sqr(deltau * invpx) + vpMath::sqr(deltav * invpy));
       double u = deltau * fr2 + u0;
       double v = deltav * fr2 + v0;

       //computation of the bilinear interpolation

       //declarations
       int _u  = (int) (u);
       int _v  = (int) (v);
       if (u < 0.f) _u = -1;
       if (v < 0.f) _v = -1;
       double  _du  = (u) - (double) _u;
       double  _dv  = (v) - (double) _v;
       Type  _v01;
       Type  _v23;
       if ( (0 <= _u) && (0 <= _v) &&
            (_u < ((width) - 1)) && (_v < ((height) - 1)) ) {
         //process interpolation
         const Type* _mp = &I[_v][_u];
         _v01 = (Type)(_mp[0] + (_du * (_mp[1] - _mp[0])));
         _mp += width;
         _v23 = (Type)(_mp[0] + (_du * (_mp[1] - _mp[0])));
         *dst = (Type)(_v01 + (_dv * (_v23 - _v01)));
       }
       else {
         *dst = 0;
       }
       dst++;
     }
   }
#endif  
}

#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
