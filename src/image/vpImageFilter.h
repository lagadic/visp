/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Various image tools, convolution, ...
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpImageFilter_H
#define vpImageFilter_H

/*!
  \file vpImageFilter.h
  \brief  Various image filter, convolution, etc...

*/

#include <visp/vpImage.h>
#include <visp/vpImageException.h>
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

/*!
  \class vpImageFilter

  \ingroup ImageFiltering

  \brief  Various image filter, convolution, etc...

*/
class VISP_EXPORT vpImageFilter
{

public:
  static void filter(const vpImage<double> &I,
		     vpImage<double>& Iu,
		     vpImage<double>& Iv,
		     const vpMatrix& M) ;


  static void filter(const vpImage<unsigned char> &I,
		     vpImage<double>& If,
		     const vpMatrix& M) ;

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  static void canny(const vpImage<unsigned char>& I,
                    vpImage<unsigned char>& Ic,
                    const unsigned int gaussianFilterSize,
                    const double thresholdCanny,
                    const unsigned int apertureSobel);
#endif

  /*!
   Apply a 5x5 Gaussian filter to an image pixel.

   \param fr : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template<class T>
  static double
  gaussianFilter(vpImage<T> & fr,
                                const unsigned int r, const unsigned int c)
  {
    //filter Gaussien
    return (
                                          15.0 * fr[r][c]
                                          + 12.0 * ( fr[r-1][c]  + fr[r][c-1]  + fr[r+1][c]   + fr[r][c+1]   )
                                          + 9.0  * ( fr[r-1][c-1] + fr[r+1][c-1] + fr[r-1][c+1] + fr[r+1][c+1])
                                          + 5.0  * ( fr[r-2][c]   + fr[r][c-2]   + fr[r+2][c]   + fr[r][c+2] )
                                          + 4.0  * ( fr[r-2][c+1] + fr[r-2][c-1] + fr[r-1][c-2] + fr[r+1][c-2] +
                                                                                  fr[r+2][c-1] + fr[r+2][c+1] + fr[r-1][c+2] + fr[r+1][c+2] )
                                          + 2.0  * ( fr[r-2][c-2] + fr[r+2][c-2] + fr[r-2][c+2] + fr[r+2][c+2] )
                                          )
          /159.0;
  }



  /*!
   Apply a 1x3 Derivative Filter to an image pixel.

   \param fr : Image to filter
   \param r: coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template<class T>
  static double
  derivativeFilterX(vpImage<T> & fr,
                                   const unsigned int r, const unsigned int c)
  {
    return (2047.0 *(fr[r][c+1] - fr[r][c-1])
            +913.0 *(fr[r][c+2] - fr[r][c-2])
            +112.0 *(fr[r][c+3] - fr[r][c-3]))/8418.0;
  }

  /*!
   Apply a 3x1 Derivative Filter to an image pixel.

   \param fr : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   */
  template<class T>
  static double
  derivativeFilterY(vpImage<T> & fr,
                                   const unsigned int r, const unsigned int c)
  {
    return (2047.0 *(fr[r+1][c] - fr[r-1][c])
            +913.0 *(fr[r+2][c] - fr[r-2][c])
            +112.0 *(fr[r+3][c] - fr[r-3][c]))/8418.0;
  }

  /*!
   build a Gaussian Derivative filter

          \param filter : array (of size t/2) that contains the filter
          \param t : size of the filter

   \warning filter has to be deallocated
   */
  static void
  coefficientGaussianDerivative(double *filter, const unsigned int t)
  {
    unsigned int i;
    //  double sigma;
          if (filter == NULL)
                  filter = new double[t/2] ;

    double s2 = vpMath::sqr((t-1)/6.0);

    for(i=1; i<=(t-1)/2; i++)
    {
      filter[i] = (i/(s2*sqrt(2*M_PI)))*exp((i*i)/(-2*s2));

    }

  }


  /*!
   Apply a 1 x size Derivative Filter in X to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using vpImageFilter::coefficientGaussianDerivative().
   \param size : size of the filter

   \sa vpImageFilter::coefficientGaussianDerivative()
   */

  template<class T>
  static double
  derivativeFilterX(vpImage<T> &I,
                                   const unsigned int r, const unsigned int c,
                                   double *filter, const unsigned int size)
  {
          unsigned int i;
          double result;

          result = 0;

          for(i=1; i<=(size-1)/2; i++)
          {
                  result += filter[i]*(I[r][c+i] - I[r][c-i]) ;
          }
          return result;
  }



  /*!
   Apply a size x 1 Derivative Filter in Y to an image pixel.

   \param I : Image to filter
   \param r : coordinates (row) of the pixel
   \param c : coordinates (column) of the pixel
   \param filter : coefficients of the filter to be initialized using vpImageFilter::coefficientGaussianDerivative().
   \param size : size of the filter

  \sa vpImageFilter::coefficientGaussianDerivative()
   */
  template<class T>
  static double
  derivativeFilterY(vpImage<T> &I,
                                   const unsigned int r, const unsigned int c,
                                   double *filter, const unsigned int size)
  {
          unsigned int i;
          double result;

          result = 0;

          for(i=1; i<=(size-1)/2; i++)
          {
                  result += filter[i]*(I[r+i][c] - I[r-i][c]) ;
          }
          return result;
  }
} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
