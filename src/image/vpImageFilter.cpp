/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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

#include <visp/vpImageFilter.h>

/*!
  Apply a filter to an image.

  \param I : Image to filter
  \param If : Filtered image.
  \param M : Filter coefficients.

*/
void
vpImageFilter::filter(const vpImage<unsigned char> &I,
		      vpImage<double>& If,
		      const vpMatrix& M)
{

  unsigned int size = M.getRows() ;
  unsigned int half_size = size/2 ;

  If.resize(I.getHeight(),I.getWidth()) ;

  If = 0 ;

  for (unsigned int i=half_size ; i < I.getHeight()-half_size ; i++)
  {
    for (unsigned int j=half_size ; j < I.getWidth()-half_size ; j++)
    {
      double   conv_x = 0 ;

      for(unsigned int a = 0 ; a < size ; a++ )
        for(unsigned int b = 0 ; b < size ; b++ )
	{
	  double val =  I[i-half_size+a][j-half_size+b] ;
	  conv_x += M[a][b] * val ;
	}
      If[i][j] = conv_x ;
    }
  }

}

/*!
  Apply a filter to an image.

  \param I : Image to filter
  \param Iu : Filtered image along the horizontal axis (u = columns).
  \param Iv : Filtered image along the vertical axis (v = rows).
  \param M : Separate filter coefficients

*/
void
vpImageFilter::filter(const vpImage<double> &I,
		      vpImage<double>& Iu,
		      vpImage<double>& Iv,
		      const vpMatrix& M)
{

  unsigned int size = M.getRows() ;
  unsigned int half_size = size/2 ;

  Iu.resize(I.getHeight(),I.getWidth()) ;
  Iv.resize(I.getHeight(),I.getWidth()) ;

  Iu = 0 ;
  Iv = 0 ;
  for (unsigned int v=half_size ; v < I.getHeight()-half_size ; v++)
  {
    for (unsigned int u=half_size ; u < I.getWidth()-half_size ; u++)
    {
      double   conv_u = 0 ;
      double   conv_v = 0 ;

      for(unsigned int a = 0 ; a < size ; a++ )
        for(unsigned int b = 0 ; b < size ; b++ )
	{
	  double val =  I[v-half_size+a][u-half_size+b] ;
	  conv_u += M[a][b] * val ;
	  conv_v += M[b][a] * val  ;
	}
      Iu[v][u] = conv_u ;
      Iv[v][u] = conv_v ;
    }
  }

}


/*!
 Apply a 5x5 Gaussian filter to an image pixel.
 
 \param fr : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 */

double 
vpImageFilter::gaussianFilter(vpImage<unsigned char> & fr, 
															const int r, const int c)
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

double 
vpImageFilter::derivativeFilterX(vpImage<unsigned char> & fr, const int r, const int c)
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

double 
vpImageFilter::derivativeFilterY(vpImage<unsigned char> & fr, const int r, const int c)
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

void 
vpImageFilter::coefficientGaussianDerivative(double *filter, const int t)
{ 
  int i;
  //  double sigma;
	if (filter == NULL)
		filter = new double[t/2] ;
	
  double s2 = vpMath::sqr((t-1)/6.0);
	
  for(i=1; i<=(t-1)/2; i++)
  {
    filter[i] = (i/(s2*sqrt(2*M_PI)))*exp(-(i*i)/(2*s2));
		
  }
	
}


/*!
 Apply a 1 x size Derivative Filter in X to an image pixel.
 
 \param I : Image to filter
 \param r : coordinates (row) of the pixel
 \param c : coordinates (column) of the pixel
 \param filter : coefficients of the filter
 (to be initialized using vpImageFilter::coefficientGaussianDerivative)
 \param size : size of the filter
 
 \sa vpImageFilter::coefficientGaussianDerivative
 */

double 
vpImageFilter::derivativeFilterX(vpImage<unsigned char> &I, 
							 const int r, const int c, 
							 double *filter, const int size)
{
	int i;
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
 \param filter : coefficients of the filter
   (to be initialized using vpImageFilter::coefficientGaussianDerivative)
 \param size : size of the filter
 
\sa vpImageFilter::coefficientGaussianDerivative
 */

double  
vpImageFilter::derivativeFilterY(vpImage<unsigned char> &I, 
							 const int r, const int c, 
							 double *filter, const int size)
{
	int i;
	double result;
	
	result = 0;
	
	for(i=1; i<=(size-1)/2; i++)
	{
		result += filter[i]*(I[r+i][c] - I[r-i][c]) ;
	}
	return result;
}

