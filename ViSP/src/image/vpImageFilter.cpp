/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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



