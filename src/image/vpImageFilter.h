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



#ifndef vpImageFilter_H
#define vpImageFilter_H

/*!
  \file vpImageFilter.h
  \brief  Various image filter, convolution, etc...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>

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

  static double gaussianFilter(vpImage<unsigned char> &I, 
			       const unsigned int r, 
			       const unsigned int c) ;

  static double  derivativeFilterX(vpImage<unsigned char> &I,
				   const unsigned int r, unsigned int c) ;

  static double  derivativeFilterY(vpImage<unsigned char> &I,
				   const unsigned int r, unsigned int c) ;

  static void  coefficientGaussianDerivative(double *fg, const int t) ;


  static double derivativeFilterX(vpImage<unsigned char> &I, 
				  const unsigned int r, const unsigned int c, 
				  double *filter, const unsigned int size)  ;
  
  static double  derivativeFilterY(vpImage<unsigned char> &I, 
				   const unsigned int r, const unsigned int c, 
				   double *filter, const unsigned int size) ;
} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
