


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageFilter.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageFilter.h,v 1.1 2006-02-06 14:49:41 fspindle Exp $
 *
 * Description
 * ============
 *   various image tools, convolution, etc...
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpImageFilter_H
#define vpImageFilter_H

/*!
  \file vpImageFilter.h
  \brief    various image tools, convolution, etc...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>

/*!
  \class vpImageFilter

  \brief   various image tools, convolution, etc...

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes


*/
class vpImageFilter
{

public:
  static void filter(const vpImage<double> &I,
		     vpImage<double>& Iu,
		     vpImage<double>& Iv,
		     const vpMatrix& M) ;


  static void filter(const vpImage<unsigned char> &I,
		     vpImage<double>& If,
		     const vpMatrix& M) ;
} ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
