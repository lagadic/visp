


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageTools.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageTools.h,v 1.1 2005-09-02 14:03:18 marchand Exp $
 *
 * Description
 * ============
 *   various image tools, convolution, etc...
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpImageTools_H
#define vpImageTools_H

/*!
  \file vpImageTools.h
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
  \class vpImageTools

  \brief   various image tools, convolution, etc...

  \author Eric Marchand  (Eric.Marchand@irisa.fr) Irisa / Inria Rennes


*/
class vpImageTools
{

public:
  static  void   filter(const vpImage<double> &I,
			vpImage<double>& Ix,
			vpImage<double>& Iy,
			const vpMatrix& M) ;


  static void   filter(const vpImage<unsigned char> &I,
		       vpImage<double>& If,
		       const vpMatrix& M) ;

} ;


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
