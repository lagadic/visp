


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
 *  $Id: vpImageTools.h,v 1.2 2005-09-19 13:38:02 fspindle Exp $
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

  template<class Type>
  static void createSubImage(const vpImage<Type> &I,
			     int i_sub, int j_sub,
			     int nrow_sub, int ncol_sub,
			     vpImage<Type> &SI);


} ;

/*!
  Extract a sub part of an image

  \param I : Input image from which a sub image will be extracted.
  \param i_sub, j_sub : coordinates of the upper left point of the sub image
  \param nrow_sub, ncol_sub : number of row, column of the sub image
  \param SI : new sub-image
*/
template<class Type>
void vpImageTools::createSubImage(const vpImage<Type> &I,
				  int i_sub, int j_sub,
				  int nrow_sub, int ncol_sub,
				  vpImage<Type> &SI)
{
  int i,j ;
  int  imax = i_sub + nrow_sub ;
  int  jmax = j_sub + ncol_sub ;

  if (imax > I.getRows())
  {
    imax = I.getRows() -1 ;
    nrow_sub = imax-i_sub ;
  }
  if (jmax > I.getCols())
  {
    jmax = I.getCols() -1 ;
    ncol_sub = jmax -j_sub ;
  }

  SI.resize(nrow_sub, ncol_sub) ;
  for (i=i_sub ; i < imax ; i++)
    for (j=j_sub ; j < jmax ; j++)
    {
      SI[i-i_sub][j-j_sub] = I[i][j] ;
    }
}



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
