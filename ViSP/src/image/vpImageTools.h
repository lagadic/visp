/****************************************************************************
 *
 * $Id: vpImageTools.h,v 1.5 2006-05-30 08:40:43 fspindle Exp $
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
  \brief Various image tools...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpConfig.h>
#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>

/*!
  \class vpImageTools

  \brief  Various image tools...

  \author Fabien Spindler  (Fabien.Spindler@irisa.fr) Irisa / Inria Rennes


*/
class VISP_EXPORT vpImageTools
{

public:
  template<class Type>
  static void createSubImage(const vpImage<Type> &I,
			     int i_sub, int j_sub,
			     int nrow_sub, int ncol_sub,
			     vpImage<Type> &SI);

  static void changeLUT(vpImage<unsigned char>& I,
			unsigned char A,
			unsigned char newA,
			unsigned char B,
			unsigned char newB);

} ;

/*
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
