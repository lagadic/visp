/****************************************************************************
 *
 * $Id: vpImageTools.h,v 1.8 2007-08-17 15:10:38 fspindle Exp $
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
#include <visp/vpRect.h>

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
			     unsigned int i_sub, unsigned int j_sub,
			     unsigned int nrow_sub, unsigned int ncol_sub,
			     vpImage<Type> &S);

  template<class Type>
  static void createSubImage(const vpImage<Type> &I,
			     const vpRect &rect,
			     vpImage<Type> &S);
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
/*
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



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
