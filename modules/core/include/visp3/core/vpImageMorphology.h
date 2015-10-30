/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Morphology tools.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpImageMorphology_H
#define vpImageMorphology_H

/*!
  \file vpImageMorphology.h
  \brief Various mathematical morphology tools, erosion, dilatation...

*/
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMatrix.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

/*!
  \class vpImageMorphology

  \ingroup group_core_image

  \brief  Various mathematical morphology tools, erosion, dilatation...

  \author Fabien Spindler  (Fabien.Spindler@irisa.fr) Irisa / Inria Rennes


*/
class vpImageMorphology
{
public:
  /*! \enum vpConnexityType
  Type of connexity 4, or 8.
  */
  typedef enum {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
		   right, up, down) */
    CONNEXITY_8 /*!< For a given pixel 8 neighbors are considered (left,
		  right, up, down, and the 4 pixels located on the diagonal) */
  } vpConnexityType;

public:
  template<class Type>
  static void erosion(vpImage<Type> &I, Type value, Type value_out,
		      vpConnexityType connexity = CONNEXITY_4);

  template<class Type>
  static void dilatation(vpImage<Type> &I, Type value, Type value_out,
			 vpConnexityType connexity = CONNEXITY_4);

} ;

/*!

  Erode a binary image using a structuring element of size one.

  \param I : Image to process.
  \param value : Values of the pixels to erode.
  \param value_out : Value to set if erosion is done.
  \param connexity : Type of connexity: 4 or 8.

  To erode a black area in an unsigned char image, set \e value to
  0 and \e value_out to 255.

  To erode a white area in an unsigned char image with one element mask, set
  \e value to 255 and \e value_out to 0.

  \sa dilatation()
*/
template<class Type>
void vpImageMorphology::erosion(vpImage<Type> &I,
				Type value,
				Type value_out,
				vpConnexityType connexity)
{
  vpImage<Type> J(I.getHeight(), I.getWidth()) ;
  J = I;

  if (connexity == CONNEXITY_4) {
    for (unsigned int i=1 ; i < I.getHeight()-1   ; i++)
      for (unsigned int j=1 ; j < I.getWidth()-1   ; j++)
      {
	if (I[i][j] == value)
	{
	  // Consider 4 neighbors
	  if ((I[i-1][j] == value_out) ||
	      (I[i+1][j] == value_out) ||
	      (I[i][j-1] == value_out) ||
	      (I[i][j+1] == value_out))
	    J[i][j] = value_out;
	}
      }
  }
  else {
    for (unsigned int i=1 ; i < I.getHeight()-1   ; i++)
      for (unsigned int j=1 ; j < I.getWidth()-1   ; j++)
      {
	if (I[i][j] == value)
	{
	  // Consider 8 neighbors
	  if ((I[i-1][j-1] == value_out) ||
	      (I[i-1][j]   == value_out) ||
	      (I[i-1][j+1] == value_out) ||
	      (I[i][j-1]   == value_out) ||
	      (I[i][j+1]   == value_out) ||
	      (I[i+1][j+1] == value_out) ||
	      (I[i+1][j+1] == value_out) ||
	      (I[i+1][j+1] == value_out) )
	    J[i][j] = value_out ;
	}
      }


  }
  I = J ;
}

/*!

  Dilate a binary image using a structuring element of size one.

  \param I : Image to process.
  \param value : Values of the pixels to dilate.
  \param value_out : Value to set if dilatation is done.
  \param connexity : Type of connexity: 4 or 8.

  To dilate a black area in an unsigned char image with one element mask, set
  \e value to 0 and \e value_out to 255.

  To dilate a white area in an unsigned char image with one element mask, set
  \e value to 255 and \e value_out to 0.

  \sa erosion()
*/
template<class Type>
void vpImageMorphology::dilatation(vpImage<Type> &I,
				   Type value,
				   Type value_out,
				   vpConnexityType connexity)
{
  vpImage<Type> J(I.getHeight(), I.getWidth()) ;
  J = I;
  if (connexity == CONNEXITY_4) {
    for (unsigned int i=1 ; i < I.getHeight()-1   ; i++)
      for (unsigned int j=1 ; j < I.getWidth()-1   ; j++)
      {
	if (I[i][j] == value_out)
	{
	  // Consider 4 neighbors
	  if ((I[i-1][j] == value) ||
	      (I[i+1][j] == value) ||
	      (I[i][j-1] == value) ||
	      (I[i][j+1] == value))
	    J[i][j] = value ;
	}
      }
  }
  else {
    for (unsigned int i=1 ; i < I.getHeight()-1   ; i++)
      for (unsigned int j=1 ; j < I.getWidth()-1   ; j++)
      {
	if (I[i][j] == value_out)
	{
	  // Consider 8 neighbors
	  if ((I[i-1][j-1] == value) ||
	      (I[i-1][j]   == value) ||
	      (I[i-1][j+1] == value) ||
	      (I[i][j-1]   == value) ||
	      (I[i][j+1]   == value) ||
	      (I[i+1][j+1] == value) ||
	      (I[i+1][j+1] == value) ||
	      (I[i+1][j+1] == value) )
	    J[i][j] = value ;
	}
      }
  }

  I = J ;
}
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
