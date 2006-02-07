
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageMorphology.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageMorphology.h,v 1.1 2006-02-07 14:45:28 fspindle Exp $
 *
 * Description
 * ============
 *   Morphology tools
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpImageMorphology_H
#define vpImageMorphology_H

/*!
  \file vpImageMorphology.h
  \brief Various mathematical morphology tools, erosion, dilatation...

*/

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <visp/vpImageException.h>
#include <visp/vpImage.h>
#include <visp/vpMatrix.h>

/*!
  \class vpImageMorphology

  \brief  Various mathematical morphology tools, erosion, dilatation...

  \author Fabien Spindler  (Fabien.Spindler@irisa.fr) Irisa / Inria Rennes


*/
class vpImageMorphology
{
public:
  /*! \enum ConnexityEnum
  Type of connexity 4, or 8.
  */
  typedef enum {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
		   right, up, down) */
    CONNEXITY_8 /*!< For a given pixel 8 neighbors are considered (left,
		  right, up, down, and the 4 pixels located on the diagonal) */
  } ConnexityEnum;

public:
  template<class Type>
  static void erosion(vpImage<Type> &I, Type value, Type value_out,
		      ConnexityEnum connexity = CONNEXITY_4);

  template<class Type>
  static void dilatation(vpImage<Type> &I, Type value, Type value_out,
			 ConnexityEnum connexity = CONNEXITY_4);

} ;

/*!

  Erode a binary image using a structuring element of size one.

  \param value : Values of the pixels to erode.
  \param value_out : Value to set if erosion is done.
  \param connexity : Type of connexity: 4 or 8.

  To erode a black area in an <unsigned char> image, set \e value to
  0 and \e value_out to 255.

  To erode a white area in an <unsigned char> image with one element mask, set
  \e value to 255 and \e value_out to 0.

  \sa dilatation()
*/
template<class Type>
void vpImageMorphology::erosion(vpImage<Type> &I,
				Type value,
				Type value_out,
				ConnexityEnum connexity)
{
  vpImage<Type> J(I.getRows(), I.getCols()) ;
  J = I;

  if (connexity == CONNEXITY_4) {
    for (int i=1 ; i < I.getRows()-1   ; i++)
      for (int j=1 ; j < I.getCols()-1   ; j++)
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
    for (int i=1 ; i < I.getRows()-1   ; i++)
      for (int j=1 ; j < I.getCols()-1   ; j++)
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

  \param value : Values of the pixels to dilate.
  \param value_out : Value to set if dilatation is done.
  \param connexity : Type of connexity: 4 or 8.

  To dilate a black area in an <unsigned char> image with one element mask, set
  \e value to 0 and \e value_out to 255.

  To dilate a white area in an <unsigned char> image with one element mask, set
  \e value to 255 and \e value_out to 0.

  \sa erosion()
*/
template<class Type>
void vpImageMorphology::dilatation(vpImage<Type> &I,
				   Type value,
				   Type value_out,
				   ConnexityEnum connexity)
{
  vpImage<Type> J(I.getRows(), I.getCols()) ;
  J = I;
  if (connexity == CONNEXITY_4) {
    for (int i=1 ; i < I.getRows()-1   ; i++)
      for (int j=1 ; j < I.getCols()-1   ; j++)
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
    for (int i=1 ; i < I.getRows()-1   ; i++)
      for (int j=1 ; j < I.getCols()-1   ; j++)
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
