/****************************************************************************
 *
 * $Id: vpImageTools.cpp,v 1.7 2007-02-27 08:42:27 fspindle Exp $
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

#include <visp/vpImageTools.h>


/*!

  Change the look up table (LUT) of an image. Considering image values
   v in the range [A, B], rescale this values in [newA, newB] by linear
  interpolation.

  - if v \f$ \in \f$ ]-inf, A], set v to newA
  - else if v \f$ \in \f$ [B, inf[ set v to newB
  - else set v to newA + (newB-newA)/(B-A)*(v-A)

  \param I : Image to process.
  \param A : Low value of the range to consider.
  \param newA : New value to attribute to pixel who's value was A
  \param B : Height value of the range to consider.
  \param newB : New value to attribute to pixel who's value was B

  This method can be used to binarize an image. For an unsigned char image
  (in the range 0-255), thresholding this image at level 128 can be done by:

  \code
  // Binarize image I:
  // - values less than or equal to 128 are set to 0,
  // - values greater than 128 are set to 255
  vpImageTools::changeLUT(I, 128, 0, 128, 255);
  \endcode

*/
void vpImageTools::changeLUT(vpImage<unsigned char>& I,
			     unsigned char A,
			     unsigned char newA,
			     unsigned char B,
			     unsigned char newB)
{
  unsigned char v;
  double _A = A;
  double _B = B;
  double _newA = newA;
  double _newB = newB;

  double factor = (_newB-_newA)/(_B-_A);

  for (int i=0 ; i < I.getRows(); i++)
    for (int j=0 ; j < I.getCols(); j++) {
      v = I[i][j];

      if (v <= A)
	I[i][j] = newA;
      else if (v >= B)
	I[i][j] = newB;
      else
	I[i][j] = (unsigned char)(newA + factor*(v-A));
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
