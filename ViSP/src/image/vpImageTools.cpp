/****************************************************************************
 *
 * $Id: vpImageTools.cpp,v 1.11 2007-09-14 08:42:47 fspindle Exp $
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

  Change the look up table (LUT) of an image. Considering pixel values
  \f$ v \f$ in the range \f$[A, B]\f$, rescale this values in
  \f$[A^*, B^*]\f$ by linear interpolation:

  \f$
  \left\{ \begin{array}{ll}
  v \in ]-\infty, A] \mbox{, } &  v = A^* \\
  v \in  [B, \infty[ \mbox{, } &  v = B^* \\
  v \in ]A, B[ \mbox{, }       &  v = A^* + (v-A) * \frac{B^*-A^*}{B-A}
  \end{array} 
  \right.
  \f$

  \param I : Image to process.
  \param A : Low value of the range to consider.
  \param A_star : New value \f$ A^*\f$ to attribute to pixel who's value was A
  \param B : Height value of the range to consider.
  \param B_star : New value \f$ B^*\f$ to attribute to pixel who's value was B

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
			     unsigned char A_star,
			     unsigned char B,
			     unsigned char B_star)
{
  unsigned char v;

  double factor = (B_star - A_star)/(B - A);

  for (unsigned int i=0 ; i < I.getHeight(); i++)
    for (unsigned int j=0 ; j < I.getWidth(); j++) {
      v = I[i][j];

      if (v <= A)
	I[i][j] = A_star;
      else if (v >= B)
	I[i][j] = B_star;
      else
	I[i][j] = (unsigned char)(A_star + factor*(v-A));
  }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
