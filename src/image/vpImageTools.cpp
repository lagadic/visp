/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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

  Change the look up table (LUT) of an image. Considering pixel gray
  level values \f$ l \f$ in the range \f$[A, B]\f$, this method allows
  to rescale these values in \f$[A^*, B^*]\f$ by linear interpolation:

  \f$
  \left\{ \begin{array}{ll}
  l \in ]-\infty, A] \mbox{, } &  l = A^* \\
  l \in  [B, \infty[ \mbox{, } &  l = B^* \\
  l \in ]A, B[ \mbox{, }       &  l = A^* + (l-A) * \frac{B^*-A^*}{B-A}
  \end{array} 
  \right.
  \f$

  \param I : Image to process.
  \param A : Low gray level value of the range to consider.
  \param A_star : New gray level value \f$ A^*\f$ to attribute to pixel 
  who's value was A
  \param B : Height gray level value of the range to consider.
  \param B_star : New gray level value \f$ B^*\f$ to attribute to pixel 
  who's value was B
  \return The modified image.

  \exception vpImageException::incorrectInitializationError If \f$B \leq A\f$.

  As shown in the example below, this method can be used to binarize
  an image. For an unsigned char image (in the range 0-255),
  thresholding this image at level 127 can be done by:

  \code
#include <visp/vpImageTools.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

int main()
{
  vpImage<unsigned char> I;
#ifdef UNIX
  std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
#elif WIN32
  std::string filename("C:/temp/ViSP-images/Klimt/Klimt.pgm");
#endif
  // Read an image from the disk
  vpImageIo::read(I, filename); 

  // Binarize image I:
  // - gray level values less than or equal to 127 are set to 0,
  // - gray level values greater than 128 are set to 255
  vpImageTools::changeLUT(I, 127, 0, 128, 255);
  
  vpImageIo::write(I, "Klimt.pgm"); // Write the image in a PGM P5 image file format 
}
  \endcode

*/
void vpImageTools::changeLUT(vpImage<unsigned char>& I,
			     unsigned char A,
			     unsigned char A_star,
			     unsigned char B,
			     unsigned char B_star)
{
  // Test if input values are valid
  if (B <= A) {
    vpERROR_TRACE("Bad gray levels") ;
    throw (vpImageException(vpImageException::incorrectInitializationError ,
			    "Bad gray levels")) ;
  }
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

/*!
  Compute the signed difference between the two images I1 and I2 for 
  visualization issue : Idiff = I1-I2

  - pixels with a null difference are set to 128. 
  - A negative difference implies a pixel value < 128
  - A positive difference implies a pixel value > 128
  
  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference.
*/
void vpImageTools::imageDifference(vpImage<unsigned char> &I1, 
				   vpImage<unsigned char> &I2,
				   vpImage<unsigned char> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth()))
  {
    throw (vpException(vpException::dimensionError, "The two images have not the same size"));
  }
  int n = I1.getHeight() * I1.getWidth() ;
  int diff ;
  for (int b = 0; b < n ; b++)
    {
      diff = I1.bitmap[b] - I2.bitmap[b] + 128;
      Idiff.bitmap[b] = (unsigned char)
	(vpMath::maximum(vpMath::minimum(diff, 255), 0));
    }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
