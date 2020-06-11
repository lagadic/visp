/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * Image morphology.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <visp3/core/vpImageMorphology.h>

#include <Simd/SimdLib.hpp>

/*!
  Erode a grayscale image using the given structuring element.

  The gray-scale erosion of \f$ A \left( x, y \right) \f$ by \f$ B \left (x, y
  \right) \f$ is defined as: \f[ \left ( A \ominus B \right ) \left( x,y
  \right) = \textbf{min} \left \{ A \left ( x+x', y+y' \right ) - B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ + \infty \f$ outside the domain
  of the image.

  In our case, gray-scale erosion is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. Gray-scale erosion using
  such a structuring element is equivalent to a local-minimum operator: \f[
    \left ( A \ominus B \right ) \left( x,y \right) = \textbf{min} \left \{ A
  \left ( x+x', y+y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

  \param I : Image to process.
  \param connexity : Type of connexity: 4 or 8.

  \sa dilatation(vpImage<unsigned char> &, const vpConnexityType &)
*/
void vpImageMorphology::erosion(vpImage<unsigned char> &I, const vpConnexityType &connexity)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 255;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    } else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  SimdImageErosion(I.bitmap, J.bitmap, I.getWidth(), I.getHeight(),
                   connexity == CONNEXITY_4 ? SimdImageConnexity4 : SimdImageConnexity8);
}

/*!
  Dilate a grayscale image using the given structuring element.

  The gray-scale dilatation of \f$ A \left( x, y \right) \f$ by \f$ B \left
  (x, y \right) \f$ is defined as: \f[ \left ( A \oplus B \right ) \left( x,y
  \right) = \textbf{max} \left \{ A \left ( x-x', y-y' \right ) + B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ - \infty \f$ outside the domain
  of the image.

  In our case, gray-scale erosion is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. Gray-scale erosion using
  such a structuring element is equivalent to a local-maximum operator: \f[
    \left ( A \oplus B \right ) \left( x,y \right) = \textbf{max} \left \{ A
  \left ( x-x', y-y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

  \param I : Image to process.
  \param connexity : Type of connexity: 4 or 8.

  \sa erosion(vpImage<unsigned char> &, const vpConnexityType &)
*/
void vpImageMorphology::dilatation(vpImage<unsigned char> &I, const vpConnexityType &connexity)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 0;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    } else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  SimdImageDilatation(I.bitmap, J.bitmap, I.getWidth(), I.getHeight(),
                      connexity == CONNEXITY_4 ? SimdImageConnexity4 : SimdImageConnexity8);
}
