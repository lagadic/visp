/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
class VISP_EXPORT vpImageMorphology
{
public:
  /*! \enum vpConnexityType
  Type of connexity 4, or 8.
  */
  typedef enum {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
                      right, up, down) */
    CONNEXITY_8  /*!< For a given pixel 8 neighbors are considered (left,
                      right, up, down, and the 4 pixels located on the
                    diagonal) */
  } vpConnexityType;

public:
  template <class Type>
  static void erosion(vpImage<Type> &I, Type value, Type value_out, vpConnexityType connexity = CONNEXITY_4);

  template <class Type>
  static void dilatation(vpImage<Type> &I, Type value, Type value_out, vpConnexityType connexity = CONNEXITY_4);

  static void erosion(vpImage<unsigned char> &I, const vpConnexityType &connexity = CONNEXITY_4);
  static void dilatation(vpImage<unsigned char> &I, const vpConnexityType &connexity = CONNEXITY_4);
};

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
template <class Type>
void vpImageMorphology::erosion(vpImage<Type> &I, Type value, Type value_out, vpConnexityType connexity)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  vpImage<Type> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = value;
      }
    } else {
      J[i][0] = value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = value;
    }
  }

  if (connexity == CONNEXITY_4) {
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (J[i + 1][j + 1] == value) {
          // Consider 4 neighbors
          if ((J[i][j + 1] == value_out) ||     // Top
              (J[i + 2][j + 1] == value_out) || // Bottom
              (J[i + 1][j] == value_out) ||     // Left
              (J[i + 1][j + 2] == value_out)) { // Right
            I[i][j] = value_out;
          }
        }
      }
    }
  } else {
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (J[i + 1][j + 1] == value) {
          // Consider 8 neighbors
          if ((J[i][j] == value_out) || (J[i][j + 1] == value_out) || (J[i][j + 2] == value_out) ||
              (J[i + 1][j] == value_out) || (J[i + 1][j + 2] == value_out) || (J[i + 2][j] == value_out) ||
              (J[i + 2][j + 1] == value_out) || (J[i + 2][j + 2] == value_out))
            I[i][j] = value_out;
        }
      }
    }
  }
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
template <class Type>
void vpImageMorphology::dilatation(vpImage<Type> &I, Type value, Type value_out, vpConnexityType connexity)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  vpImage<Type> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = value_out;
      }
    } else {
      J[i][0] = value_out;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = value_out;
    }
  }

  if (connexity == CONNEXITY_4) {
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (J[i + 1][j + 1] == value_out) {
          // Consider 4 neighbors
          if ((J[i][j + 1] == value) ||     // Top
              (J[i + 2][j + 1] == value) || // Bottom
              (J[i + 1][j] == value) ||     // Left
              (J[i + 1][j + 2] == value)) { // Right
            I[i][j] = value;
          }
        }
      }
    }
  } else {
    for (unsigned int i = 0; i < I.getHeight(); i++) {
      for (unsigned int j = 0; j < I.getWidth(); j++) {
        if (J[i + 1][j + 1] == value_out) {
          // Consider 8 neighbors
          if ((J[i][j] == value) || (J[i][j + 1] == value) || (J[i][j + 2] == value) || (J[i + 1][j] == value) ||
              (J[i + 1][j + 2] == value) || (J[i + 2][j] == value) || (J[i + 2][j + 1] == value) ||
              (J[i + 2][j + 2] == value)) {
            I[i][j] = value;
          }
        }
      }
    }
  }
}
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
