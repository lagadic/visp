/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \file vpImageMorphology.h
  \brief Various mathematical morphology tools, erosion, dilatation...

*/

#ifndef VP_IMAGE_MORPHOLOGY_H
#define VP_IMAGE_MORPHOLOGY_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageException.h>
#include <visp3/core/vpMatrix.h>

#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

BEGIN_VISP_NAMESPACE
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
  typedef enum
  {
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

  template <typename T>
  static void erosion(vpImage<T> &I, const vpConnexityType &connexity = CONNEXITY_4);

  template <typename T>
  static void dilatation(vpImage<T> &I, const vpConnexityType &connexity = CONNEXITY_4);

  template <typename T>
  static void erosion(vpImage<T> &I, const int &size);

  template <typename T>
  static void dilatation(vpImage<T> &I, const int &size);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
    \brief  An erosion is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The erosion using
  such a structuring element is equivalent to a local-minimum operator: \f[
    \left ( A \ominus B \right ) \left( x,y \right) = \textbf{min} \left \{ A
  \left ( x+x', y+y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]
    \deprecated Provided only for compat with previous releases. Use rather the template function erosion(vpImage<T> &, const vpConnexityType &)
    \param I : Gray-scale image to process.
    \param connexity : Type of connexity: 4 or 8.
   */
  VP_DEPRECATED static void erosion(vpImage<unsigned char> &I, const vpConnexityType &connexity = CONNEXITY_4)
  {
    vpImageMorphology::erosion<unsigned char>(I, connexity);
  }

  /*!
    \brief A dilatation is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The erosion using
  such a structuring element is equivalent to a local-maximum operator: \f[
    \left ( A \ominus B \right ) \left( x,y \right) = \textbf{max} \left \{ A
  \left ( x+x', y+y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

    \deprecated Provided only for compat with previous releases. Use rather the template function dilatation(vpImage<T> &, const vpConnexityType &)
    \param I : Gray-scale image to process.
    \param connexity : Type of connexity: 4 or 8.
   */
  VP_DEPRECATED static void dilatation(vpImage<unsigned char> &I, const vpConnexityType &connexity = CONNEXITY_4)
  {
    vpImageMorphology::dilatation<unsigned char>(I, connexity);
  }
  //@}
#endif

private:
  /**
   * \brief Modify the image by applying the \b operation on each of its elements on a 3x3
   * grid.
   *
   * \tparam T Either a class such as vpRGBa or a type such as double, unsigned char ...
   * \param[out] I The image we want to modify.
   * \param[in] null_value The value that is padded to the input image to manage the borders.
   * \param[in] operation The operation to apply to its elements on a 3x3 grid.
   * \param[in] connexity Either a 4-connexity, if we want to take into account only the horizontal
   * and vertical neighbors, or a 8-connexity, if we want to also take into account the diagonal neighbors.
   */
  template <typename T>
  static void imageOperation(vpImage<T> &I, const T &null_value, const T &(*operation)(const T &, const T &), const vpConnexityType &connexity = CONNEXITY_4);

  /**
   * \brief Modify the image by applying the \b operation on each of its elements on a \b size x \b size
   * grid.
   *
   * \tparam T Any type such as double, unsigned char ...
   * \param[out] I The image we want to modify.
   * \param[in] operation The operation to apply to its elements on a the grid.
   * \param[in] size Size of the kernel of the operation.
   */
  template <typename T>
  static void imageOperation(vpImage<T> &I, const T &(*operation)(const T &, const T &), const int &size = 3);

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
  unsigned int j_height = J.getHeight();
  unsigned int j_width = J.getWidth();
  for (unsigned int i = 0; i < j_height; ++i) {
    if ((i == 0) || (i == (j_height - 1))) {
      for (unsigned int j = 0; j < j_width; ++j) {
        J[i][j] = value;
      }
    }
    else {
      J[i][0] = value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = value;
    }
  }

  if (connexity == CONNEXITY_4) {
    unsigned int i_height = I.getHeight();
    unsigned int i_width = I.getWidth();
    for (unsigned int i = 0; i < i_height; ++i) {
      for (unsigned int j = 0; j < i_width; ++j) {
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
  }
  else {
    unsigned int i_height = I.getHeight();
    unsigned int i_width = I.getWidth();
    for (unsigned int i = 0; i < i_height; ++i) {
      for (unsigned int j = 0; j < i_width; ++j) {
        if (J[i + 1][j + 1] == value) {
          // Consider 8 neighbors
          bool cond4firstneighbors = (J[i][j] == value_out) || (J[i][j + 1] == value_out) ||
            (J[i][j + 2] == value_out) || (J[i + 1][j] == value_out);
          bool cond4secondneighbors = (J[i + 1][j + 2] == value_out) || (J[i + 2][j] == value_out) ||
            (J[i + 2][j + 1] == value_out) || (J[i + 2][j + 2] == value_out);
          if (cond4firstneighbors || cond4secondneighbors) {
            I[i][j] = value_out;
          }
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
  unsigned int j_height = J.getHeight();
  unsigned int j_width = J.getWidth();
  for (unsigned int i = 0; i < j_height; ++i) {
    if ((i == 0) || (i == (j_height - 1))) {
      for (unsigned int j = 0; j < j_width; ++j) {
        J[i][j] = value_out;
      }
    }
    else {
      J[i][0] = value_out;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = value_out;
    }
  }

  if (connexity == CONNEXITY_4) {
    unsigned int i_height = I.getHeight();
    unsigned int i_width = I.getWidth();
    for (unsigned int i = 0; i < i_height; ++i) {
      for (unsigned int j = 0; j < i_width; ++j) {
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
  }
  else {
    unsigned int i_height = I.getHeight();
    unsigned int i_width = I.getWidth();
    for (unsigned int i = 0; i < i_height; ++i) {
      for (unsigned int j = 0; j < i_width; ++j) {
        if (J[i + 1][j + 1] == value_out) {
          // Consider 8 neighbors
          bool cond4firstneighbors = (J[i][j] == value) || (J[i][j + 1] == value) || (J[i][j + 2] == value) || (J[i + 1][j] == value);
          bool cond4secondneighbors = (J[i + 1][j + 2] == value) || (J[i + 2][j] == value) || (J[i + 2][j + 1] == value) ||
            (J[i + 2][j + 2] == value);
          if (cond4firstneighbors || cond4secondneighbors) {
            I[i][j] = value;
          }
        }
      }
    }
  }
}

template<typename T>
void vpImageMorphology::imageOperation(vpImage<T> &I, const T &null_value, const T &(*operation)(const T &, const T &), const vpConnexityType &connexity)
{
  const int width_in = I.getWidth();
  const int height_in = I.getHeight();
  const int width_dilat = width_in + 2;
  const int height_dilat = height_in + 2;
  vpImage<T> J(height_dilat, width_dilat, null_value);

  // Copy I to J and add border
  J.insert(I, vpImagePoint(1, 1));

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    const int nbOffset = 5;
    int offset_x[nbOffset] = { 0, -1, 0, 1, 0 };
    int offset_y[nbOffset] = { -1,  0, 0, 0, 1 };

    for (int i = 0; i < height_in; ++i) {
      for (int j = 0; j < width_in; ++j) {
        T value = null_value;
        for (int k = 0; k < nbOffset; ++k) {
          value = operation(value, J[i + 1 + offset_y[k]][j + 1 + offset_x[k]]);
        }

        I[i][j] = value;
      }
    }
  }
  else {
    const int nbOffset = 9;
    int offset_x[nbOffset] = { -1, 0, 1,-1, 0, 1,-1, 0, 1 };
    int offset_y[nbOffset] = { -1,-1,-1, 0, 0, 0, 1, 1, 1 };

    for (int i = 0; i < height_in; ++i) {
      for (int j = 0; j < width_in; ++j) {
        T value = null_value;
        for (int k = 0; k < nbOffset; ++k) {
          value = operation(value, J[i + 1 + offset_y[k]][j + 1 + offset_x[k]]);
        }

        I[i][j] = value;
      }
    }
  }
}

/*!
  Erode an image using the given structuring element.

  The erosion of \f$ A \left( x, y \right) \f$ by \f$ B \left (x, y
  \right) \f$ is defined as: \f[ \left ( A \ominus B \right ) \left( x,y
  \right) = \textbf{min} \left \{ A \left ( x+x', y+y' \right ) - B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ + \infty \f$ outside the domain
  of the image.

  In our case, the erosion is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The erosion using
  such a structuring element is equivalent to a local-minimum operator: \f[
    \left ( A \ominus B \right ) \left( x,y \right) = \textbf{min} \left \{ A
  \left ( x+x', y+y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

  \param I : Image to process.
  \param connexity : Type of connexity: 4 or 8.

  \sa dilatation(vpImage<T> &, const vpConnexityType &)
*/
template <typename T>
void vpImageMorphology::erosion(vpImage<T> &I, const vpConnexityType &connexity)
{
  const T &(*operation)(const T & a, const T & b) = std::min;
  vpImageMorphology::imageOperation(I, std::numeric_limits<T>::max(), operation, connexity);
}

/*!
  Dilate an image using the given structuring element.

  The dilatation of \f$ A \left( x, y \right) \f$ by \f$ B \left
  (x, y \right) \f$ is defined as: \f[ \left ( A \oplus B \right ) \left( x,y
  \right) = \textbf{max} \left \{ A \left ( x-x', y-y' \right ) + B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ - \infty \f$ outside the domain
  of the image.

  In our case, the dilatation is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The dilatation using
  such a structuring element is equivalent to a local-maximum operator: \f[
    \left ( A \oplus B \right ) \left( x,y \right) = \textbf{max} \left \{ A
  \left ( x-x', y-y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

  \param I : Image to process.
  \param connexity : Type of connexity: 4 or 8.

  \sa erosion(vpImage<T> &, const vpConnexityType &)
*/
template <typename T>
void vpImageMorphology::dilatation(vpImage<T> &I, const vpConnexityType &connexity)
{
  const T &(*operation)(const T & a, const T & b) = std::max;
  vpImageMorphology::imageOperation(I, std::numeric_limits<T>::min(), operation, connexity);
}

template<typename T>
void vpImageMorphology::imageOperation(vpImage<T> &I, const T &(*operation)(const T &, const T &), const int &size)
{
  if ((size % 2) != 1) {
    throw(vpException(vpException::badValue, "Dilatation/erosion kernel must be odd."));
  }

  const int width_in = I.getWidth();
  const int height_in = I.getHeight();
  int halfKernelSize = size / 2;
  vpImage<T> J = I;

  for (int r = 0; r < height_in; ++r) {
    // Computing the rows we can explore without going outside the limits of the image
    int r_iterator_start = -halfKernelSize, r_iterator_stop = halfKernelSize + 1;
    if ((r - halfKernelSize) < 0) {
      r_iterator_start = -r;
    }
    else if ((r + halfKernelSize) >= height_in) {
      r_iterator_stop = height_in - r;
    }
    for (int c = 0; c < width_in; ++c) {
      T value = I[r][c];
      // Computing the columns we can explore without going outside the limits of the image
      int c_iterator_start = -halfKernelSize, c_iterator_stop = halfKernelSize + 1;
      if ((c - halfKernelSize) < 0) {
        c_iterator_start = -c;
      }
      else if ((c + halfKernelSize) >= width_in) {
        c_iterator_stop = width_in - c;
      }
      for (int r_iterator = r_iterator_start; r_iterator < r_iterator_stop; ++r_iterator) {
        for (int c_iterator = c_iterator_start; c_iterator < c_iterator_stop; ++c_iterator) {
          value = operation(value, J[r + r_iterator][c + c_iterator]);
        }
      }
      I[r][c] = value;
    }
  }
}

/*!
 * \brief Erosion of \b size >=3 with 8-connectivity.
  Erode an image using the given structuring element.

  The erosion of \f$ A \left( x, y \right) \f$ by \f$ B \left (x, y
  \right) \f$ is defined as: \f[ \left ( A \ominus B \right ) \left( x,y
  \right) = \textbf{min} \left \{ A \left ( x+x', y+y' \right ) - B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ + \infty \f$ outside the domain
  of the image.

  In our case, the erosion is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The erosion using
  such a structuring element is equivalent to a local-minimum operator: \f[
    \left ( A \ominus B \right ) \left( x,y \right) = \textbf{min} \left \{ A
  \left ( x+x', y+y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]

 * \tparam T Any type of image, except vpRGBa .
 * \param[out] I The image to which the erosion must be applied, where the erosion corresponds
 * to a min operator on a window of size \b size.
 * \param[in] size The size of the window on which is performed the min operator for each pixel.

  \sa dilatation(vpImage<T> &, const int &)
*/
template <typename T>
void vpImageMorphology::erosion(vpImage<T> &I, const int &size)
{
  const T &(*operation)(const T & a, const T & b) = std::min;
  vpImageMorphology::imageOperation(I, operation, size);
}

/**
 * \brief Dilatation of \b size >=3 with 8-connectivity.
 *
 * The dilatation of \f$ A \left( x, y \right) \f$ by \f$ B \left
  (x, y \right) \f$ is defined as: \f[ \left ( A \oplus B \right ) \left( x,y
  \right) = \textbf{max} \left \{ A \left ( x-x', y-y' \right ) + B \left (
  x', y'\right ) | \left ( x', y'\right ) \subseteq D_B \right \} \f] where
  \f$ D_B \f$ is the domain of the structuring element \f$ B \f$ and \f$ A
  \left( x,y \right) \f$ is assumed to be \f$ - \infty \f$ outside the domain
  of the image.

  In our case, the dilatation is performed with a flat structuring element
  \f$ \left( B \left( x,y \right) = 0 \right) \f$. The dilatation using
  such a structuring element is equivalent to a local-maximum operator: \f[
    \left ( A \oplus B \right ) \left( x,y \right) = \textbf{max} \left \{ A
  \left ( x-x', y-y' \right ) | \left ( x', y'\right ) \subseteq D_B \right \}
  \f]
 *
 * \tparam T Any type of image, except vpRGBa .
 * \param[out] I The image to which the dilatation must be applied, where the dilatation corresponds
 * to a max operator on a window of size \b size.
 * \param[in] size The size of the window on which is performed the max operator for each pixel.
 *
 * \sa erosion(vpImage<T> &, const int &)
 */
template<typename T>
void vpImageMorphology::dilatation(vpImage<T> &I, const int &size)
{
  const T &(*operation)(const T & a, const T & b) = std::max;
  vpImageMorphology::imageOperation(I, operation, size);
}
END_VISP_NAMESPACE
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
