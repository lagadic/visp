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
 * Image handling.
 */

#ifndef VP_IMAGE_OPERATOR_H
#define VP_IMAGE_OPERATOR_H

// Warning: this file shouldn't be included by the user. Internal usage only to reduce length of vpImage.h

template <class Type> std::ostream &operator<<(std::ostream &s, const vpImage<Type> &I)
{
  if (I.bitmap == nullptr) {
    return s;
  }

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < (i_width - 1); ++j) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][i_width - 1];

    // We don't add a \n character at the end of the last row line
    if (i < (i_height - 1)) {
      s << std::endl;
    }
  }

  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<unsigned char> &I)
{
  if (I.bitmap == nullptr) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  const unsigned int magic_3 = 3;

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < (i_width - 1); ++j) {
      s << std::setw(magic_3) << static_cast<unsigned>(I[i][j]) << " ";
    }

    // We don't add "  " after the last column element
    s << std::setw(magic_3) << static_cast<unsigned>(I[i][I.getWidth() - 1]);

    // We don't add a \n character at the end of the last row line
    if (i < (i_height - 1)) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<char> &I)
{
  if (I.bitmap == nullptr) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  const unsigned int magic_4 = 4;

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < (i_width - 1); ++j) {
      s << std::setw(magic_4) << static_cast<int>(I[i][j]) << " ";
    }

    // We don't add "  " after the last column element
    s << std::setw(magic_4) << static_cast<int>(I[i][i_width - 1]);

    // We don't add a \n character at the end of the last row line
    if (i < (i_height - 1)) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<float> &I)
{
  if (I.bitmap == nullptr) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  const unsigned int magic_9 = 9;
  s.precision(magic_9); // http://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < (i_width - 1); ++j) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][i_width - 1];

    // We don't add a \n character at the end of the last row line
    if (i < (i_height - 1)) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

inline std::ostream &operator<<(std::ostream &s, const vpImage<double> &I)
{
  if (I.bitmap == nullptr) {
    return s;
  }

  std::ios_base::fmtflags original_flags = s.flags();
  const unsigned int magic_17 = 17;
  s.precision(magic_17); // http://en.cppreference.com/w/cpp/types/numeric_limits/max_digits10

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < (i_width - 1); ++j) {
      s << I[i][j] << " ";
    }

    // We don't add "  " after the last column element
    s << I[i][i_width - 1];

    // We don't add a \n character at the end of the last row line
    if (i < (i_height - 1)) {
      s << std::endl;
    }
  }

  s.flags(original_flags); // restore s to standard state
  return s;
}

/*!
  \brief Copy operator
*/
template <class Type> vpImage<Type> &vpImage<Type>::operator=(vpImage<Type> other)
{
  swap(*this, other);
  if (other.display != nullptr) {
    display = other.display;
  }

  return *this;
}

/*!
  \brief = operator : Set all the element of the bitmap to a given  value \e
  v. \f$ A = v <=> A[i][j] = v \f$

   \warning = must be defined for \f$ <\f$ Type \f$ > \f$
*/
template <class Type> vpImage<Type> &vpImage<Type>::operator=(const Type &v)
{
  for (unsigned int i = 0; i < npixels; ++i) {
    bitmap[i] = v;
  }

  return *this;
}

/*!
  Compare two images.

  \return true if the images are the same, false otherwise.
*/
template <class Type> bool vpImage<Type>::operator==(const vpImage<Type> &I) const
{
  if (this->width != I.getWidth()) {
    return false;
  }
  if (this->height != I.getHeight()) {
    return false;
  }

  /*
  //  printf("wxh: %dx%d bitmap: %p I.bitmap %p\n", width, height, bitmap,
  //  I.bitmap);
  */
  for (unsigned int i = 0; i < npixels; ++i) {
    if (bitmap[i] != I.bitmap[i]) {
      /*
      //      std::cout << "differ for pixel " << i << " (" << i%this->height
      //      << ", " << i - i%this->height << ")" << std::endl;
      */
      return false;
    }
  }
  return true;
}
/*!
  Compare two images.

  \return true if the images are different, false if they are the same.
*/
template <class Type> bool vpImage<Type>::operator!=(const vpImage<Type> &I) const { return !(*this == I); }

/*!
  Operation  A - B (A is unchanged).

  \code
  #include <visp3/core/vpImage.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<unsigned char> A(288, 384);
    vpImage<unsigned char> B(288, 384);
    vpImage<unsigned char> C;

    A = 128;
    B = 120;

    // operator-() : C = A - B
    C = A - B;

    return 0;
  }
  \endcode

  \sa sub(const vpImage<Type> &, const vpImage<Type> &, vpImage<Type> &) to
  avoid matrix allocation for each use.
*/
template <class Type> vpImage<Type> vpImage<Type>::operator-(const vpImage<Type> &B) const
{
  vpImage<Type> C;
  sub(*this, B, C);
  return C;
}

#endif
