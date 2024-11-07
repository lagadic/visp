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
 * RGBA pixel.
 */

/*!
  \file vpRGBa.cpp
  \brief Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used - yet -)
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpRGBa.h>

BEGIN_VISP_NAMESPACE
/*!
  Copy operator that initializes all the components to `v`.

  \param v : Value used to initialize the object ( R = G = B = v ).
*/
vpRGBa &vpRGBa::operator=(const unsigned char &v)
{
  this->R = v;
  this->G = v;
  this->B = v;
  this->A = v;
  return *this;
}

/*!
  Copy operator that initializes all the components to `v`.

  \param v : Value used to initialize the object ( R = G = B = v ).
*/
vpRGBa &vpRGBa::operator=(const unsigned int &v)
{
  assert(v < 256);
  this->R = v;
  this->G = v;
  this->B = v;
  this->A = v;
  return *this;
}

/*!
  Copy operator that initializes all the components to `v`.

  \param v : Value used to initialize the object ( R = G = B = v ).
*/
vpRGBa &vpRGBa::operator=(const int &v)
{
  assert(v < 256);
  this->R = v;
  this->G = v;
  this->B = v;
  this->A = v;
  return *this;
}

/*!
  Copy operator.
*/
vpRGBa &vpRGBa::operator=(const vpRGBa &v)
{
  this->R = v.R;
  this->G = v.G;
  this->B = v.B;
  this->A = v.A;
  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Move operator.
*/
vpRGBa &vpRGBa::operator=(const vpRGBa &&v)
{
  this->R = std::move(v.R);
  this->G = std::move(v.G);
  this->B = std::move(v.B);
  this->A = std::move(v.A);
  return *this;
}
#endif

/*!
  Cast a vpColVector in a vpRGBa

  \param v : Input vector. v[0], v[1], v[2], v[3] are to make into
  relation with respectively R, G, B and A.

  \exception vpException::dimensionError : If v is not a 4 four
  dimension vector.
*/
vpRGBa &vpRGBa::operator=(const vpColVector &v)
{
  if (v.getRows() != 4) {
    throw(vpException(vpException::dimensionError, "Bad vector dimension "));
  }
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  R = static_cast<unsigned char>(v[index_0]);
  G = static_cast<unsigned char>(v[index_1]);
  B = static_cast<unsigned char>(v[index_2]);
  A = static_cast<unsigned char>(v[index_3]);
  return *this;
}

/*!
  Compare two RGBa values.

  \return true if the values are the same, false otherwise.
*/
bool vpRGBa::operator==(const vpRGBa &v) const
{
  return (R == v.R) && (G == v.G) && (B == v.B) && (A == v.A);
}
/*!
  Compare two color pixels.

  \return true if the images are different, false if they are the same.
*/
bool vpRGBa::operator!=(const vpRGBa &v) const { return ((R != v.R) || (G != v.G) || (B != v.B) || (A != v.A)); }

/*!
  subtraction operator : "this" - v.
  \param v : Color to subtract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBa::operator-(const vpRGBa &v) const
{
  vpColVector n(4); // new color
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  n[index_0] = static_cast<double>(R) - static_cast<double>(v.R);
  n[index_1] = static_cast<double>(G) - static_cast<double>(v.G);
  n[index_2] = static_cast<double>(B) - static_cast<double>(v.B);
  n[index_3] = static_cast<double>(A) - static_cast<double>(v.A);
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
  \warning in case of overflow : e.g. 128+128 returns 0 for all 4 channels
*/
vpRGBa vpRGBa::operator+(const vpRGBa &v) const
{
  vpRGBa n; // new color
  n.R = static_cast<unsigned char>(R + v.R);
  n.G = static_cast<unsigned char>(G + v.G);
  n.B = static_cast<unsigned char>(B + v.B);
  n.A = static_cast<unsigned char>(A + v.A);
  return n;
}

/*!
  subtraction operator : "this" - v.
  \param v : Color to subtract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBa::operator-(const vpColVector &v) const
{
  vpColVector n(4); // new color
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  n[index_0] = R - v[index_0];
  n[index_1] = G - v[index_1];
  n[index_2] = B - v[index_2];
  n[index_3] = A - v[index_3];
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
*/
vpColVector vpRGBa::operator+(const vpColVector &v) const
{
  vpColVector n(4); // new color
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  n[index_0] = R + v[index_0];
  n[index_1] = G + v[index_1];
  n[index_2] = B + v[index_2];
  n[index_3] = A + v[index_3];
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector vpRGBa::operator*(const float &v) const
{
  vpColVector n(4);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  n[index_0] = R * v;
  n[index_1] = G * v;
  n[index_2] = B * v;
  n[index_3] = A * v;
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector vpRGBa::operator*(const double &v) const
{
  vpColVector n(4);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  n[index_0] = R * v;
  n[index_1] = G * v;
  n[index_2] = B * v;
  n[index_3] = A * v;
  return n;
}

bool vpRGBa::operator<(const vpRGBa &v) const
{
  double gray1 = (0.2126 * R) + (0.7152 * G) + (0.0722 * B);
  double gray2 = (0.2126 * v.R) + (0.7152 * v.G) + (0.0722 * v.B);

  return (gray1 < gray2);
}

bool vpRGBa::operator>(const vpRGBa &v) const
{
  double gray1 = (0.2126 * R) + (0.7152 * G) + (0.0722 * B);
  double gray2 = (0.2126 * v.R) + (0.7152 * v.G) + (0.0722 * v.B);

  return (gray1 > gray2);
}

/*!
 * Scale RGB components by x. Alpha component remain unchanged.
 *
 * @param x : Value used to scale RGB color components.
 * @param rgb : RGB color components to rescale.
 * @return Rescaled components with RGB * x.
 */
vpRGBa operator*(const double &x, const  vpRGBa &rgb)
{
  vpRGBa rgba;
  rgba.R = static_cast<unsigned char>(rgb.R * x);
  rgba.G = static_cast<unsigned char>(rgb.G * x);
  rgba.B = static_cast<unsigned char>(rgb.B * x);
  rgba.A = rgb.A;
  return rgba;
}

/*!
  \relates vpRGBa

  Writes the RGBA values to the stream \e os, and
  returns a reference to the stream. The
  coordinates are separated by a comma.

  The following code prints the intensity of the pixel in the middle of the image:
  \code
  #include <visp3/core/vpImage.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<vpRGBa> I(480,640);

    std::cout << "RGB: " << I[240][320] << std::endl;

    return 0;
  }
  \endcode
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRGBa &rgba)
{
  os << "(" << static_cast<int>(rgba.R) << "," << static_cast<int>(rgba.G) << "," << static_cast<int>(rgba.B) << "," << static_cast<int>(rgba.A) << ")";
  return os;
}
END_VISP_NAMESPACE
