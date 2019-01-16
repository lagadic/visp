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
 * RGBA pixel.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpRGBa.cpp
  \brief Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used - yet -)
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpRGBa.h>

/*!
  Copy operator (from an unsigned char value)

  \param v : Input color ( R = G = B = v )
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

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
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
  dimention vector.
*/
vpRGBa &vpRGBa::operator=(const vpColVector &v)
{
  if (v.getRows() != 4) {
    vpERROR_TRACE("Bad vector dimension ");
    throw(vpException(vpException::dimensionError, "Bad vector dimension "));
  }
  R = (unsigned char)v[0];
  G = (unsigned char)v[1];
  B = (unsigned char)v[2];
  A = (unsigned char)v[3];
  return *this;
}

/*!
  Compare two RGBa values.

  \return true if the values are the same, false otherwise.
*/
bool vpRGBa::operator==(const vpRGBa &v)
{
  if (R != v.R)
    return false;
  if (G != v.G)
    return false;
  if (B != v.B)
    return false;
  if (A != v.A)
    return false;

  return true;
}
/*!
  Compare two color pixels.

  \return true if the images are different, false if they are the same.
*/
bool vpRGBa::operator!=(const vpRGBa &v) { return (R != v.R || G != v.G || B != v.B || A != v.A); }

/*!
  Substraction operator : "this" - v.
  \param v : Color to substract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBa::operator-(const vpRGBa &v) const
{
  vpColVector n(4); // new color
  n[0] = (double)R - (double)v.R;
  n[1] = (double)G - (double)v.G;
  n[2] = (double)B - (double)v.B;
  n[3] = (double)A - (double)v.A;
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
  Substraction operator : "this" - v.
  \param v : Color to substract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBa::operator-(const vpColVector &v) const
{
  vpColVector n(4); // new color
  n[0] = R - v[0];
  n[1] = G - v[1];
  n[2] = B - v[2];
  n[3] = A - v[3];
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
  n[0] = R + v[0];
  n[1] = G + v[1];
  n[2] = B + v[2];
  n[3] = A + v[3];
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
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  n[3] = A * v;
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
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  n[3] = A * v;
  return n;
}

bool vpRGBa::operator<(const vpRGBa &v) const
{
  double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
  double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

  return (gray1 < gray2);
}

bool vpRGBa::operator>(const vpRGBa &v) const
{
  double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
  double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

  return (gray1 > gray2);
}

vpRGBa operator*(const double &x, const vpRGBa &rgb) { return rgb * x; }

/*!

  \relates vpRGBa

  Writes the RGBA values to the stream \e os, and
  returns a reference to the stream. The
  coordinates are separated by a comma.

  The following code prints the intensity of the pixel in the middle of the image:
\code
#include <visp3/core/vpImage.h>

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
  os << "(" << (int)rgba.R << "," << (int)rgba.G << "," << (int)rgba.B << "," << (int)rgba.A << ")";
  return os;
}
