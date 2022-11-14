/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * 32-bit floating point RGB pixel.
 *
 *****************************************************************************/

/*!
  \file vpRGBf.cpp
  \brief Define the object vpRGBf that is used to build color
  images (it defines a RGB 32-bit floating point structure)
*/

#include <visp3/core/vpColor.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpRGBf.h>

/*!
  Copy operator (from a floating-point value)

  \param v : Input color ( R = G = B = v )
*/
vpRGBf &vpRGBf::operator=(float v)
{
  this->R = v;
  this->G = v;
  this->B = v;
  return *this;
}

/*!
  Copy operator.
*/
vpRGBf &vpRGBf::operator=(const vpRGBf &v)
{
  this->R = v.R;
  this->G = v.G;
  this->B = v.B;
  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Move operator.
*/
vpRGBf &vpRGBf::operator=(const vpRGBf &&v)
{
  this->R = std::move(v.R);
  this->G = std::move(v.G);
  this->B = std::move(v.B);
  return *this;
}
#endif

/*!
  Cast a vpColVector in a vpRGBf

  \param v : Input vector. v[0], v[1], v[2] are to make into
  relation with respectively R, G and B.

  \exception vpException::dimensionError : If v is not a 3-dimentional vector.
*/
vpRGBf &vpRGBf::operator=(const vpColVector &v)
{
  if (v.getRows() != 3) {
    vpERROR_TRACE("Bad vector dimension");
    throw(vpException(vpException::dimensionError, "Bad vector dimension"));
  }
  R = (float) v[0];
  G = (float) v[1];
  B = (float) v[2];
  return *this;
}

/*!
  Compare two RGB values.

  \return true if the values are exactly the same, false otherwise.
*/
bool vpRGBf::operator==(const vpRGBf &v)
{
  if (R != v.R)
    return false;
  if (G != v.G)
    return false;
  if (B != v.B)
    return false;

  return true;
}
/*!
  Compare two color pixels.

  \return true if the values are different, false if they are exactly the same.
*/
bool vpRGBf::operator!=(const vpRGBf &v) { return (R != v.R || G != v.G || B != v.B); }

/*!
  subtraction operator : "this" - v.
  \param v : Color to subtract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBf::operator-(const vpRGBf &v) const
{
  vpColVector n(3); // new color
  n[0] = (double)R - (double)v.R;
  n[1] = (double)G - (double)v.G;
  n[2] = (double)B - (double)v.B;
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
  \warning in case of overflow : e.g. 128+128 returns 0 for all 4 channels
*/
vpRGBf vpRGBf::operator+(const vpRGBf &v) const
{
  vpRGBf n; // new color
  n.R = R + v.R;
  n.G = G + v.G;
  n.B = B + v.B;
  return n;
}

/*!
  subtraction operator : "this" - v.
  \param v : Color to subtract to the current object "this".
  \return "this" - v
*/
vpColVector vpRGBf::operator-(const vpColVector &v) const
{
  vpColVector n(3); // new color
  n[0] = R - v[0];
  n[1] = G - v[1];
  n[2] = B - v[2];
  return n;
}

/*!
  Addition operator : "this" + v.
  \param v : Color to add to the current object "this".
  \return "this" + v
*/
vpColVector vpRGBf::operator+(const vpColVector &v) const
{
  vpColVector n(3); // new color
  n[0] = R + v[0];
  n[1] = G + v[1];
  n[2] = B + v[2];
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector vpRGBf::operator*(float v) const
{
  vpColVector n(3);
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  return n;
}

/*!
  Multiplication operator : v * "this".
  \param v : Value to multiply.
  \return v * "this"
*/
vpColVector vpRGBf::operator*(double v) const
{
  vpColVector n(3);
  n[0] = R * v;
  n[1] = G * v;
  n[2] = B * v;
  return n;
}

bool vpRGBf::operator<(const vpRGBf &v) const
{
  double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
  double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

  return (gray1 < gray2);
}

bool vpRGBf::operator>(const vpRGBf &v) const
{
  double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
  double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

  return (gray1 > gray2);
}

vpRGBf operator*(double x, const vpRGBf &rgb) { return rgb * x; }

/*!
  \relates vpRGBf

  Writes the RGB values to the stream \e os, and
  returns a reference to the stream. The
  coordinates are separated by a comma.

  The following code prints the intensity of the pixel in the middle of the image:
\code
#include <visp3/core/vpImage.h>

int main()
{
  vpImage<vpRGBf> I(480,640);

  std::cout << "RGB: " << I[240][320] << std::endl;

  return 0;
}
  \endcode
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRGBf &rgb)
{
  os << "(" << rgb.R << "," << rgb.G << "," << rgb.B << ")";
  return os;
}
