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

#ifndef vpPixel_h
#define vpPixel_h

/*!
  \file vpPixel.h
  \brief Define the object vpPixel that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)
*/

#include <visp3/core/vpColVector.h>

/*!
  \class vpPixel

  \ingroup group_core_image

  Class that defines a RGBa 32 bits structure that is used to build color
  images. RGBa stands for red green blue alpha color space.

  The alpha channel is normally used as an opacity channel. If a pixel has a
  value of 0 in its alpha channel, it is fully transparent, whereas a value of
  255 in the alpha channel gives a fully opaque pixel.

  By default the alpha channel is set to vpPixel::alpha_default.
  \sa vpImage
*/
template <class Type, typename std::enable_if<std::is_same<Type, unsigned char >::value ||
                                              std::is_same<Type, char          >::value ||
                                              std::is_same<Type, unsigned short>::value ||
                                              std::is_same<Type, short         >::value ||
                                              std::is_same<Type, unsigned int  >::value ||
                                              std::is_same<Type, int           >::value ||
                                              std::is_same<Type, float         >::value ||
                                              std::is_same<Type, double        >::value>::type* = nullptr> class vpPixel
{
public:
  enum AlphaDefault { alpha_default = 255 };

  /*!
    Basic constructor.

    Build a black value.

  */
  inline vpPixel() : R(0), G(0), B(0), A(vpPixel::alpha_default) {}

  /*!
    Constructor.

    Initialize the color with R, G, B, A values.

    \param r : Red value.
    \param g : Green value.
    \param b : Blue value.
    \param a : Additional value.
  */
  inline vpPixel(Type r, Type g, Type b, Type a = vpPixel::alpha_default)
    : R(r), G(g), B(b), A(a)
  {
  }

  /*!
    Constructor.

    Initialize all the R, G, B, A components to \e v.

    \param v : Value to set.
  */
  inline vpPixel(Type v) : R(v), G(v), B(v), A(v) {}

  /*!
    Copy constructor.
  */
  inline vpPixel(const vpPixel &v) : R(v.R), G(v.G), B(v.B), A(v.A) {}

  /*!
    Create a RGBa value from a 4 dimension column vector.

    R=v[0]
    G=v[1]
    B=v[2]
    A=v[3]
  */
  inline vpPixel(const vpColVector &v) : R(0), G(0), B(0), A(vpPixel::alpha_default) { *this = v; }

  // We cannot add here the following destructor without changing the
  // hypothesis that the size of this class is 4. With the destructor it
  // becomes 16 that does break a lot of things around image conversions
  // virtual ~vpPixel() {}; // Not to implement

  vpPixel &operator=(const Type &v)
  {
    this->R = v;
    this->G = v;
    this->B = v;
    this->A = v;
    return *this;
  }

  vpPixel &operator=(const vpPixel &v)
  {
    this->R = v.R;
    this->G = v.G;
    this->B = v.B;
    this->A = v.A;
    return *this;
  }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpPixel &operator=(const vpPixel &&v)
  {
    this->R = std::move(v.R);
    this->G = std::move(v.G);
    this->B = std::move(v.B);
    this->A = std::move(v.A);
    return *this;
  }
#endif
  vpPixel &operator=(const vpColVector &v)
  {
    if (v.getRows() != 4) {
      // vpERROR_TRACE("Bad vector dimension ");
      throw(vpException(vpException::dimensionError, "Bad vector dimension "));
    }
    R = static_cast<Type>(v[0]);
    G = static_cast<Type>(v[1]);
    B = static_cast<Type>(v[2]);
    A = static_cast<Type>(v[3]);
    return *this;
  }

  bool operator==(const vpPixel &v)
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

  bool operator!=(const vpPixel &v)
  {
    return (R != v.R || G != v.G || B != v.B || A != v.A);
  }

  vpColVector operator-(const vpPixel &v) const
  {
    vpColVector n(4); // new color
    n[0] = (double)R - (double)v.R;
    n[1] = (double)G - (double)v.G;
    n[2] = (double)B - (double)v.B;
    n[3] = (double)A - (double)v.A;
    return n;
  }

  vpPixel operator+(const vpPixel &v) const
  {
    vpPixel n; // new color
    // n.R = static_cast<unsigned char>(R + v.R);
    // n.G = static_cast<unsigned char>(G + v.G);
    // n.B = static_cast<unsigned char>(B + v.B);
    // n.A = static_cast<unsigned char>(A + v.A);
    n.R = static_cast<Type>(R + v.R);
    n.G = static_cast<Type>(G + v.G);
    n.B = static_cast<Type>(B + v.B);
    n.A = static_cast<Type>(A + v.A);
    return n;
  }

  vpColVector operator-(const vpColVector &v) const
  {
    vpColVector n(4); // new color
    n[0] = R - v[0];
    n[1] = G - v[1];
    n[2] = B - v[2];
    n[3] = A - v[3];
    return n;
  }

  vpColVector operator+(const vpColVector &v) const
  {
    vpColVector n(4); // new color
    n[0] = R + v[0];
    n[1] = G + v[1];
    n[2] = B + v[2];
    n[3] = A + v[3];
    return n;
  }

  vpColVector operator*(const float &v) const
  {
    vpColVector n(4);
    n[0] = R * v;
    n[1] = G * v;
    n[2] = B * v;
    n[3] = A * v;
    return n;
  }

  vpColVector operator*(const double &v) const
  {
    vpColVector n(4);
    n[0] = R * v;
    n[1] = G * v;
    n[2] = B * v;
    n[3] = A * v;
    return n;
  }

  bool operator<(const vpPixel &v) const
  {
    double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
    double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

    return (gray1 < gray2);
  }

  bool operator>(const vpPixel &v) const
  {
    double gray1 = 0.2126 * R + 0.7152 * G + 0.0722 * B;
    double gray2 = 0.2126 * v.R + 0.7152 * v.G + 0.0722 * v.B;

    return (gray1 > gray2);
  }

  // vpPixel operator*(const double &x, const vpPixel &rgb)
  // {
  //   return rgb * x;
  // }

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPixel &rgba)
  {
    os << "(" << (int)rgba.R << "," << (int)rgba.G << "," << (int)rgba.B << "," << (int)rgba.A << ")";
    return os;
  }

public:
  Type R; //!< Red component.
  Type G; //!< Green component.
  Type B; //!< Blue component.
  Type A; //!< Additionnal component.

  // friend VISP_EXPORT vpPixel operator*(const double &x, const vpPixel &rgb);
};

#endif
