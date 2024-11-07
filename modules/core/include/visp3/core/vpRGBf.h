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
 * 32-bit floating point RGB pixel.
 */

/*!
  \file vpRGBf.h
  \brief Define the object vpRGBf that is used to build color
  images (it defines a RGB 32-bit floating point structure)
*/

#ifndef VP_RGBF_H
#define VP_RGBF_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

BEGIN_VISP_NAMESPACE

/*!
  \class vpRGBf

  \ingroup group_core_image

  Class that defines a RGB 32-bit floating point structure that is used to build color
  images. RGB stands for red green blue color space.

  \sa vpRGBa
  \sa vpImage
*/
class VISP_EXPORT vpRGBf
{
public:
  /*!
    Basic constructor.

    Build a black value.
  */
  inline vpRGBf() : R(0), G(0), B(0) { }

  /*!
    Constructor.

    Initialize the color with R, G, B values.

    \param r : Red value.
    \param g : Green value.
    \param b : Blue value.
  */
  inline vpRGBf(float r, float g, float b)
    : R(r), G(g), B(b)
  { }

  /*!
    Constructor.

    Initialize all the R, G, B components to \e v.

    \param v : Value to set.
  */
  VP_EXPLICIT inline vpRGBf(float v) : R(v), G(v), B(v) { }

  /*!
    Constructor.

    Initialize all the R, G, B components to \e v.

    \param v : Value to set.
  */
  VP_EXPLICIT inline vpRGBf(int v)
  {
    *this = v;
  }

  /*!
    Copy constructor.
  */
  inline vpRGBf(const vpRGBf &v) : R(v.R), G(v.G), B(v.B) { }

  /*!
    Create a RGB value from a 3 dimensional column vector.

    R=v[0]
    G=v[1]
    B=v[2]
  */
  VP_EXPLICIT inline vpRGBf(const vpColVector &v) : R(0), G(0), B(0) { *this = v; }

  vpRGBf &operator=(float v);
  vpRGBf &operator=(int v);
  vpRGBf &operator=(const vpRGBf &v);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRGBf &operator=(const vpRGBf &&v);
#endif
  vpRGBf &operator=(const vpColVector &v);
  bool operator==(const vpRGBf &v) const;
  bool operator!=(const vpRGBf &v) const;

  vpColVector operator-(const vpRGBf &v) const;
  vpRGBf operator+(const vpRGBf &v) const;
  vpColVector operator-(const vpColVector &v) const;
  vpColVector operator+(const vpColVector &v) const;
  vpColVector operator*(float v) const;
  vpColVector operator*(double v) const;

  bool operator<(const vpRGBf &v) const;
  bool operator>(const vpRGBf &v) const;

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRGBf &rgb);

public:
  float R; //!< Red component.
  float G; //!< Green component.
  float B; //!< Blue component.

  friend VISP_EXPORT vpRGBf operator*(double x, const vpRGBf &rgb);
  friend VISP_EXPORT vpRGBf operator*(float x, const vpRGBf &rgb);
};
END_VISP_NAMESPACE
#endif
