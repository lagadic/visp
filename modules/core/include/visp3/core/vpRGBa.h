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
  \file vpRGBa.h
  \brief Define the object vpRGBa that is used to build color
  images (it defines a RGB 32 bits structure, fourth byte is not used)
*/

#ifndef VP_RGBA_H
#define VP_RGBA_H

#include <assert.h>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
#include <type_traits>
#endif


BEGIN_VISP_NAMESPACE
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
template <typename T, bool>
class vpHSV;
#endif
/*!
  \class vpRGBa

  \ingroup group_core_image

  Class that defines a RGBa 32 bits structure that is used to build color
  images. RGBa stands for red green blue alpha color space.

  The alpha channel is normally used as an opacity channel. If a pixel has a
  value of 0 in its alpha channel, it is fully transparent, whereas a value of
  255 in the alpha channel gives a fully opaque pixel.

  By default the alpha channel is set to vpRGBa::alpha_default.
  \sa vpImage
*/
class VISP_EXPORT vpRGBa
{
public:
  enum AlphaDefault { alpha_default = 255 };

  /*!
    Basic constructor.

    Build a black value.
  */
  inline vpRGBa() : R(0), G(0), B(0), A(vpRGBa::alpha_default) { }

  /*!
    Constructor.

    Initialize the color with R, G, B, A values.

    \param r : Red value.
    \param g : Green value.
    \param b : Blue value.
    \param a : Additional value.
  */
  inline vpRGBa(unsigned char r, unsigned char g, unsigned char b, unsigned char a = vpRGBa::alpha_default)
    : R(r), G(g), B(b), A(a)
  { }

  /*!
    Constructor.

    Initialize all the R, G, B, A components to \e v.

    \param v : Value to set.
  */
  VP_EXPLICIT inline vpRGBa(unsigned char v) : R(v), G(v), B(v), A(v) { }

  /*!
    Constructor.

    Initialize all the R, G, B, A components to \e v that should be in 0 - 255 range.

    \param v : Value to set.
  */
  VP_EXPLICIT inline vpRGBa(unsigned int v)
  {
    assert(v < 256);
    unsigned char v_uc = static_cast<unsigned char>(v);
    R = v_uc;
    G = v_uc;
    B = v_uc;
    A = v_uc;
  }

  /*!
    Constructor.

    Initialize all the R, G, B, A components to \e v that should be in 0 - 255 range.

    \param v : Value to set.
  */
  VP_EXPLICIT inline vpRGBa(int v)
  {
    assert(v >=0 && v < 256);
    unsigned char v_uc = static_cast<unsigned char>(v);
    R = v_uc;
    G = v_uc;
    B = v_uc;
    A = v_uc;
  }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifndef VISP_PYTHON_PREPROCESSOR_RUNNING
  /**
   * \brief Construct a new vpRGBa object from an vpHSV pbject.
   *
   * \tparam T The type of the channels of the vpHSV pixels.
   * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; 180].
   * \param[in] hsv The vpHSV object for which we want to have the corresponding vpRGBa object.
   */
  template <typename T, bool useFullScale, typename std::enable_if<std::is_same<T, unsigned char>::value, int>::type = 0>
  VP_EXPLICIT vpRGBa(const vpHSV<T, useFullScale> &hsv)
  {
    buildFrom<T, useFullScale, float>(hsv);
  }

  template <typename T, bool useFullScale, typename std::enable_if<std::is_floating_point<T>::value, int>::type = 0>
  VP_EXPLICIT vpRGBa(const vpHSV<T, useFullScale> &hsv)
  {
    buildFrom<T, useFullScale>(hsv);
  }
#endif

  /**
   * \brief Build a vpRGBa object from a vpHSV<unsigned char> object.
   *
   * \tparam T The type of the channels of the vpHSV pixels.
   * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; 180].
   * \tparam std::enable_if<std::is_same<T, unsigned char>::value, T>::type Enable the method only if T is unsigned char.
   * \param[in] other The vpHSV from which we want to build our object.
   * \return vpRGBa& The current object after conversion.
   */
  template<typename T, bool useFullScale, typename U = float >
  typename std::enable_if<std::is_same<U, float>::value &&std::is_same<T, unsigned char>::value, vpRGBa &>::type buildFrom(const vpHSV<T, useFullScale> &other)
  {
    vpHSV<U, useFullScale> hsv(other);
    buildFrom(hsv);
    return *this;
  }

  /**
   * \brief Build a vpRGBa object from a vpHSV<double> or vpHSV<float> object.
   *
   * \tparam T The type of the channels of the vpHSV pixels.
   * \tparam useFullScale True if vpHSV uses unsigned char and the full range [0; 255], false if vpHSV uses unsigned char and the limited range [0; 180].
   * \tparam std::enable_if<std::is_floating_point<T>::value, int>::type Enable the method only if T is a floating point number. The "int" is here because we cannot use a floating point type,
   * otherwise we get "{float/double} is not a valid type for a template non-type parameter".
   * \param[in] other The vpHSV from which we want to build our object.
   * \return vpRGBa& The current object after conversion.
   */
  template<typename T, bool useFullScale>
  typename std::enable_if<std::is_floating_point<T>::value, vpRGBa &>::type buildFrom(const vpHSV<T, useFullScale> &hsv)
  {
    T hue = hsv.H, saturation = hsv.S, value = hsv.V;
    T h = hue * 6.0;
    T s = saturation;
    T v = value;

    if (vpMath::equal(h, static_cast<T>(6.0), std::numeric_limits<T>::epsilon())) {
      h = 0.0;
    }

    T f = h - static_cast<int>(h);
    T p = v * (1.0 - s);
    T q = v * (1.0 - (s * f));
    T t = v * (1.0 - (s * (1.0 - f)));

    const int val_2 = 2;
    const int val_3 = 3;
    const int val_4 = 4;
    switch (static_cast<int>(h)) {
    case 0:
      hue = v;
      saturation = t;
      value = p;
      break;

    case 1:
      hue = q;
      saturation = v;
      value = p;
      break;

    case val_2:
      hue = p;
      saturation = v;
      value = t;
      break;

    case val_3:
      hue = p;
      saturation = q;
      value = v;
      break;

    case val_4:
      hue = t;
      saturation = p;
      value = v;
      break;

    default: // case 5:
      hue = v;
      saturation = p;
      value = q;
      break;
    }

    R = static_cast<unsigned char>(vpMath::round(hue * 255.0));
    G = static_cast<unsigned char>(vpMath::round(saturation * 255.0));
    B = static_cast<unsigned char>(vpMath::round(value * 255.0));
    A = alpha_default;
    return *this;
  }
#endif

/*!
 * Copy constructor.
 */
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  inline vpRGBa(const vpRGBa &v) = default;
#else
  inline vpRGBa(const vpRGBa &v) : R(v.R), G(v.G), B(v.B), A(v.A) { }
#endif

  /*!
    Create a RGBa value from a 4 dimension column vector.

    R=v[0]
    G=v[1]
    B=v[2]
    A=v[3]
  */
  VP_EXPLICIT inline vpRGBa(const vpColVector &v) : R(0), G(0), B(0), A(vpRGBa::alpha_default) { *this = v; }

  // We cannot add here the following destructor without changing the
  // hypothesis that the size of this class is 4. With the destructor it
  // becomes 16 that does break a lot of things around image conversions
  // virtual ~vpRGBa() {}; // Not to implement

  vpRGBa &operator=(const unsigned char &v);
  vpRGBa &operator=(const unsigned int &v);
  vpRGBa &operator=(const int &v);
#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  vpRGBa &operator=(vpRGBa &&v) = default;
  vpRGBa &operator=(const vpRGBa &v) = default;
#else
  vpRGBa &operator=(const vpRGBa &v)
  {
    this->R = v.R;
    this->G = v.G;
    this->B = v.B;
    this->A = v.A;
    return *this;
  }
#endif
  vpRGBa &operator=(const vpColVector &v);
  bool operator==(const vpRGBa &v) const;
  bool operator!=(const vpRGBa &v) const;

  vpColVector operator-(const vpRGBa &v) const;
  vpRGBa operator+(const vpRGBa &v) const;
  vpColVector operator-(const vpColVector &v) const;
  vpColVector operator+(const vpColVector &v) const;
  vpColVector operator*(const float &v) const;
  vpColVector operator*(const double &v) const;

  bool operator<(const vpRGBa &v) const;
  bool operator>(const vpRGBa &v) const;

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpRGBa &rgba);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /**
   * \brief Number of channels a vpRGBa object is made of.
   */
  static constexpr unsigned char nbChannels = 4;
#endif
public:
  unsigned char R; //!< Red component.
  unsigned char G; //!< Green component.
  unsigned char B; //!< Blue component.
  unsigned char A; //!< Additional component.

  friend VISP_EXPORT vpRGBa operator*(const double &x, const vpRGBa &rgb);
};

#if ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
static_assert(std::is_trivially_assignable_v<vpRGBa, vpRGBa>);
static_assert(std::is_trivially_copyable_v<vpRGBa>);
#endif

END_VISP_NAMESPACE
#endif
