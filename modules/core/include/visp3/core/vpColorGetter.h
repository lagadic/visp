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
 * Color getter.
 */

#ifndef VP_COLOR_GETTER_H
#define VP_COLOR_GETTER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpRGBa.h>

BEGIN_VISP_NAMESPACE
/**
 * \brief Helper class that permits to get a channel of a color type by its index instead of its name.
 *
 * \tparam ID The index of the channel.
 */
  template <int ID>
class vpColorGetter
{
public:
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/**
 * \brief Setter for a channel of a HSV pixel.
 *
 * \tparam ArithmeticType The encoding type of a channel.
 * \tparam useFullScale When using unsigned charn, either true if the Hue channel uses the full range
 * of unsigned char or false if it uses a limited range.
 * \param[in] col The pixel we want to set a channel value.
 * \return ArithmeticType& The channel value.
 */
  template <typename ArithmeticType, bool useFullScale >
  static ArithmeticType &get(vpHSV<ArithmeticType, useFullScale> &col);

  /**
   * \brief Getter for a channel of a HSV pixel.
   *
   * \tparam ArithmeticType The encoding type of a channel.
   * \tparam useFullScale When using unsigned charn, either true if the Hue channel uses the full range
   * of unsigned char or false if it uses a limited range.
   * \param[in] col The pixel we want to get a channel value.
   * \return ArithmeticType& The channel value.
   */
  template <typename ArithmeticType, bool useFullScale >
  static const ArithmeticType &get(const vpHSV<ArithmeticType, useFullScale> &col);
#endif

  /**
  * \brief Setter for a vpRGBa pixel.
  *
  * \param[in] col The pixel.
  * \return unsigned char& The channel after the change has been performed.
  */
  static unsigned char &get(vpRGBa &col);

  /**
   * \brief Getter for a vpRGBA pixel.
   *
   * \param[in] col The pixel.
   * \return const unsigned char& The value of the desired channel.
   */
  static const unsigned char &get(const vpRGBa &col);
};

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
template<>
template<typename ArithmeticType, bool useFullScale>
inline ArithmeticType &vpColorGetter<0>::get(vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.H;
}

template<>
template<typename ArithmeticType, bool useFullScale>
inline ArithmeticType &vpColorGetter<1>::get(vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.S;
}

template<>
template<typename ArithmeticType, bool useFullScale>
inline ArithmeticType &vpColorGetter<2>::get(vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.V;
}

template<>
template<typename ArithmeticType, bool useFullScale>
inline const ArithmeticType &vpColorGetter<0>::get(const vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.H;
}

template<>
template<typename ArithmeticType, bool useFullScale>
inline const ArithmeticType &vpColorGetter<1>::get(const vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.S;
}

template<>
template<typename ArithmeticType, bool useFullScale>
inline const ArithmeticType &vpColorGetter<2>::get(const vpHSV<ArithmeticType, useFullScale> &hsv)
{
  return hsv.V;
}
#endif

template<>
inline unsigned char &vpColorGetter<0>::get(vpRGBa &rgba)
{
  return rgba.R;
}

template<>
inline unsigned char &vpColorGetter<1>::get(vpRGBa &rgba)
{
  return rgba.G;
}

template<>
inline unsigned char &vpColorGetter<2>::get(vpRGBa &rgba)
{
  return rgba.B;
}

template<>
inline unsigned char &vpColorGetter<3>::get(vpRGBa &rgba)
{
  return rgba.A;
}

template<>
inline const unsigned char &vpColorGetter<0>::get(const vpRGBa &rgba)
{
  return rgba.R;
}

template<>
inline const unsigned char &vpColorGetter<1>::get(const vpRGBa &rgba)
{
  return rgba.G;
}

template<>
inline const unsigned char &vpColorGetter<2>::get(const vpRGBa &rgba)
{
  return rgba.B;
}

template<>
inline const unsigned char &vpColorGetter<3>::get(const vpRGBa &rgba)
{
  return rgba.A;
}
END_VISP_NAMESPACE
#endif
