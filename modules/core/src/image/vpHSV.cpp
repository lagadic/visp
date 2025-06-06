/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * HSV color scale.
 */

#include <visp3/core/vpHSV.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
BEGIN_VISP_NAMESPACE

template<>
vpHSV<double> &vpHSV<double>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  set(hsv);
  return *this;
}

template<>
vpHSV<unsigned char, true> &vpHSV<unsigned char, true>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= 255.;
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv);
  return *this;
}

template<>
vpHSV<unsigned char, false> &vpHSV<unsigned char, false>::buildFrom(const vpRGBa &rgba)
{
  vpColVector hsv = computeNormalizedHSV(rgba);
  hsv[0] *= static_cast<double>(maxHueUsingLimitedRange);
  hsv[1] *= 255.;
  hsv[2] *= 255.;

  set(hsv);
  return *this;
}

END_VISP_NAMESPACE
#else
void dummy_vpHSV() { }
#endif
