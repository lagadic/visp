/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Colormap class to recolor an image with different grayscale values into
 * some corresponding color values, for better visualization for example.
 */

/*!
 * \file vpColormap.h
 *
 * \brief Colormap tool to have a mapping between 256 values and RGB values.
 */

#ifndef _vpColormap_h_
#define _vpColormap_h_

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpColormap
 *
 * \ingroup group_core_image
 *
 * \brief Creates a colormap class to be able to recolor an image with different grayscale
 * values into some corresponding color values, for better visualization for example.
*/
class VISP_EXPORT vpColormap
{
public:
  enum vpColormapType
  {
    COLORMAP_AUTUMN,
    COLORMAP_CIVIDIS,
    COLORMAP_COOL,
    COLORMAP_GIST_EARTH,
    COLORMAP_GNUPLOT,
    COLORMAP_GNUPLOT2,
    COLORMAP_HOT,
    COLORMAP_HSV,
    COLORMAP_INFERNO,
    COLORMAP_JET,
    COLORMAP_MAGMA,
    COLORMAP_OCEAN,
    COLORMAP_PLASMA,
    COLORMAP_RAINBOW,
    COLORMAP_SPRING,
    COLORMAP_SUMMER,
    COLORMAP_TERRAIN,
    COLORMAP_TURBO,
    COLORMAP_TWILIGHT,
    COLORMAP_TWILIGHT_SHIFTED,
    COLORMAP_VIRIDIS,
    COLORMAP_WINTER
  };

  vpColormap(const vpColormapType &colormapType);

  void convert(const vpImage<unsigned char> &I, vpImage<vpRGBa> &Icolor, bool normalize = false);
  void convert(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Icolor, bool normalize = false);

  void convert(const vpImage<float> &I, vpImage<vpRGBa> &Icolor);
  void convert(const vpImage<vpRGBf> &I, vpImage<vpRGBa> &Icolor);

private:
  vpColormapType m_colormapType;
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  unsigned char m_colormapSrgbBytes[256][3] = {};
#else
  unsigned char m_colormapSrgbBytes[256][3];
#endif
};
END_VISP_NAMESPACE
#endif
