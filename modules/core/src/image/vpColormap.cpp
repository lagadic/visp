/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 Inria. All rights reserved.
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
 * Colormap.
 *
 *****************************************************************************/

#include <visp3/core/vpColormap.h>

// TODO: doc
/*!
  Creates a new font class with given height.

  \note The vpFontFamily::GENERIC_MONOSPACE font supports ASCII characters only. It was generated on the base of the
  generic monospace font from Gdiplus.

  \param [in] height : Initial font height value in pixels. By default it is equal to 16 pixels.
  \param [in] fontFamily : Font family in TTF format.
  \param [in] ttfFilename : Path to the TTF file if needed. Can contain multiple paths separated by `;`character. The
  first valid path that is found is used.
*/
vpColormap::vpColormap(const vpColormapType& colormapType) : m_colormapType(colormapType) {
  for (unsigned int i = 0; i < 256; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      switch (m_colormapType) {
      case COLORMAP_AUTUMN:
        m_colormapSrgbBytes[i][j] = m_autumnSrgbBytes[i][j];
        break;
      case COLORMAP_CIVIDIS:
        m_colormapSrgbBytes[i][j] = m_cividisSrgbBytes[i][j];
        break;
      case COLORMAP_COOL:
        m_colormapSrgbBytes[i][j] = m_coolSrgbBytes[i][j];
        break;
      case COLORMAP_GIST_EARTH:
        m_colormapSrgbBytes[i][j] = m_gistEarthSrgbBytes[i][j];
        break;
      case COLORMAP_GNUPLOT:
        m_colormapSrgbBytes[i][j] = m_gnuplotSrgbBytes[i][j];
        break;
      case COLORMAP_GNUPLOT2:
        m_colormapSrgbBytes[i][j] = m_gnuplot2SrgbBytes[i][j];
        break;
      case COLORMAP_HOT:
        m_colormapSrgbBytes[i][j] = m_hotSrgbBytes[i][j];
        break;
      case COLORMAP_HSV:
        m_colormapSrgbBytes[i][j] = m_hsvSrgbBytes[i][j];
        break;
      case COLORMAP_INFERNO:
        m_colormapSrgbBytes[i][j] = m_infernoSrgbBytes[i][j];
        break;
      case COLORMAP_JET:
        m_colormapSrgbBytes[i][j] = m_jetSrgbBytes[i][j];
        break;
      case COLORMAP_MAGMA:
        m_colormapSrgbBytes[i][j] = m_magmaSrgbBytes[i][j];
        break;
      case COLORMAP_OCEAN:
        m_colormapSrgbBytes[i][j] = m_oceanSrgbBytes[i][j];
        break;
      case COLORMAP_PLASMA:
        m_colormapSrgbBytes[i][j] = m_plasmaSrgbBytes[i][j];
        break;
      case COLORMAP_RAINBOW:
        m_colormapSrgbBytes[i][j] = m_rainbowSrgbBytes[i][j];
        break;
      case COLORMAP_SPRING:
        m_colormapSrgbBytes[i][j] = m_springSrgbBytes[i][j];
        break;
      case COLORMAP_SUMMER:
        m_colormapSrgbBytes[i][j] = m_summerSrgbBytes[i][j];
        break;
      case COLORMAP_TERRAIN:
        m_colormapSrgbBytes[i][j] = m_terrainSrgbBytes[i][j];
        break;
      case COLORMAP_TURBO:
        m_colormapSrgbBytes[i][j] = m_turboSrgbBytes[i][j];
        break;
      case COLORMAP_TWILIGHT:
        m_colormapSrgbBytes[i][j] = m_twilightSrgbBytes[i][j];
        break;
      case COLORMAP_TWILIGHT_SHIFTED:
        m_colormapSrgbBytes[i][j] = m_twilightShiftedSrgbBytes[i][j];
        break;
      case COLORMAP_VIRIDIS:
        m_colormapSrgbBytes[i][j] = m_viridisSrgbBytes[i][j];
        break;
      case COLORMAP_WINTER:
        m_colormapSrgbBytes[i][j] = m_winterSrgbBytes[i][j];
        break;
      default:
        break;
      }
    }
  }
}

// TODO: doc
void vpColormap::convert(const vpImage<float>& I, vpImage<vpRGBa>& Icolor) 
{
  float minVal = 0, maxVal = 1;
  I.getMinMaxValue(minVal, maxVal);

  // convert to 256 grayscale values
  float a = 255.0f / (maxVal - minVal);
  float b = -255 * minVal / (maxVal - minVal);
  vpImage<unsigned char> Inorm(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      Inorm[i][j] = static_cast<unsigned char>(a * I[i][j] + b);
    }
  }

  Icolor.resize(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < Icolor.getHeight(); i++) {
    for (unsigned int j = 0; j < Icolor.getWidth(); j++) {
      unsigned char gray = Inorm[i][j];
      Icolor[i][j] =
          vpRGBa(m_colormapSrgbBytes[gray][0], m_colormapSrgbBytes[gray][1], m_colormapSrgbBytes[gray][2]);
    }
  }
}

// TODO: doc
void vpColormap::convert(const vpImage<vpRGBf> &I, vpImage<vpRGBa> &Icolor)
{
  vpImage<float> I_float(I.getHeight(), I.getWidth());
   for (unsigned int i = 0; i < I.getHeight(); i++) {
     for (unsigned int j = 0; j < I.getWidth(); j++) {
       I_float[i][j] = 0.299f * I[i][j].R + 0.587f * I[i][j].G + 0.114 * I[i][j].B;
     }
   }
   convert(I_float, Icolor);
}
