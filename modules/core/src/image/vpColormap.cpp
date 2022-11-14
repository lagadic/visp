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
 * Draw text in an image.
 *
 *****************************************************************************/

#include <visp3/core/vpColormap.h>

/*!
  Creates a new font class with given height.

  \note The vpFontFamily::GENERIC_MONOSPACE font supports ASCII characters only. It was generated on the base of the
  generic monospace font from Gdiplus.

  \param [in] height : Initial font height value in pixels. By default it is equal to 16 pixels.
  \param [in] fontFamily : Font family in TTF format.
  \param [in] ttfFilename : Path to the TTF file if needed. Can contain multiple paths separated by `;`character. The
  first valid path that is found is used.
*/
vpColormap::vpColormap()
{
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
      Icolor[i][j] = vpRGBa(turbo_srgb_bytes[gray][0], turbo_srgb_bytes[gray][1], turbo_srgb_bytes[gray][2]);
    }
  }
}

// TODO: doc
void vpColormap::convert(const vpImage<vpRGBf> &I, vpImage<vpRGBa> &Icolor)
{
  //vpRGBf minVal = 0, maxVal = 1;
  //I.getMinMaxValue(minVal, maxVal);
  //// TODO:
  //std::cout << "min_R: " << minVal.R << " ; max_R: " << maxVal.R << std::endl;
  //std::cout << "min_G: " << minVal.G << " ; max_G: " << maxVal.G << std::endl;
  //std::cout << "min_B: " << minVal.B << " ; max_B: " << maxVal.B << std::endl;

  //// convert to 256 grayscale values
  //float a_R = 255.0f / (maxVal.R - minVal.R);
  //float b_R = -255.0f * minVal.R / (maxVal.R - minVal.R);
  //float a_G = 255.0f / (maxVal.G - minVal.G);
  //float b_G = -255.0f * minVal.G / (maxVal.G - minVal.G);
  //float a_B = 255.0f / (maxVal.B - minVal.B);
  //float b_B = -255.0f * minVal.B / (maxVal.B - minVal.B);
  //std::cout << "a_R: " << a_R << " ; b_R: " << b_R << std::endl;
  //std::cout << "a_G: " << a_G << " ; b_G: " << b_G << std::endl;
  //std::cout << "a_B: " << a_B << " ; b_B: " << b_B << std::endl;

  //Icolor.resize(I.getHeight(), I.getWidth());
  //for (unsigned int i = 0; i < I.getHeight(); i++) {
  //  for (unsigned int j = 0; j < I.getWidth(); j++) {
  //    Icolor[i][j].R = static_cast<unsigned char>(a_R * I[i][j].R + b_R);
  //    Icolor[i][j].G = static_cast<unsigned char>(a_G * I[i][j].G + b_G);
  //    Icolor[i][j].B = static_cast<unsigned char>(a_B * I[i][j].B + b_B);
  //  }
  //}

  vpImage<float> I_float(I.getHeight(), I.getWidth());
   for (unsigned int i = 0; i < I.getHeight(); i++) {
     for (unsigned int j = 0; j < I.getWidth(); j++) {
       I_float[i][j] = 0.299f * I[i][j].R + 0.587f * I[i][j].G + 0.114 * I[i][j].B;
     }
   }
   convert(I_float, Icolor);
}
