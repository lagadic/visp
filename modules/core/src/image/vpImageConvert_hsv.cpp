/****************************************************************************
 *
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
 * Convert image types.
 *
*****************************************************************************/

/*!
  \file vpImageConvert_hsv.cpp
  \brief HSV from/to conversion.
*/

#if defined(_OPENMP)
#include <omp.h>
#endif

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>

/*!
 * Convert an HSV image to a RGB or RGBa image depending on the value of \e step.
 * \param[in] hue_ : Input image H channel.
 * \param[in] saturation_ : Input image S channel.
 * \param[in] value_ : Input image V channel.
 * \param[out] rgb : Pointer to the 24-bit or 32-bits color image that should be allocated with a size of
 * width * height * step.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the output color image; 3 for an RGB image, 4 for an RGBA image.
 */
void vpImageConvert::HSV2RGB(const double *hue_, const double *saturation_, const double *value_, unsigned char *rgb,
                             unsigned int size, unsigned int step)
{
  for (unsigned int i = 0; i < size; ++i) {
    double hue = hue_[i], saturation = saturation_[i], value = value_[i];

    if (vpMath::equal(saturation, 0.0, std::numeric_limits<double>::epsilon())) {
      hue = value;
      saturation = value;
    }
    else {
      double h = hue * 6.0;
      double s = saturation;
      double v = value;

      if (vpMath::equal(h, 6.0, std::numeric_limits<double>::epsilon())) {
        h = 0.0;
      }

      double f = h - static_cast<int>(h);
      double p = v * (1.0 - s);
      double q = v * (1.0 - (s * f));
      double t = v * (1.0 - (s * (1.0 - f)));

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

      case 2:
        hue = p;
        saturation = v;
        value = t;
        break;

      case 3:
        hue = p;
        saturation = q;
        value = v;
        break;

      case 4:
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
    }

    rgb[i * step] = static_cast<unsigned char>(vpMath::round(hue * 255.0));
    rgb[i * step + 1] = static_cast<unsigned char>(vpMath::round(saturation * 255.0));
    rgb[i * step + 2] = static_cast<unsigned char>(vpMath::round(value * 255.0));
    if (step == 4) {// alpha
      rgb[i * step + 3] = vpRGBa::alpha_default;
    }
  }
}

/*!
 * Convert an RGB or RGBa color image depending on the value of \e step into an HSV image.
 * \param[out] rgb : Pointer to the 24-bit or 32-bits color image that should be allocated with a size of
 * width * height * step.
 * \param[out] hue : Output H channel with values in range [0, 1].
 * \param[out] saturation : Output S channel with values in range [0, 1].
 * \param[out] value : Output V channel with values in range [0, 1].
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the input color image; 3 for an RGB image, 4 for an RGBA image.
 */
void vpImageConvert::RGB2HSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
                             unsigned int size, unsigned int step)
{
  for (unsigned int i = 0; i < size; ++i) {
    double red, green, blue;
    double h, s, v;
    double min, max;

    red = rgb[i * step] / 255.0;
    green = rgb[i * step + 1] / 255.0;
    blue = rgb[i * step + 2] / 255.0;

    if (red > green) {
      max = ((std::max))(red, blue);
      min = ((std::min))(green, blue);
    }
    else {
      max = ((std::max))(green, blue);
      min = ((std::min))(red, blue);
    }

    v = max;

    if (!vpMath::equal(max, 0.0, std::numeric_limits<double>::epsilon())) {
      s = (max - min) / max;
    }
    else {
      s = 0.0;
    }

    if (vpMath::equal(s, 0.0, std::numeric_limits<double>::epsilon())) {
      h = 0.0;
    }
    else {
      double delta = max - min;
      if (vpMath::equal(delta, 0.0, std::numeric_limits<double>::epsilon())) {
        delta = 1.0;
      }

      if (vpMath::equal(red, max, std::numeric_limits<double>::epsilon())) {
        h = (green - blue) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<double>::epsilon())) {
        h = 2 + (blue - red) / delta;
      }
      else {
        h = 4 + (red - green) / delta;
      }

      h /= 6.0;
      if (h < 0.0) {
        h += 1.0;
      }
      else if (h > 1.0) {
        h -= 1.0;
      }
    }

    hue[i] = h;
    saturation[i] = s;
    value[i] = v;
  }
}

/*!
  Converts an array of hue, saturation and value to an array of RGBa values.

  Alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] hue : Array of hue values (range between [0 - 1]).
  \param[in] saturation : Array of saturation values (range between [0 - 1]).
  \param[in] value : Array of value values (range between [0 - 1]).
  \param[out] rgba : Pointer to the 32-bit RGBA image that should
  be allocated with a size of width * height * 4. Alpha channel is here set to vpRGBa::alpha_default.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::HSVToRGBa(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
                               unsigned int size)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgba, size, 4);
}

/*!
  Converts an array of hue, saturation and value to an array of RGBa values.

  Alpha component of the converted image is set to vpRGBa::alpha_default.

  \param[in] hue : Array of hue values (range between [0 - 255]).
  \param[in] saturation : Array of saturation values (range between [0 - 255]).
  \param[in] value : Array of value values (range between [0 - 255]).
  \param[out] rgba : Pointer to the 32-bit RGBA image that should
  be allocated with a size of width * height * 4. Alpha channel is here set to vpRGBa::alpha_default.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::HSVToRGBa(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                               unsigned char *rgba, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    double h = hue[i] / 255.0, s = saturation[i] / 255.0, v = value[i] / 255.0;

    vpImageConvert::HSVToRGBa(&h, &s, &v, (rgba + (i * 4)), 1);
  }
}

/*!
  Converts an array of RGBa to an array of hue, saturation, value values.
  The alpha channel is not used.

  \param[in] rgba : Pointer to the 32-bits RGBA bitmap.
  \param[out] hue : Array of hue values converted from RGB color space in range [0 - 1].
  \param[out] saturation : Array of saturation values converted from RGB color space in range [0 - 1].
  \param[out] value : Array of value values converted from RGB color space in range [0 - 1].
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::RGBaToHSV(const unsigned char *rgba, double *hue, double *saturation, double *value,
                               unsigned int size)
{
  unsigned int step = 4;
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < size; ++i) {
    double red, green, blue;
    double h, s, v;
    double min, max;

    unsigned int i_ = i * step;

    red = rgba[i_];
    green = rgba[++i_];
    blue = rgba[++i_];

    if (red > green) {
      max = std::max<double>(red, blue);
      min = std::min<double>(green, blue);
    }
    else {
      max = std::max<double>(green, blue);
      min = std::min<double>(red, blue);
    }

    v = max;

    if (!vpMath::equal(max, 0., std::numeric_limits<double>::epsilon())) {
      s = 255. * (max - min) / max;
    }
    else {
      s = 0.;
    }

    if (vpMath::equal(s, 0., std::numeric_limits<double>::epsilon())) {
      h = 0.;
    }
    else {
      double delta = max - min;
      if (vpMath::equal(delta, 0., std::numeric_limits<double>::epsilon())) {
        delta = 255.;
      }

      if (vpMath::equal(red, max, std::numeric_limits<double>::epsilon())) {
        h = 43. * (green - blue) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<double>::epsilon())) {
        h = 85. + 43. * (blue - red) / delta;
      }
      else {
        h = 171. + 43. * (red - green) / delta;
      }

      if (h < 0.) {
        h += 255.;
      }
      else if (h > 255.) {
        h -= 255.;
      }
    }

    hue[i] = h;
    saturation[i] = s;
    value[i] = v;
  }
}

/*!
  Converts an array of RGBa to an array of hue, saturation, value values.
  The alpha channel is not used.

  \param[in] rgba : Pointer to the 32-bits RGBA bitmap.
  \param[out] hue : Array of hue values converted from RGB color space in range [0, 255].
  \param[out] saturation : Array of saturation values converted from RGB color space in range [0 - 255].
  \param[out] value : Array of value values converted from RGB color space in range [0 - 255].
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::RGBaToHSV(const unsigned char *rgba, unsigned char *hue, unsigned char *saturation,
                               unsigned char *value, unsigned int size)
{
  unsigned int step = 4;
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < size; ++i) {
    float red, green, blue;
    float h, s, v;
    float min, max;
    unsigned int i_ = i * step;

    red = rgba[i_];
    green = rgba[++i_];
    blue = rgba[++i_];

    if (red > green) {
      max = std::max<float>(red, blue);
      min = std::min<float>(green, blue);
    }
    else {
      max = std::max<float>(green, blue);
      min = std::min<float>(red, blue);
    }

    v = max;

    if (!vpMath::equal(max, 0.f, std::numeric_limits<float>::epsilon())) {
      s = 255.f * (max - min) / max;
    }
    else {
      s = 0.f;
    }

    if (vpMath::equal(s, 0.f, std::numeric_limits<float>::epsilon())) {
      h = 0.f;
    }
    else {
      float delta = max - min;
      if (vpMath::equal(delta, 0.0, std::numeric_limits<float>::epsilon())) {
        delta = 255.f;
      }

      if (vpMath::equal(red, max, std::numeric_limits<float>::epsilon())) {
        h = 43.f * (green - blue) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<float>::epsilon())) {
        h = 85.f + 43.f * (blue - red) / delta;
      }
      else {
        h = 171.f + 43.f * (red - green) / delta;
      }

      if (h < 0.f) {
        h += 255.f;
      }
      else if (h > 255.f) {
        h -= 255.f;
      }
    }

    hue[i] = static_cast<unsigned char>(h);
    saturation[i] = static_cast<unsigned char>(s);
    value[i] = static_cast<unsigned char>(v);
  }
}

/*!
  Converts an array of hue, saturation and value to an array of RGB values.

  \param[in] hue : Array of hue values (range between [0 - 1]).
  \param[in] saturation : Array of saturation values (range between [0 - 1]).
  \param[in] value : Array of value values (range between [0 - 1]).
  \param[out] rgb : Pointer to the 24-bit RGB image that should be allocated with a size of
  width * height * 3.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::HSVToRGB(const double *hue, const double *saturation, const double *value, unsigned char *rgb,
                              unsigned int size)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgb, size, 3);
}

/*!
  Converts an array of hue, saturation and value to an array of RGB values.

  \param[in] hue : Array of hue values (range between [0 - 255]).
  \param[in] saturation : Array of saturation values (range between [0 - 255]).
  \param[in] value : Array of value values (range between [0 - 255]).
  \param[out] rgb : Pointer to the 24-bit RGB image that should be allocated with a size of width * height * 3.
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::HSVToRGB(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                              unsigned char *rgb, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    double h = hue[i] / 255.0, s = saturation[i] / 255.0, v = value[i] / 255.0;

    vpImageConvert::HSVToRGB(&h, &s, &v, (rgb + (i * 3)), 1);
  }
}

/*!
  Converts an array of RGB to an array of hue, saturation, value values.

  \param[in] rgb : Pointer to the 24-bits RGB bitmap.
  \param[out] hue : Array of hue values converted from RGB color space(range between[0 - 255]).
  \param[out] saturation : Array of saturation values converted from RGB color space(range between[0 - 255]).
  \param[out] value : Array of value values converted from RGB color space(range between[0 - 255]).
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::RGBToHSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
                              unsigned int size)
{
  vpImageConvert::RGB2HSV(rgb, hue, saturation, value, size, 3);
}

/*!
  Converts an array of RGB to an array of hue, saturation, value values.

  \param[in] rgb : Pointer to the 24-bits RGB bitmap.
  \param[out] hue : Array of hue values converted from RGB color space (range between [0 - 255]).
  \param[out] saturation : Array of saturation values converted from RGB color space (range between [0 - 255]).
  \param[out] value : Array of value values converted from RGB color space (range between [0 - 255]).
  \param[in] size : The image size or the number of pixels corresponding to the image width * height.
*/
void vpImageConvert::RGBToHSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation,
                              unsigned char *value, unsigned int size)
{
  for (unsigned int i = 0; i < size; ++i) {
    double h, s, v;

    vpImageConvert::RGBToHSV((rgb + (i * 3)), &h, &s, &v, 1);

    hue[i] = static_cast<unsigned char>(255.0 * h);
    saturation[i] = static_cast<unsigned char>(255.0 * s);
    value[i] = static_cast<unsigned char>(255.0 * v);
  }
}
