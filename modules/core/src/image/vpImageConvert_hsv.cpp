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

BEGIN_VISP_NAMESPACE
/*!
 * Convert an HSV image to a RGB or RGBa image depending on the value of \e step.
 * \param[in] hue_ : Image hue channel in range [0,1].
 * \param[in] saturation_ : Image saturation channel in range [0,1].
 * \param[in] value_ : Image value channel in range [0,1].
 * \param[out] rgb : Pointer to the RGB (24-bit) or RGBa (32-bits) color image that should be allocated with a size of
 * width * height * step.
 * This array should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the output color image; 3 for an RGB image, 4 for an RGBA image.
 */
  void vpImageConvert::HSV2RGB(const double *hue_, const double *saturation_, const double *value_, unsigned char *rgb,
                               unsigned int size, unsigned int step)
{
  int size_ = static_cast<int>(size);
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < size_; ++i) {
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

    int i_step = i * step;
    rgb[i_step] = static_cast<unsigned char>(vpMath::round(hue * 255.0));
    rgb[++i_step] = static_cast<unsigned char>(vpMath::round(saturation * 255.0));
    rgb[++i_step] = static_cast<unsigned char>(vpMath::round(value * 255.0));
    if ((++i_step) == 3) { // alpha
      rgb[i_step] = vpRGBa::alpha_default;
    }
  }
}

/*!
 * Convert an HSV image to a RGB or RGBa image depending on the value of \e step.
 * \param[in] hue_ : Image hue channel. Range depends on `h_full` parameter.
 * \param[in] saturation_ : Image saturation channel in range [0,255].
 * \param[in] value_ : Image value channel in range [0,255].
 * \param[out] rgb : Pointer to the RGB (24-bit) or RGBa (32-bits) color image that should be allocated with a size of
 * width * height * step.
 * This array should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the output color image; 3 for an RGB image, 4 for an RGBA image.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 */
void vpImageConvert::HSV2RGB(const unsigned char *hue_, const unsigned char *saturation_, const unsigned char *value_,
                             unsigned char *rgb, unsigned int size, unsigned int step, bool h_full)
{
  float h_max;
  if (h_full) {
    h_max = 255.f;
  }
  else {
    h_max = 180.f;
  }
  int size_ = static_cast<int>(size);
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < size_; ++i) {
    float hue = hue_[i] / h_max;
    float saturation = saturation_[i] / 255.f;
    float value = value_[i] / 255.f;

    if (vpMath::equal(saturation, 0.f, std::numeric_limits<float>::epsilon())) {
      hue = value;
      saturation = value;
    }
    else {
      float h = hue * 6.f;
      float s = saturation;
      float v = value;

      if (vpMath::equal(h, 6.f, std::numeric_limits<float>::epsilon())) {
        h = 0.0f;
      }
      float f = h - static_cast<int>(h);
      float p = v * (1.0f - s);
      float q = v * (1.0f - (s * f));
      float t = v * (1.0f - (s * (1.0f - f)));

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

    int i_step = i * step;
    rgb[i_step] = static_cast<unsigned char>(hue * 255.f);
    rgb[++i_step] = static_cast<unsigned char>(saturation * 255.0f);
    rgb[++i_step] = static_cast<unsigned char>(value * 255.0f);
    if ((++i_step) == 3) { // alpha
      rgb[i_step] = vpRGBa::alpha_default;
    }
  }
}

/*!
 * Convert an RGB or RGBa color image depending on the value of \e step into an HSV image.
 * \param[in] rgb : Pointer to the RGB (24-bits) or RGBa (32-bits) color image that should be allocated with a size of
 * width * height * step.
 * \param[out] hue : Converted hue channel with values in range [0, 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] saturation : Converted saturation channel with values in range [0, 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] value : Converted value channel with values in range [0, 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the input color image; 3 for an RGB image, 4 for an RGBa image.
 */
void vpImageConvert::RGB2HSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
                             unsigned int size, unsigned int step)
{
  int size_ = static_cast<int>(size);
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < size_; ++i) {
    double red, green, blue;
    double h, s, v;
    double min, max;
    int i_ = i * step;

    red = rgb[i_] / 255.0;
    green = rgb[++i_] / 255.0;
    blue = rgb[++i_] / 255.0;

    if (red > green) {
      max = std::max<double>(red, blue);
      min = std::min<double>(green, blue);
    }
    else {
      max = std::max<double>(green, blue);
      min = std::min<double>(red, blue);
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

      if (vpMath::equal(red, max, std::numeric_limits<double>::epsilon())) {
        h = (green - blue) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<double>::epsilon())) {
        h = 2 + ((blue - red) / delta);
      }
      else {
        h = 4 + ((red - green) / delta);
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
 * Convert an RGB or RGBa color image depending on the value of \e step into an HSV image.
 * \param[in] rgb : Pointer to the RGB (24-bit) or RGBa (32-bits) color image that should be allocated with a size of
 * width * height * step.
 * \param[out] hue : Converted hue channel. Range depends on `h_full` parameter.
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] saturation : Converted saturation channel with values in range [0, 255].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] value : Converted value channel with values in range [0, 255].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] step : Number of channels of the input color image; 3 for an RGB image, 4 for an RGBA image.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 */
void vpImageConvert::RGB2HSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation, unsigned char *value,
                             unsigned int size, unsigned int step, bool h_full)
{
  int size_ = static_cast<int>(size);
  std::vector<float> h_scale(4);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  if (h_full) {
    h_scale[index_0] = 42.5f;
    h_scale[index_1] = 85.f;
    h_scale[index_2] = 170.f;
    h_scale[index_3] = 255.f;
  }
  else {
    h_scale[index_0] = 30.f;
    h_scale[index_1] = 60.f;
    h_scale[index_2] = 120.f;
    h_scale[index_3] = 180.f;
  }
#if defined(_OPENMP)
#pragma omp parallel for
#endif
  for (int i = 0; i < size_; ++i) {
    float red, green, blue;
    float h, s, v;
    float min, max;
    unsigned int i_ = i * step;

    red = rgb[i_];
    green = rgb[++i_];
    blue = rgb[++i_];

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
      s = (255.f * (max - min)) / max;
    }
    else {
      s = 0.f;
    }

    if (vpMath::equal(s, 0.f, std::numeric_limits<float>::epsilon())) {
      h = 0.f;
    }
    else {
      float delta = max - min;

      if (vpMath::equal(red, max, std::numeric_limits<float>::epsilon())) {
        h = (h_scale[index_0] * (green - blue)) / delta;
      }
      else if (vpMath::equal(green, max, std::numeric_limits<float>::epsilon())) {
        h = h_scale[index_1] + ((h_scale[index_0] * (blue - red)) / delta);
      }
      else {
        h = h_scale[index_2] + ((h_scale[index_0] * (red - green)) / delta);
      }

      if (h < 0.f) {
        h += h_scale[index_3];
      }
    }

    hue[i] = static_cast<unsigned char>(h);
    saturation[i] = static_cast<unsigned char>(s);
    value[i] = static_cast<unsigned char>(v);
  }
}

/*!
 * Converts an array of hue, saturation and value (HSV) to an array of RGBa values.
 *
 * Alpha component of the converted image is set to vpRGBa::alpha_default.
 *
 * \param[in] hue : Array of hue values in range [0,1].
 * \param[in] saturation : Array of saturation values in range [0,1].
 * \param[in] value : Array of value values in range [0,1].
 * \param[out] rgba : Pointer to the 32-bit RGBa image that should
 * be allocated prior to calling this function with a size of width * height * 4.
 * Alpha channel is here set to vpRGBa::alpha_default.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 */
void vpImageConvert::HSVToRGBa(const double *hue, const double *saturation, const double *value, unsigned char *rgba,
                               unsigned int size)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgba, size, 4);
}

/*!
 * Converts an array of hue, saturation and value (HSV) to an array of RGBa values.
 *
 * Alpha component of the converted image is set to vpRGBa::alpha_default.
 *
 * \param[in] hue : Array of hue values. Range depends on `h_full` parameter.
 * \param[in] saturation : Array of saturation values in range [0,255].
 * \param[in] value : Array of value values in range [0,255].
 * \param[out] rgba : Pointer to the 32-bit RGBa image that should
 * be allocated prior to calling this function with a size of width * height * 4. Alpha channel is here set
 * to vpRGBa::alpha_default.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 */
void vpImageConvert::HSVToRGBa(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                               unsigned char *rgba, unsigned int size, bool h_full)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgba, size, 4, h_full);
}

/*!
 * Converts an array of RGBa to an array of hue, saturation, value (HSV) values.
 * The alpha channel is not used.
 *
 * \param[in] rgba : Pointer to the 32-bits RGBa bitmap.
 * \param[out] hue : Array of hue values converted from RGB color space in range [0 - 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] saturation : Array of saturation values converted from RGB color space in range [0 - 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] value : Array of value values converted from RGB color space in range [0 - 1].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 */
void vpImageConvert::RGBaToHSV(const unsigned char *rgba, double *hue, double *saturation, double *value,
                               unsigned int size)
{
  vpImageConvert::RGB2HSV(rgba, hue, saturation, value, size, 4);
}

/*!
 * Converts an array of RGBa to an array of hue, saturation, value (HSV) values.
 * The alpha channel is not used.
 *
 * \param[in] rgba : Pointer to the 32-bits RGBA bitmap that has a dimension of `size * 4`.
 * \param[out] hue : Array of hue values converted from RGB color space. Range depends on `h_full` parameter.
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] saturation : Array of saturation values converted from RGB color space in range [0 - 255].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[out] value : Array of value values converted from RGB color space in range [0 - 255].
 * This array of dimension `size` should be allocated prior to calling this function.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 *
 * \sa vpImageTools::inRange()
 */
void vpImageConvert::RGBaToHSV(const unsigned char *rgba, unsigned char *hue, unsigned char *saturation,
                               unsigned char *value, unsigned int size, bool h_full)
{
  vpImageConvert::RGB2HSV(rgba, hue, saturation, value, size, 4, h_full);
}

/*!
 * Converts an array of hue, saturation and value to an array of RGB values.
 *
 * \param[in] hue : Array of hue values in range [0,1].
 * The dimension of this array corresponds to `size` parameter.
 * \param[in] saturation : Array of saturation values in range [0,1].
 * The dimension of this array corresponds to `size` parameter.
 * \param[in] value : Array of value values in range [0,1].
 * The dimension of this array corresponds to `size` parameter.
 * \param[out] rgb : Pointer to the 24-bit RGB image that should be allocated prior to calling this function
 * with a size of `width * height * 3` where `width * height` corresponds to `size` parameter.
 * \param[in] size : The image size or the number of pixels corresponding to the image `width * height`.
 */
void vpImageConvert::HSVToRGB(const double *hue, const double *saturation, const double *value, unsigned char *rgb,
                              unsigned int size)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgb, size, 3);
}

/*!
 * Converts an array of hue, saturation and value to an array of RGB values.
 *
 * \param[in] hue : Array of hue values. Range depends on `h_full` parameter.
 * The dimension of this array corresponds to `size` parameter.
 * \param[in] saturation : Array of saturation values in range [0,255].
 * The dimension of this array corresponds to `size` parameter.
 * \param[in] value : Array of value values  in range [0,255].
 * The dimension of this array corresponds to `size` parameter.
 * \param[out] rgb : Pointer to the 24-bit RGB image that should be allocated prior to calling this function
 * with a size of `width * height * 3` where `width * height` corresponds to `size` parameter.
 * \param[in] size : The image size or the number of pixels corresponding to the image `width * height`.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 */
void vpImageConvert::HSVToRGB(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                              unsigned char *rgb, unsigned int size, bool h_full)
{
  vpImageConvert::HSV2RGB(hue, saturation, value, rgb, size, 3, h_full);
}

/*!
 * Converts an array of RGB to an array of hue, saturation, value values.
 *
 * \param[in] rgb : Pointer to the 24-bits RGB bitmap. Its size corresponds to `size * 3`.
 * \param[out] hue : Array of hue values in range [0,1] converted from RGB color space.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[out] saturation : Array of saturation values in range [0,1]  converted from RGB color space.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[out] value : Array of value values in range [0,1] converted from RGB color space.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 */
void vpImageConvert::RGBToHSV(const unsigned char *rgb, double *hue, double *saturation, double *value,
                              unsigned int size)
{
  vpImageConvert::RGB2HSV(rgb, hue, saturation, value, size, 3);
}

/*!
 * Converts an array of RGB to an array of hue, saturation, value values.
 *
 * \param[in] rgb : Pointer to the 24-bits RGB bitmap. Its size corresponds to `size * 3`.
 * \param[out] hue : Array of hue values converted from RGB color space. Range depends on `h_full` parameter.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[out] saturation : Array of saturation values in range [0,255] converted from RGB color space.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[out] value : Array of value values in range [0,255] converted from RGB color space.
 * This array should be allocated prior to calling this function. Its size corresponds to `size` parameter.
 * \param[in] size : The image size or the number of pixels corresponding to the image width * height.
 * \param[in] h_full : When true, hue range is in [0, 255]. When false, hue range is in [0, 180].
 *
 * \sa vpImageTools::inRange()
 */
void vpImageConvert::RGBToHSV(const unsigned char *rgb, unsigned char *hue, unsigned char *saturation,
                              unsigned char *value, unsigned int size, bool h_full)
{
  vpImageConvert::RGB2HSV(rgb, hue, saturation, value, size, 3, h_full);
}
END_VISP_NAMESPACE
