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
 * Image handling.
 */

#ifndef VP_IMAGE_GETTERS_H
#define VP_IMAGE_GETTERS_H

// Warning: this file shouldn't be included by the user. Internal usage only to reduce length of vpImage.h

/*!
  Retrieves pixel value from an image containing values of type `Type`.

  Gets the value of a sub-pixel with coordinates (i,j).

  \param i : Pixel coordinate along the rows.
  \param j : Pixel coordinate along the columns.

  \return Pixel value.

  \exception vpImageException::notInTheImage : If (i,j) is out of the image.
*/
template <class Type> inline Type vpImage<Type>::getValue(unsigned int i, unsigned int j) const
{
  if ((i >= height) || (j >= width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside the image"));
  }

  return row[i][j];
}

/*!
  Retrieves pixel value from an image containing values of type `Type` with
  sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation.

  See also vpImageTools::interpolate() for a similar result, but with a choice of the interpolation method.

  \param i : Sub-pixel coordinate along the rows.
  \param j : Sub-pixel coordinate along the columns.

  \return Interpolated sub-pixel value from the four neighbours.

  \exception vpImageException::notInTheImage : If (i,j) is out of the image.
*/
template <class Type> Type vpImage<Type>::getValue(double i, double j) const
{
  if ((i < 0) || (j < 0) || ((i + 1) > height) || ((j + 1) > width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if ((height * width) == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = std::min<unsigned int>(height - 1, iround + 1);
  unsigned int jround_1 = std::min<unsigned int>(width - 1, jround + 1);

  double value =
    (((static_cast<double>(row[iround][jround]) * rfrac) + (static_cast<double>(row[iround_1][jround]) * rratio)) * cfrac) +
    (((static_cast<double>(row[iround][jround_1]) * rfrac) + (static_cast<double>(row[iround_1][jround_1]) * rratio)) *
    cratio);

  return static_cast<Type>(vpMath::round(value));
}

/*!
  \relates vpImage
*/
template <> inline double vpImage<double>::getValue(double i, double j) const
{
  if ((i < 0) || (j < 0) || ((i + 1) > height) || ((j + 1) > width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if ((height * width) == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = std::min<unsigned int>(height - 1, iround + 1);
  unsigned int jround_1 = std::min<unsigned int>(width - 1, jround + 1);

  return (((row[iround][jround] * rfrac) + (row[iround_1][jround] * rratio)) * cfrac) +
    (((row[iround][jround_1] * rfrac) + (row[iround_1][jround_1] * rratio)) * cratio);
}

/*!
  \relates vpImage
 */
template <> inline unsigned char vpImage<unsigned char>::getValue(double i, double j) const
{
  if ((i < 0) || (j < 0) || ((i + 1) > height) || ((j + 1) > width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if ((height * width) == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  // alpha architecture is bi-endianness. The following optimization makes testImageGetValue failing
#if (defined(VISP_LITTLE_ENDIAN) || defined(VISP_BIG_ENDIAN)) && !(defined(__alpha__) || defined(_M_ALPHA))
  // Fixed-point arithmetic
  const uint32_t magic_8 = 8;
  const uint32_t magic_16 = 16;
  const uint32_t magic_32 = 32;
  const uint32_t magic_0x00FF = 0x00FF;
  const uint32_t precision = 1U << magic_16;
  uint64_t y = static_cast<uint64_t>(i * precision);
  uint64_t x = static_cast<uint64_t>(j * precision);

  uint64_t iround = y & (~0xFFFFU);
  uint64_t jround = x & (~0xFFFFU);

  uint64_t rratio = y - iround;
  uint64_t cratio = x - jround;

  uint64_t rfrac = precision - rratio;
  uint64_t cfrac = precision - cratio;

  uint64_t x_ = x >> magic_16;
  uint64_t y_ = y >> magic_16;

  if (((y_ + 1) < height) && ((x_ + 1) < width)) {
    uint16_t up = vpEndian::reinterpret_cast_uchar_to_uint16_LE(bitmap + (y_ * width) + x_);
    uint16_t down = vpEndian::reinterpret_cast_uchar_to_uint16_LE(bitmap + ((y_ + 1) * width) + x_);

    return static_cast<unsigned char>((((((up & magic_0x00FF) * rfrac) + ((down & magic_0x00FF) * rratio)) * cfrac)
                                       + (((up >> magic_8) * rfrac) + ((down >> magic_8) * rratio)) * cratio) >> magic_32);
  }
  else if ((y_ + 1) < height) {
    return static_cast<unsigned char>(((row[y_][x_] * rfrac) + (row[y_ + 1][x_] * rratio)) >> magic_16);
  }
  else if ((x_ + 1) < width) {
    uint16_t up = vpEndian::reinterpret_cast_uchar_to_uint16_LE(bitmap + (y_ * width) + x_);
    return static_cast<unsigned char>((((up & magic_0x00FF) * cfrac) + ((up >> magic_8) * cratio)) >> magic_16);
  }
  else {
    return row[y_][x_];
  }
#else
  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  if (iround >= height || jround >= width) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside the image"));
  }

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = std::min<unsigned int>(height - 1, iround + 1);
  unsigned int jround_1 = std::min<unsigned int>(width - 1, jround + 1);

  double value =
    (static_cast<double>(row[iround][jround]) * rfrac + static_cast<double>(row[iround_1][jround]) * rratio) * cfrac +
    (static_cast<double>(row[iround][jround_1]) * rfrac + static_cast<double>(row[iround_1][jround_1]) * rratio) *
    cratio;
  return static_cast<unsigned char>(vpMath::round(value));
#endif
}

/*!
  \relates vpImage
 */
template <> inline vpRGBa vpImage<vpRGBa>::getValue(double i, double j) const
{
  if ((i < 0) || (j < 0) || ((i + 1) > height) || ((j + 1) > width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if ((height * width) == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = std::min<unsigned int>(height - 1, iround + 1);
  unsigned int jround_1 = std::min<unsigned int>(width - 1, jround + 1);

  double valueR =
    (((static_cast<double>(row[iround][jround].R) * rfrac) + (static_cast<double>(row[iround_1][jround].R) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].R) * rfrac) + (static_cast<double>(row[iround_1][jround_1].R) * rratio)) *
    cratio);
  double valueG =
    (((static_cast<double>(row[iround][jround].G) * rfrac) + (static_cast<double>(row[iround_1][jround].G) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].G) * rfrac) + (static_cast<double>(row[iround_1][jround_1].G) * rratio)) *
    cratio);
  double valueB =
    (((static_cast<double>(row[iround][jround].B) * rfrac) + (static_cast<double>(row[iround_1][jround].B) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].B) * rfrac) + (static_cast<double>(row[iround_1][jround_1].B) * rratio)) *
    cratio);

  return vpRGBa(static_cast<unsigned char>(vpMath::round(valueR)), static_cast<unsigned char>(vpMath::round(valueG)),
                static_cast<unsigned char>(vpMath::round(valueB)));
}

/*!
  \relates vpImage
 */
template <> inline vpRGBf vpImage<vpRGBf>::getValue(double i, double j) const
{
  if ((i < 0) || (j < 0) || ((i + 1) > height) || ((j + 1) > width)) {
    throw(vpException(vpImageException::notInTheImage, "Pixel outside of the image"));
  }
  if ((height * width) == 0) {
    throw vpException(vpImageException::notInitializedError, "Empty image!");
  }

  unsigned int iround = static_cast<unsigned int>(floor(i));
  unsigned int jround = static_cast<unsigned int>(floor(j));

  double rratio = i - static_cast<double>(iround);
  double cratio = j - static_cast<double>(jround);

  double rfrac = 1.0 - rratio;
  double cfrac = 1.0 - cratio;

  unsigned int iround_1 = std::min<unsigned int>(height - 1, iround + 1);
  unsigned int jround_1 = std::min<unsigned int>(width - 1, jround + 1);

  double valueR =
    (((static_cast<double>(row[iround][jround].R) * rfrac) + (static_cast<double>(row[iround_1][jround].R) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].R) * rfrac) + (static_cast<double>(row[iround_1][jround_1].R) * rratio)) *
    cratio);
  double valueG =
    (((static_cast<double>(row[iround][jround].G) * rfrac) + (static_cast<double>(row[iround_1][jround].G) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].G) * rfrac) + (static_cast<double>(row[iround_1][jround_1].G) * rratio)) *
    cratio);
  double valueB =
    (((static_cast<double>(row[iround][jround].B) * rfrac) + (static_cast<double>(row[iround_1][jround].B) * rratio)) *
    cfrac) +
    (((static_cast<double>(row[iround][jround_1].B) * rfrac) + (static_cast<double>(row[iround_1][jround_1].B) * rratio)) *
    cratio);

  return vpRGBf(static_cast<float>(valueR), static_cast<float>(valueG), static_cast<float>(valueB));
}

/*!
  Retrieves pixel value from an image containing values of type \e Type with
  sub-pixel accuracy.

  Gets the value of a sub-pixel with coordinates (i,j) with bilinear
  interpolation.

  See also vpImageTools::interpolate() for a similar result, but with a choice of the interpolation method.

  \param ip : Sub-pixel coordinates of a point in the image.

  \return Interpolated sub-pixel value from the four neighbors.

  \exception vpImageException::notInTheImage : If the image point \e ip is out
  of the image.
*/
template <class Type> inline Type vpImage<Type>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

/*!
  \relates vpImage
 */
template <> inline double vpImage<double>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

/*!
  \relates vpImage
 */
template <> inline unsigned char vpImage<unsigned char>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

/*!
  \relates vpImage
 */
template <> inline vpRGBa vpImage<vpRGBa>::getValue(const vpImagePoint &ip) const
{
  return getValue(ip.get_i(), ip.get_j());
}

/*!
 * \brief Return the maximum value within the bitmap
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMinValue()
 */
template <class Type> Type vpImage<Type>::getMaxValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute maximum value of an empty image"));
  }
  Type m = bitmap[0];
  for (unsigned int i = 0; i < npixels; ++i) {
    if (bitmap[i] > m) {
      m = bitmap[i];
    }
  }
  (void)onlyFiniteVal;
  return m;
}

/*!
 * \relates vpImage
 * \brief Return the maximum value within the double bitmap.
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMinValue()
 */
template <> inline double vpImage<double>::getMaxValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute maximum value of an empty image"));
  }
  double m = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if ((bitmap[i] > m) && (vpMath::isFinite(bitmap[i]))) {
        m = bitmap[i];
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] > m) {
        m = bitmap[i];
      }
    }
  }
  return m;
}

/*!
 * \relates vpImage
 * \brief Return the maximum value within the float bitmap.
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMinValue()
 */
template <> inline float vpImage<float>::getMaxValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute maximum value of an empty image"));
  }
  float m = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if ((bitmap[i] > m) && (vpMath::isFinite(bitmap[i]))) {
        m = bitmap[i];
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] > m) {
        m = bitmap[i];
      }
    }
  }
  return m;
}

/*!
 * \brief Return the minimum value within the bitmap
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMaxValue()
 */
template <class Type> Type vpImage<Type>::getMinValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute minimum value of an empty image"));
  }
  Type m = bitmap[0];
  for (unsigned int i = 0; i < npixels; ++i) {
    if (bitmap[i] < m) {
      m = bitmap[i];
    }
  }
  (void)onlyFiniteVal;
  return m;
}

/*!
 * \relates vpImage
 * \brief Return the minimum value within the double bitmap.
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMaxValue()
 */
template <> inline double vpImage<double>::getMinValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute minimum value of an empty image"));
  }
  double m = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if ((bitmap[i] < m) && (vpMath::isFinite(bitmap[i]))) {
        m = bitmap[i];
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] < m) {
        m = bitmap[i];
      }
    }
  }
  return m;
}

/*!
 * \relates vpImage
 * \brief Return the minimum value within the float bitmap.
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 * If true, consider only finite values.
 *
 * \sa getMaxValue()
 */
template <> inline float vpImage<float>::getMinValue(bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot compute minimum value of an empty image"));
  }
  float m = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if ((bitmap[i] < m) && (vpMath::isFinite(bitmap[i]))) {
        m = bitmap[i];
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] < m) {
        m = bitmap[i];
      }
    }
  }
  return m;
}

/*!
 * \brief Look for the minimum and the maximum value within the bitmap
 * \param min : The minimal value within the bitmap.
 * \param max : The maximal value within the bitmap.
 * \param onlyFiniteVal : This parameter is ignored for non double or non float bitmap.
 *
 * \sa getMaxValue()
 * \sa getMinValue()
 * \sa getMinMaxLoc()
 */
template <class Type> void vpImage<Type>::getMinMaxValue(Type &min, Type &max, bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot get minimum/maximum values of an empty image"));
  }

  min = bitmap[0];
  max = bitmap[0];
  for (unsigned int i = 0; i < npixels; ++i) {
    if (bitmap[i] < min) {
      min = bitmap[i];
    }
    if (bitmap[i] > max) {
      max = bitmap[i];
    }
  }
  (void)onlyFiniteVal;
}

/*!
 * \relates vpImage
 * \brief Look for the minimum and the maximum value within the double bitmap
 * \param min : The minimal value within the bitmap.
 * \param max : The maximal value within the bitmap.
 * \param onlyFiniteVal : If true, consider only finite values.
 *
 * \sa getMaxValue()
 * \sa getMinValue()
 * \sa getMinMaxLoc()
 */
template <> inline void vpImage<double>::getMinMaxValue(double &min, double &max, bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot get minimum/maximum values of an empty image"));
  }

  min = bitmap[0];
  max = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (vpMath::isFinite(bitmap[i])) {
        if (bitmap[i] < min) {
          min = bitmap[i];
        }
        if (bitmap[i] > max) {
          max = bitmap[i];
        }
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] < min) {
        min = bitmap[i];
      }
      if (bitmap[i] > max) {
        max = bitmap[i];
      }
    }
  }
}

/*!
 * \relates vpImage
 * \brief Look for the minimum and the maximum value within the float bitmap
 * \param min : The minimal value within the bitmap.
 * \param max : The maximal value within the bitmap.
 * \param onlyFiniteVal : If true, consider only finite values.
 *
 * \sa getMaxValue()
 * \sa getMinValue()
 * \sa getMinMaxLoc()
 */
template <> inline void vpImage<float>::getMinMaxValue(float &min, float &max, bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot get minimum/maximum values of an empty image"));
  }

  min = bitmap[0];
  max = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (vpMath::isFinite(bitmap[i])) {
        if (bitmap[i] < min) {
          min = bitmap[i];
        }
        if (bitmap[i] > max) {
          max = bitmap[i];
        }
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i] < min) {
        min = bitmap[i];
      }
      if (bitmap[i] > max) {
        max = bitmap[i];
      }
    }
  }
}

/*!
  \brief Look for the minimum and the maximum value within the 3-channels float bitmap
  \param min : The minimal values within the bitmap.
  \param max : The maximal values within the bitmap.
  \param onlyFiniteVal : If true, consider only finite values.

  \sa getMaxValue()
  \sa getMinValue()
  \sa getMinMaxLoc()
*/
template <> inline void vpImage<vpRGBf>::getMinMaxValue(vpRGBf &min, vpRGBf &max, bool onlyFiniteVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot get minimum/maximum values of an empty image"));
  }

  min = bitmap[0];
  max = bitmap[0];
  if (onlyFiniteVal) {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (vpMath::isFinite(bitmap[i].R)) {
        if (bitmap[i].R < min.R) {
          min.R = bitmap[i].R;
        }
        if (bitmap[i].R > max.R) {
          max.R = bitmap[i].R;
        }
      }
      if (vpMath::isFinite(bitmap[i].G)) {
        if (bitmap[i].G < min.G) {
          min.G = bitmap[i].G;
        }
        if (bitmap[i].G > max.G) {
          max.G = bitmap[i].G;
        }
      }
      if (vpMath::isFinite(bitmap[i].B)) {
        if (bitmap[i].B < min.B) {
          min.B = bitmap[i].B;
        }
        if (bitmap[i].B > max.B) {
          max.B = bitmap[i].B;
        }
      }
    }
  }
  else {
    for (unsigned int i = 0; i < npixels; ++i) {
      if (bitmap[i].R < min.R) {
        min.R = bitmap[i].R;
      }
      if (bitmap[i].R > max.R) {
        max.R = bitmap[i].R;
      }

      if (bitmap[i].G < min.G) {
        min.G = bitmap[i].G;
      }
      if (bitmap[i].G > max.G) {
        max.G = bitmap[i].G;
      }

      if (bitmap[i].B < min.B) {
        min.B = bitmap[i].B;
      }
      if (bitmap[i].B > max.B) {
        max.B = bitmap[i].B;
      }
    }
  }
}

/*!
  \brief Get the position of the minimum and/or the maximum pixel value within the bitmap and
  the corresponding value.
  Following code allows retrieving only minimum value and position:
  \code
  vpImage<double> I(h, w);
  //[...] Fill I
  vpImagePoint min_loc;
  double min_val = 0.0;
  I.getMinMaxLoc(&min_loc, nullptr, &min_val, nullptr);
  \endcode

  \param minLoc : Position of the pixel with minimum value if not nullptr.
  \param maxLoc : Position of the pixel with maximum value if not nullptr.
  \param minVal : Minimum pixel value if not nullptr.
  \param maxVal : Maximum pixel value if not nullptr.

  \sa getMaxValue()
  \sa getMinValue()
  \sa getMinMaxValue()
*/
template <class Type>
void vpImage<Type>::getMinMaxLoc(vpImagePoint *minLoc, vpImagePoint *maxLoc, Type *minVal, Type *maxVal) const
{
  if (npixels == 0) {
    throw(vpException(vpException::fatalError, "Cannot get location of minimum/maximum "
                      "values of an empty image"));
  }

  Type min = bitmap[0], max = bitmap[0];
  vpImagePoint minLoc_, maxLoc_;
  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      if (row[i][j] < min) {
        min = row[i][j];
        minLoc_.set_ij(i, j);
      }

      if (row[i][j] > max) {
        max = row[i][j];
        maxLoc_.set_ij(i, j);
      }
    }
  }

  if (minLoc != nullptr) {
    *minLoc = minLoc_;
  }

  if (maxLoc != nullptr) {
    *maxLoc = maxLoc_;
  }

  if (minVal != nullptr) {
    *minVal = min;
  }

  if (maxVal != nullptr) {
    *maxVal = max;
  }
}

/*!
 * \brief Return the mean value of the bitmap.
 *
 * For vpRGBa and vpRGBf image types, the sum of image intensities is computed by (R+G+B).
 *
 * \param[in] p_mask Optional parameter. If not set to nullptr, a boolean mask that indicates which points must be
 * considered, if set to true.
 * \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
 * valid according to the boolean mask or image size when `p_mask` is set to nullptr.
 */
template <class Type> double vpImage<Type>::getMeanValue(const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  unsigned int nbPointsInMask = 0;
  double sum = getSum(p_mask, &nbPointsInMask);
  if (nbPointsInMask == 0) {
    throw(vpException(vpException::divideByZeroError, "Division by zero in vpImage::getMeanValue()"));
  }
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return sum / nbPointsInMask;
}

/*!
* \brief Return the standard deviation of the bitmap
*
* - For a vpRGBa or a vpRGBf image, we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c].R + I[r][c].G + I[r][c].B - \mu)^2}\f]
* - For a unary type image (unsigned char, float, double), we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c] - \mu)^2}\f]
*
* where \f$ \mu \f$ is the mean of the image as computed by \b vpImage::getMeanValue() and \f$ \mbox{size} \f$
* is the number of pixels to consider in the mask.
*
* \param[in] p_mask A boolean mask that indicates which points must be considered, if set to true.
* \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
* valid according to the boolean mask or image size when `p_mask` is set to nullptr.
*/
template <class Type> double vpImage<Type>::getStdev(const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  double mean = getMeanValue(p_mask, nbValidPoints);
  return getStdev(mean, p_mask);
}

/*!
* \brief Return the standard deviation of the bitmap
*
* - For a vpRGBa or a vpRGBf image, we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c].R + I[r][c].G + I[r][c].B - \mu)^2}\f]
* - For a unary type image (unsigned char, float, double), we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c] - \mu)^2}\f]
*
* where \f$ \mu \f$ is the mean of the image as computed by \b vpImage::getMeanValue() and \f$ \mbox{size} \f$
* is the number of pixels to consider in the mask.
*
* \param[in] mean The mean of the image.
* \param[in] p_mask Optional parameter. When different from nullptr, a boolean mask that indicates which pixels must
* be considered, if set to true.
* \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
* valid according to the boolean mask or image size when `p_mask` is set to nullptr.
* \return double The standard deviation taking into account only the points for which the mask is true.
*/
template <class Type> double vpImage<Type>::getStdev(const double &mean, const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  const unsigned int size = width * height;
  double sum = 0.;
  unsigned int nbPointsInMask = 0;
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute standard deviation: image and mask size differ"));
    }
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        sum += (bitmap[i] - mean) * (bitmap[i] - mean);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < size; ++i) {
      sum += (bitmap[i] - mean) * (bitmap[i] - mean);
    }
    nbPointsInMask = size;
  }
  sum /= static_cast<double>(nbPointsInMask);
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return std::sqrt(sum);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
* \brief Return the standard deviation of the bitmap
*
* - For a vpRGBa or a vpRGBf image, we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c].R + I[r][c].G + I[r][c].B - \mu)^2}\f]
* - For a unary type image (unsigned char, float, double), we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c] - \mu)^2}\f]
*
* where \f$ \mu \f$ is the mean of the image as computed by \b vpImage::getMeanValue() and \f$ \mbox{size} \f$
* is the number of pixels to consider in the mask.
*
* \param[in] mean The mean of the image.
* \param[in] p_mask Optional parameter. When different from nullptr, a boolean mask that indicates which pixels must
* be considered, if set to true.
* \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
* valid according to the boolean mask or image size when `p_mask` is set to nullptr.
* \return double The standard deviation taking into account only the points for which the mask is true.
*/
template <> inline double vpImage<vpRGBa>::getStdev(const double &mean, const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  const unsigned int size = width * height;
  double sum = 0.;
  unsigned int nbPointsInMask = 0;
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute standard deviation: image and mask size differ"));
    }
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        double val = static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
        sum += (val - mean) * (val - mean);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < size; ++i) {
      double val = static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
      sum += (val - mean) * (val - mean);
    }
    nbPointsInMask = size;
  }
  sum /= static_cast<double>(nbPointsInMask);
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return std::sqrt(sum);
}

/*!
* \brief Return the standard deviation of the bitmap
*
* - For a vpRGBa or a vpRGBf image, we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c].R + I[r][c].G + I[r][c].B - \mu)^2}\f]
* - For a unary type image (unsigned char, float, double), we compute the standard deviation as follow:
*   \f[ stdev = \sqrt{\frac{1}{size} \sum_{r = 0}^{height-1} \sum_{c = 0}^{width-1} (I[r][c] - \mu)^2}\f]
*
* where \f$ \mu \f$ is the mean of the image as computed by \b vpImage::getMeanValue() and \f$ \mbox{size} \f$
* is the number of pixels to consider in the mask.
*
* \param[in] mean The mean of the image.
* \param[in] p_mask Optional parameter. When different from nullptr, a boolean mask that indicates which pixels must
* be considered, if set to true.
* \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
* valid according to the boolean mask or image size when `p_mask` is set to nullptr.
* \return double The standard deviation taking into account only the points for which the mask is true.
*/
template <> inline double vpImage<vpRGBf>::getStdev(const double &mean, const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  const unsigned int size = width * height;
  double sum = 0.;
  unsigned int nbPointsInMask = 0;
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute standard deviation: image and mask size differ"));
    }
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        double val = static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
        sum += (val - mean) * (val - mean);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < size; ++i) {
      double val = static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
      sum += (val - mean) * (val - mean);
    }
    nbPointsInMask = size;
  }
  sum /= static_cast<double>(nbPointsInMask);
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return std::sqrt(sum);
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

/**
 * \brief Compute the sum of image intensities.
 * - For unary image types (unsigned char, float, double), compute the sum of image intensities.
 * - For vpRGBa image type, compute the sum (R+G+B) of image intensities.
 * - For vpRGBf image type, compute the sum (R+G+B) of image intensities.
 *
 * \param[in] p_mask Optional parameter. If not set to nullptr, pointer to a boolean mask that indicates the valid
 * points by a true flag.
 * \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
 * valid according to the boolean mask or image size when `p_mask` is set to nullptr.
 */
template <class Type> inline double vpImage<Type>::getSum(const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    if (nbValidPoints) {
      *nbValidPoints = 0;
    }
    return 0.0;
  }
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute sum: image and mask size differ"));
    }
  }
  double res = 0.0;
  unsigned int nbPointsInMask = 0;
  unsigned int size = height * width;
  if (p_mask) {
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        res += static_cast<double>(bitmap[i]);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < size; ++i) {
      res += static_cast<double>(bitmap[i]);
    }
    nbPointsInMask = size;
  }
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }

  return res;
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * \brief Compute the sum of image intensities.
 * - For unary image types (unsigned char, float, double), compute the sum of image intensities.
 * - For vpRGBa image type, compute the sum (R+G+B) of image intensities.
 * - For vpRGBf image type, compute the sum (R+G+B) of image intensities.
 *
 * \param[in] p_mask Optional parameter. If not set to nullptr, pointer to a boolean mask that indicates the valid
 * points by a true flag.
 * \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
 * valid according to the boolean mask or image size when `p_mask` is set to nullptr.
 */
template <> inline double vpImage<vpRGBa>::getSum(const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  double res = 0.0;
  unsigned int nbPointsInMask = 0;
  unsigned int size = height * width;
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute sum: image and mask size differ"));
    }
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        res += static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < (height * width); ++i) {
      res += static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
    }
    nbPointsInMask = size;
  }
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return res;
}

/**
 * \brief Compute the sum of image intensities.
 * - For unary image types (unsigned char, float, double), compute the sum of image intensities.
 * - For vpRGBa image type, compute the sum (R+G+B) of image intensities.
 * - For vpRGBf image type, compute the sum (R+G+B) of image intensities.
 *
 * \param[in] p_mask Optional parameter. If not set to nullptr, pointer to a boolean mask that indicates the valid
 * points by a true flag.
 * \param[out] nbValidPoints Optional parameter. When different from nullptr contains the number of points that are
 * valid according to the boolean mask or image size when `p_mask` is set to nullptr.
 */
template <> inline double vpImage<vpRGBf>::getSum(const vpImage<bool> *p_mask, unsigned int *nbValidPoints) const
{
  if ((height == 0) || (width == 0)) {
    return 0.0;
  }
  double res = 0.0;
  unsigned int nbPointsInMask = 0;
  unsigned int size = height * width;
  if (p_mask) {
    if ((p_mask->getWidth() != width) || (p_mask->getHeight() != height)) {
      throw(vpException(vpException::fatalError, "Cannot compute sum: image and mask size differ"));
    }
    for (unsigned int i = 0; i < size; ++i) {
      if (p_mask->bitmap[i]) {
        res += static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
        ++nbPointsInMask;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < (height * width); ++i) {
      res += static_cast<double>(bitmap[i].R) + static_cast<double>(bitmap[i].G) + static_cast<double>(bitmap[i].B);
    }
    nbPointsInMask = size;
  }
  if (nbValidPoints) {
    *nbValidPoints = nbPointsInMask;
  }
  return res;
}
#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif
