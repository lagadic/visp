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

#ifndef VP_IMAGE_TOOLS_WARP_H
#define VP_IMAGE_TOOLS_WARP_H

// Warning: this file shouldn't be included by the user. Internal usage only to reduce length of vpImage.h
#include <visp3/core/vpImageTools.h>

/*!
  Apply a warping (affine or perspective) transformation to an image.

  \param src : Input image.
  \param T : Transformation / warping matrix, a `2x3` matrix for an affine transformation
  or a `3x3` matrix for a perspective transformation (homography).
  \param dst : Output image, if empty it will be of the same size than src and zero-initialized.
  \param interpolation : Interpolation method (only INTERPOLATION_NEAREST and INTERPOLATION_LINEAR
  are accepted, if INTERPOLATION_CUBIC is passed, INTERPOLATION_NEAREST will be used instead).
  \param fixedPointArithmetic : If true and if `pixelCenter` is false, fixed-point arithmetic is used if
  possible. Otherwise (e.g. the input image is too big) it fallbacks to the default implementation.
  \param pixelCenter : If true, pixel coordinates are at (0.5, 0.5), otherwise at (0,0). Fixed-point
  arithmetic cannot be used with `pixelCenter` option.
*/
template <class Type>
void vpImageTools::warpImage(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst,
                             const vpImageInterpolationType &interpolation, bool fixedPointArithmetic, bool pixelCenter)
{
  if (((T.getRows() != 2) && (T.getRows() != 3)) || (T.getCols() != 3)) {
    std::cerr << "Input transformation must be a (2x3) or (3x3) matrix." << std::endl;
    return;
  }

  if (src.getSize() == 0) {
    return;
  }

  const bool affine = (T.getRows() == 2);
  const bool interp_NN = (interpolation == INTERPOLATION_NEAREST) || (interpolation == INTERPOLATION_CUBIC);

  if (dst.getSize() == 0) {
    dst.resize(src.getHeight(), src.getWidth(), Type(0));
  }

  vpMatrix M = T;
  if (affine) {
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    double D = (M[index_0][index_0] * M[index_1][index_1]) - (M[index_0][index_1] * M[index_1][index_0]);
    D = !vpMath::nul(D, std::numeric_limits<double>::epsilon()) ? (1.0 / D) : 0;
    double A11 = M[index_1][index_1] * D, A22 = M[index_0][index_0] * D;
    M[index_0][index_0] = A11;
    M[index_0][index_1] *= -D;
    M[index_1][index_0] *= -D;
    M[index_1][index_1] = A22;
    double b1 = (-M[index_0][index_0] * M[index_0][index_2]) - (M[index_0][index_1] * M[index_1][index_2]);
    double b2 = (-M[index_1][index_0] * M[index_0][index_2]) - (M[index_1][index_1] * M[index_1][index_2]);
    M[index_0][index_2] = b1;
    M[index_1][index_2] = b2;
  }
  else {
    M = T.inverseByLU();
  }

  if (fixedPointArithmetic && (!pixelCenter)) {
    fixedPointArithmetic = checkFixedPoint(0, 0, M, affine) && checkFixedPoint(dst.getWidth() - 1, 0, M, affine) &&
      checkFixedPoint(0, dst.getHeight() - 1, M, affine) &&
      checkFixedPoint(dst.getWidth() - 1, dst.getHeight() - 1, M, affine);
  }

  if (interp_NN) {
    // nearest neighbor interpolation
    warpNN(src, M, dst, affine, pixelCenter, fixedPointArithmetic);
  }
  else {
    // bilinear interpolation
    warpLinear(src, M, dst, affine, pixelCenter, fixedPointArithmetic);
  }
}

template <class Type>
void vpImageTools::warpNN(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine,
                          bool centerCorner, bool fixedPoint)
{
  if (fixedPoint && (!centerCorner)) {
    const int nbits = 16;
    const int32_t precision = 1 << nbits;
    const float precision_1 = 1 / static_cast<float>(precision);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    int32_t a0_i32 = static_cast<int32_t>(T[index_0][index_0] * precision);
    int32_t a1_i32 = static_cast<int32_t>(T[index_0][index_1] * precision);
    int32_t a2_i32 = static_cast<int32_t>(T[index_0][index_2] * precision);
    int32_t a3_i32 = static_cast<int32_t>(T[index_1][index_0] * precision);
    int32_t a4_i32 = static_cast<int32_t>(T[index_1][index_1] * precision);
    int32_t a5_i32 = static_cast<int32_t>(T[index_1][index_2] * precision);
    int32_t a6_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[index_2][index_0] * precision) : 0;
    int32_t a7_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[index_2][index_1] * precision) : 0;
    int32_t a8_i32 = T.getRows() == 3 ? static_cast<int32_t>(T[index_2][index_2] * precision) : 1;

    int32_t height_1_i32 = static_cast<int32_t>((src.getHeight() - 1) * precision) + 0x8000;
    int32_t width_1_i32 = static_cast<int32_t>((src.getWidth() - 1) * precision) + 0x8000;

    if (affine) {
      unsigned int dst_height = dst.getHeight();
      unsigned int dst_width = dst.getWidth();
      for (unsigned int i = 0; i < dst_height; ++i) {
        int32_t xi = a2_i32;
        int32_t yi = a5_i32;

        for (unsigned int j = 0; j < dst_width; ++j) {
          if ((yi >= 0) && (yi < height_1_i32) && (xi >= 0) && (xi < width_1_i32)) {
            float x_ = (xi >> nbits) + ((xi & 0xFFFF) * precision_1);
            float y_ = (yi >> nbits) + ((yi & 0xFFFF) * precision_1);

            int x = vpMath::round(x_);
            int y = vpMath::round(y_);
            dst[i][j] = src[y][x];
          }

          xi += a0_i32;
          yi += a3_i32;
        }

        a2_i32 += a1_i32;
        a5_i32 += a4_i32;
      }
    }
    else {
      unsigned int dst_height = dst.getHeight();
      unsigned int dst_width = dst.getWidth();
      int src_height = static_cast<int>(src.getHeight());
      int src_width = static_cast<int>(src.getWidth());
      for (unsigned int i = 0; i < dst_height; ++i) {
        int64_t xi = a2_i32;
        int64_t yi = a5_i32;
        int64_t wi = a8_i32;

        for (unsigned int j = 0; j < dst_width; ++j) {
          bool cond_on_y = (yi >= 0) && (yi <= ((src_height - 1) * wi));
          bool cond_on_x = (xi >= 0) && (xi <= ((src_width - 1) * wi));
          if ((wi != 0) && cond_on_y && cond_on_x) {
            float w_ = (wi >> nbits) + ((wi & 0xFFFF) * precision_1);
            float x_ = ((xi >> nbits) + ((xi & 0xFFFF) * precision_1)) / w_;
            float y_ = ((yi >> nbits) + ((yi & 0xFFFF) * precision_1)) / w_;

            int x = vpMath::round(x_);
            int y = vpMath::round(y_);

            dst[i][j] = src[y][x];
          }

          xi += a0_i32;
          yi += a3_i32;
          wi += a6_i32;
        }

        a2_i32 += a1_i32;
        a5_i32 += a4_i32;
        a8_i32 += a7_i32;
      }
    }
  }
  else {
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    double a0 = T[index_0][index_0];
    double a1 = T[index_0][index_1];
    double a2 = T[index_0][index_2];
    double a3 = T[index_1][index_0];
    double a4 = T[index_1][index_1];
    double a5 = T[index_1][index_2];
    double a6 = affine ? 0.0 : T[index_2][index_0];
    double a7 = affine ? 0.0 : T[index_2][index_1];
    double a8 = affine ? 1.0 : T[index_2][index_2];

    unsigned int dst_height = dst.getHeight();
    unsigned int dst_width = dst.getWidth();
    for (unsigned int i = 0; i < dst_height; ++i) {
      for (unsigned int j = 0; j < dst_width; ++j) {
        double x = ((a0 * (centerCorner ? (j + 0.5) : j)) + (a1 * (centerCorner ? (i + 0.5) : i))) + a2;
        double y = ((a3 * (centerCorner ? (j + 0.5) : j)) + (a4 * (centerCorner ? (i + 0.5) : i))) + a5;
        double w = ((a6 * (centerCorner ? (j + 0.5) : j)) + (a7 * (centerCorner ? (i + 0.5) : i))) + a8;

        if (vpMath::nul(w, std::numeric_limits<double>::epsilon())) {
          w = 1.0;
        }

        int x_ = centerCorner ? coordCast(x / w) : vpMath::round(x / w);
        int y_ = centerCorner ? coordCast(y / w) : vpMath::round(y / w);

        if ((x_ >= 0) && (x_ < static_cast<int>(src.getWidth())) && (y_ >= 0) && (y_ < static_cast<int>(src.getHeight()))) {
          dst[i][j] = src[y_][x_];
        }
      }
    }
  }
}

template <class Type>
void vpImageTools::warpLinear(const vpImage<Type> &src, const vpMatrix &T, vpImage<Type> &dst, bool affine,
                              bool centerCorner, bool fixedPoint)
{
  if (fixedPoint && (!centerCorner)) {
    const int nbits = 16;
    const uint64_t precision = 1 << nbits;
    const float precision_1 = 1 / static_cast<float>(precision);
    const uint64_t precision2 = 1ULL << (2 * nbits);
    const float precision_2 = 1 / static_cast<float>(precision2);
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;

    int64_t a0_i64 = static_cast<int64_t>(T[index_0][index_0] * precision);
    int64_t a1_i64 = static_cast<int64_t>(T[index_0][index_1] * precision);
    int64_t a2_i64 = static_cast<int64_t>(T[index_0][index_2] * precision);
    int64_t a3_i64 = static_cast<int64_t>(T[index_1][index_0] * precision);
    int64_t a4_i64 = static_cast<int64_t>(T[index_1][index_1] * precision);
    int64_t a5_i64 = static_cast<int64_t>(T[index_1][index_2] * precision);
    int64_t a6_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[index_2][index_0] * precision) : 0;
    int64_t a7_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[index_2][index_1] * precision) : 0;
    int64_t a8_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[index_2][index_2] * precision) : 1;

    int64_t height_i64 = static_cast<int64_t>(src.getHeight() * precision);
    int64_t width_i64 = static_cast<int64_t>(src.getWidth() * precision);

    if (affine) {
      unsigned int dst_height = dst.getHeight();
      unsigned int dst_width = dst.getWidth();
      for (unsigned int i = 0; i < dst_height; ++i) {
        int64_t xi_ = a2_i64;
        int64_t yi_ = a5_i64;

        for (unsigned int j = 0; j < dst_width; ++j) {
          if ((yi_ >= 0) && (yi_ < height_i64) && (xi_ >= 0) && (xi_ < width_i64)) {
            const int64_t xi_lower = xi_ & (~0xFFFF);
            const int64_t yi_lower = yi_ & (~0xFFFF);

            const int64_t t = yi_ - yi_lower;
            const int64_t t_1 = precision - t;
            const int64_t s = xi_ - xi_lower;
            const int64_t s_1 = precision - s;

            const int x_ = static_cast<int>(xi_ >> nbits);
            const int y_ = static_cast<int>(yi_ >> nbits);

            if ((y_ < (static_cast<int>(src.getHeight()) - 1)) && (x_ < (static_cast<int>(src.getWidth()) - 1))) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_ + 1];
              const Type val10 = src[y_ + 1][x_];
              const Type val11 = src[y_ + 1][x_ + 1];
              const int64_t interp_i64 =
                static_cast<int64_t>(((s_1 * t_1) * val00) + ((s * t_1) * val01) + ((s_1 * t) * val10) + ((s * t) * val11));
              const float interp = (interp_i64 >> (nbits * 2)) + ((interp_i64 & 0xFFFFFFFFU) * precision_2);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else if (y_ < (static_cast<int>(src.getHeight()) - 1)) {
              const Type val00 = src[y_][x_];
              const Type val10 = src[y_ + 1][x_];
              const int64_t interp_i64 = static_cast<int64_t>((t_1 * val00) + (t * val10));
              const float interp = (interp_i64 >> nbits) + ((interp_i64 & 0xFFFF) * precision_1);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else if (x_ < (static_cast<int>(src.getWidth()) - 1)) {
              const Type val00 = src[y_][x_];
              const Type val01 = src[y_][x_ + 1];
              const int64_t interp_i64 = static_cast<int64_t>((s_1 * val00) + (s * val01));
              const float interp = (interp_i64 >> nbits) + ((interp_i64 & 0xFFFF) * precision_1);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi_ += a0_i64;
          yi_ += a3_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
      }
    }
    else {
      unsigned int dst_height = dst.getHeight();
      unsigned int dst_width = dst.getWidth();
      int src_height = static_cast<int>(src.getHeight());
      int src_width = static_cast<int>(src.getWidth());
      for (unsigned int i = 0; i < dst_height; ++i) {
        int64_t xi = a2_i64;
        int64_t yi = a5_i64;
        int64_t wi = a8_i64;

        for (unsigned int j = 0; j < dst_width; ++j) {
          bool cond_on_y = (yi >= 0) && (yi <= ((src_height - 1) * wi));
          bool cond_on_x = (xi >= 0) && (xi <= ((src_width - 1) * wi));
          if ((wi != 0) && cond_on_y && cond_on_x) {
            const float wi_ = (wi >> nbits) + ((wi & 0xFFFF) * precision_1);
            const float xi_ = ((xi >> nbits) + ((xi & 0xFFFF) * precision_1)) / wi_;
            const float yi_ = ((yi >> nbits) + ((yi & 0xFFFF) * precision_1)) / wi_;

            const int x_ = static_cast<int>(xi_);
            const int y_ = static_cast<int>(yi_);

            const float t = yi_ - y_;
            const float s = xi_ - x_;

            if ((y_ < (src_height - 1)) && (x_ < (src_width - 1))) {
              const float val00 = static_cast<float>(src[y_][x_]);
              const float val01 = static_cast<float>(src[y_][x_ + 1]);
              const float val10 = static_cast<float>(src[y_ + 1][x_]);
              const float val11 = static_cast<float>(src[y_ + 1][x_ + 1]);
              const float col0 = lerp(val00, val01, s);
              const float col1 = lerp(val10, val11, s);
              const float interp = lerp(col0, col1, t);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else if (y_ < (src_height - 1)) {
              const float val00 = static_cast<float>(src[y_][x_]);
              const float val10 = static_cast<float>(src[y_ + 1][x_]);
              const float interp = lerp(val00, val10, t);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else if (x_ < (src_width - 1)) {
              const float val00 = static_cast<float>(src[y_][x_]);
              const float val01 = static_cast<float>(src[y_][x_ + 1]);
              const float interp = lerp(val00, val01, s);
              dst[i][j] = vpMath::saturate<Type>(interp);
            }
            else {
              dst[i][j] = src[y_][x_];
            }
          }

          xi += a0_i64;
          yi += a3_i64;
          wi += a6_i64;
        }

        a2_i64 += a1_i64;
        a5_i64 += a4_i64;
        a8_i64 += a7_i64;
      }
    }
  }
  else {
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    double a0 = T[index_0][index_0];
    double a1 = T[index_0][index_1];
    double a2 = T[index_0][index_2];
    double a3 = T[index_1][index_0];
    double a4 = T[index_1][index_1];
    double a5 = T[index_1][index_2];
    double a6 = affine ? 0.0 : T[index_2][index_0];
    double a7 = affine ? 0.0 : T[index_2][index_1];
    double a8 = affine ? 1.0 : T[index_2][index_2];

    unsigned int dst_height = dst.getHeight();
    unsigned int dst_width = dst.getWidth();
    int src_height = static_cast<int>(src.getHeight());
    int src_width = static_cast<int>(src.getWidth());
    for (unsigned int i = 0; i < dst_height; ++i) {
      for (unsigned int j = 0; j < dst_width; ++j) {
        double x = (a0 * (centerCorner ? (j + 0.5) : j)) + (a1 * (centerCorner ? (i + 0.5) : i)) + a2;
        double y = (a3 * (centerCorner ? (j + 0.5) : j)) + (a4 * (centerCorner ? (i + 0.5) : i)) + a5;
        double w = (a6 * (centerCorner ? (j + 0.5) : j)) + (a7 * (centerCorner ? (i + 0.5) : i)) + a8;
        if (vpMath::nul(w, std::numeric_limits<double>::epsilon())) {
          w = 1;
        }

        x = (x / w) - (centerCorner ? 0.5 : 0);
        y = (y / w) - (centerCorner ? 0.5 : 0);

        int x_lower = static_cast<int>(x);
        int y_lower = static_cast<int>(y);
        bool stop_for_loop = false;
        if ((y_lower >= src_height) || (x_lower >= src_width) || (y < 0) || (x < 0)) {
          stop_for_loop = true;
        }
        if (!stop_for_loop) {
          double s = x - x_lower;
          double t = y - y_lower;

          if ((y_lower < (src_height - 1)) && (x_lower < (src_width - 1))) {
            const double val00 = static_cast<double>(src[y_lower][x_lower]);
            const double val01 = static_cast<double>(src[y_lower][x_lower + 1]);
            const double val10 = static_cast<double>(src[y_lower + 1][x_lower]);
            const double val11 = static_cast<double>(src[y_lower + 1][x_lower + 1]);
            const double col0 = lerp(val00, val01, s);
            const double col1 = lerp(val10, val11, s);
            const double interp = lerp(col0, col1, t);
            dst[i][j] = vpMath::saturate<Type>(interp);
          }
          else if (y_lower < (src_height - 1)) {
            const double val00 = static_cast<double>(src[y_lower][x_lower]);
            const double val10 = static_cast<double>(src[y_lower + 1][x_lower]);
            const double interp = lerp(val00, val10, t);
            dst[i][j] = vpMath::saturate<Type>(interp);
          }
          else if (x_lower < (src_width - 1)) {
            const double val00 = static_cast<double>(src[y_lower][x_lower]);
            const double val01 = static_cast<double>(src[y_lower][x_lower + 1]);
            const double interp = lerp(val00, val01, s);
            dst[i][j] = vpMath::saturate<Type>(interp);
          }
          else {
            dst[i][j] = src[y_lower][x_lower];
          }
        }
      }
    }
  }
}

inline void vpImageTools::warpLinearFixedPointNotCenter(const vpImage<vpRGBa> &src, const vpMatrix &T,
                                                        vpImage<vpRGBa> &dst, bool affine)
{
  const unsigned int index_0 = 0, index_1 = 1, index_2 = 2;
  const int nbits = 16;
  const int64_t precision = 1 << nbits;
  const float precision_1 = 1.f / static_cast<float>(precision);
  const int64_t precision2 = 1ULL << (2 * nbits);
  const float precision_2 = 1.f / static_cast<float>(precision2);

  int64_t a0_i64 = static_cast<int64_t>(T[index_0][index_0] * precision);
  int64_t a1_i64 = static_cast<int64_t>(T[index_0][index_1] * precision);
  int64_t a2_i64 = static_cast<int64_t>(T[index_0][index_2] * precision);
  int64_t a3_i64 = static_cast<int64_t>(T[index_1][index_0] * precision);
  int64_t a4_i64 = static_cast<int64_t>(T[index_1][index_1] * precision);
  int64_t a5_i64 = static_cast<int64_t>(T[index_1][index_2] * precision);
  int64_t a6_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[index_2][index_0] * precision) : 0;
  int64_t a7_i64 = T.getRows() == 3 ? static_cast<int64_t>(T[index_2][index_1] * precision) : 0;
  int64_t a8_i64 = precision;

  int64_t height_i64 = static_cast<int64_t>(src.getHeight() * precision);
  int64_t width_i64 = static_cast<int64_t>(src.getWidth() * precision);

  if (affine) {
    unsigned int dst_height = dst.getHeight();
    unsigned int dst_width = dst.getWidth();
    int src_height = static_cast<int>(src.getHeight());
    int src_width = static_cast<int>(src.getWidth());
    for (unsigned int i = 0; i < dst_height; ++i) {
      int64_t xi = a2_i64;
      int64_t yi = a5_i64;

      for (unsigned int j = 0; j < dst_width; ++j) {
        if ((yi >= 0) && (yi < height_i64) && (xi >= 0) && (xi < width_i64)) {
          const int64_t xi_lower = xi & (~0xFFFF);
          const int64_t yi_lower = yi & (~0xFFFF);

          const int64_t t = yi - yi_lower;
          const int64_t t_1 = precision - t;
          const int64_t s = xi - xi_lower;
          const int64_t s_1 = precision - s;

          const int x_ = static_cast<int>(xi >> nbits);
          const int y_ = static_cast<int>(yi >> nbits);

          if ((y_ < (src_height - 1)) && (x_ < (src_width - 1))) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val01 = src[y_][x_ + 1];
            const vpRGBa val10 = src[y_ + 1][x_];
            const vpRGBa val11 = src[y_ + 1][x_ + 1];
            const int64_t interpR_i64 =
              static_cast<int64_t>((s_1 * t_1 * val00.R) + (s * t_1 * val01.R) + (s_1 * t * val10.R) + (s * t * val11.R));
            const float interpR = (interpR_i64 >> (nbits * 2)) + ((interpR_i64 & 0xFFFFFFFFU) * precision_2);

            const int64_t interpG_i64 =
              static_cast<int64_t>((s_1 * t_1 * val00.G) + (s * t_1 * val01.G) + (s_1 * t * val10.G) + (s * t * val11.G));
            const float interpG = (interpG_i64 >> (nbits * 2)) + ((interpG_i64 & 0xFFFFFFFFU) * precision_2);

            const int64_t interpB_i64 =
              static_cast<int64_t>((s_1 * t_1 * val00.B) + (s * t_1 * val01.B) + (s_1 * t * val10.B) + (s * t * val11.B));
            const float interpB = (interpB_i64 >> (nbits * 2)) + ((interpB_i64 & 0xFFFFFFFFU) * precision_2);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (y_ < (src_height - 1)) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val10 = src[y_ + 1][x_];
            const int64_t interpR_i64 = static_cast<int64_t>(t_1 * val00.R + t * val10.R);
            const float interpR = (interpR_i64 >> nbits) + ((interpR_i64 & 0xFFFF) * precision_1);

            const int64_t interpG_i64 = static_cast<int64_t>((t_1 * val00.G) + (t * val10.G));
            const float interpG = (interpG_i64 >> nbits) + ((interpG_i64 & 0xFFFF) * precision_1);

            const int64_t interpB_i64 = static_cast<int64_t>((t_1 * val00.B) + (t * val10.B));
            const float interpB = (interpB_i64 >> nbits) + ((interpB_i64 & 0xFFFF) * precision_1);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (x_ < (src_width - 1)) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val01 = src[y_][x_ + 1];
            const int64_t interpR_i64 = static_cast<int64_t>((s_1 * val00.R) + (s * val01.R));
            const float interpR = (interpR_i64 >> nbits) + ((interpR_i64 & 0xFFFF) * precision_1);

            const int64_t interpG_i64 = static_cast<int64_t>((s_1 * val00.G) + (s * val01.G));
            const float interpG = (interpG_i64 >> nbits) + ((interpG_i64 & 0xFFFF) * precision_1);

            const int64_t interpB_i64 = static_cast<int64_t>((s_1 * val00.B) + (s * val01.B));
            const float interpB = (interpB_i64 >> nbits) + ((interpB_i64 & 0xFFFF) * precision_1);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else {
            dst[i][j] = src[y_][x_];
          }
        }

        xi += a0_i64;
        yi += a3_i64;
      }

      a2_i64 += a1_i64;
      a5_i64 += a4_i64;
    }
  }
  else {
    unsigned int dst_height = dst.getHeight();
    unsigned int dst_width = dst.getWidth();
    int src_height = static_cast<int>(src.getHeight());
    int src_width = static_cast<int>(src.getWidth());
    for (unsigned int i = 0; i < dst_height; ++i) {
      int64_t xi = a2_i64;
      int64_t yi = a5_i64;
      int64_t wi = a8_i64;

      for (unsigned int j = 0; j < dst_width; ++j) {
        if ((yi >= 0) && (yi <= ((src_height - 1) * wi)) && (xi >= 0) &&
            (xi <= ((src_width - 1) * wi))) {
          const float wi_ = (wi >> nbits) + ((wi & 0xFFFF) * precision_1);
          const float xi_ = ((xi >> nbits) + ((xi & 0xFFFF) * precision_1)) / wi_;
          const float yi_ = ((yi >> nbits) + ((yi & 0xFFFF) * precision_1)) / wi_;

          const int x_ = static_cast<int>(xi_);
          const int y_ = static_cast<int>(yi_);

          const float t = yi_ - y_;
          const float s = xi_ - x_;

          if ((y_ < (src_height - 1)) && (x_ < (src_width - 1))) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val01 = src[y_][x_ + 1];
            const vpRGBa val10 = src[y_ + 1][x_];
            const vpRGBa val11 = src[y_ + 1][x_ + 1];
            const float colR0 = lerp(val00.R, val01.R, s);
            const float colR1 = lerp(val10.R, val11.R, s);
            const float interpR = lerp(colR0, colR1, t);

            const float colG0 = lerp(val00.G, val01.G, s);
            const float colG1 = lerp(val10.G, val11.G, s);
            const float interpG = lerp(colG0, colG1, t);

            const float colB0 = lerp(val00.B, val01.B, s);
            const float colB1 = lerp(val10.B, val11.B, s);
            const float interpB = lerp(colB0, colB1, t);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (y_ < (src_height - 1)) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val10 = src[y_ + 1][x_];
            const float interpR = lerp(val00.R, val10.R, t);
            const float interpG = lerp(val00.G, val10.G, t);
            const float interpB = lerp(val00.B, val10.B, t);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (x_ < (src_width - 1)) {
            const vpRGBa val00 = src[y_][x_];
            const vpRGBa val01 = src[y_][x_ + 1];
            const float interpR = lerp(val00.R, val01.R, s);
            const float interpG = lerp(val00.G, val01.G, s);
            const float interpB = lerp(val00.B, val01.B, s);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else {
            dst[i][j] = src[y_][x_];
          }
        }

        xi += a0_i64;
        yi += a3_i64;
        wi += a6_i64;
      }

      a2_i64 += a1_i64;
      a5_i64 += a4_i64;
      a8_i64 += a7_i64;
    }
  }

}

template <>
inline void vpImageTools::warpLinear(const vpImage<vpRGBa> &src, const vpMatrix &T, vpImage<vpRGBa> &dst, bool affine,
                                     bool centerCorner, bool fixedPoint)
{
  const unsigned int index_0 = 0, index_1 = 1, index_2 = 2;
  if (fixedPoint && (!centerCorner)) {
    warpLinearFixedPointNotCenter(src, T, dst, affine);
  }
  else {
    double a0 = T[index_0][index_0];
    double a1 = T[index_0][index_1];
    double a2 = T[index_0][index_2];
    double a3 = T[index_1][index_0];
    double a4 = T[index_1][index_1];
    double a5 = T[index_1][index_2];
    double a6 = affine ? 0.0 : T[index_2][index_0];
    double a7 = affine ? 0.0 : T[index_2][index_1];
    double a8 = affine ? 1.0 : T[index_2][index_2];

    unsigned int dst_height = dst.getHeight();
    unsigned int dst_width = dst.getWidth();
    int src_height = static_cast<int>(src.getHeight());
    int src_width = static_cast<int>(src.getWidth());
    for (unsigned int i = 0; i < dst_height; ++i) {
      for (unsigned int j = 0; j < dst_width; ++j) {
        double x = (a0 * (centerCorner ? (j + 0.5) : j)) + (a1 * (centerCorner ? (i + 0.5) : i)) + a2;
        double y = (a3 * (centerCorner ? (j + 0.5) : j)) + (a4 * (centerCorner ? (i + 0.5) : i)) + a5;
        double w = (a6 * (centerCorner ? (j + 0.5) : j)) + (a7 * (centerCorner ? (i + 0.5) : i)) + a8;

        x = (x / w) - (centerCorner ? 0.5 : 0);
        y = (y / w) - (centerCorner ? 0.5 : 0);

        int x_lower = static_cast<int>(x);
        int y_lower = static_cast<int>(y);

        bool stop_for_loop = false;
        if ((y_lower >= src_height) || (x_lower >= src_width) || (y < 0) || (x < 0)) {
          stop_for_loop = true;
        }
        if (!stop_for_loop) {
          double s = x - x_lower;
          double t = y - y_lower;

          if ((y_lower < (src_height - 1)) && (x_lower < (src_width - 1))) {
            const vpRGBa val00 = src[y_lower][x_lower];
            const vpRGBa val01 = src[y_lower][x_lower + 1];
            const vpRGBa val10 = src[y_lower + 1][x_lower];
            const vpRGBa val11 = src[y_lower + 1][x_lower + 1];
            const double colR0 = lerp(val00.R, val01.R, s);
            const double colR1 = lerp(val10.R, val11.R, s);
            const double interpR = lerp(colR0, colR1, t);

            const double colG0 = lerp(val00.G, val01.G, s);
            const double colG1 = lerp(val10.G, val11.G, s);
            const double interpG = lerp(colG0, colG1, t);

            const double colB0 = lerp(val00.B, val01.B, s);
            const double colB1 = lerp(val10.B, val11.B, s);
            const double interpB = lerp(colB0, colB1, t);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (y_lower < (src_height - 1)) {
            const vpRGBa val00 = src[y_lower][x_lower];
            const vpRGBa val10 = src[y_lower + 1][x_lower];
            const double interpR = lerp(val00.R, val10.R, t);
            const double interpG = lerp(val00.G, val10.G, t);
            const double interpB = lerp(val00.B, val10.B, t);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else if (x_lower < (src_width - 1)) {
            const vpRGBa val00 = src[y_lower][x_lower];
            const vpRGBa val01 = src[y_lower][x_lower + 1];
            const double interpR = lerp(val00.R, val01.R, s);
            const double interpG = lerp(val00.G, val01.G, s);
            const double interpB = lerp(val00.B, val01.B, s);

            dst[i][j] = vpRGBa(vpMath::saturate<unsigned char>(interpR), vpMath::saturate<unsigned char>(interpG),
                               vpMath::saturate<unsigned char>(interpB), 255);
          }
          else {
            dst[i][j] = src[y_lower][x_lower];
          }
        }
      }
    }
  }
}

#endif
