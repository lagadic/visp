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
 * Common functions for color conversion and image resize tests.
 *
*****************************************************************************/

#ifndef common_HPP
#define common_HPP

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageMorphology.h>

#if defined(VISP_HAVE_OPENCV)
#include <opencv2/core/core.hpp>
#endif

namespace common_tools
{
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
static const int g_nearest_neighbor = 0;
static const int g_bilinear = 1;

void fill(vpImage<unsigned char> &img)
{
  for (unsigned int i = 0; i < img.getSize(); i++) {
    img.bitmap[i] = static_cast<unsigned char>(i);
  }
}

void fill(vpImage<vpRGBa> &img)
{
  for (unsigned int i = 0; i < img.getSize(); i++) {
    img.bitmap[i].R = static_cast<unsigned char>(i);
    img.bitmap[i].G = static_cast<unsigned char>(i * 2);
    img.bitmap[i].B = static_cast<unsigned char>(i * 3);
    img.bitmap[i].A = vpRGBa::alpha_default;
  }
}

void fill(std::vector<unsigned char> &img)
{
  for (size_t i = 0; i < img.size(); i++) {
    img[i] = static_cast<unsigned char>(i);
  }
}

bool almostEqual(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, double threshold, double &error)
{
  error = 0.0;

  if (I1.getHeight() != I2.getHeight() || I1.getWidth() != I2.getWidth()) {
    return false;
  }

  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      error += vpMath::sqr(I1[i][j] - I2[i][j]);
    }
  }

  error = sqrt(error / I1.getSize());
  return error < threshold;
}

bool almostEqual(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, double threshold)
{
  double error = 0.0;
  return almostEqual(I1, I2, threshold, error);
}

bool almostEqual(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, double threshold, double &error)
{
  error = 0.0;

  if (I1.getHeight() != I2.getHeight() || I1.getWidth() != I2.getWidth()) {
    return false;
  }

  for (unsigned int i = 0; i < I1.getHeight(); i++) {
    for (unsigned int j = 0; j < I1.getWidth(); j++) {
      error += vpMath::sqr(I1[i][j].R - I2[i][j].R);
      error += vpMath::sqr(I1[i][j].G - I2[i][j].G);
      error += vpMath::sqr(I1[i][j].B - I2[i][j].B);
    }
  }

  error = sqrt(error / (3 * I1.getSize()));
  return error < threshold;
}

bool almostEqual(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, double threshold)
{
  double error = 0.0;
  return almostEqual(I1, I2, threshold, error);
}

/// Image resize
unsigned char getPixelClamped(const vpImage<unsigned char> &I, float x, float y)
{
  int j = vpMath::round(x);
  int i = vpMath::round(y);
  j = std::max<int>(0, std::min<int>(j, static_cast<int>(I.getWidth()) - 1));
  i = std::max<int>(0, std::min<int>(i, static_cast<int>(I.getHeight()) - 1));

  return I[i][j];
}

vpRGBa getPixelClamped(const vpImage<vpRGBa> &I, float x, float y)
{
  int j = vpMath::round(x);
  int i = vpMath::round(y);
  j = std::max<int>(0, std::min<int>(j, static_cast<int>(I.getWidth()) - 1));
  i = std::max<int>(0, std::min<int>(i, static_cast<int>(I.getHeight()) - 1));

  return I[i][j];
}

float lerp(float A, float B, float t) { return A * (1.0f - t) + B * t; }

void resizeBilinear(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires, unsigned int i, unsigned int j,
                    int u0, int v0, float xFrac, float yFrac)
{
  int u1 = std::min<int>(static_cast<int>(I.getWidth()) - 1, u0 + 1);
  int v1 = v0;

  int u2 = u0;
  int v2 = std::min<int>(static_cast<int>(I.getHeight()) - 1, v0 + 1);

  int u3 = u1;
  int v3 = v2;

  float col0 = lerp(I[v0][u0], I[v1][u1], xFrac);
  float col1 = lerp(I[v2][u2], I[v3][u3], xFrac);
  float value = lerp(col0, col1, yFrac);

  Ires[i][j] = vpMath::saturate<unsigned char>(value);
  // Ires[i][j] = static_cast<unsigned char>(value);
}

void resizeBilinear(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, unsigned int i, unsigned int j, int u0, int v0,
                    float xFrac, float yFrac)
{
  int u1 = std::min<int>(static_cast<int>(I.getWidth()) - 1, u0 + 1);
  int v1 = v0;

  int u2 = u0;
  int v2 = std::min<int>(static_cast<int>(I.getHeight()) - 1, v0 + 1);

  int u3 = u1;
  int v3 = v2;

  float col0 = lerp(I[v0][u0].R, I[v1][u1].R, xFrac);
  float col1 = lerp(I[v2][u2].R, I[v3][u3].R, xFrac);
  float valueR = lerp(col0, col1, yFrac);

  col0 = lerp(I[v0][u0].G, I[v1][u1].G, xFrac);
  col1 = lerp(I[v2][u2].G, I[v3][u3].G, xFrac);
  float valueG = lerp(col0, col1, yFrac);

  col0 = lerp(I[v0][u0].B, I[v1][u1].B, xFrac);
  col1 = lerp(I[v2][u2].B, I[v3][u3].B, xFrac);
  float valueB = lerp(col0, col1, yFrac);

  Ires[i][j].R = vpMath::saturate<unsigned char>(valueR);
  Ires[i][j].G = vpMath::saturate<unsigned char>(valueG);
  Ires[i][j].B = vpMath::saturate<unsigned char>(valueB);
}

void resizeRef(const vpImage<unsigned char> &Isrc, vpImage<unsigned char> &Idst, int method)
{
  const float scaleX = Isrc.getWidth() / static_cast<float>(Idst.getWidth());
  const float scaleY = Isrc.getHeight() / static_cast<float>(Idst.getHeight());
  const float half = 0.5f;

  for (unsigned int i = 0; i < Idst.getHeight(); i++) {
    const float v = (i + half) * scaleY - half;
    const int v0 = static_cast<int>(v);
    const float yFrac = v - v0;

    for (unsigned int j = 0; j < Idst.getWidth(); j++) {
      const float u = (j + half) * scaleX - half;
      const int u0 = static_cast<int>(u);
      const float xFrac = u - u0;

      if (method == 0) { // nearest neighbor
        Idst[i][j] = getPixelClamped(Isrc, u, v);
      }
      else if (method == 1) { // bilinear
        resizeBilinear(Isrc, Idst, i, j, u0, v0, xFrac, yFrac);
      }
      else { // bicubic
     // no bicubic ref test for now
      }
    }
  }
}

void resizeRef(const vpImage<vpRGBa> &Isrc, vpImage<vpRGBa> &Idst, int method)
{
  const float scaleX = Isrc.getWidth() / static_cast<float>(Idst.getWidth());
  const float scaleY = Isrc.getHeight() / static_cast<float>(Idst.getHeight());
  const float half = 0.5f;

  for (unsigned int i = 0; i < Idst.getHeight(); i++) {
    const float v = (i + half) * scaleY - half;
    const int v0 = static_cast<int>(v);
    const float yFrac = v - v0;

    for (unsigned int j = 0; j < Idst.getWidth(); j++) {
      const float u = (j + half) * scaleX - half;
      const int u0 = static_cast<int>(u);
      const float xFrac = u - u0;

      if (method == 0) { // nearest neighbor
        Idst[i][j] = getPixelClamped(Isrc, u, v);
      }
      else if (method == 1) { // bilinear
        resizeBilinear(Isrc, Idst, i, j, u0, v0, xFrac, yFrac);
      }
      else { // bicubic
     // no bicubic ref test for now
      }
    }
  }
}

/// Color conversion
void RGBaToBGR(const vpImage<vpRGBa> &rgba, std::vector<unsigned char> &bgr)
{
  bgr.resize(rgba.getSize() * 3);

  vpImage<vpRGBa> bgra(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> R(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> G(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> B(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> A(rgba.getHeight(), rgba.getWidth());

  vpImageConvert::split(rgba, &R, &G, &B, &A);
  vpImageConvert::merge(&B, &G, &R, &A, bgra);

  vpImageConvert::RGBaToRGB(reinterpret_cast<unsigned char *>(bgra.bitmap),
                            reinterpret_cast<unsigned char *>(bgr.data()), bgra.getSize());
}

/// Color conversion
void RGBaToBGRa(const vpImage<vpRGBa> &rgba, std::vector<unsigned char> &bgra)
{
  bgra.resize(rgba.getSize() * 4);

  vpImage<vpRGBa> bgra_(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> R(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> G(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> B(rgba.getHeight(), rgba.getWidth());
  vpImage<unsigned char> A(rgba.getHeight(), rgba.getWidth());

  vpImageConvert::split(rgba, &R, &G, &B, &A);
  vpImageConvert::merge(&B, &G, &R, &A, bgra_);

  memcpy(reinterpret_cast<unsigned char *>(bgra.data()), reinterpret_cast<unsigned char *>(bgra_.bitmap),
         rgba.getSize() * 4);
}

void grayToRGBaRef(unsigned char *grey, unsigned char *rgba, unsigned int size)
{
  unsigned char *pt_input = grey;
  unsigned char *pt_end = grey + size;
  unsigned char *pt_output = rgba;

  while (pt_input != pt_end) {
    unsigned char p = *pt_input;
    *(pt_output) = p;                         // R
    *(pt_output + 1) = p;                     // G
    *(pt_output + 2) = p;                     // B
    *(pt_output + 3) = vpRGBa::alpha_default; // A

    pt_input++;
    pt_output += 4;
  }
}

void RGBaToGrayRef(unsigned char *rgba, unsigned char *grey, unsigned int size)
{
  unsigned char *pt_input = rgba;
  unsigned char *pt_end = rgba + size * 4;
  unsigned char *pt_output = grey;

  while (pt_input != pt_end) {
    *pt_output = (unsigned char)(0.2126 * (*pt_input) + 0.7152 * (*(pt_input + 1)) + 0.0722 * (*(pt_input + 2)));
    pt_input += 4;
    pt_output++;
  }
}

void RGBToGrayRef(unsigned char *rgb, unsigned char *grey, unsigned int width, unsigned int height, bool flip)
{
  // if we have to flip the image, we start from the end last scanline so
  // the  step is negative
  int lineStep = (flip) ? -(int)(width * 3) : (int)(width * 3);

  // starting source address = last line if we need to flip the image
  unsigned char *src = (flip) ? rgb + (width * height * 3) + lineStep : rgb;

  unsigned int j = 0;
  unsigned int i = 0;

  unsigned r, g, b;

  for (i = 0; i < height; i++) {
    unsigned char *line = src;
    for (j = 0; j < width; j++) {
      r = *(line++);
      g = *(line++);
      b = *(line++);
      *grey++ = (unsigned char)(0.2126 * r + 0.7152 * g + 0.0722 * b);
    }

    // go to the next line
    src += lineStep;
  }
}

/// Image Add / Sub
void imageAddRef(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, vpImage<unsigned char> &Ires,
                 bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;

  for (unsigned int cpt = 0; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 + (short int)*ptr_I2) : *ptr_I1 + *ptr_I2;
  }
}

void imageSubtractRef(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2, vpImage<unsigned char> &Ires,
                      bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;

  for (unsigned int cpt = 0; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 - (short int)*ptr_I2) : *ptr_I1 - *ptr_I2;
  }
}

/// Image morphology
void imageErosionRef(vpImage<unsigned char> &I,
                     vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 255;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    }
    else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    unsigned int offset[5] = { 1, J.getWidth(), J.getWidth() + 1, J.getWidth() + 2, J.getWidth() * 2 + 1 };

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char min_value = null_value;
        for (int k = 0; k < 5; k++) {
          min_value = std::min<unsigned char>(min_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = min_value;
      }
    }
  }
  else {
 // CONNEXITY_8
    unsigned int offset[9] = { 0,
                              1,
                              2,
                              J.getWidth(),
                              J.getWidth() + 1,
                              J.getWidth() + 2,
                              J.getWidth() * 2,
                              J.getWidth() * 2 + 1,
                              J.getWidth() * 2 + 2 };

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char min_value = null_value;
        for (int k = 0; k < 9; k++) {
          min_value = std::min<unsigned char>(min_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = min_value;
      }
    }
  }
}

// Dilatation in the general case on grayscale images
void imageDilatationRef(vpImage<unsigned char> &I,
                        vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 0;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    }
    else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    unsigned int offset[5] = { 1, J.getWidth(), J.getWidth() + 1, J.getWidth() + 2, J.getWidth() * 2 + 1 };

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char max_value = null_value;
        for (int k = 0; k < 5; k++) {
          max_value = std::max<unsigned char>(max_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = max_value;
      }
    }
  }
  else {
    // CONNEXITY_8
    unsigned int offset[9] = { 0,
                              1,
                              2,
                              J.getWidth(),
                              J.getWidth() + 1,
                              J.getWidth() + 2,
                              J.getWidth() * 2,
                              J.getWidth() * 2 + 1,
                              J.getWidth() * 2 + 2 };

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char max_value = null_value;
        for (int k = 0; k < 9; k++) {
          max_value = std::max<unsigned char>(max_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = max_value;
      }
    }
  }
}

void magicSquare(vpImage<unsigned char> &magic_square, int N)
{
  magic_square.resize((unsigned int)N, (unsigned int)N, 0);

  int n = 1;
  int i = 0, j = N / 2;

  while (n <= N * N) {
    magic_square[i][j] = vpMath::saturate<unsigned char>(n);
    n++;

    int newi = vpMath::modulo((i - 1), N), newj = vpMath::modulo((j + 1), N);

    if (magic_square[newi][newj]) {
      i++;
    }
    else {
      i = newi;
      j = newj;
    }
  }
}

void BGRToGrayRef(unsigned char *bgr, unsigned char *grey, unsigned int width, unsigned int height, bool flip)
{
  // if we have to flip the image, we start from the end last scanline so
  // the  step is negative
  int lineStep = (flip) ? -(int)(width * 3) : (int)(width * 3);

  // starting source address = last line if we need to flip the image
  unsigned char *src = (flip) ? bgr + (width * height * 3) + lineStep : bgr;

  for (unsigned int i = 0; i < height; i++) {
    unsigned char *line = src;
    for (unsigned int j = 0; j < width; j++) {
      *grey++ = (unsigned char)(0.2126 * *(line + 2) + 0.7152 * *(line + 1) + 0.0722 * *(line + 0));
      line += 3;
    }

    // go to the next line
    src += lineStep;
  }
}

void BGRToRGBaRef(unsigned char *bgr, unsigned char *rgba, unsigned int width, unsigned int height, bool flip)
{
  // if we have to flip the image, we start from the end last scanline so the
  // step is negative
  int lineStep = (flip) ? -(int)(width * 3) : (int)(width * 3);

  // starting source address = last line if we need to flip the image
  unsigned char *src = (flip) ? (bgr + (width * height * 3) + lineStep) : bgr;

  for (unsigned int i = 0; i < height; i++) {
    unsigned char *line = src;
    for (unsigned int j = 0; j < width; j++) {
      *rgba++ = *(line + 2);
      *rgba++ = *(line + 1);
      *rgba++ = *(line + 0);
      *rgba++ = vpRGBa::alpha_default;

      line += 3;
    }
    // go to the next line
    src += lineStep;
  }
}

void BGRaToRGBaRef(unsigned char *bgra, unsigned char *rgba, unsigned int width, unsigned int height, bool flip)
{
  // if we have to flip the image, we start from the end last scanline so the
  // step is negative
  int lineStep = (flip) ? -(int)(width * 4) : (int)(width * 4);

  // starting source address = last line if we need to flip the image
  unsigned char *src = (flip) ? (bgra + (width * height * 4) + lineStep) : bgra;

  for (unsigned int i = 0; i < height; i++) {
    unsigned char *line = src;
    for (unsigned int j = 0; j < width; j++) {
      *rgba++ = *(line + 2);
      *rgba++ = *(line + 1);
      *rgba++ = *(line + 0);
      *rgba++ = *(line + 3);

      line += 4;
    }
    // go to the next line
    src += lineStep;
  }
}

#if defined(VISP_HAVE_OPENCV)
void fill(cv::Mat &img)
{
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (img.type() == CV_8UC1) {
        img.at<uchar>(i, j) = static_cast<uchar>(i * img.cols + j);
      }
      else if (img.type() == CV_8UC3) {
        img.at<cv::Vec3b>(i, j)[0] = static_cast<uchar>((i * img.cols + j) * 3);
        img.at<cv::Vec3b>(i, j)[1] = static_cast<uchar>((i * img.cols + j) * 3 + 1);
        img.at<cv::Vec3b>(i, j)[2] = static_cast<uchar>((i * img.cols + j) * 3 + 2);
      }
    }
  }
}
#endif
} // namespace common_tools
#endif
