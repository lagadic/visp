/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Image tools.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpImageTools.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1
#endif

/*!
  Change the look up table (LUT) of an image. Considering pixel gray
  level values \f$ l \f$ in the range \f$[A, B]\f$, this method allows
  to rescale these values in \f$[A^*, B^*]\f$ by linear interpolation:

  \f$
  \left\{ \begin{array}{ll}
  l \in ]-\infty, A] \mbox{, } &  l = A^* \\
  l \in  [B, \infty[ \mbox{, } &  l = B^* \\
  l \in ]A, B[ \mbox{, }       &  l = A^* + (l-A) * \frac{B^*-A^*}{B-A}
  \end{array}
  \right.
  \f$

  \param I : Image to process.
  \param A : Low gray level value of the range to consider.
  \param A_star : New gray level value \f$ A^*\f$ to attribute to pixel
  who's value was A
  \param B : Height gray level value of the range to consider.
  \param B_star : New gray level value \f$ B^*\f$ to attribute to pixel
  who's value was B
  \return The modified image.

  \exception vpImageException::incorrectInitializationError If \f$B \leq A\f$.

  As shown in the example below, this method can be used to binarize
  an image. For an unsigned char image (in the range 0-255),
  thresholding this image at level 127 can be done by:

  \code
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/io/vpImageIo.h>

int main()
{
  vpImage<unsigned char> I;
#ifdef _WIN32
  std::string filename("C:/temp/ViSP-images/Klimt/Klimt.ppm");
#else
  std::string filename("/local/soft/ViSP/ViSP-images/Klimt/Klimt.ppm");
#endif

  // Read an image from the disk
  vpImageIo::read(I, filename);

  // Binarize image I:
  // - gray level values less than or equal to 127 are set to 0,
  // - gray level values greater than 128 are set to 255
  vpImageTools::changeLUT(I, 127, 0, 128, 255);

  vpImageIo::write(I, "Klimt.pgm"); // Write the image in a PGM P5 image file format
}
  \endcode

*/
void vpImageTools::changeLUT(vpImage<unsigned char> &I, unsigned char A, unsigned char A_star, unsigned char B,
                             unsigned char B_star)
{
  // Test if input values are valid
  if (B <= A) {
    vpERROR_TRACE("Bad gray levels");
    throw(vpImageException(vpImageException::incorrectInitializationError, "Bad gray levels"));
  }
  unsigned char v;

  double factor = (double)(B_star - A_star) / (double)(B - A);

  for (unsigned int i = 0; i < I.getHeight(); i++)
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      v = I[i][j];

      if (v <= A)
        I[i][j] = A_star;
      else if (v >= B)
        I[i][j] = B_star;
      else
        I[i][j] = (unsigned char)(A_star + factor * (v - A));
    }
}

/*!
  Compute the signed difference between the two images I1 and I2 for
  visualization issue : Idiff = I1-I2

  - pixels with a null difference are set to 128.
  - A negative difference implies a pixel value < 128
  - A positive difference implies a pixel value > 128

  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference.
*/
void vpImageTools::imageDifference(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                   vpImage<unsigned char> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images have not the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diff = I1.bitmap[b] - I2.bitmap[b] + 128;
    Idiff.bitmap[b] = (unsigned char)(vpMath::maximum(vpMath::minimum(diff, 255), 0));
  }
}

/*!
  Compute the signed difference between the two images I1 and I2 RGB
  components for visualization issue : Idiff = I1-I2. The fourth component
  named A is not compared. It is set to 0 in the resulting difference image.

  - pixels with a null difference are set to R=128, G=128, B=128.
  - A negative difference implies a pixel R, G, B value < 128
  - A positive difference implies a pixel R, G, B value > 128

  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference between RGB components.
*/
void vpImageTools::imageDifference(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, vpImage<vpRGBa> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "Cannot compute image difference. The two images "
                                                   "(%ux%u) and (%ux%u) have not the same size",
                      I1.getWidth(), I1.getHeight(), I2.getWidth(), I2.getHeight()));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diffR = I1.bitmap[b].R - I2.bitmap[b].R + 128;
    int diffG = I1.bitmap[b].G - I2.bitmap[b].G + 128;
    int diffB = I1.bitmap[b].B - I2.bitmap[b].B + 128;
    int diffA = I1.bitmap[b].A - I2.bitmap[b].A + 128;
    Idiff.bitmap[b].R = (unsigned char)(vpMath::maximum(vpMath::minimum(diffR, 255), 0));
    Idiff.bitmap[b].G = (unsigned char)(vpMath::maximum(vpMath::minimum(diffG, 255), 0));
    Idiff.bitmap[b].B = (unsigned char)(vpMath::maximum(vpMath::minimum(diffB, 255), 0));
    Idiff.bitmap[b].A = (unsigned char)(vpMath::maximum(vpMath::minimum(diffA, 255), 0));
  }
}

/*!
  Compute the difference between the two images I1 and I2
  \warning : This is NOT for visualization
  If you want to visualize difference images during servo, please use
  vpImageTools::imageDifference(..,..,..) function.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference.
*/
void vpImageTools::imageDifferenceAbsolute(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                           vpImage<unsigned char> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diff = I1.bitmap[b] - I2.bitmap[b];
    Idiff.bitmap[b] = diff;
  }
}

/*!
  Compute the difference between the two images I1 and I2.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference.
*/
void vpImageTools::imageDifferenceAbsolute(const vpImage<double> &I1, const vpImage<double> &I2, vpImage<double> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  double diff;
  for (unsigned int b = 0; b < n; b++) {
    diff = I1.bitmap[b] - I2.bitmap[b];
    Idiff.bitmap[b] = vpMath::abs(diff);
  }
}

/*!
  Compute the difference between the two images I1 and I2 RGB components.
  The fourth component named A is not compared.
  It is set to 0 in the resulting difference image.

  \warning : This is NOT for visualization.
  If you want to visualize difference images during servo, please use
  vpImageTools::imageDifference(..,..,..) function.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Idiff : The result of the difference between RGB components.
*/
void vpImageTools::imageDifferenceAbsolute(const vpImage<vpRGBa> &I1, const vpImage<vpRGBa> &I2, vpImage<vpRGBa> &Idiff)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth()))
    Idiff.resize(I1.getHeight(), I1.getWidth());

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; b++) {
    int diffR = I1.bitmap[b].R - I2.bitmap[b].R;
    int diffG = I1.bitmap[b].G - I2.bitmap[b].G;
    int diffB = I1.bitmap[b].B - I2.bitmap[b].B;
    // int diffA = I1.bitmap[b].A - I2.bitmap[b].A;
    Idiff.bitmap[b].R = diffR;
    Idiff.bitmap[b].G = diffG;
    Idiff.bitmap[b].B = diffB;
    // Idiff.bitmap[b].A = diffA;
    Idiff.bitmap[b].A = 0;
  }
}

/*!
  Compute the image addition: \f$ Ires = I1 + I2 \f$.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Ires : \f$ Ires = I1 + I2 \f$
  \param saturate : If true, saturate the result to [0 ; 255] using
  vpMath::saturate, otherwise overflow may occur.
*/
void vpImageTools::imageAdd(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            vpImage<unsigned char> &Ires, const bool saturate)
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
  unsigned int cpt = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2() && Ires.getSize() >= 16) {
    for (; cpt <= Ires.getSize() - 16; cpt += 16, ptr_I1 += 16, ptr_I2 += 16, ptr_Ires += 16) {
      const __m128i v1 = _mm_loadu_si128((const __m128i *)ptr_I1);
      const __m128i v2 = _mm_loadu_si128((const __m128i *)ptr_I2);
      const __m128i vres = saturate ? _mm_adds_epu8(v1, v2) : _mm_add_epi8(v1, v2);

      _mm_storeu_si128((__m128i *)ptr_Ires, vres);
    }
  }
#endif

  for (; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 + (short int)*ptr_I2) : *ptr_I1 + *ptr_I2;
  }
}

/*!
  Compute the image addition: \f$ Ires = I1 - I2 \f$.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Ires : \f$ Ires = I1 - I2 \f$
  \param saturate : If true, saturate the result to [0 ; 255] using
  vpMath::saturate, otherwise overflow may occur.
*/
void vpImageTools::imageSubtract(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                 vpImage<unsigned char> &Ires, const bool saturate)
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
  unsigned int cpt = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2() && Ires.getSize() >= 16) {
    for (; cpt <= Ires.getSize() - 16; cpt += 16, ptr_I1 += 16, ptr_I2 += 16, ptr_Ires += 16) {
      const __m128i v1 = _mm_loadu_si128((const __m128i *)ptr_I1);
      const __m128i v2 = _mm_loadu_si128((const __m128i *)ptr_I2);
      const __m128i vres = saturate ? _mm_subs_epu8(v1, v2) : _mm_sub_epi8(v1, v2);

      _mm_storeu_si128((__m128i *)ptr_Ires, vres);
    }
  }
#endif

  for (; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>((short int)*ptr_I1 - (short int)*ptr_I2) : *ptr_I1 - *ptr_I2;
  }
}

/*!
  Compute a correlation between 2 images.

  \param I1 : The first image.
  \param I2 : The second image.
*/
double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    std::cerr << "Error: in vpImageTools::normalizedCorrelation(): "
              << "image dimensions mismatch." << std::endl;
    exit(EXIT_FAILURE);
  }

  double a = I1.getMeanValue();
  double b = I2.getMeanValue();

  double ab = 0.0;
  double a2 = 0.0;
  double b2 = 0.0;

  for (unsigned int i = 0; i < I1.getHeight(); ++i)
    for (unsigned int j = 0; j < I1.getWidth(); ++j) {
      ab += (I1(i, j) - a) * (I2(i, j) - b);
      a2 += vpMath::sqr(I1(i, j) - a);
      b2 += vpMath::sqr(I2(i, j) - b);
    }

  return ab / sqrt(a2 * b2);
}

/*!
  Compute the column-wise mean intensities.

  \param I : The image.
  \param V : The result vector.
*/

void vpImageTools::columnMean(const vpImage<double> &I, vpRowVector &V)
{
  unsigned int height(I.getHeight()), width(I.getWidth());
  V.resize(width);
  for (unsigned int j = 0; j < width; ++j)
    V[j] = 0.0;
  for (unsigned int i = 0; i < height; ++i)
    for (unsigned int j = 0; j < width; ++j)
      V[j] += I[i][j];
  for (unsigned int j = 0; j < width; ++j)
    V[j] /= height;
}

/*!
  Normalize the image intensities.
  \param I : The image to normalize.
*/
void vpImageTools::normalize(vpImage<double> &I)
{
  double s = I.getSum();
  for (unsigned int i = 0; i < I.getHeight(); ++i)
    for (unsigned int j = 0; j < I.getWidth(); ++j)
      I(i, j, I(i, j) / s);
}

/*!
  Get the interpolated value at a given location.
  \param I : The image to perform intepolation in.
  \param point : The image point.
  \param it : The interpolation type.
*/
double vpImageTools::interpolate(const vpImage<unsigned char> &I, vpImagePoint point, vpImageInterpolationType it)
{
  switch (it) {
  case INTERPOLATION_NEAREST:
    return I(vpMath::round(point.get_i()), vpMath::round(point.get_j()));
  case INTERPOLATION_LINEAR: {
    int x1 = (int)floor(point.get_i());
    int x2 = (int)ceil(point.get_i());
    int y1 = (int)floor(point.get_j());
    int y2 = (int)ceil(point.get_j());
    double v1, v2;
    if (x1 == x2) {
      v1 = I(x1, y1);
      v2 = I(x1, y2);
    } else {
      v1 = (x2 - point.get_i()) * I(x1, y1) + (point.get_i() - x1) * I(x2, y1);
      v2 = (x2 - point.get_i()) * I(x1, y2) + (point.get_i() - x1) * I(x2, y2);
    }
    if (y1 == y2)
      return v1;
    return (y2 - point.get_j()) * v1 + (point.get_j() - y1) * v2;
  }
  case INTERPOLATION_CUBIC: {
    throw vpException(vpException::notImplementedError,
                      "vpImageTools::interpolate(): bi-cubic interpolation is not implemented.");
  }
  default: {
    throw vpException(vpException::notImplementedError, "vpImageTools::interpolate(): invalid interpolation type");
  }
  }
}

/*!
  Extract a rectangular region from an image.
  \param Src : The source image.
  \param Dst : The resulting image.
  \param r : The rectangle area.
*/
void vpImageTools::extract(const vpImage<unsigned char> &Src, vpImage<unsigned char> &Dst, const vpRectOriented &r)
{
  unsigned int x_d = vpMath::round(r.getHeight());
  unsigned int y_d = vpMath::round(r.getWidth());
  double x1 = r.getTopLeft().get_i();
  double y1 = r.getTopLeft().get_j();
  double t = r.getOrientation();
  Dst.resize(x_d, y_d);
  for (unsigned int x = 0; x < x_d; ++x) {
    for (unsigned int y = 0; y < y_d; ++y) {
      Dst(x, y,
          (unsigned char)interpolate(Src, vpImagePoint(x1 + x * cos(t) + y * sin(t), y1 - x * sin(t) + y * cos(t)),
                                     vpImageTools::INTERPOLATION_LINEAR));
    }
  }
}

/*!
  Extract a rectangular region from an image.
  \param Src : The source image.
  \param Dst : The resulting image.
  \param r : The rectangle area.
*/
void vpImageTools::extract(const vpImage<unsigned char> &Src, vpImage<double> &Dst, const vpRectOriented &r)
{
  unsigned int x_d = vpMath::round(r.getHeight());
  unsigned int y_d = vpMath::round(r.getWidth());
  double x1 = r.getTopLeft().get_i();
  double y1 = r.getTopLeft().get_j();
  double t = r.getOrientation();
  Dst.resize(x_d, y_d);
  for (unsigned int x = 0; x < x_d; ++x) {
    for (unsigned int y = 0; y < y_d; ++y) {
      Dst(x, y, interpolate(Src, vpImagePoint(x1 + x * cos(t) + y * sin(t), y1 - x * sin(t) + y * cos(t)),
                            vpImageTools::INTERPOLATION_LINEAR));
    }
  }
}

// Reference:
// http://blog.demofox.org/2015/08/15/resizing-images-with-bicubic-interpolation/
// t is a value that goes from 0 to 1 to interpolate in a C1 continuous way
// across uniformly sampled data points. when t is 0, this will return B.
// When t is 1, this will return C. In between values will return an
// interpolation between B and C. A and B are used to calculate the slopes at
// the edges.
float vpImageTools::cubicHermite(const float A, const float B, const float C, const float D, const float t)
{
  float a = (-A + 3.0f * B - 3.0f * C + D) / 2.0f;
  float b = A + 2.0f * C - (5.0f * B + D) / 2.0f;
  float c = (-A + C) / 2.0f;
  float d = B;

  return a * t * t * t + b * t * t + c * t + d;
}

float vpImageTools::lerp(const float A, const float B, const float t) { return A * (1.0f - t) + B * t; }
