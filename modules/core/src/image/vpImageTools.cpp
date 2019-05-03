/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1

#if defined __SSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <pmmintrin.h>
#define VISP_HAVE_SSE3 1
#endif
#if defined __SSSE3__ || (defined _MSC_VER && _MSC_VER >= 1500)
#include <tmmintrin.h>
#define VISP_HAVE_SSSE3 1
#endif
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

  bool checkSSSE3 = vpCPUFeatures::checkSSSE3();
#if !VISP_HAVE_SSSE3
  checkSSSE3 = false;
#endif

  unsigned int i = 0;
  if (checkSSSE3) {
#if VISP_HAVE_SSSE3
    if (I1.getSize() >= 16) {
      const __m128i mask1 = _mm_set_epi8(-1, 14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0);
      const __m128i mask2 = _mm_set_epi8(-1, 15, -1, 13, -1, 11, -1, 9, -1, 7, -1, 5, -1, 3, -1, 1);

      const __m128i mask_out2 = _mm_set_epi8(14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0, -1);

      for (; i <= I1.getSize()-16; i+= 16) {
        const __m128i vdata1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(I1.bitmap + i));
        const __m128i vdata2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(I2.bitmap + i));

        __m128i vdata1_reorg = _mm_shuffle_epi8(vdata1, mask1);
        __m128i vdata2_reorg = _mm_shuffle_epi8(vdata2, mask1);

        const __m128i vshift = _mm_set1_epi16(128);
        __m128i vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);

        const __m128i v255 = _mm_set1_epi16(255);
        const __m128i vzero = _mm_setzero_si128();
        const __m128i vdata_diff_min_max1 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

        vdata1_reorg = _mm_shuffle_epi8(vdata1, mask2);
        vdata2_reorg = _mm_shuffle_epi8(vdata2, mask2);

        vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);
        const __m128i vdata_diff_min_max2 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

        _mm_storeu_si128(reinterpret_cast<__m128i *>(Idiff.bitmap + i), _mm_or_si128(_mm_shuffle_epi8(vdata_diff_min_max1, mask1),
                                                                                     _mm_shuffle_epi8(vdata_diff_min_max2, mask_out2)));
      }
    }
#endif
  }

  for (; i < I1.getSize(); i++) {
    int diff = I1.bitmap[i] - I2.bitmap[i] + 128;
    Idiff.bitmap[i] = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diff, 255), 0));
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

  bool checkSSSE3 = vpCPUFeatures::checkSSSE3();
#if !VISP_HAVE_SSSE3
  checkSSSE3 = false;
#endif

  unsigned int i = 0;
  if (checkSSSE3) {
#if VISP_HAVE_SSSE3
    if (I1.getSize() >= 4) {
      const __m128i mask1 = _mm_set_epi8(-1, 14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0);
      const __m128i mask2 = _mm_set_epi8(-1, 15, -1, 13, -1, 11, -1, 9, -1, 7, -1, 5, -1, 3, -1, 1);

      const __m128i mask_out2 = _mm_set_epi8(14, -1, 12, -1, 10, -1, 8, -1, 6, -1, 4, -1, 2, -1, 0, -1);

      for (; i <= I1.getSize()-4; i+= 4) {
        const __m128i vdata1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(I1.bitmap + i));
        const __m128i vdata2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(I2.bitmap + i));

        __m128i vdata1_reorg = _mm_shuffle_epi8(vdata1, mask1);
        __m128i vdata2_reorg = _mm_shuffle_epi8(vdata2, mask1);

        const __m128i vshift = _mm_set1_epi16(128);
        __m128i vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);

        const __m128i v255 = _mm_set1_epi16(255);
        const __m128i vzero = _mm_setzero_si128();
        const __m128i vdata_diff_min_max1 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

        vdata1_reorg = _mm_shuffle_epi8(vdata1, mask2);
        vdata2_reorg = _mm_shuffle_epi8(vdata2, mask2);

        vdata_diff = _mm_add_epi16(_mm_sub_epi16(vdata1_reorg, vdata2_reorg), vshift);
        const __m128i vdata_diff_min_max2 = _mm_max_epi16(_mm_min_epi16(vdata_diff, v255), vzero);

        _mm_storeu_si128(reinterpret_cast<__m128i *>(Idiff.bitmap + i), _mm_or_si128(_mm_shuffle_epi8(vdata_diff_min_max1, mask1),
                                                                                     _mm_shuffle_epi8(vdata_diff_min_max2, mask_out2)));
      }
    }
#endif
  }

  for (; i < I1.getSize(); i++) {
    int diffR = I1.bitmap[i].R - I2.bitmap[i].R + 128;
    int diffG = I1.bitmap[i].G - I2.bitmap[i].G + 128;
    int diffB = I1.bitmap[i].B - I2.bitmap[i].B + 128;
    int diffA = I1.bitmap[i].A - I2.bitmap[i].A + 128;
    Idiff.bitmap[i].R = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffR, 255), 0));
    Idiff.bitmap[i].G = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffG, 255), 0));
    Idiff.bitmap[i].B = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffB, 255), 0));
    Idiff.bitmap[i].A = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffA, 255), 0));
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
    Idiff.bitmap[b] = static_cast<unsigned char>(vpMath::abs(diff));
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
  for (unsigned int b = 0; b < n; b++) {
    Idiff.bitmap[b] = vpMath::abs(I1.bitmap[b] - I2.bitmap[b]);
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
    Idiff.bitmap[b].R = static_cast<unsigned char>(vpMath::abs(diffR));
    Idiff.bitmap[b].G = static_cast<unsigned char>(vpMath::abs(diffG));
    Idiff.bitmap[b].B = static_cast<unsigned char>(vpMath::abs(diffB));
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
      const __m128i v1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_I1));
      const __m128i v2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_I2));
      const __m128i vres = saturate ? _mm_adds_epu8(v1, v2) : _mm_add_epi8(v1, v2);

      _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_Ires), vres);
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
      const __m128i v1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_I1));
      const __m128i v2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(ptr_I2));
      const __m128i vres = saturate ? _mm_subs_epu8(v1, v2) : _mm_sub_epi8(v1, v2);

      _mm_storeu_si128(reinterpret_cast<__m128i *>(ptr_Ires), vres);
    }
  }
#endif

  for (; cpt < Ires.getSize(); cpt++, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ?
          vpMath::saturate<unsigned char>(static_cast<short int>(*ptr_I1) - static_cast<short int>(*ptr_I2)) :
          *ptr_I1 - *ptr_I2;
  }
}

/*!
  Compute the undistortion transformation map.

  \param cam : Camera intrinsic parameters with distortion coefficients.
  \param width : Image width.
  \param height : Image height.
  \param mapU : 2D array that contains at each coordinate the u-coordinate in the distorted image.
  \param mapV : 2D array that contains at each coordinate the v-coordinate in the distorted image.
  \param mapDu : 2D array that contains at each coordinate the \f$ \Delta u \f$ for the interpolation.
  \param mapDv : 2D array that contains at each coordinate the \f$ \Delta v \f$ for the interpolation.
*/
void vpImageTools::initUndistortMap(const vpCameraParameters &cam, unsigned int width, unsigned int height,
                                    vpArray2D<int> &mapU, vpArray2D<int> &mapV,
                                    vpArray2D<float> &mapDu, vpArray2D<float> &mapDv)
{
  mapU.resize(height, width, false, false);
  mapV.resize(height, width, false, false);
  mapDu.resize(height, width, false, false);
  mapDv.resize(height, width, false, false);

  float u0 = static_cast<float>(cam.get_u0());
  float v0 = static_cast<float>(cam.get_v0());
  float px = static_cast<float>(cam.get_px());
  float py = static_cast<float>(cam.get_py());
  float kud = static_cast<float>(cam.get_kud());

  if (std::fabs(static_cast<double>(kud)) <= std::numeric_limits<double>::epsilon()) {
    // There is no need to undistort the image
    for (unsigned int i = 0; i < height; i++) {
      for (unsigned int j = 0; j < width; j++) {
        mapU[i][j] = static_cast<int>(j);
        mapV[i][j] = static_cast<int>(i);
        mapDu[i][j] = 0;
        mapDv[i][j] = 0;
      }
    }

    return;
  }

  float invpx = 1.0f / px;
  float invpy = 1.0f / py;

  float kud_px2 = kud * invpx * invpx;
  float kud_py2 = kud * invpy * invpy;

  for (unsigned int v = 0; v < height; v++) {
    float deltav = v - v0;
    float fr1 = 1.0f + kud_py2 * deltav * deltav;

    for (unsigned int u = 0; u < width; u++) {
      // computation of u,v : corresponding pixel coordinates in I.
      float deltau = u - u0;
      float fr2 = fr1 + kud_px2 * deltau * deltau;

      float u_float = deltau * fr2 + u0;
      float v_float = deltav * fr2 + v0;

      int u_round = static_cast<int>(u_float);
      int v_round = static_cast<int>(v_float);

      mapU[v][u] = u_round;
      mapV[v][u] = v_round;

      mapDu[v][u] = u_float - u_round;
      mapDv[v][u] = v_float - v_round;
    }
  }
}

/*!
  Compute the integral images:

  \f$ II(u,v)=\sum_{u^{'}\leq u, v^{'}\leq v}I(u,v) \f$

  \f$ IIsq(u,v)=\sum_{u^{'}\leq u, v^{'}\leq v}I(u,v)^2 \f$.

  \param I : Input image.
  \param II : Integral image II.
  \param IIsq : Integral image IIsq.
*/
void vpImageTools::integralImage(const vpImage<unsigned char> &I, vpImage<double> &II, vpImage<double> &IIsq)
{
  if (I.getSize() == 0) {
    std::cerr << "Error, input image is empty." << std::endl;
    return;
  }

  II.resize(I.getHeight() + 1, I.getWidth() + 1, 0.0);
  IIsq.resize(I.getHeight() + 1, I.getWidth() + 1, 0.0);

  for (unsigned int i = 1; i < II.getHeight(); i++) {
    for (unsigned int j = 1; j < II.getWidth(); j++) {
      II[i][j] = I[i - 1][j - 1] + II[i - 1][j] + II[i][j - 1] - II[i - 1][j - 1];
      IIsq[i][j] = vpMath::sqr(I[i - 1][j - 1]) + IIsq[i - 1][j] + IIsq[i][j - 1] - IIsq[i - 1][j - 1];
    }
  }
}

/*!
  Compute a correlation between 2 images.

  \param I1 : The first image.
  \param I2 : The second image.
  \param useOptimized : Use SSE if true and available.
*/
double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2,
#if VISP_HAVE_SSE2
                                           const bool useOptimized)
#else
                                           const bool)
#endif
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw vpException(vpException::dimensionError, "Error: in vpImageTools::normalizedCorrelation(): "
                                                   "image dimension mismatch between I1=%ux%u and I2=%ux%u",
                      I1.getHeight(), I1.getWidth(), I2.getHeight(), I2.getWidth());
  }

  const double a = I1.getMeanValue();
  const double b = I2.getMeanValue();

  double ab = 0.0;
  double a2 = 0.0;
  double b2 = 0.0;

  unsigned int cpt = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2() && I1.getSize() >= 2 && useOptimized) {
    const double *ptr_I1 = I1.bitmap;
    const double *ptr_I2 = I2.bitmap;

    const __m128d v_mean_a = _mm_set1_pd(a);
    const __m128d v_mean_b = _mm_set1_pd(b);
    __m128d v_ab = _mm_setzero_pd();
    __m128d v_a2 = _mm_setzero_pd();
    __m128d v_b2 = _mm_setzero_pd();

    for (; cpt <= I1.getSize() - 2; cpt += 2, ptr_I1 += 2, ptr_I2 += 2) {
      const __m128d v1 = _mm_loadu_pd(ptr_I1);
      const __m128d v2 = _mm_loadu_pd(ptr_I2);
      const __m128d norm_a = _mm_sub_pd(v1, v_mean_a);
      const __m128d norm_b = _mm_sub_pd(v2, v_mean_b);
      v_ab = _mm_add_pd(v_ab, _mm_mul_pd(norm_a, norm_b));
      v_a2 = _mm_add_pd(v_a2, _mm_mul_pd(norm_a, norm_a));
      v_b2 = _mm_add_pd(v_b2, _mm_mul_pd(norm_b, norm_b));
    }

    double v_res_ab[2], v_res_a2[2], v_res_b2[2];
    _mm_storeu_pd(v_res_ab, v_ab);
    _mm_storeu_pd(v_res_a2, v_a2);
    _mm_storeu_pd(v_res_b2, v_b2);

    ab = v_res_ab[0] + v_res_ab[1];
    a2 = v_res_a2[0] + v_res_a2[1];
    b2 = v_res_b2[0] + v_res_b2[1];
  }
#endif

  for (; cpt < I1.getSize(); cpt++) {
    ab += (I1.bitmap[cpt] - a) * (I2.bitmap[cpt] - b);
    a2 += vpMath::sqr(I1.bitmap[cpt] - a);
    b2 += vpMath::sqr(I2.bitmap[cpt] - b);
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
  unsigned int height = I.getHeight(), width = I.getWidth();
  V.resize(width); // resize and nullify

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
  \param method : The interpolation method (only interpolation with vpImageTools::INTERPOLATION_NEAREST and
  vpImageTools::INTERPOLATION_LINEAR are implemented).
*/
double vpImageTools::interpolate(const vpImage<unsigned char> &I, const vpImagePoint &point,
                                 const vpImageInterpolationType &method)
{
  switch (method) {
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

/*!
  Match a template image into another image using zero-mean normalized cross-correlation:

  \f$\frac{\sum_{u^{'},v^{'}} (I(u+u^{'},v+v^{'})-\bar{I}_{u^{'},v^{'}})
(T(u^{'},v^{'})-\bar{T}_{u^{'},v^{'}})}{\sqrt{\sum_{u^{'},v^{'}}
(I(u+u^{'},v+v^{'})-\bar{I}_{u^{'},v^{'}})^2
\sum_{u^{'},v^{'}}(T(u^{'},v^{'})-\bar{T}_{u^{'},v^{'}})^2}}\f$
  \param I : Input image.
  \param I_tpl : Template image.
  \param I_score : Output template matching score.
  \param step_u : Step in u-direction to speed-up the computation.
  \param step_v : Step in v-direction to speed-up the computation.
  \param useOptimized : Use optimized version (SSE, OpenMP, integral images, ...) if true and available.
*/
void vpImageTools::templateMatching(const vpImage<unsigned char> &I, const vpImage<unsigned char> &I_tpl,
                                    vpImage<double> &I_score, const unsigned int step_u, const unsigned int step_v,
                                    const bool useOptimized)
{
  if (I.getSize() == 0) {
    std::cerr << "Error, input image is empty." << std::endl;
    return;
  }

  if (I_tpl.getSize() == 0) {
    std::cerr << "Error, template image is empty." << std::endl;
    return;
  }

  if (I_tpl.getHeight() > I.getHeight() || I_tpl.getWidth() > I.getWidth()) {
    std::cerr << "Error, template image is bigger than input image." << std::endl;
    return;
  }

  vpImage<double> I_double, I_tpl_double;
  vpImageConvert::convert(I, I_double);
  vpImageConvert::convert(I_tpl, I_tpl_double);

  const unsigned int height_tpl = I_tpl.getHeight(), width_tpl = I_tpl.getWidth();
  I_score.resize(I.getHeight() - height_tpl, I.getWidth() - width_tpl, 0.0);

  if (useOptimized) {
    vpImage<double> II, IIsq;
    integralImage(I, II, IIsq);

    vpImage<double> II_tpl, IIsq_tpl;
    integralImage(I_tpl, II_tpl, IIsq_tpl);

    // zero-mean template image
    const double sum2 = (II_tpl[height_tpl][width_tpl] + II_tpl[0][0] - II_tpl[0][width_tpl] - II_tpl[height_tpl][0]);
    const double mean2 = sum2 / I_tpl.getSize();
    for (unsigned int cpt = 0; cpt < I_tpl_double.getSize(); cpt++) {
      I_tpl_double.bitmap[cpt] -= mean2;
    }

#if defined _OPENMP && _OPENMP >= 200711 // OpenMP 3.1
#pragma omp parallel for schedule(dynamic)
    for (unsigned int i = 0; i < I.getHeight() - height_tpl; i += step_v) {
      for (unsigned int j = 0; j < I.getWidth() - width_tpl; j += step_u) {
        I_score[i][j] = normalizedCorrelation(I_double, I_tpl_double, II, IIsq, II_tpl, IIsq_tpl, i, j);
      }
    }
#else
    // error C3016: 'i': index variable in OpenMP 'for' statement must have signed integral type
    int end = (int)((I.getHeight() - height_tpl) / step_v) + 1;
    std::vector<unsigned int> vec_step_v((size_t)end);
    for (unsigned int cpt = 0, idx = 0; cpt < I.getHeight() - height_tpl; cpt += step_v, idx++) {
      vec_step_v[(size_t)idx] = cpt;
    }
#if defined _OPENMP // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
    for (int cpt = 0; cpt < end; cpt++) {
      for (unsigned int j = 0; j < I.getWidth() - width_tpl; j += step_u) {
        I_score[vec_step_v[cpt]][j] =
            normalizedCorrelation(I_double, I_tpl_double, II, IIsq, II_tpl, IIsq_tpl, vec_step_v[cpt], j);
      }
    }
#endif
  } else {
    vpImage<double> I_cur;

    for (unsigned int i = 0; i < I.getHeight() - height_tpl; i += step_v) {
      for (unsigned int j = 0; j < I.getWidth() - width_tpl; j += step_u) {
        vpRect roi(vpImagePoint(i, j), vpImagePoint(i + height_tpl - 1, j + width_tpl - 1));
        vpImageTools::crop(I_double, roi, I_cur);

        I_score[i][j] = vpImageTools::normalizedCorrelation(I_cur, I_tpl_double, useOptimized);
      }
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

float vpImageTools::lerp(const float A, const float B, const float t) {
  return A * (1.0f - t) + B * t;
}

int64_t vpImageTools::lerp2(int64_t A, int64_t B, int64_t t, int64_t t_1) {
  return A * t_1 + B * t;
}

double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2,
                                           const vpImage<double> &II, const vpImage<double> &IIsq,
                                           const vpImage<double> &II_tpl, const vpImage<double> &IIsq_tpl,
                                           const unsigned int i0, const unsigned int j0)
{
  double ab = 0.0;
#if VISP_HAVE_SSE2
  bool use_sse_version = true;
  if (vpCPUFeatures::checkSSE2() && I2.getWidth() >= 2) {
    const double *ptr_I1 = I1.bitmap;
    const double *ptr_I2 = I2.bitmap;

    __m128d v_ab = _mm_setzero_pd();

    for (unsigned int i = 0; i < I2.getHeight(); i++) {
      unsigned int j = 0;
      ptr_I1 = &I1.bitmap[(i0 + i) * I1.getWidth() + j0];

      for (; j <= I2.getWidth() - 2; j += 2, ptr_I1 += 2, ptr_I2 += 2) {
        const __m128d v1 = _mm_loadu_pd(ptr_I1);
        const __m128d v2 = _mm_loadu_pd(ptr_I2);
        v_ab = _mm_add_pd(v_ab, _mm_mul_pd(v1, v2));
      }

      for (; j < I2.getWidth(); j++) {
        ab += (I1[i0 + i][j0 + j]) * I2[i][j];
      }
    }

    double v_res_ab[2];
    _mm_storeu_pd(v_res_ab, v_ab);

    ab += v_res_ab[0] + v_res_ab[1];
  } else {
    use_sse_version = false;
  }
#else
  bool use_sse_version = false;
#endif

  if (!use_sse_version) {
    for (unsigned int i = 0; i < I2.getHeight(); i++) {
      for (unsigned int j = 0; j < I2.getWidth(); j++) {
        ab += (I1[i0 + i][j0 + j]) * I2[i][j];
      }
    }
  }

  const unsigned int height_tpl = I2.getHeight(), width_tpl = I2.getWidth();
  const double sum1 =
      (II[i0 + height_tpl][j0 + width_tpl] + II[i0][j0] - II[i0][j0 + width_tpl] - II[i0 + height_tpl][j0]);
  const double sum2 = (II_tpl[height_tpl][width_tpl] + II_tpl[0][0] - II_tpl[0][width_tpl] - II_tpl[height_tpl][0]);

  double a2 = ((IIsq[i0 + I2.getHeight()][j0 + I2.getWidth()] + IIsq[i0][j0] - IIsq[i0][j0 + I2.getWidth()] -
                IIsq[i0 + I2.getHeight()][j0]) -
               (1.0 / I2.getSize()) * vpMath::sqr(sum1));

  double b2 = ((IIsq_tpl[I2.getHeight()][I2.getWidth()] + IIsq_tpl[0][0] - IIsq_tpl[0][I2.getWidth()] -
                IIsq_tpl[I2.getHeight()][0]) -
               (1.0 / I2.getSize()) * vpMath::sqr(sum2));
  return ab / sqrt(a2 * b2);
}

/*!
  Apply the transformation map to the image.

  \param I : Input grayscale image.
  \param mapU : Map that contains at each destination coordinate the u-coordinate in the source image.
  \param mapV : Map that contains at each destination coordinate the v-coordinate in the source image.
  \param mapDu : Map that contains at each destination coordinate the \f$ \Delta u \f$ for the interpolation.
  \param mapDv : Map that contains at each destination coordinate the \f$ \Delta v \f$ for the interpolation.
  \param Iundist : Output transformed grayscale image.
*/
void vpImageTools::remap(const vpImage<unsigned char> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                         const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<unsigned char> &Iundist)
{
  Iundist.resize(I.getHeight(), I.getWidth());

#if defined _OPENMP // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i_ = 0; i_ < static_cast<int>(I.getHeight()); i_++) {
    const unsigned int i = static_cast<unsigned int>(i_);
    for (unsigned int j = 0; j < I.getWidth(); j++) {

      int u_round = mapU[i][j];
      int v_round = mapV[i][j];

      float du = mapDu[i][j];
      float dv = mapDv[i][j];

      if (0 <= u_round && 0 <= v_round && u_round < static_cast<int>(I.getWidth()) - 1
          && v_round < static_cast<int>(I.getHeight()) - 1) {
        // process interpolation
        float col0 = lerp(I[v_round][u_round], I[v_round][u_round + 1], du);
        float col1 = lerp(I[v_round + 1][u_round], I[v_round + 1][u_round + 1], du);
        float value = lerp(col0, col1, dv);

        Iundist[i][j] = static_cast<unsigned char>(value);
      } else {
        Iundist[i][j] = 0;
      }
    }
  }
}

/*!
  Apply the transformation map to the image.

  \param I : Input color image.
  \param mapU : Map that contains at each destination coordinate the u-coordinate in the source image.
  \param mapV : Map that contains at each destination coordinate the v-coordinate in the source image.
  \param mapDu : Map that contains at each destination coordinate the \f$ \Delta u \f$ for the interpolation.
  \param mapDv : Map that contains at each destination coordinate the \f$ \Delta v \f$ for the interpolation.
  \param Iundist : Output transformed color image.
*/
void vpImageTools::remap(const vpImage<vpRGBa> &I, const vpArray2D<int> &mapU, const vpArray2D<int> &mapV,
                         const vpArray2D<float> &mapDu, const vpArray2D<float> &mapDv, vpImage<vpRGBa> &Iundist)
{
  Iundist.resize(I.getHeight(), I.getWidth());

  bool checkSSE2 = vpCPUFeatures::checkSSE2();
#if !VISP_HAVE_SSE2
  checkSSE2 = false;
#endif

  if (checkSSE2) {
#if defined VISP_HAVE_SSE2
#if defined _OPENMP // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i_ = 0; i_ < static_cast<int>(I.getHeight()); i_++) {
      const unsigned int i = static_cast<unsigned int>(i_);
      for (unsigned int j = 0; j < I.getWidth(); j++) {

        int u_round = mapU[i][j];
        int v_round = mapV[i][j];

        const __m128 vdu = _mm_set1_ps(mapDu[i][j]);
        const __m128 vdv = _mm_set1_ps(mapDv[i][j]);

        if (0 <= u_round && 0 <= v_round && u_round < static_cast<int>(I.getWidth()) - 1
            && v_round < static_cast<int>(I.getHeight()) - 1) {
  #define VLERP(va, vb, vt) _mm_add_ps(va, _mm_mul_ps(_mm_sub_ps(vb, va), vt));

          // process interpolation
          const __m128 vdata1 =
              _mm_set_ps(static_cast<float>(I[v_round][u_round].A), static_cast<float>(I[v_round][u_round].B),
                         static_cast<float>(I[v_round][u_round].G), static_cast<float>(I[v_round][u_round].R));

          const __m128 vdata2 =
              _mm_set_ps(static_cast<float>(I[v_round][u_round + 1].A), static_cast<float>(I[v_round][u_round + 1].B),
                         static_cast<float>(I[v_round][u_round + 1].G), static_cast<float>(I[v_round][u_round + 1].R));

          const __m128 vdata3 =
              _mm_set_ps(static_cast<float>(I[v_round + 1][u_round].A), static_cast<float>(I[v_round + 1][u_round].B),
                         static_cast<float>(I[v_round + 1][u_round].G), static_cast<float>(I[v_round + 1][u_round].R));

          const __m128 vdata4 = _mm_set_ps(
              static_cast<float>(I[v_round + 1][u_round + 1].A), static_cast<float>(I[v_round + 1][u_round + 1].B),
              static_cast<float>(I[v_round + 1][u_round + 1].G), static_cast<float>(I[v_round + 1][u_round + 1].R));

          const __m128 vcol0 = VLERP(vdata1, vdata2, vdu);
          const __m128 vcol1 = VLERP(vdata3, vdata4, vdu);
          const __m128 vvalue = VLERP(vcol0, vcol1, vdv);

  #undef VLERP

          float values[4];
          _mm_storeu_ps(values, vvalue);
          Iundist[i][j].R = static_cast<unsigned char>(values[0]);
          Iundist[i][j].G = static_cast<unsigned char>(values[1]);
          Iundist[i][j].B = static_cast<unsigned char>(values[2]);
          Iundist[i][j].A = static_cast<unsigned char>(values[3]);
        } else {
          Iundist[i][j] = 0;
        }
      }
    }
#endif
  } else {
#if defined _OPENMP // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i_ = 0; i_ < static_cast<int>(I.getHeight()); i_++) {
      const unsigned int i = static_cast<unsigned int>(i_);
      for (unsigned int j = 0; j < I.getWidth(); j++) {

        int u_round = mapU[i][j];
        int v_round = mapV[i][j];

        float du = mapDu[i][j];
        float dv = mapDv[i][j];

        if (0 <= u_round && 0 <= v_round && u_round < static_cast<int>(I.getWidth()) - 1
            && v_round < static_cast<int>(I.getHeight()) - 1) {
          // process interpolation
          float col0 = lerp(I[v_round][u_round].R, I[v_round][u_round + 1].R, du);
          float col1 = lerp(I[v_round + 1][u_round].G, I[v_round + 1][u_round + 1].G, du);
          float value = lerp(col0, col1, dv);

          Iundist[i][j].R = static_cast<unsigned char>(value);

          col0 = lerp(I[v_round][u_round].G, I[v_round][u_round + 1].G, du);
          col1 = lerp(I[v_round + 1][u_round].G, I[v_round + 1][u_round + 1].G, du);
          value = lerp(col0, col1, dv);

          Iundist[i][j].G = static_cast<unsigned char>(value);

          col0 = lerp(I[v_round][u_round].B, I[v_round][u_round + 1].B, du);
          col1 = lerp(I[v_round + 1][u_round].B, I[v_round + 1][u_round + 1].B, du);
          value = lerp(col0, col1, dv);

          Iundist[i][j].B = static_cast<unsigned char>(value);

          col0 = lerp(I[v_round][u_round].A, I[v_round][u_round + 1].A, du);
          col1 = lerp(I[v_round + 1][u_round].A, I[v_round + 1][u_round + 1].A, du);
          value = lerp(col0, col1, dv);

          Iundist[i][j].A = static_cast<unsigned char>(value);
        } else {
          Iundist[i][j] = 0;
        }
      }
    }
  }
}
