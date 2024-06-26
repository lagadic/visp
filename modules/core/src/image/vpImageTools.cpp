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
 * Image tools.
 *
*****************************************************************************/

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageException.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.hpp>
#endif

BEGIN_VISP_NAMESPACE
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

  \exception vpImageException::incorrectInitializationError If \f$B \leq A\f$.

  As shown in the example below, this method can be used to binarize
  an image. For an unsigned char image (in the range 0-255),
  thresholding this image at level 127 can be done by:

  \code
  #include <visp3/core/vpImage.h>
  #include <visp3/core/vpImageTools.h>
  #include <visp3/io/vpImageIo.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpImage<unsigned char> I;
  #ifdef _WIN32
    std::string filename("C:/Temp/visp-images/Klimt/Klimt.ppm");
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
    throw(vpImageException(vpImageException::incorrectInitializationError, "Bad gray levels"));
  }
  unsigned char v;

  double factor = static_cast<double>((B_star - A_star) / static_cast<double>((B - A)));

  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < i_width; ++j) {
      v = I[i][j];

      if (v <= A) {
        I[i][j] = A_star;
      }
      else if (v >= B) {
        I[i][j] = B_star;
      }
      else {
        I[i][j] = static_cast<unsigned char>(A_star + (factor * (v - A)));
      }
    }
  }
}

/*!
  Compute the signed difference between the two images I1 and I2 for
  visualization purpose: Idiff = I1-I2

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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

#if defined(VISP_HAVE_SIMDLIB)
  SimdImageDifference(I1.bitmap, I2.bitmap, I1.getSize(), Idiff.bitmap);
#else
  for (unsigned int i = 0; i < I1.getSize(); ++i) {
    int diff = (I1.bitmap[i] - I2.bitmap[i]) + 128;
    Idiff.bitmap[i] = static_cast<unsigned char>(std::max<unsigned char>(std::min<unsigned char>(diff, 255), 0));
  }
#endif
}

/*!
  Compute the signed difference between the two images I1 and I2 RGB
  components for visualization purpose: Idiff = I1-I2. The fourth component
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
    throw(vpException(vpException::dimensionError,
                      "Cannot compute image difference. The two images "
                      "(%ux%u) and (%ux%u) have not the same size",
                      I1.getWidth(), I1.getHeight(), I2.getWidth(), I2.getHeight()));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

#if defined(VISP_HAVE_SIMDLIB)
  SimdImageDifference(reinterpret_cast<unsigned char *>(I1.bitmap), reinterpret_cast<unsigned char *>(I2.bitmap),
                      I1.getSize() * 4, reinterpret_cast<unsigned char *>(Idiff.bitmap));
#else
  unsigned int i1_size = I1.getSize();
  for (unsigned int i = 0; i < (i1_size * 4); ++i) {
    int diffR = (I1.bitmap[i].R - I2.bitmap[i].R) + 128;
    int diffG = (I1.bitmap[i].G - I2.bitmap[i].G) + 128;
    int diffB = (I1.bitmap[i].B - I2.bitmap[i].B) + 128;
    int diffA = (I1.bitmap[i].A - I2.bitmap[i].A) + 128;
    Idiff.bitmap[i].R = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffR, 255), 0));
    Idiff.bitmap[i].G = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffG, 255), 0));
    Idiff.bitmap[i].B = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffB, 255), 0));
    Idiff.bitmap[i].A = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diffA, 255), 0));
  }
#endif
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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; ++b) {
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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; ++b) {
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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

  unsigned int n = I1.getHeight() * I1.getWidth();
  for (unsigned int b = 0; b < n; ++b) {
    int diffR = I1.bitmap[b].R - I2.bitmap[b].R;
    int diffG = I1.bitmap[b].G - I2.bitmap[b].G;
    int diffB = I1.bitmap[b].B - I2.bitmap[b].B;
    // --comment: int diffA eq I1 dot bitmap[b] dot A minus I2 dot bitmap[b] dot A
    Idiff.bitmap[b].R = static_cast<unsigned char>(vpMath::abs(diffR));
    Idiff.bitmap[b].G = static_cast<unsigned char>(vpMath::abs(diffG));
    Idiff.bitmap[b].B = static_cast<unsigned char>(vpMath::abs(diffB));
    // --comment: Idiff dot bitmap[b] dot A eq diffA
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

  \note The simd lib is used to accelerate processing on x86 and ARM architecture.

  \warning This function does not work in-place (Ires object must be different from I1 and I2).
*/
void vpImageTools::imageAdd(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                            vpImage<unsigned char> &Ires, bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

#if defined(VISP_HAVE_SIMDLIB)
  typedef Simd::View<Simd::Allocator> View;
  View img1(I1.getWidth(), I1.getHeight(), I1.getWidth(), View::Gray8, I1.bitmap);
  View img2(I2.getWidth(), I2.getHeight(), I2.getWidth(), View::Gray8, I2.bitmap);
  View imgAdd(Ires.getWidth(), Ires.getHeight(), Ires.getWidth(), View::Gray8, Ires.bitmap);

  Simd::OperationBinary8u(img1, img2, imgAdd,
                          saturate ? SimdOperationBinary8uSaturatedAddition : SimdOperationBinary8uAddition);
#else
  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;
  unsigned int ires_size = Ires.getSize();
  for (unsigned int cpt = 0; cpt < ires_size; ++cpt, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ? vpMath::saturate<unsigned char>(static_cast<short int>(*ptr_I1) + static_cast<short int>(*ptr_I2)) : ((*ptr_I1) + (*ptr_I2));
  }
#endif
}

/*!
  Compute the image addition: \f$ Ires = I1 - I2 \f$.

  \param I1 : The first image.
  \param I2 : The second image.
  \param Ires : \f$ Ires = I1 - I2 \f$
  \param saturate : If true, saturate the result to [0 ; 255] using
  vpMath::saturate, otherwise overflow may occur.

  \note The simd lib is used to accelerate processing on x86 and ARM architecture.

  \warning This function does not work in-place (Ires object must be different from I1 and I2).
*/
void vpImageTools::imageSubtract(const vpImage<unsigned char> &I1, const vpImage<unsigned char> &I2,
                                 vpImage<unsigned char> &Ires, bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

#if defined(VISP_HAVE_SIMDLIB)
  typedef Simd::View<Simd::Allocator> View;
  View img1(I1.getWidth(), I1.getHeight(), I1.getWidth(), View::Gray8, I1.bitmap);
  View img2(I2.getWidth(), I2.getHeight(), I2.getWidth(), View::Gray8, I2.bitmap);
  View imgAdd(Ires.getWidth(), Ires.getHeight(), Ires.getWidth(), View::Gray8, Ires.bitmap);

  Simd::OperationBinary8u(img1, img2, imgAdd,
                          saturate ? SimdOperationBinary8uSaturatedSubtraction : SimdOperationBinary8uSubtraction);
#else
  unsigned char *ptr_I1 = I1.bitmap;
  unsigned char *ptr_I2 = I2.bitmap;
  unsigned char *ptr_Ires = Ires.bitmap;
  unsigned int ires_size = Ires.getSize();
  for (unsigned int cpt = 0; cpt < ires_size; ++cpt, ++ptr_I1, ++ptr_I2, ++ptr_Ires) {
    *ptr_Ires = saturate ?
      vpMath::saturate<unsigned char>(static_cast<short int>(*ptr_I1) - static_cast<short int>(*ptr_I2)) :
      ((*ptr_I1) - (*ptr_I2));
  }
#endif
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
                                    vpArray2D<int> &mapU, vpArray2D<int> &mapV, vpArray2D<float> &mapDu,
                                    vpArray2D<float> &mapDv)
{
  mapU.resize(height, width, false, false);
  mapV.resize(height, width, false, false);
  mapDu.resize(height, width, false, false);
  mapDv.resize(height, width, false, false);

  vpCameraParameters::vpCameraParametersProjType projModel = cam.get_projModel();
  bool is_KannalaBrandt =
    (projModel == vpCameraParameters::ProjWithKannalaBrandtDistortion); // Check the projection model used

  float u0 = static_cast<float>(cam.get_u0());
  float v0 = static_cast<float>(cam.get_v0());
  float px = static_cast<float>(cam.get_px());
  float py = static_cast<float>(cam.get_py());
  float kud = 0;
  std::vector<double> dist_coefs;

  if (!is_KannalaBrandt) {
    kud = static_cast<float>(cam.get_kud());
  }
  else {
    dist_coefs = cam.getKannalaBrandtDistortionCoefficients();
  }

  if ((!is_KannalaBrandt) && (std::fabs(static_cast<double>(kud)) <= std::numeric_limits<double>::epsilon())) {
    // There is no need to undistort the image (Perpective projection)
    for (unsigned int i = 0; i < height; ++i) {
      for (unsigned int j = 0; j < width; ++j) {
        mapU[i][j] = static_cast<int>(j);
        mapV[i][j] = static_cast<int>(i);
        mapDu[i][j] = 0;
        mapDv[i][j] = 0;
      }
    }

    return;
  }

  float invpx, invpy;
  float kud_px2 = 0., kud_py2 = 0., deltau_px, deltav_py = 0;
  float fr1 = 0, fr2;
  float deltav, deltau;
  float u_float, v_float;
  int u_round, v_round;
  double r, scale;
  double theta, theta_d;
  double theta2, theta4, theta6, theta8;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  invpx = 1.0f / px;
  invpy = 1.0f / py;

  if (!is_KannalaBrandt) {
    kud_px2 = kud * invpx * invpx;
    kud_py2 = kud * invpy * invpy;
  }

  for (unsigned int v = 0; v < height; ++v) {
    deltav = v - v0;

    if (!is_KannalaBrandt) {
      fr1 = 1.0f + (kud_py2 * deltav * deltav);
    }
    else {
      deltav_py = deltav * invpy;
    }

    for (unsigned int u = 0; u < width; ++u) {
      // computation of u,v : corresponding pixel coordinates in I.
      deltau = u - u0;
      if (!is_KannalaBrandt) {
        fr2 = fr1 + (kud_px2 * deltau * deltau);

        u_float = (deltau * fr2) + u0;
        v_float = (deltav * fr2) + v0;
      }

      else {
        deltau_px = deltau * invpx;
        r = sqrt(vpMath::sqr(deltau_px) + vpMath::sqr(deltav_py));
        theta = atan(r);

        theta2 = vpMath::sqr(theta);
        theta4 = vpMath::sqr(theta2);
        theta6 = theta2 * theta4;
        theta8 = vpMath::sqr(theta4);

        theta_d = theta * (1 + (dist_coefs[index_0] * theta2) + (dist_coefs[index_1] * theta4) + (dist_coefs[index_2] * theta6) +
                           (dist_coefs[index_3] * theta8));

        // --comment: scale eq (r == 0) 1.0 otherwise theta_d / r
        scale = (std::fabs(r) < std::numeric_limits<double>::epsilon()) ? 1.0 : (theta_d / r);
        u_float = static_cast<float>((deltau * scale) + u0);
        v_float = static_cast<float>((deltav * scale) + v0);
      }

      u_round = static_cast<int>(u_float);
      v_round = static_cast<int>(v_float);

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

  unsigned int ii_height = II.getHeight();
  unsigned int ii_width = II.getWidth();
  for (unsigned int i = 1; i < ii_height; ++i) {
    for (unsigned int j = 1; j < ii_width; ++j) {
      II[i][j] = (I[i - 1][j - 1] + II[i - 1][j] + II[i][j - 1]) - II[i - 1][j - 1];
      IIsq[i][j] = (vpMath::sqr(I[i - 1][j - 1]) + IIsq[i - 1][j] + IIsq[i][j - 1]) - IIsq[i - 1][j - 1];
    }
  }
}

/*!
  Compute a correlation between 2 images.

  \param I1 : The first image.
  \param I2 : The second image.
  \param useOptimized : Use SSE if true and available.
*/
double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2, bool useOptimized)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw vpException(vpException::dimensionError,
                      "Error: in vpImageTools::normalizedCorrelation(): "
                      "image dimension mismatch between I1=%ux%u and I2=%ux%u",
                      I1.getHeight(), I1.getWidth(), I2.getHeight(), I2.getWidth());
  }

  const double a = I1.getMeanValue();
  const double b = I2.getMeanValue();

  double ab = 0.0;
  double a2 = 0.0;
  double b2 = 0.0;

#if defined(VISP_HAVE_SIMDLIB)
  SimdNormalizedCorrelation(I1.bitmap, a, I2.bitmap, b, I1.getSize(), a2, b2, ab, useOptimized);
#else
  unsigned int i1_size = I1.getSize();
  for (unsigned int cpt = 0; cpt < i1_size; ++cpt) {
    ab += (I1.bitmap[cpt] - a) * (I2.bitmap[cpt] - b);
    a2 += vpMath::sqr(I1.bitmap[cpt] - a);
    b2 += vpMath::sqr(I2.bitmap[cpt] - b);
  }
  (void)useOptimized;
#endif

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

  for (unsigned int i = 0; i < height; ++i) {
    for (unsigned int j = 0; j < width; ++j) {
      V[j] += I[i][j];
    }
  }
  for (unsigned int j = 0; j < width; ++j) {
    V[j] /= height;
  }
}

/*!
  Normalize the image intensities.
  \param I : The image to normalize.
*/
void vpImageTools::normalize(vpImage<double> &I)
{
  double s = I.getSum();
  unsigned int i_height = I.getHeight();
  unsigned int i_width = I.getWidth();
  for (unsigned int i = 0; i < i_height; ++i) {
    for (unsigned int j = 0; j < i_width; ++j) {
      I(i, j, I(i, j) / s);
    }
  }
}

namespace
{
/*!
* Get the interpolated value at a given location using the nearest points.
* \param I : The image to perform intepolation in.
* \param point : The image point.
*/
double interpolationNearest(const vpImage<unsigned char> &I, const vpImagePoint &point)
{
  int x1 = static_cast<int>(floor(point.get_i()));
  int x2 = static_cast<int>(ceil(point.get_i()));
  int y1 = static_cast<int>(floor(point.get_j()));
  int y2 = static_cast<int>(ceil(point.get_j()));
  double v1, v2;
  if (x1 == x2) {
    v1 = I(x1, y1);
    v2 = I(x1, y2);
  }
  else {
    v1 = ((x2 - point.get_i()) * I(x1, y1)) + ((point.get_i() - x1) * I(x2, y1));
    v2 = ((x2 - point.get_i()) * I(x1, y2)) + ((point.get_i() - x1) * I(x2, y2));
  }
  if (y1 == y2) {
    return v1;
  }
  return ((y2 - point.get_j()) * v1) + ((point.get_j() - y1) * v2);
}
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
    return interpolationNearest(I, point);
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
  \param src : The source image.
  \param dst : The resulting image.
  \param r : The rectangle area.
*/
void vpImageTools::extract(const vpImage<unsigned char> &src, vpImage<unsigned char> &dst, const vpRectOriented &r)
{
  unsigned int x_d = vpMath::round(r.getHeight());
  unsigned int y_d = vpMath::round(r.getWidth());
  double x1 = r.getTopLeft().get_i();
  double y1 = r.getTopLeft().get_j();
  double t = r.getOrientation();
  double cos_t = cos(t);
  double sin_t = sin(t);
  dst.resize(x_d, y_d);
  for (unsigned int x = 0; x < x_d; ++x) {
    double x_cos_t = x * cos_t;
    double x_sin_t = x * sin_t;
    for (unsigned int y = 0; y < y_d; ++y) {
      dst(x, y,
          static_cast<unsigned char>(interpolate(src, vpImagePoint(x1 + x_cos_t + (y * sin_t), (y1 - x_sin_t) + (y * cos_t)),
                                                 vpImageTools::INTERPOLATION_LINEAR)));
    }
  }
}

/*!
  Extract a rectangular region from an image.
  \param src : The source image.
  \param dst : The resulting image.
  \param r : The rectangle area.
*/
void vpImageTools::extract(const vpImage<unsigned char> &src, vpImage<double> &dst, const vpRectOriented &r)
{
  unsigned int x_d = vpMath::round(r.getHeight());
  unsigned int y_d = vpMath::round(r.getWidth());
  double x1 = r.getTopLeft().get_i();
  double y1 = r.getTopLeft().get_j();
  double t = r.getOrientation();
  double cos_t = cos(t);
  double sin_t = sin(t);
  dst.resize(x_d, y_d);
  for (unsigned int x = 0; x < x_d; ++x) {
    double x_cos_t = x * cos_t;
    double x_sin_t = x * sin_t;
    for (unsigned int y = 0; y < y_d; ++y) {
      dst(x, y,
          interpolate(src, vpImagePoint(x1 + x_cos_t + (y * sin_t), (y1 - x_sin_t) + (y * cos_t)),
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
                                    vpImage<double> &I_score, unsigned int step_u, unsigned int step_v,
                                    bool useOptimized)
{
  if (I.getSize() == 0) {
    std::cerr << "Error, input image is empty." << std::endl;
    return;
  }

  if (I_tpl.getSize() == 0) {
    std::cerr << "Error, template image is empty." << std::endl;
    return;
  }

  if ((I_tpl.getHeight() > I.getHeight()) || (I_tpl.getWidth() > I.getWidth())) {
    std::cerr << "Error, template image is bigger than input image." << std::endl;
    return;
  }

  vpImage<double> I_double, I_tpl_double;
  vpImageConvert::convert(I, I_double);
  vpImageConvert::convert(I_tpl, I_tpl_double);

  unsigned int height_tpl = I_tpl.getHeight(), width_tpl = I_tpl.getWidth();
  I_score.resize(I.getHeight() - height_tpl, I.getWidth() - width_tpl, 0.0);

  if (useOptimized) {
    vpImage<double> II, IIsq;
    integralImage(I, II, IIsq);

    vpImage<double> II_tpl, IIsq_tpl;
    integralImage(I_tpl, II_tpl, IIsq_tpl);

    // zero-mean template image
    const double sum2 = (((II_tpl[height_tpl][width_tpl] + II_tpl[0][0]) - II_tpl[0][width_tpl]) - II_tpl[height_tpl][0]);
    const double mean2 = sum2 / I_tpl.getSize();
    unsigned int i_tpl_double_size = I_tpl_double.getSize();
    for (unsigned int cpt = 0; cpt < i_tpl_double_size; ++cpt) {
      I_tpl_double.bitmap[cpt] -= mean2;
    }

#if defined(_OPENMP) && (_OPENMP >= 200711) // OpenMP 3.1
#pragma omp parallel for schedule(dynamic)
    for (unsigned int i = 0; i < I.getHeight() - height_tpl; i += step_v) {
      for (unsigned int j = 0; j < I.getWidth() - width_tpl; j += step_u) {
        I_score[i][j] = normalizedCorrelation(I_double, I_tpl_double, II, IIsq, II_tpl, IIsq_tpl, i, j);
      }
    }
#else
    // error C3016: 'i': index variable in OpenMP 'for' statement must have signed integral type
    int end = static_cast<int>((I.getHeight() - height_tpl) / step_v) + 1;
    std::vector<unsigned int> vec_step_v(static_cast<size_t>(end));
    unsigned int i_height = I.getHeight();
    for (unsigned int cpt = 0, idx = 0; cpt < (i_height - height_tpl); cpt += step_v, ++idx) {
      vec_step_v[static_cast<size_t>(idx)] = cpt;
    }
#if defined(_OPENMP) // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
    for (int cpt = 0; cpt < end; ++cpt) {
      unsigned int i_width = I.getWidth();
      for (unsigned int j = 0; j < (i_width - width_tpl); j += step_u) {
        I_score[vec_step_v[cpt]][j] =
          normalizedCorrelation(I_double, I_tpl_double, II, IIsq, II_tpl, IIsq_tpl, vec_step_v[cpt], j);
      }
    }
#endif
  }
  else {
    vpImage<double> I_cur;

    unsigned int i_height = I.getHeight();
    unsigned int i_width = I.getWidth();
    for (unsigned int i = 0; i < (i_height - height_tpl); i += step_v) {
      for (unsigned int j = 0; j < (i_width - width_tpl); j += step_u) {
        vpRect roi(vpImagePoint(i, j), vpImagePoint(((i + height_tpl) - 1), ((j + width_tpl) - 1)));
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
  float a = (((-A + (3.0f * B)) - (3.0f * C)) + D) / 2.0f;
  float b = (A + (2.0f * C)) - (((5.0f * B) + D) / 2.0f);
  float c = (-A + C) / 2.0f;
  float d = B;

  return (a * t * t * t) + (b * t * t) + (c * t) + d;
}

int vpImageTools::coordCast(double x) { return x < 0 ? -1 : static_cast<int>(x); }

double vpImageTools::lerp(double A, double B, double t) { return (A * (1.0 - t)) + (B * t); }

float vpImageTools::lerp(float A, float B, float t) { return (A * (1.0f - t)) + (B * t); }

int64_t vpImageTools::lerp2(int64_t A, int64_t B, int64_t t, int64_t t_1) { return (A * t_1) + (B * t); }

double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2,
                                           const vpImage<double> &II, const vpImage<double> &IIsq,
                                           const vpImage<double> &II_tpl, const vpImage<double> &IIsq_tpl,
                                           unsigned int i0, unsigned int j0)
{
  double ab = 0.0;

#if defined(VISP_HAVE_SIMDLIB)
  SimdNormalizedCorrelation2(I1.bitmap, I1.getWidth(), I2.bitmap, I2.getWidth(), I2.getHeight(), i0, j0, ab);
#else
  unsigned int i2_height = I2.getHeight();
  unsigned int i2_width = I2.getWidth();
  for (unsigned int i = 0; i < i2_height; ++i) {
    for (unsigned int j = 0; j < i2_width; ++j) {
      ab += (I1[i0 + i][j0 + j]) * I2[i][j];
    }
  }
#endif

  unsigned int height_tpl = I2.getHeight(), width_tpl = I2.getWidth();
  const double sum1 =
    (((II[i0 + height_tpl][j0 + width_tpl] + II[i0][j0]) - II[i0][j0 + width_tpl]) - II[i0 + height_tpl][j0]);
  const double sum2 = (((II_tpl[height_tpl][width_tpl] + II_tpl[0][0]) - II_tpl[0][width_tpl]) - II_tpl[height_tpl][0]);

  double a2 = ((((IIsq[i0 + I2.getHeight()][j0 + I2.getWidth()] + IIsq[i0][j0]) - IIsq[i0][j0 + I2.getWidth()]) -
                IIsq[i0 + I2.getHeight()][j0]) -
               ((1.0 / I2.getSize()) * vpMath::sqr(sum1)));

  double b2 = ((((IIsq_tpl[I2.getHeight()][I2.getWidth()] + IIsq_tpl[0][0]) - IIsq_tpl[0][I2.getWidth()]) -
                IIsq_tpl[I2.getHeight()][0]) -
               ((1.0 / I2.getSize()) * vpMath::sqr(sum2)));
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

#if defined(_OPENMP) // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i_ = 0; i_ < static_cast<int>(I.getHeight()); ++i_) {
    const unsigned int i = static_cast<unsigned int>(i_);
    unsigned int i_width = I.getWidth();
    for (unsigned int j = 0; j < i_width; ++j) {

      int u_round = mapU[i][j];
      int v_round = mapV[i][j];

      float du = mapDu[i][j];
      float dv = mapDv[i][j];

      if ((0 <= u_round) && (0 <= v_round) && (u_round < (static_cast<int>(I.getWidth()) - 1)) &&
          (v_round < (static_cast<int>(I.getHeight()) - 1))) {
        // process interpolation
        float col0 = lerp(I[v_round][u_round], I[v_round][u_round + 1], du);
        float col1 = lerp(I[v_round + 1][u_round], I[v_round + 1][u_round + 1], du);
        float value = lerp(col0, col1, dv);

        Iundist[i][j] = static_cast<unsigned char>(value);
      }
      else {
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

#if defined(_OPENMP) // only to disable warning: ignoring #pragma omp parallel [-Wunknown-pragmas]
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < static_cast<int>(I.getHeight()); ++i) {
#if defined(VISP_HAVE_SIMDLIB)
    SimdRemap(reinterpret_cast<unsigned char *>(I.bitmap), 4, I.getWidth(), I.getHeight(), i * I.getWidth(), mapU.data,
              mapV.data, mapDu.data, mapDv.data, reinterpret_cast<unsigned char *>(Iundist.bitmap));
#else
    const unsigned int i_ = static_cast<unsigned int>(i);
    unsigned int i_width = I.getWidth();
    for (unsigned int j = 0; j < i_width; ++j) {

      int u_round = mapU[i_][j];
      int v_round = mapV[i_][j];

      float du = mapDu[i_][j];
      float dv = mapDv[i_][j];

      if ((0 <= u_round) && (0 <= v_round) && (u_round < (static_cast<int>(I.getWidth()) - 1))
          && (v_round < (static_cast<int>(I.getHeight()) - 1))) {
        // process interpolation
        float col0 = lerp(I[v_round][u_round].R, I[v_round][u_round + 1].R, du);
        float col1 = lerp(I[v_round + 1][u_round].R, I[v_round + 1][u_round + 1].R, du);
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
      }
      else {
        Iundist[i][j] = 0;
      }
    }
#endif
  }
}

#if defined(VISP_HAVE_SIMDLIB)
void vpImageTools::resizeSimdlib(const vpImage<vpRGBa> &Isrc, unsigned int resizeWidth, unsigned int resizeHeight,
                                 vpImage<vpRGBa> &Idst, int method)
{
  Idst.resize(resizeHeight, resizeWidth);

  typedef Simd::View<Simd::Allocator> View;
  View src(Isrc.getWidth(), Isrc.getHeight(), Isrc.getWidth() * sizeof(vpRGBa), View::Bgra32, Isrc.bitmap);
  View dst(Idst.getWidth(), Idst.getHeight(), Idst.getWidth() * sizeof(vpRGBa), View::Bgra32, Idst.bitmap);

  Simd::Resize(src, dst, method == INTERPOLATION_LINEAR ? SimdResizeMethodBilinear : SimdResizeMethodArea);
}

void vpImageTools::resizeSimdlib(const vpImage<unsigned char> &Isrc, unsigned int resizeWidth,
                                 unsigned int resizeHeight, vpImage<unsigned char> &Idst, int method)
{
  Idst.resize(resizeHeight, resizeWidth);

  typedef Simd::View<Simd::Allocator> View;
  View src(Isrc.getWidth(), Isrc.getHeight(), Isrc.getWidth(), View::Gray8, Isrc.bitmap);
  View dst(Idst.getWidth(), Idst.getHeight(), Idst.getWidth(), View::Gray8, Idst.bitmap);

  Simd::Resize(src, dst, method == INTERPOLATION_LINEAR ? SimdResizeMethodBilinear : SimdResizeMethodArea);
}
#endif

bool vpImageTools::checkFixedPoint(unsigned int x, unsigned int y, const vpMatrix &T, bool affine)
{
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

  double w = (a6 * x) + (a7 * y) + a8;
  double x2 = ((a0 * x) + (a1 * y) + a2) / w;
  double y2 = ((a3 * x) + (a4 * y) + a5) / w;

  const double limit = 1 << 15;
  return (vpMath::abs(x2) < limit) && (vpMath::abs(y2) < limit);
}

/*!
 * Keep the part of an image that is in the mask.
 * @param[in] I : Input image.
 * @param[in] mask : Mask where pixels to consider have values that differ from 0.
 * @param[out] I_mask : Resulting image where pixels that are in the mask are kept.
 * @return The number of pixels that are in the mask.
 */
int vpImageTools::inMask(const vpImage<vpRGBa> &I, const vpImage<unsigned char> &mask, vpImage<vpRGBa> &I_mask)
{
  if ((I.getHeight() != mask.getHeight()) || (I.getWidth() != mask.getWidth())) {
    throw(vpImageException(vpImageException::incorrectInitializationError,
                           "Error in vpImageTools::inMask(): image (%dx%d) and mask (%dx%d) size doesn't match",
                           I.getWidth(), I.getHeight(), mask.getWidth(), mask.getHeight()));
  }
  vpRGBa black(0, 0, 0);
  I_mask.resize(I.getHeight(), I.getWidth());
  int cpt_in_mask = 0;
  int size_ = static_cast<int>(I.getSize());
#if defined(_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_mask)
#endif
  for (int i = 0; i < size_; ++i) {
    if (mask.bitmap[i] == 0) {
      I_mask.bitmap[i] = black;
    }
    else {
      I_mask.bitmap[i] = I.bitmap[i];
      ++cpt_in_mask;
    }
  }
  return cpt_in_mask;
}

/*!
 * Keep the part of an image that is in the mask.
 * @param[in] I : Input image.
 * @param[in] mask : Mask where pixels to consider have values that differ from 0.
 * @param[out] I_mask : Resulting image where pixels that are in the mask are kept.
 * @return The number of pixels that are in the mask.
 */
int vpImageTools::inMask(const vpImage<unsigned char> &I, const vpImage<unsigned char> &mask, vpImage<unsigned char> &I_mask)
{
  if ((I.getHeight() != mask.getHeight()) || (I.getWidth() != mask.getWidth())) {
    throw(vpImageException(vpImageException::incorrectInitializationError,
                           "Error in vpImageTools::inMask(): image (%dx%d) and mask (%dx%d) size doesn't match",
                           I.getWidth(), I.getHeight(), mask.getWidth(), mask.getHeight()));
  }
  I_mask.resize(I.getHeight(), I.getWidth());
  int cpt_in_mask = 0;
  int size_ = static_cast<int>(I.getSize());
#if defined(_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_mask)
#endif
  for (int i = 0; i < size_; ++i) {
    if (mask.bitmap[i] == 0) {
      I_mask.bitmap[i] = 0;
    }
    else {
      I_mask.bitmap[i] = I.bitmap[i];
      ++cpt_in_mask;
    }
  }
  return cpt_in_mask;
}

/*!
 * Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
 * \param[in] hue : Pointer to an array of hue values. Its dimension is equal to the `size` parameter.
 * \param[in] saturation : Pointer to an array of saturation values. Its dimension is equal to the `size` parameter.
 * \param[in] value : Pointer to an array of values. Its dimension is equal to the `size` parameter.
 * \param[in] hsv_range : 6-dim vector that contains the low/high range values for each HSV channel respectively.
 * Each element of this vector should be in [0,255] range. Note that there is also tutorial-hsv-tuner.cpp that may help
 * to determine low/high HSV values.
 * \param[out] mask : Pointer to a resulting mask of dimension `size`. When HSV value is in the boundaries, the mask
 * element is set to 255, otherwise to 0. The mask should be allocated prior calling this function. Its dimension
 * is equal to the `size` parameter.
 * \param[in] size : Size of `hue`, `saturation`, `value` and `mask` arrays.
 *
 * \sa vpImageConvert::RGBToHSV(const unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned int, bool)
 * \sa vpImageConvert::RGBaToHSV(const unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned int, bool)
 */
int vpImageTools::inRange(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                          const vpColVector &hsv_range, unsigned char *mask, unsigned int size)
{
  if ((hue == nullptr) || (saturation == nullptr) || (value == nullptr)) {
    throw(vpImageException(vpImageException::notInitializedError,
                           "Error in vpImageTools::inRange(): hsv pointer are empty"));
  }
  else if (hsv_range.size() != 6) {
    throw(vpImageException(vpImageException::notInitializedError,
                           "Error in vpImageTools::inRange(): wrong values vector size (%d)", hsv_range.size()));
  }
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  unsigned char h_low = static_cast<unsigned char>(hsv_range[index_0]);
  unsigned char h_high = static_cast<unsigned char>(hsv_range[index_1]);
  unsigned char s_low = static_cast<unsigned char>(hsv_range[index_2]);
  unsigned char s_high = static_cast<unsigned char>(hsv_range[index_3]);
  unsigned char v_low = static_cast<unsigned char>(hsv_range[index_4]);
  unsigned char v_high = static_cast<unsigned char>(hsv_range[index_5]);
  int size_ = static_cast<int>(size);
  int cpt_in_range = 0;
#if defined(_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_range)
#endif
  for (int i = 0; i < size_; ++i) {
    bool check_h_low_high_hue = (h_low <= hue[i]) && (hue[i] <= h_high);
    bool check_s_low_high_saturation = (s_low <= saturation[i]) && (saturation[i] <= s_high);
    bool check_v_low_high_value = (v_low <= value[i]) && (value[i] <= v_high);
    if (check_h_low_high_hue && check_s_low_high_saturation && check_v_low_high_value) {
      mask[i] = 255;
      ++cpt_in_range;
    }
    else {
      mask[i] = 0;
    }
  }
  return cpt_in_range;
}

/*!
 * Create binary mask by checking if HSV (hue, saturation, value) channels lie between low and high HSV thresholds.
 * \param[in] hue : Pointer to an array of hue values. Its dimension is equal to the `size` parameter.
 * \param[in] saturation : Pointer to an array of saturation values. Its dimension is equal to the `size` parameter.
 * \param[in] value : Pointer to an array of values. Its dimension is equal to the `size` parameter.
 * \param[in] hsv_range : 6-dim vector that contains the low/high range values for each HSV channel respectively.
 * Each element of this vector should be in [0,255] range. Note that there is also tutorial-hsv-tuner.cpp that may help
 * to determine low/high HSV values.
 * \param[out] mask : Pointer to a resulting mask of dimension `size`. When HSV value is in the boundaries, the mask
 * element is set to 255, otherwise to 0. The mask should be allocated prior calling this function. Its dimension
 * is equal to the `size` parameter.
 * \param[in] size : Size of `hue`, `saturation`, `value` and `mask` arrays.
 * \return The number of pixels that are in the HSV range.
 *
 * \sa vpImageConvert::RGBToHSV(const unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned int, bool)
 * \sa vpImageConvert::RGBaToHSV(const unsigned char *, unsigned char *, unsigned char *, unsigned char *, unsigned int, bool)
 */
int vpImageTools::inRange(const unsigned char *hue, const unsigned char *saturation, const unsigned char *value,
                           const std::vector<int> &hsv_range, unsigned char *mask, unsigned int size)
{
  if ((hue == nullptr) || (saturation == nullptr) || (value == nullptr)) {
    throw(vpImageException(vpImageException::notInitializedError,
                           "Error in vpImageTools::inRange(): hsv pointer are empty"));
  }
  else if (hsv_range.size() != 6) {
    throw(vpImageException(vpImageException::notInitializedError,
                           "Error in vpImageTools::inRange(): wrong values vector size (%d)", hsv_range.size()));
  }
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  unsigned char h_low = static_cast<unsigned char>(hsv_range[index_0]);
  unsigned char h_high = static_cast<unsigned char>(hsv_range[index_1]);
  unsigned char s_low = static_cast<unsigned char>(hsv_range[index_2]);
  unsigned char s_high = static_cast<unsigned char>(hsv_range[index_3]);
  unsigned char v_low = static_cast<unsigned char>(hsv_range[index_4]);
  unsigned char v_high = static_cast<unsigned char>(hsv_range[index_5]);
  int size_ = static_cast<int>(size);
  int cpt_in_range = 0;
#if defined(_OPENMP)
#pragma omp parallel for reduction(+:cpt_in_range)
#endif
  for (int i = 0; i < size_; ++i) {
    bool check_h_low_high_hue = (h_low <= hue[i]) && (hue[i] <= h_high);
    bool check_s_low_high_saturation = (s_low <= saturation[i]) && (saturation[i] <= s_high);
    bool check_v_low_high_value = (v_low <= value[i]) && (value[i] <= v_high);
    if (check_h_low_high_hue && check_s_low_high_saturation && check_v_low_high_value) {
      mask[i] = 255;
      ++cpt_in_range;
    }
    else {
      mask[i] = 0;
    }
  }
  return cpt_in_range;
}
END_VISP_NAMESPACE
