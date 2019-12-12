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

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>

#include <Simd/SimdLib.hpp>

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

  for (unsigned int i = 0; i < I1.getSize(); i++) {
    int diff = I1.bitmap[i] - I2.bitmap[i] + 128;
    Idiff.bitmap[i] = static_cast<unsigned char>(vpMath::maximum(vpMath::minimum(diff, 255), 0));
  }
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
    throw(vpException(vpException::dimensionError, "Cannot compute image difference. The two images "
                                                   "(%ux%u) and (%ux%u) have not the same size",
                      I1.getWidth(), I1.getHeight(), I2.getWidth(), I2.getHeight()));
  }

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

  for (unsigned int i = 0; i < I1.getSize(); i++) {
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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

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

  if ((I1.getHeight() != Idiff.getHeight()) || (I1.getWidth() != Idiff.getWidth())) {
    Idiff.resize(I1.getHeight(), I1.getWidth());
  }

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

  \note The simd lib is used to accelerate processing on x86 and ARM architecture.

  \warning This function does not work in-place (Ires object must be different from I1 and I2).
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

  typedef Simd::View<Simd::Allocator> View;
  View img1(I1.getWidth(), I1.getHeight(), I1.getWidth(), View::Gray8, I1.bitmap);
  View img2(I2.getWidth(), I2.getHeight(), I2.getWidth(), View::Gray8, I2.bitmap);
  View imgAdd(Ires.getWidth(), Ires.getHeight(), Ires.getWidth(), View::Gray8, Ires.bitmap);

  Simd::OperationBinary8u(img1, img2, imgAdd, saturate ? SimdOperationBinary8uSaturatedAddition : SimdOperationBinary8uAddition);
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
                                 vpImage<unsigned char> &Ires, const bool saturate)
{
  if ((I1.getHeight() != I2.getHeight()) || (I1.getWidth() != I2.getWidth())) {
    throw(vpException(vpException::dimensionError, "The two images do not have the same size"));
  }

  if ((I1.getHeight() != Ires.getHeight()) || (I1.getWidth() != Ires.getWidth())) {
    Ires.resize(I1.getHeight(), I1.getWidth());
  }

  typedef Simd::View<Simd::Allocator> View;
  View img1(I1.getWidth(), I1.getHeight(), I1.getWidth(), View::Gray8, I1.bitmap);
  View img2(I2.getWidth(), I2.getHeight(), I2.getWidth(), View::Gray8, I2.bitmap);
  View imgAdd(Ires.getWidth(), Ires.getHeight(), Ires.getWidth(), View::Gray8, Ires.bitmap);

  Simd::OperationBinary8u(img1, img2, imgAdd, saturate ? SimdOperationBinary8uSaturatedSubtraction : SimdOperationBinary8uSubtraction);
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
  Compute a correlation between 2 images.

  \param I1 : The first image.
  \param I2 : The second image.
*/
double vpImageTools::normalizedCorrelation(const vpImage<double> &I1, const vpImage<double> &I2)
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

  for (unsigned int cpt = 0; cpt < I1.getSize(); cpt++) {
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

int vpImageTools::coordCast(double x)
{
  return x < 0 ? -1 : static_cast<int>(x);
}

double vpImageTools::lerp(double A, double B, double t) {
  return A * (1.0 - t) + B * t;
}

float vpImageTools::lerp(float A, float B, float t) {
  return A * (1.0f - t) + B * t;
}

int64_t vpImageTools::lerp2(int64_t A, int64_t B, int64_t t, int64_t t_1) {
  return A * t_1 + B * t;
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
        } else {
          Iundist[i][j] = 0;
        }
      }
    }
}

void vpImageTools::resizeSimdlib(const vpImage<vpRGBa> &Isrc, unsigned int resizeWidth,
                                 unsigned int resizeHeight, vpImage<vpRGBa> &Idst,
                                 int method)
{
  Idst.resize(resizeHeight, resizeWidth);

  typedef Simd::View<Simd::Allocator> View;
  View src(Isrc.getWidth(), Isrc.getHeight(), Isrc.getWidth() * sizeof(vpRGBa), View::Bgra32, Isrc.bitmap);
  View dst(Idst.getWidth(), Idst.getHeight(), Idst.getWidth() * sizeof(vpRGBa), View::Bgra32, Idst.bitmap);

  Simd::Resize(src, dst, method == INTERPOLATION_LINEAR ? SimdResizeMethodBilinear : SimdResizeMethodArea);
}

void vpImageTools::resizeSimdlib(const vpImage<unsigned char> &Isrc, unsigned int resizeWidth,
                                 unsigned int resizeHeight, vpImage<unsigned char> &Idst,
                                 int method)
{
  Idst.resize(resizeHeight, resizeWidth);

  typedef Simd::View<Simd::Allocator> View;
  View src(Isrc.getWidth(), Isrc.getHeight(), Isrc.getWidth(), View::Gray8, Isrc.bitmap);
  View dst(Idst.getWidth(), Idst.getHeight(), Idst.getWidth(), View::Gray8, Idst.bitmap);

  Simd::Resize(src, dst, method == INTERPOLATION_LINEAR ? SimdResizeMethodBilinear : SimdResizeMethodArea);
}

bool vpImageTools::checkFixedPoint(unsigned int x, unsigned int y, const vpMatrix &T, bool affine)
{
  double a0 = T[0][0];  double a1 = T[0][1];  double a2 = T[0][2];
  double a3 = T[1][0];  double a4 = T[1][1];  double a5 = T[1][2];
  double a6 = affine ? 0.0 : T[2][0];
  double a7 = affine ? 0.0 : T[2][1];
  double a8 = affine ? 1.0 : T[2][2];

  double w  = a6 * x + a7 * y + a8;
  double x2 = (a0 * x + a1 * y + a2) / w;
  double y2 = (a3 * x + a4 * y + a5) / w;

  const double limit = 1 << 15;
  return (vpMath::abs(x2) < limit) && (vpMath::abs(y2) < limit);
}
