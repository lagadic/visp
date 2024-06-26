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
 */
/* Autostretch HSV 0.10 --- image filter plug-in for GIMP
 *
 * Copyright (C) 1997 Scott Goehring
 * Copyright (C) 1996 Federico Mena Quintero
 *
 * You can contact me at scott@poverty.bloomington.in.us
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*!
  \file vpImgproc.cpp
  \brief Basic image processing functions.
*/

#include <visp3/core/vpGaussianFilter.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/imgproc/vpImgproc.h>

namespace VISP_NAMESPACE_NAME
{
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS) && defined(ENABLE_VISP_NAMESPACE)
using namespace VISP_NAMESPACE_NAME;
#endif

std::string vpGammaMethodList(const std::string &pref, const std::string &sep, const std::string &suf)
{
  std::string list(pref);
  for (unsigned int i = 0; i < (GAMMA_METHOD_COUNT - 1); ++i) {
    vpGammaMethod type = static_cast<vpGammaMethod>(i);
    list += vpGammaMethodToString(type);
    list += sep;
  }
  vpGammaMethod type = static_cast<vpGammaMethod>(GAMMA_METHOD_COUNT - 1);
  list += vpGammaMethodToString(type);
  list += suf;
  return list;
}

std::string vpGammaMethodToString(const vpGammaMethod &type)
{
  std::string name;
  switch (type) {
  case GAMMA_MANUAL:
    name = "gamma_manual";
    break;
  case  GAMMA_LOG_BASED:
    name = "gamma_log";
    break;
  case  GAMMA_NONLINEAR_BASED:
    name = "gamma_nonlinear";
    break;
  case  GAMMA_CDF_BASED:
    name = "gamma_cdf";
    break;
  case  GAMMA_CLASSIFICATION_BASED:
    name = "gamma_classification";
    break;
  case  GAMMA_SPATIAL_VARIANT_BASED:
    name = "gamma_spatial_variant";
    break;
  case  GAMMA_METHOD_COUNT:
  default:
    name = "gamma_method_unknown";
  }
  return name;
}

vpGammaMethod vpGammaMethodFromString(const std::string &name)
{
  vpGammaMethod type(GAMMA_METHOD_COUNT);
  unsigned int count = static_cast<unsigned int>(GAMMA_METHOD_COUNT);
  bool notFound = true;
  unsigned int i = 0;
  while ((i < count) && notFound) {
    vpGammaMethod temp = static_cast<vpGammaMethod>(i);
    if (name == vpGammaMethodToString(temp)) {
      type = temp;
      notFound = false;
    }
    ++i;
  }
  return type;
}

std::string vpGammaColorHandlingList(const std::string &pref, const std::string &sep, const std::string &suf)
{
  std::string list(pref);
  for (unsigned int i = 0; i < (GAMMA_COLOR_HANDLING_COUNT - 1); ++i) {
    vpGammaColorHandling type = static_cast<vpGammaColorHandling>(i);
    list += vpGammaColorHandlingToString(type);
    list += sep;
  }
  vpGammaColorHandling type = static_cast<vpGammaColorHandling>(GAMMA_COLOR_HANDLING_COUNT - 1);
  list += vpGammaColorHandlingToString(type);
  list += suf;
  return list;
}

std::string vpGammaColorHandlingToString(const vpGammaColorHandling &type)
{
  std::string name;
  switch (type) {
  case  GAMMA_RGB:
    name = "gamma_color_rgb";
    break;
  case  GAMMA_HSV:
    name = "gamma_color_hsv";
    break;
  case  GAMMA_COLOR_HANDLING_COUNT:
  default:
    name = "gamma_color_unknown";
  }
  return name;
}

vpGammaColorHandling vpGammaColorHandlingFromString(const std::string &name)
{
  vpGammaColorHandling type(GAMMA_COLOR_HANDLING_COUNT);
  unsigned int count = static_cast<unsigned int>(GAMMA_COLOR_HANDLING_COUNT);
  bool notFound = true;
  unsigned int i = 0;
  while ((i < count) && notFound) {
    vpGammaColorHandling temp = static_cast<vpGammaColorHandling>(i);
    if (name == vpGammaColorHandlingToString(temp)) {
      type = temp;
      notFound = false;
    }
    ++i;
  }
  return type;
}

void adjust(vpImage<unsigned char> &I, double alpha, double beta)
{
  // Construct the look-up table
  const unsigned int lutSize = 256;
  unsigned char lut[lutSize];
  for (unsigned int i = 0; i < lutSize; ++i) {
    lut[i] = vpMath::saturate<unsigned char>((alpha * i) + beta);
  }

  // Apply the transformation using a LUT
  I.performLut(lut);
}

void adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, double alpha, double beta)
{
  // Copy I1 to I2
  I2 = I1;

  adjust(I2, alpha, beta);
}

void adjust(vpImage<vpRGBa> &I, double alpha, double beta)
{
  // Construct the look-up table
  const unsigned int lutSize = 256;
  vpRGBa lut[lutSize];
  for (unsigned int i = 0; i < lutSize; ++i) {
    lut[i].R = vpMath::saturate<unsigned char>((alpha * i) + beta);
    lut[i].G = vpMath::saturate<unsigned char>((alpha * i) + beta);
    lut[i].B = vpMath::saturate<unsigned char>((alpha * i) + beta);
    lut[i].A = vpMath::saturate<unsigned char>((alpha * i) + beta);
  }

  // Apply the transformation using a LUT
  I.performLut(lut);
}

void adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, double alpha, double beta)
{
  // Copy I1 to I2
  I2 = I1;

  adjust(I2, alpha, beta);
}

void equalizeHistogram(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  vpImage<unsigned char> Icpy = I;
  equalizeHistogram(Icpy, I, p_mask);
}

void equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2,
                       const vpImage<bool> *p_mask)
{
  if ((I1.getWidth() * I1.getHeight()) == 0) {
    return;
  }

  // Calculate the histogram
  vpHistogram hist;
  hist.setMask(p_mask);
  hist.equalize(I1, I2);
}

void equalizeHistogram(vpImage<vpRGBa> &I, bool useHSV)
{
  if ((I.getWidth() * I.getHeight()) == 0) {
    return;
  }

  if (!useHSV) {
    // Split the RGBa image into 4 images
    vpImage<unsigned char> pR(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pG(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pB(I.getHeight(), I.getWidth());
    vpImage<unsigned char> pa(I.getHeight(), I.getWidth());

    vpImageConvert::split(I, &pR, &pG, &pB, &pa);

    // Apply histogram equalization for each channel
    equalizeHistogram(pR);
    equalizeHistogram(pG);
    equalizeHistogram(pB);

   // Merge the result in I
    unsigned int size = I.getWidth() * I.getHeight();
    unsigned char *ptrStart = reinterpret_cast<unsigned char *>(I.bitmap);
    unsigned char *ptrEnd = ptrStart + (size * 4);
    unsigned char *ptrCurrent = ptrStart;

    unsigned int cpt = 0;
    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = pR.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pG.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pB.bitmap[cpt];
      ++ptrCurrent;

      *ptrCurrent = pa.bitmap[cpt];
      ++ptrCurrent;

      ++cpt;
    }
  }
  else {
    vpImage<unsigned char> hue(I.getHeight(), I.getWidth());
    vpImage<unsigned char> saturation(I.getHeight(), I.getWidth());
    vpImage<unsigned char> value(I.getHeight(), I.getWidth());

    unsigned int size = I.getWidth() * I.getHeight();
    // Convert from RGBa to HSV
    vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap), reinterpret_cast<unsigned char *>(hue.bitmap),
                              reinterpret_cast<unsigned char *>(saturation.bitmap), reinterpret_cast<unsigned char *>(value.bitmap), size);

    // Histogram equalization on the value plane
    equalizeHistogram(value);

   // Convert from HSV to RGBa
    vpImageConvert::HSVToRGBa(reinterpret_cast<unsigned char *>(hue.bitmap), reinterpret_cast<unsigned char *>(saturation.bitmap),
                              reinterpret_cast<unsigned char *>(value.bitmap), reinterpret_cast<unsigned char *>(I.bitmap), size);
  }
}

void equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, bool useHSV)
{
  I2 = I1;
  equalizeHistogram(I2, useHSV);
}

/**
 * \brief This method is an implementation of the article "Towards Real-time Hardware Gamma Correction
 * for Dynamic Contrast Enhancement" by Jesse Scott, Michael Pusateri, IEEE Applied Imagery Pattern Recognition
 * Workshop (AIPR 2009), 2009
 *
 * The gamma factor depends on the mean of the original image and its intensity range.
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionLogMethod(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  float mean = static_cast<float>(I.getMeanValue(p_mask));
  unsigned char inputMin = 0, inputMax = 0;
  I.getMinMaxValue(inputMin, inputMax);
  unsigned char inputRange = inputMax - inputMin;

  float gamma_computed = static_cast<float>((std::log(128.f) - std::log(256.f)) / (std::log(mean) - std::log(inputRange)));
  float inverse_gamma = 1.f / gamma_computed;

  // Construct the look-up table
  unsigned char lut[256];
  float inputRangeAsFloat = static_cast<float>(inputRange);
  for (unsigned int i = inputMin; i <= inputMax; ++i) {
    lut[i] = vpMath::saturate<unsigned char>(std::pow(static_cast<float>(i - inputMin) / inputRangeAsFloat, inverse_gamma) * 255.f);
  }

  I.performLut(lut);
}

/**
 * \brief This method is an implementation of the article "REDUCING ILLUMINATION BASED ON NONLINEAR GAMMA CORRECTION"
 * by Yihua Shi, Jinfeng Yang, Renbiao Wu, International Conference on Image Processing · September 2007
 *
 * The gamma factor is the result of the sum of non-linear functions whose values depend on
 * the pixel intensity.
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionNonLinearMethod(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  (void)p_mask;
  const float a = 0.2f;
  const float b = 0.3f;
  const float c = 0.3f;
  const float x_m = 127.5f;
  const float alpha = std::atan2(-b, x_m);
  const float rho = 0.1f;
  const unsigned int lutSize = 256;
  unsigned char lut[lutSize];
  for (unsigned int i = 0; i < lutSize; ++i) {
    float x = static_cast<float>(i);
    float phi = (M_PI_FLOAT * x) / (2.f * x_m);
    float f1 = a * std::cos(phi);
    float k = rho * std::sin((4 * M_PI_FLOAT * x) / 255.f);
    float f2 = ((k + b)*std::cos(alpha)) + (x * std::sin(alpha));
    float r = c * std::abs((x / x_m) - 1.f);
    float f3 = r * std::cos((3.f * M_PI_FLOAT * x) / 255.f);
    float g = f1 + f2 + f3;
    float gamma = 1 + g;
    float inverse_gamma = 1.f / gamma;
    lut[i] = vpMath::saturate<unsigned char>(std::pow(static_cast<float>(i) / 255.f, inverse_gamma) * 255.f);
  }
  I.performLut(lut);
}

/**
 * \brief This method is an implementation of the article "An adaptive gamma correction for image
 * enhancement", Shanto Rahman, Md Mostafijur Rahman, M. Abdullah-Al-Wadud, Golam Dastegir Al-Quaderi and
 * Mohammad Shoyaib, EURASIP Journal on Image and Video Processing (2016)
 *
 * The gamma factor depends of the contrast of the image. The constant depends on the brightness of
 * the image.
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionClassBasedMethod(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  double mean = I.getMeanValue(p_mask);
  double stdev = I.getStdev(p_mask);
  double meanNormalized = mean / 255.;
  double stdevNormalized = stdev / 255.;
  const float tau = 3.f;
  bool isAlreadyHighContrast = (4. * stdevNormalized) > (1./tau);
  const unsigned int lutSize = 256;
  unsigned char lut[lutSize];
  float gamma = 0.f;
  if (isAlreadyHighContrast) {
    // Case medium to high contrast image
    gamma = static_cast<float>(std::exp((1.f - (meanNormalized + stdevNormalized))/2.f));
  }
  else {
    // Case low contrast image
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    gamma = -static_cast<float>(std::log2(stdevNormalized));
#else
    gamma = -static_cast<float>(std::log(stdevNormalized) / std::log(2.f));
#endif
  }
  if (meanNormalized < 0.5) {
      // Case dark image
    float meanPowerGamma = static_cast<float>(std::pow(meanNormalized, gamma));
    for (unsigned int i = 0; i < lutSize; ++i) {
      float iNormalized = static_cast<float>(i)/255.f;
      float iPowerGamma = std::pow(iNormalized, gamma);
      lut[i] = vpMath::saturate<unsigned char>(255.f * (iPowerGamma / (iPowerGamma + ((1.f - iPowerGamma) * meanPowerGamma))));
    }
  }
  else {
    // Case bright image
    for (unsigned int i = 0; i < lutSize; ++i) {
      float iNormalized = static_cast<float>(i)/255.f;
      lut[i] = vpMath::saturate<unsigned char>(std::pow(iNormalized, gamma) * 255.f);
    }
  }
  I.performLut(lut);
}

/**
 * \brief This technique comes from the article "Efficient Contrast Enhancement Using Adaptive
 * Gamma Correction With Weighting Distribution" by Shih-Chia Huang, Fan-Chieh Cheng, and Yi-Sheng Chiu,
 * IEEE TRANSACTIONS ON IMAGE PROCESSING, VOL. 22, NO. 3, MARCH 2013.
 *
 * It works well on globally dark images that must be brightened, but overcompensate
 * images that are already "bright enough".
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionProbBasedMethod(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  const unsigned int nbBins = 256;
  vpHistogram histo;
  histo.setMask(p_mask);
  histo.calculate(I, nbBins);
  unsigned int totalNbPoints = histo.getTotal();
  unsigned int minHisto = histo[0];
  unsigned int maxHisto = histo[0];
  for (unsigned int i = 1; i < nbBins; ++i) {
    minHisto = std::min(minHisto, histo[i]);
    maxHisto = std::max(maxHisto, histo[i]);
  }
  float pdfMin = static_cast<float>(minHisto) / static_cast<float>(totalNbPoints);
  float pdfMax = static_cast<float>(maxHisto) / static_cast<float>(totalNbPoints);
  float pdf_w[nbBins];
  float sum_pdf_w = 0.f;
  for (unsigned int i = 0; i < nbBins; ++i) {
    float pdf = static_cast<float>(histo[i])/static_cast<float>(totalNbPoints);
    pdf_w[i] = pdfMax * std::sqrt((pdf - pdfMin)/(pdfMax - pdfMin)); // alpha = 0.5
    sum_pdf_w += pdf_w[i];
  }
  unsigned char lut[nbBins];
  float cdf_w = 0;
  for (unsigned int i = 0; i < nbBins; ++i) {
    cdf_w += pdf_w[i] / sum_pdf_w;
    float gamma = 1.f - cdf_w;
    float iNormalized = static_cast<float>(i)/255.f;
    lut[i] = vpMath::saturate<unsigned char>(std::pow(iNormalized, gamma) * 255.f);
  }
  I.performLut(lut);
}

/**
 * \brief This technique comes from the article "A Space-Variant Luminance Map based Color Image Enhancement" by Sungmok
 * Lee, Homin Kwon, Hagyong Han, Gidong Lee, and Bongsoon Kang,
 * IEEE Transactions on Consumer Electronics, Vol. 56, No. 4, November 2010.
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionSpatialBased(vpImage<unsigned char> &I, const vpImage<bool> *p_mask)
{
  unsigned int width = I.getWidth(), height = I.getHeight();
  const unsigned int scale2 = 2, scale4 = 4, scale8 = 8;
  vpImage<unsigned char> I_2, I_4, I_8;
  I.subsample(scale2, scale2, I_2);
  I.subsample(scale4, scale4, I_4);
  I.subsample(scale8, scale8, I_8);
  vpImage<float> I_blur, I_2_blur, I_4_blur, I_8_blur;
  const bool normalize = true;
  const unsigned int gaussKernelSize = 3;
  vpImageFilter::gaussianBlur(I, I_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_2, I_2_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_4, I_4_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_8, I_8_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImage<float> L, L_2, L_4, L_8;
  vpImageTools::resize(I_blur, L, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_2_blur, L_2, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_4_blur, L_4, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_8_blur, L_8, width, height, vpImageTools::INTERPOLATION_CUBIC);
  const float alpha = 0.5f;
  unsigned int size = height * width;
  float stdev = static_cast<float>(I.getStdev(p_mask));
  float p;
  const float stdevThresh1 = 40., stdevThresh2 = 80.;
  if (stdev <= stdevThresh1) {
    p = 2.f;
  }
  else if (stdev <= stdevThresh2) {
    p = (-0.025f * stdev) + 3.f;
  }
  else {
    p = 1.f;
  }

  for (unsigned int i = 0; i < size; ++i) {
    bool hasToCompute = true;
    if (p_mask != nullptr) {
      hasToCompute = p_mask->bitmap[i];
    }
    if (hasToCompute) {
      float svlm = (L.bitmap[i] + L_2.bitmap[i] + L_4.bitmap[i] + L_8.bitmap[i]) / 4.f; // Computation of the space-variant luminance map
      float gamma = std::pow(alpha, (128.f - svlm)/128.f);
      float iNormalized = static_cast<float>(I.bitmap[i])/255.f;
      float o = std::pow(iNormalized, gamma) * 255.f; // Computation of the luminance
      float r = svlm / o;
      float e = std::pow(r, p);
      float s = 255.f * std::pow(o / 255.f, e);
      I.bitmap[i] = vpMath::saturate<unsigned char>((s * static_cast<float>(I.bitmap[i])) / o);
    }
  }
}

/**
 * \brief This technique comes from the article "A Space-Variant Luminance Map based Color Image Enhancement" by Sungmok
 * Lee, Homin Kwon, Hagyong Han, Gidong Lee, and Bongsoon Kang,
 * IEEE Transactions on Consumer Electronics, Vol. 56, No. 4, November 2010.
 *
 * \param[out] I The image on which gamma correction must be applied.
 * \param[in] p_mask Boolean that indicates which points must be taken into account (true value)
 * or must be ignored (false value).
 */
void gammaCorrectionSpatialBased(vpImage<vpRGBa> &I, const vpImage<bool> *p_mask)
{
  unsigned int width = I.getWidth(), height = I.getHeight();
  unsigned int size = height * width;
  vpImage<unsigned char> I_gray(height, width);
  for (unsigned int i = 0; i < size; ++i) {
    vpRGBa rgb = I.bitmap[i];
    I_gray.bitmap[i] = static_cast<unsigned char>((0.299 * rgb.R) + (0.587 * rgb.G) + (0.114 * rgb.B));
  }
  vpImage<unsigned char> I_2, I_4, I_8;
  const unsigned int scale2 = 2, scale4 = 4, scale8 = 8;
  I_gray.subsample(scale2, scale2, I_2);
  I_gray.subsample(scale4, scale4, I_4);
  I_gray.subsample(scale8, scale8, I_8);
  vpImage<float> I_blur, I_2_blur, I_4_blur, I_8_blur;
  const bool normalize = true;
  const unsigned int gaussKernelSize = 3;
  vpImageFilter::gaussianBlur(I_gray, I_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_2, I_2_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_4, I_4_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImageFilter::gaussianBlur(I_8, I_8_blur, gaussKernelSize, 0.f, normalize, p_mask);
  vpImage<float> L, L_2, L_4, L_8;
  vpImageTools::resize(I_blur, L, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_2_blur, L_2, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_4_blur, L_4, width, height, vpImageTools::INTERPOLATION_CUBIC);
  vpImageTools::resize(I_8_blur, L_8, width, height, vpImageTools::INTERPOLATION_CUBIC);
  const float alpha = 0.5f;

  float stdev = static_cast<float>(I.getStdev(p_mask));
  float p;
  const float stdevThresh1 = 40., stdevThresh2 = 80.;
  if (stdev <= stdevThresh1) {
    p = 2.f;
  }
  else if (stdev <= stdevThresh2) {
    p = (-0.025f * stdev) + 3.f;
  }
  else {
    p = 1.f;
  }
  for (unsigned int i = 0; i < size; ++i) {
    bool hasToCompute = true;
    if (p_mask != nullptr) {
      hasToCompute = p_mask->bitmap[i];
    }
    if (hasToCompute) {
      float svlm = (L.bitmap[i] + L_2.bitmap[i] + L_4.bitmap[i] + L_8.bitmap[i]) / 4.f; // Computation of the space-variant luminance map
      float gamma = std::pow(alpha, (128.f - svlm)/128.f);
      float iNormalized = static_cast<float>(I_gray.bitmap[i])/255.f;
      float o = std::pow(iNormalized, gamma) * 255.f; // Computation of the luminance
      float r = svlm / o;
      float e = std::pow(r, p);
      float s = 255.f * std::pow(o / 255.f, e);
      I.bitmap[i].R = vpMath::saturate<unsigned char>((s * static_cast<float>(I.bitmap[i].R)) / o);
      I.bitmap[i].G = vpMath::saturate<unsigned char>((s * static_cast<float>(I.bitmap[i].G)) / o);
      I.bitmap[i].B = vpMath::saturate<unsigned char>((s * static_cast<float>(I.bitmap[i].B)) / o);
    }
  }
}

void gammaCorrection(vpImage<unsigned char> &I, const float &gamma, const vpGammaMethod &method, const vpImage<bool> *p_mask)
{
  float inverse_gamma = 1.0;
  if ((gamma > 0) && (method ==  GAMMA_MANUAL)) {
    inverse_gamma = 1.0f / gamma;
    // Construct the look-up table
    const unsigned int lutSize = 256;
    unsigned char lut[lutSize];
    for (unsigned int i = 0; i < lutSize; ++i) {
      lut[i] = vpMath::saturate<unsigned char>(std::pow(static_cast<float>(i) / 255.0, inverse_gamma) * 255.0);
    }

    I.performLut(lut);
  }
  else if (method ==  GAMMA_MANUAL) {
    std::stringstream errMsg;
    errMsg << "ERROR: gamma correction factor (";
    errMsg << gamma << ") cannot be negative when using a constant user-defined factor." << std::endl;
    throw(vpException(vpException::badValue, errMsg.str()));
  }
  else if (gamma > 0) {
    std::stringstream errMsg;
    errMsg << "ERROR: asking for automatic gamma correction but setting a user-defined factor (" << gamma << ")." << std::endl;
    throw(vpException(vpException::badValue, errMsg.str()));
  }
  else {
    if (method ==  GAMMA_NONLINEAR_BASED) {
      gammaCorrectionNonLinearMethod(I, p_mask);
    }
    else if (method ==  GAMMA_LOG_BASED) {
      gammaCorrectionLogMethod(I, p_mask);
    }
    else if (method ==  GAMMA_CLASSIFICATION_BASED) {
      gammaCorrectionClassBasedMethod(I, p_mask);
    }
    else if (method ==  GAMMA_CDF_BASED) {
      gammaCorrectionProbBasedMethod(I, p_mask);
    }
    else if (method ==  GAMMA_SPATIAL_VARIANT_BASED) {
      gammaCorrectionSpatialBased(I, p_mask);
    }
    else {
      std::stringstream errMsg;
      errMsg << "Gamma automatic method \"" << vpGammaMethodToString(method) << "\" is not handled." << std::endl;
      throw(vpException(vpException::badValue, errMsg.str()));
    }
  }
}

void gammaCorrection(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const float &gamma,
                     const vpGammaMethod &method, const vpImage<bool> *p_mask)
{
  I2 = I1;
  gammaCorrection(I2, gamma, method, p_mask);
}

void gammaCorrection(vpImage<vpRGBa> &I, const float &gamma, const vpGammaColorHandling &colorHandling,
                     const vpGammaMethod &method, const vpImage<bool> *p_mask)
{
  if (method ==  GAMMA_SPATIAL_VARIANT_BASED) {
    gammaCorrectionSpatialBased(I, p_mask);
  }
  else {
    if (colorHandling ==  GAMMA_HSV) {
      const unsigned int height = I.getHeight(), width = I.getWidth();
      unsigned int size = height * width;
      std::vector<unsigned char> hue(size);
      std::vector<unsigned char> saturation(size);
      std::vector<unsigned char> value(size);

      vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap), &hue.front(), &saturation.front(), &value.front(), size);
      vpImage<unsigned char> I_hue(&hue.front(), height, width);
      vpImage<unsigned char> I_saturation(&saturation.front(), height, width);
      vpImage<unsigned char> I_value(&value.front(), height, width);

      gammaCorrection(I_value, gamma, method, p_mask);

      vpImageConvert::HSVToRGBa(I_hue.bitmap, I_saturation.bitmap, I_value.bitmap, reinterpret_cast<unsigned char *>(I.bitmap), size);
    }
    else if (colorHandling ==  GAMMA_RGB) {
      vpImage<unsigned char> pR, pG, pB, pa;
      vpImageConvert::split(I, &pR, &pG, &pB, &pa);
      gammaCorrection(pR, gamma, method, p_mask);
      gammaCorrection(pG, gamma, method, p_mask);
      gammaCorrection(pB, gamma, method, p_mask);
      gammaCorrection(pa, gamma, method, p_mask);
      vpImageConvert::merge(&pR, &pG, &pB, &pa, I);
    }
    else {
      std::stringstream errMsg;
      errMsg << "Gamma color handling mode \"" << vpGammaColorHandlingToString(colorHandling);
      errMsg << "\" is not handled." << std::endl;
      throw(vpException(vpException::badValue, errMsg.str()));
    }
  }
}

void gammaCorrection(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const float &gamma,
                     const vpGammaColorHandling &colorHandling, const vpGammaMethod &method,
                     const vpImage<bool> *p_mask)
{
  I2 = I1;
  gammaCorrection(I2, gamma, colorHandling, method, p_mask);
}

void stretchContrast(vpImage<unsigned char> &I)
{
  // Find min and max intensity values
  unsigned char min = 255, max = 0;
  I.getMinMaxValue(min, max);

  unsigned char range = max - min;

  // Construct the look-up table
  const unsigned int lutSize = 256, maxVal = lutSize - 1;
  unsigned char lut[lutSize];
  if (range > 0) {
    for (unsigned int x = min; x <= max; ++x) {
      lut[x] = (maxVal * (x - min)) / range;
    }
  }
  else {
    lut[min] = min;
  }

  I.performLut(lut);
}

void stretchContrast(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2)
{
  // Copy I1 to I2
  I2 = I1;
  stretchContrast(I2);
}

void stretchContrast(vpImage<vpRGBa> &I)
{
  // Find min and max intensity values
  vpRGBa min(255), max(0);

  // Split the RGBa image into 4 images
  vpImage<unsigned char> pR(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pG(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pB(I.getHeight(), I.getWidth());
  vpImage<unsigned char> pa(I.getHeight(), I.getWidth());

  vpImageConvert::split(I, &pR, &pG, &pB, &pa);
  // Min max values calculated for each channel
  unsigned char minChannel, maxChannel;
  pR.getMinMaxValue(minChannel, maxChannel);
  min.R = minChannel;
  max.R = maxChannel;

  pG.getMinMaxValue(minChannel, maxChannel);
  min.G = minChannel;
  max.G = maxChannel;

  pB.getMinMaxValue(minChannel, maxChannel);
  min.B = minChannel;
  max.B = maxChannel;

  pa.getMinMaxValue(minChannel, maxChannel);
  min.A = minChannel;
  max.A = maxChannel;

  // Construct the look-up table
  const unsigned int lutSize = 256, maxVal = lutSize - 1;
  vpRGBa lut[lutSize];
  unsigned char rangeR = max.R - min.R;
  if (rangeR > 0) {
    for (unsigned int x = min.R; x <= max.R; ++x) {
      lut[x].R = (maxVal * (x - min.R)) / rangeR;
    }
  }
  else {
    lut[min.R].R = min.R;
  }

  unsigned char rangeG = max.G - min.G;
  if (rangeG > 0) {
    for (unsigned int x = min.G; x <= max.G; ++x) {
      lut[x].G = (maxVal * (x - min.G)) / rangeG;
    }
  }
  else {
    lut[min.G].G = min.G;
  }

  unsigned char rangeB = max.B - min.B;
  if (rangeB > 0) {
    for (unsigned int x = min.B; x <= max.B; ++x) {
      lut[x].B = (maxVal * (x - min.B)) / rangeB;
    }
  }
  else {
    lut[min.B].B = min.B;
  }

  unsigned char rangeA = max.A - min.A;
  if (rangeA > 0) {
    for (unsigned int x = min.A; x <= max.A; ++x) {
      lut[x].A = (maxVal * (x - min.A)) / rangeA;
    }
  }
  else {
    lut[min.A].A = min.A;
  }

  I.performLut(lut);
}

void stretchContrast(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2)
{
  // Copy I1 to I2
  I2 = I1;
  stretchContrast(I2);
}

void stretchContrastHSV(vpImage<vpRGBa> &I)
{
  unsigned int size = I.getWidth() * I.getHeight();

  // Convert RGB to HSV
  vpImage<double> hueImage(I.getHeight(), I.getWidth()), saturationImage(I.getHeight(), I.getWidth()),
    valueImage(I.getHeight(), I.getWidth());
  vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap), hueImage.bitmap, saturationImage.bitmap, valueImage.bitmap,
                            size);

  // Find min and max Saturation and Value
  double minSaturation, maxSaturation, minValue, maxValue;
  saturationImage.getMinMaxValue(minSaturation, maxSaturation);
  valueImage.getMinMaxValue(minValue, maxValue);

  double *ptrStart = saturationImage.bitmap;
  double *ptrEnd = saturationImage.bitmap + size;
  double *ptrCurrent = ptrStart;

  // Stretch Saturation
  if ((maxSaturation - minSaturation) > 0.0) {
    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = (*ptrCurrent - minSaturation) / (maxSaturation - minSaturation);
      ++ptrCurrent;
    }
  }

  // Stretch Value
  if ((maxValue - minValue) > 0.0) {
    ptrStart = valueImage.bitmap;
    ptrEnd = valueImage.bitmap + size;
    ptrCurrent = ptrStart;

    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = (*ptrCurrent - minValue) / (maxValue - minValue);
      ++ptrCurrent;
    }
  }

  // Convert HSV to RGBa
  vpImageConvert::HSVToRGBa(hueImage.bitmap, saturationImage.bitmap, valueImage.bitmap, reinterpret_cast<unsigned char *>(I.bitmap),
                            size);
}

void stretchContrastHSV(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2)
{
  // Copy I1 to I2
  I2 = I1;
  stretchContrastHSV(I2);
}

void unsharpMask(vpImage<unsigned char> &I, float sigma, double weight)
{
  if ((weight < 1.0) && (weight >= 0.0)) {
#if defined(VISP_HAVE_SIMDLIB)
    // Gaussian blurred image
    vpGaussianFilter gaussian_filter(I.getWidth(), I.getHeight(), sigma);
    vpImage<unsigned char> I_blurred;
    gaussian_filter.apply(I, I_blurred);
#else
    // Gaussian blurred image
    vpImage<double> I_blurred;
    unsigned int size = 7;
    (void)sigma;
    vpImageFilter::gaussianBlur(I, I_blurred, size);
#endif

    // Unsharp mask
    unsigned int i_size = I.getSize();
    for (unsigned int cpt = 0; cpt < i_size; ++cpt) {
      double val = (I.bitmap[cpt] - (weight * I_blurred.bitmap[cpt])) / (1 - weight);
      I.bitmap[cpt] = vpMath::saturate<unsigned char>(val); // val > 255 ? 255 : (val < 0 ? 0 : val);
    }
  }
}

void unsharpMask(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires, float sigma, double weight)
{
  // Copy I to Ires
  Ires = I;
  unsharpMask(Ires, sigma, weight);
}

void unsharpMask(vpImage<vpRGBa> &I, float sigma, double weight)
{
  if ((weight < 1.0) && (weight >= 0.0)) {
#if defined(VISP_HAVE_SIMDLIB)
    // Gaussian blurred image
    vpGaussianFilter gaussian_filter(I.getWidth(), I.getHeight(), sigma);
    vpImage<vpRGBa> I_blurred;
    gaussian_filter.apply(I, I_blurred);
#else
    // Gaussian blurred image
    vpImage<double> I_blurred_R, I_blurred_G, I_blurred_B;
    vpImage<unsigned char> I_R, I_G, I_B;
    unsigned int size = 7;
    (void)sigma;

    vpImageConvert::split(I, &I_R, &I_G, &I_B);
    vpImageFilter::gaussianBlur(I_R, I_blurred_R, size);
    vpImageFilter::gaussianBlur(I_G, I_blurred_G, size);
    vpImageFilter::gaussianBlur(I_B, I_blurred_B, size);
#endif

    // Unsharp mask
    unsigned int i_size = I.getSize();
    for (unsigned int cpt = 0; cpt < i_size; ++cpt) {
#if defined(VISP_HAVE_SIMDLIB)
      double val_R = (I.bitmap[cpt].R - (weight * I_blurred.bitmap[cpt].R)) / (1 - weight);
      double val_G = (I.bitmap[cpt].G - (weight * I_blurred.bitmap[cpt].G)) / (1 - weight);
      double val_B = (I.bitmap[cpt].B - (weight * I_blurred.bitmap[cpt].B)) / (1 - weight);
#else
      double val_R = (I.bitmap[cpt].R - (weight * I_blurred_R.bitmap[cpt])) / (1 - weight);
      double val_G = (I.bitmap[cpt].G - (weight * I_blurred_G.bitmap[cpt])) / (1 - weight);
      double val_B = (I.bitmap[cpt].B - (weight * I_blurred_B.bitmap[cpt])) / (1 - weight);
#endif
      I.bitmap[cpt].R = vpMath::saturate<unsigned char>(val_R);
      I.bitmap[cpt].G = vpMath::saturate<unsigned char>(val_G);
      I.bitmap[cpt].B = vpMath::saturate<unsigned char>(val_B);
    }
  }
}

void unsharpMask(const vpImage<vpRGBa> &I, vpImage<vpRGBa> &Ires, float sigma, double weight)
{
  // Copy I to Ires
  Ires = I;
  unsharpMask(Ires, sigma, weight);
}

} // namespace
