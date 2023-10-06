/*
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
#include <visp3/core/vpMath.h>
#include <visp3/imgproc/vpImgproc.h>

namespace vp
{
void adjust(vpImage<unsigned char> &I, double alpha, double beta)
{
  // Construct the look-up table
  unsigned char lut[256];
  for (unsigned int i = 0; i < 256; i++) {
    lut[i] = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  // Apply the transformation using a LUT
  I.performLut(lut);
}

void adjust(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, double alpha, double beta)
{
  // Copy I1 to I2
  I2 = I1;

  vp::adjust(I2, alpha, beta);
}

void adjust(vpImage<vpRGBa> &I, double alpha, double beta)
{
  // Construct the look-up table
  vpRGBa lut[256];
  for (unsigned int i = 0; i < 256; i++) {
    lut[i].R = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].G = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].B = vpMath::saturate<unsigned char>(alpha * i + beta);
    lut[i].A = vpMath::saturate<unsigned char>(alpha * i + beta);
  }

  // Apply the transformation using a LUT
  I.performLut(lut);
}

void adjust(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, double alpha, double beta)
{
  // Copy I1 to I2
  I2 = I1;

  vp::adjust(I2, alpha, beta);
}

void equalizeHistogram(vpImage<unsigned char> &I)
{
  vpImage<unsigned char> Icpy = I;
  vp::equalizeHistogram(Icpy, I);
}

void equalizeHistogram(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2)
{
  if (I1.getWidth() * I1.getHeight() == 0) {
    return;
  }

  // Calculate the histogram
  vpHistogram hist;
  hist.equalize(I1, I2);
}

void equalizeHistogram(vpImage<vpRGBa> &I, bool useHSV)
{
  if (I.getWidth() * I.getHeight() == 0) {
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
    vp::equalizeHistogram(pR);
    vp::equalizeHistogram(pG);
    vp::equalizeHistogram(pB);

    // Merge the result in I
    unsigned int size = I.getWidth() * I.getHeight();
    unsigned char *ptrStart = (unsigned char *)I.bitmap;
    unsigned char *ptrEnd = ptrStart + size * 4;
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

      cpt++;
    }
  }
  else {
    vpImage<unsigned char> hue(I.getHeight(), I.getWidth());
    vpImage<unsigned char> saturation(I.getHeight(), I.getWidth());
    vpImage<unsigned char> value(I.getHeight(), I.getWidth());

    unsigned int size = I.getWidth() * I.getHeight();
    // Convert from RGBa to HSV
    vpImageConvert::RGBaToHSV((unsigned char *)I.bitmap, (unsigned char *)hue.bitmap,
                              (unsigned char *)saturation.bitmap, (unsigned char *)value.bitmap, size);

    // Histogram equalization on the value plane
    vp::equalizeHistogram(value);

    // Convert from HSV to RGBa
    vpImageConvert::HSVToRGBa((unsigned char *)hue.bitmap, (unsigned char *)saturation.bitmap,
                              (unsigned char *)value.bitmap, (unsigned char *)I.bitmap, size);
  }
}

void equalizeHistogram(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, bool useHSV)
{
  I2 = I1;
  vp::equalizeHistogram(I2, useHSV);
}

void gammaCorrection(vpImage<unsigned char> &I, double gamma)
{
  double inverse_gamma = 1.0;
  if (gamma > 0) {
    inverse_gamma = 1.0 / gamma;
  }
  else {
    throw vpException(vpException::badValue, "The gamma value must be positive !");
  }

  // Construct the look-up table
  unsigned char lut[256];
  for (unsigned int i = 0; i < 256; i++) {
    lut[i] = vpMath::saturate<unsigned char>(pow((double)i / 255.0, inverse_gamma) * 255.0);
  }

  I.performLut(lut);
}

void gammaCorrection(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, double gamma)
{
  I2 = I1;
  vp::gammaCorrection(I2, gamma);
}

void gammaCorrection(vpImage<vpRGBa> &I, double gamma)
{
  double inverse_gamma = 1.0;
  if (gamma > 0) {
    inverse_gamma = 1.0 / gamma;
  }
  else {
    throw vpException(vpException::badValue, "The gamma value must be positive !");
  }

  // Construct the look-up table
  vpRGBa lut[256];
  for (unsigned int i = 0; i < 256; i++) {
    lut[i].R = vpMath::saturate<unsigned char>(pow((double)i / 255.0, inverse_gamma) * 255.0);
    lut[i].G = vpMath::saturate<unsigned char>(pow((double)i / 255.0, inverse_gamma) * 255.0);
    lut[i].B = vpMath::saturate<unsigned char>(pow((double)i / 255.0, inverse_gamma) * 255.0);
    lut[i].A = vpMath::saturate<unsigned char>(pow((double)i / 255.0, inverse_gamma) * 255.0);
  }

  I.performLut(lut);
}

void gammaCorrection(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, double gamma)
{
  I2 = I1;
  vp::gammaCorrection(I2, gamma);
}

void stretchContrast(vpImage<unsigned char> &I)
{
  // Find min and max intensity values
  unsigned char min = 255, max = 0;
  I.getMinMaxValue(min, max);

  unsigned char range = max - min;

  // Construct the look-up table
  unsigned char lut[256];
  if (range > 0) {
    for (unsigned int x = min; x <= max; x++) {
      lut[x] = 255 * (x - min) / range;
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
  vp::stretchContrast(I2);
}

void stretchContrast(vpImage<vpRGBa> &I)
{
  // Find min and max intensity values
  vpRGBa min = 255, max = 0;

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
  vpRGBa lut[256];
  unsigned char rangeR = max.R - min.R;
  if (rangeR > 0) {
    for (unsigned int x = min.R; x <= max.R; x++) {
      lut[x].R = 255 * (x - min.R) / rangeR;
    }
  }
  else {
    lut[min.R].R = min.R;
  }

  unsigned char rangeG = max.G - min.G;
  if (rangeG > 0) {
    for (unsigned int x = min.G; x <= max.G; x++) {
      lut[x].G = 255 * (x - min.G) / rangeG;
    }
  }
  else {
    lut[min.G].G = min.G;
  }

  unsigned char rangeB = max.B - min.B;
  if (rangeB > 0) {
    for (unsigned int x = min.B; x <= max.B; x++) {
      lut[x].B = 255 * (x - min.B) / rangeB;
    }
  }
  else {
    lut[min.B].B = min.B;
  }

  unsigned char rangeA = max.A - min.A;
  if (rangeA > 0) {
    for (unsigned int x = min.A; x <= max.A; x++) {
      lut[x].A = 255 * (x - min.A) / rangeA;
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
  vp::stretchContrast(I2);
}

void stretchContrastHSV(vpImage<vpRGBa> &I)
{
  unsigned int size = I.getWidth() * I.getHeight();

  // Convert RGB to HSV
  vpImage<double> hueImage(I.getHeight(), I.getWidth()), saturationImage(I.getHeight(), I.getWidth()),
    valueImage(I.getHeight(), I.getWidth());
  vpImageConvert::RGBaToHSV((unsigned char *)I.bitmap, hueImage.bitmap, saturationImage.bitmap, valueImage.bitmap,
                            size);

  // Find min and max Saturation and Value
  double minSaturation, maxSaturation, minValue, maxValue;
  saturationImage.getMinMaxValue(minSaturation, maxSaturation);
  valueImage.getMinMaxValue(minValue, maxValue);

  double *ptrStart = saturationImage.bitmap;
  double *ptrEnd = saturationImage.bitmap + size;
  double *ptrCurrent = ptrStart;

  // Stretch Saturation
  if (maxSaturation - minSaturation > 0.0) {
    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = (*ptrCurrent - minSaturation) / (maxSaturation - minSaturation);
      ++ptrCurrent;
    }
  }

  // Stretch Value
  if (maxValue - minValue > 0.0) {
    ptrStart = valueImage.bitmap;
    ptrEnd = valueImage.bitmap + size;
    ptrCurrent = ptrStart;

    while (ptrCurrent != ptrEnd) {
      *ptrCurrent = (*ptrCurrent - minValue) / (maxValue - minValue);
      ++ptrCurrent;
    }
  }

  // Convert HSV to RGBa
  vpImageConvert::HSVToRGBa(hueImage.bitmap, saturationImage.bitmap, valueImage.bitmap, (unsigned char *)I.bitmap,
                            size);
}

void stretchContrastHSV(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2)
{
  // Copy I1 to I2
  I2 = I1;
  vp::stretchContrastHSV(I2);
}

void unsharpMask(vpImage<unsigned char> &I, float sigma, double weight)
{
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpGaussianFilter gaussian_filter(I.getWidth(), I.getHeight(), sigma);
    vpImage<unsigned char> I_blurred;
    gaussian_filter.apply(I, I_blurred);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I.getSize(); cpt++) {
      double val = (I.bitmap[cpt] - weight * I_blurred.bitmap[cpt]) / (1 - weight);
      I.bitmap[cpt] = vpMath::saturate<unsigned char>(val); // val > 255 ? 255 : (val < 0 ? 0 : val);
    }
  }
}

void unsharpMask(const vpImage<unsigned char> &I, vpImage<unsigned char> &Ires, float sigma, double weight)
{
  // Copy I to Ires
  Ires = I;
  vp::unsharpMask(Ires, sigma, weight);
}

void unsharpMask(vpImage<vpRGBa> &I, float sigma, double weight)
{
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpGaussianFilter gaussian_filter(I.getWidth(), I.getHeight(), sigma);
    vpImage<vpRGBa> I_blurred;
    gaussian_filter.apply(I, I_blurred);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I.getSize(); cpt++) {
      double val_R = (I.bitmap[cpt].R - weight * I_blurred.bitmap[cpt].R) / (1 - weight);
      double val_G = (I.bitmap[cpt].G - weight * I_blurred.bitmap[cpt].G) / (1 - weight);
      double val_B = (I.bitmap[cpt].B - weight * I_blurred.bitmap[cpt].B) / (1 - weight);

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
  vp::unsharpMask(Ires, sigma, weight);
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \ingroup group_imgproc_sharpening

  \deprecated This function is deprecated. You should rather use unsharpMask() with sigma Gaussian parameter.
  You can use sigma = 1.0f to have similar results.

  Sharpen a grayscale image using the unsharp mask technique.

  \param I : The grayscale image to sharpen.
  \param size : Size (must be odd) of the Gaussian blur kernel.
  \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
void unsharpMask(vpImage<unsigned char> &I, unsigned int size, double weight)
{
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpImage<double> I_blurred;
    vpImageFilter::gaussianBlur(I, I_blurred, size);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I.getSize(); cpt++) {
      double val = (I.bitmap[cpt] - weight * I_blurred.bitmap[cpt]) / (1 - weight);
      I.bitmap[cpt] = vpMath::saturate<unsigned char>(val); // val > 255 ? 255 : (val < 0 ? 0 : val);
    }
  }
}

/*!
  \ingroup group_imgproc_sharpening

  \deprecated This function is deprecated. You should rather use unsharpMask() with sigma Gaussian parameter.
  You can use sigma = 1.0f to have similar results.

  Sharpen a grayscale image using the unsharp mask technique.

  \param I1 : The first input grayscale image.
  \param I2 : The second output grayscale image.
  \param size : Size (must be odd) of the Gaussian blur kernel.
  \param weight : Weight (between [0 - 1[) for the sharpening process.
*/
void unsharpMask(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, unsigned int size, double weight)
{
  // Copy I1 to I2
  I2 = I1;
#if 0
  vp::unsharpMask(I2, size, weight);
#else
  // To avoid:
  // warning: ‘void vp::unsharpMask(vpImage<unsigned char>&, unsigned int, double)’ is deprecated
  // [-Wdeprecated-declarations]
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpImage<double> I_blurred;
    vpImageFilter::gaussianBlur(I2, I_blurred, size);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I2.getSize(); cpt++) {
      double val = (I2.bitmap[cpt] - weight * I_blurred.bitmap[cpt]) / (1 - weight);
      I2.bitmap[cpt] = vpMath::saturate<unsigned char>(val); // val > 255 ? 255 : (val < 0 ? 0 : val);
    }
  }
#endif
}

/*!
  \ingroup group_imgproc_sharpening

  \deprecated This function is deprecated. You should rather use unsharpMask() with sigma Gaussian parameter.
  You can use sigma = 1.0f to have similar results.

  Sharpen a color image using the unsharp mask technique.

  \param I : The color image to sharpen.
  \param size : Size (must be odd) of the Gaussian blur kernel.
  \param weight : Weight (between [0 - 1[) for the sharpening process.
 */
void unsharpMask(vpImage<vpRGBa> &I, unsigned int size, double weight)
{
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpImage<double> I_blurred_R, I_blurred_G, I_blurred_B;
    vpImage<unsigned char> I_R, I_G, I_B;

    vpImageConvert::split(I, &I_R, &I_G, &I_B);
    vpImageFilter::gaussianBlur(I_R, I_blurred_R, size);
    vpImageFilter::gaussianBlur(I_G, I_blurred_G, size);
    vpImageFilter::gaussianBlur(I_B, I_blurred_B, size);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I.getSize(); cpt++) {
      double val_R = (I.bitmap[cpt].R - weight * I_blurred_R.bitmap[cpt]) / (1 - weight);
      double val_G = (I.bitmap[cpt].G - weight * I_blurred_G.bitmap[cpt]) / (1 - weight);
      double val_B = (I.bitmap[cpt].B - weight * I_blurred_B.bitmap[cpt]) / (1 - weight);

      I.bitmap[cpt].R = vpMath::saturate<unsigned char>(val_R);
      I.bitmap[cpt].G = vpMath::saturate<unsigned char>(val_G);
      I.bitmap[cpt].B = vpMath::saturate<unsigned char>(val_B);
    }
  }
}

/*!
  \ingroup group_imgproc_sharpening

  \deprecated This function is deprecated. You should rather use unsharpMask() with sigma Gaussian parameter.
  You can use sigma = 1.0f to have similar results.

  Sharpen a color image using the unsharp mask technique.

  \param I1 : The first input color image.
  \param I2 : The second output color image.
  \param size : Size (must be odd) of the Gaussian blur kernel.
  \param weight : Weight (between [0 - 1[) for the sharpening process.
*/
void unsharpMask(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, unsigned int size, double weight)
{
  // Copy I1 to I2
  I2 = I1;
#if 0
  vp::unsharpMask(I2, size, weight);
#else
  // To avoid:
  // warning: ‘void vp::unsharpMask(vpImage<vpRGBa>&, unsigned int, double)’ is deprecated [-Wdeprecated-declarations]
  if (weight < 1.0 && weight >= 0.0) {
    // Gaussian blurred image
    vpImage<double> I_blurred_R, I_blurred_G, I_blurred_B;
    vpImage<unsigned char> I_R, I_G, I_B;

    vpImageConvert::split(I2, &I_R, &I_G, &I_B);
    vpImageFilter::gaussianBlur(I_R, I_blurred_R, size);
    vpImageFilter::gaussianBlur(I_G, I_blurred_G, size);
    vpImageFilter::gaussianBlur(I_B, I_blurred_B, size);

    // Unsharp mask
    for (unsigned int cpt = 0; cpt < I2.getSize(); cpt++) {
      double val_R = (I2.bitmap[cpt].R - weight * I_blurred_R.bitmap[cpt]) / (1 - weight);
      double val_G = (I2.bitmap[cpt].G - weight * I_blurred_G.bitmap[cpt]) / (1 - weight);
      double val_B = (I2.bitmap[cpt].B - weight * I_blurred_B.bitmap[cpt]) / (1 - weight);

      I2.bitmap[cpt].R = vpMath::saturate<unsigned char>(val_R);
      I2.bitmap[cpt].G = vpMath::saturate<unsigned char>(val_G);
      I2.bitmap[cpt].B = vpMath::saturate<unsigned char>(val_B);
    }
  }
#endif
}
#endif // Deprecated
};
