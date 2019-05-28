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
 * Convert image types.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/* Retinex_.java Using ImageJ Gaussian Filter
 * Retinex filter algorithm based on the plugin for GIMP.
 *
 *@author Jimenez-Hernandez Francisco <jimenezf@fi.uaemex.mx>
 *Developed at Birmingham University, School of Dentistry. Supervised by
 *Gabriel Landini
 *@version 1.0
 *
 * 8 July 2010
 *
 * This version uses ImageJ Gaussian blurring instead of GIMP's linear
 *Gaussian because there is a bug in GIMP's implementation that shifts the
 *results of the blurring to the right of the image when using more than 3
 *scales.
 *
 * Based on:
 * MSRCR Retinex
 * (Multi-Scale Retinex with Color Restoration)
 *  2003 Fabien Pelisson <Fabien.Pelisson@inrialpes.fr>
 * Retinex GIMP plug-in
 *
 * Copyright (C) 2009 MAO Y.B
 *               2009. 3. 3
 *               Visual Information Processing (VIP) Group, NJUST
 *
 * D. J. Jobson, Z. Rahman, and G. A. Woodell. A multi-scale
 * Retinex for bridging the gap between color images and the
 * human observation of scenes. IEEE Transactions on Image Processing,
 * 1997, 6(7): 965-976
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*!
  \file vpRetinex.cpp
  \brief Retinex algorithm
*/

#include <functional>
#include <numeric>

#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpMath.h>
#include <visp3/imgproc/vpImgproc.h>

#define MAX_RETINEX_SCALES 8

std::vector<double> retinexScalesDistribution(const int scaleDiv, const int level, const int scale)
{
  std::vector<double> scales(MAX_RETINEX_SCALES);

  if (scaleDiv == 1) {
    scales[0] = scale / 2.0;
  } else if (scaleDiv == 2) {
    scales[0] = scale / 2.0;
    scales[1] = scale;
  } else {
    double size_step = scale / (double)scaleDiv;
    int i;

    switch (level) {
    case vp::RETINEX_UNIFORM:
      for (i = 0; i < scaleDiv; i++) {
        scales[(size_t)i] = 2.0 + i * size_step;
      }
      break;

    case vp::RETINEX_LOW:
      size_step = std::log(scale - 2.0) / (double)scaleDiv;
      for (i = 0; i < scaleDiv; i++) {
        scales[(size_t)i] = 2.0 + std::pow(10.0, (i * size_step) / std::log(10.0));
      }
      break;

    case vp::RETINEX_HIGH:
      size_step = std::log(scale - 2.0) / (double)scaleDiv;
      for (i = 0; i < scaleDiv; i++) {
        scales[(size_t)i] = scale - std::pow(10.0, (i * size_step) / std::log(10.0));
      }
      break;

    default:
      break;
    }
  }

  return scales;
}

// See: http://imagej.net/Retinex and
// https://docs.gimp.org/en/plug-in-retinex.html
void MSRCR(vpImage<vpRGBa> &I, const int _scale, const int scaleDiv, const int level, const double dynamic,
           const int _kernelSize)
{
  // Calculate the scales of filtering according to the number of filter and
  // their distribution.
  std::vector<double> retinexScales = retinexScalesDistribution(scaleDiv, level, _scale);

  // Filtering according to the various scales.
  // Summarize the results of the various filters according to a specific
  // weight(here equivalent for all).
  double weight = 1.0 / (double)scaleDiv;

  std::vector<vpImage<double> > doubleRGB(3);
  std::vector<vpImage<double> > doubleResRGB(3);
  unsigned int size = I.getSize();

  int kernelSize = _kernelSize;
  if (kernelSize == -1) {
    // Compute the kernel size from the input image size
    kernelSize = (int)(std::min(I.getWidth(), I.getHeight()) / 2.0);
    kernelSize = (kernelSize - kernelSize % 2) + 1;
  }

  for (int channel = 0; channel < 3; channel++) {
    doubleRGB[(size_t)channel] = vpImage<double>(I.getHeight(), I.getWidth());
    doubleResRGB[(size_t)channel] = vpImage<double>(I.getHeight(), I.getWidth());

    for (unsigned int cpt = 0; cpt < size; cpt++) {
      // Shift the pixel values by 1 to avoid problem with log(0)
      switch (channel) {
      case 0:
        doubleRGB[(size_t)channel].bitmap[cpt] = I.bitmap[cpt].R + 1.0;
        break;

      case 1:
        doubleRGB[(size_t)channel].bitmap[cpt] = I.bitmap[cpt].G + 1.0;
        break;

      case 2:
        doubleRGB[(size_t)channel].bitmap[cpt] = I.bitmap[cpt].B + 1.0;
        break;

      default:
        break;
      }
    }

    for (int sc = 0; sc < scaleDiv; sc++) {
      vpImage<double> blurImage;
      double sigma = retinexScales[(size_t)sc];
      vpImageFilter::gaussianBlur(doubleRGB[(size_t)channel], blurImage, (unsigned int)kernelSize, sigma);

      for (unsigned int cpt = 0; cpt < size; cpt++) {
        // Summarize the filtered values.
        // In fact one calculates a ratio between the original values and the
        // filtered values.
        doubleResRGB[(size_t)channel].bitmap[cpt] +=
            weight * (std::log(doubleRGB[(size_t)channel].bitmap[cpt]) - std::log(blurImage.bitmap[cpt]));
      }
    }
  }

  std::vector<double> dest(size * 3);
  const double gain = 1.0, alpha = 128.0, offset = 0.0;

  for (unsigned int cpt = 0; cpt < size; cpt++) {
    double logl = std::log((double)(I.bitmap[cpt].R + I.bitmap[cpt].G + I.bitmap[cpt].B + 3.0));

    dest[cpt * 3] = gain * (std::log(alpha * doubleRGB[0].bitmap[cpt]) - logl) * doubleResRGB[0].bitmap[cpt] + offset;
    dest[cpt * 3 + 1] =
        gain * (std::log(alpha * doubleRGB[1].bitmap[cpt]) - logl) * doubleResRGB[1].bitmap[cpt] + offset;
    dest[cpt * 3 + 2] =
        gain * (std::log(alpha * doubleRGB[2].bitmap[cpt]) - logl) * doubleResRGB[2].bitmap[cpt] + offset;
  }

  double sum = std::accumulate(dest.begin(), dest.end(), 0.0);
  double mean = sum / dest.size();

  std::vector<double> diff(dest.size());
  std::transform(dest.begin(), dest.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / dest.size());

  double mini = mean - dynamic * stdev;
  double maxi = mean + dynamic * stdev;
  double range = maxi - mini;

  if (vpMath::nul(range)) {
    range = 1.0;
  }

  for (unsigned int cpt = 0; cpt < size; cpt++) {
    I.bitmap[cpt].R = vpMath::saturate<unsigned char>((255.0 * (dest[cpt * 3 + 0] - mini) / range));
    I.bitmap[cpt].G = vpMath::saturate<unsigned char>((255.0 * (dest[cpt * 3 + 1] - mini) / range));
    I.bitmap[cpt].B = vpMath::saturate<unsigned char>((255.0 * (dest[cpt * 3 + 2] - mini) / range));
  }
}

/*!
  \ingroup group_imgproc_retinex

  Apply the Retinex algorithm (the input image is modified).
  \param I : The color image after application of the Retinex technique.
  \param scale : Specifies the depth of the retinex effect. Minimum value is
  16, a value providing gross, unrefined filtering. Maximum value is 250.
  Optimal and default value is 240. \param scaleDiv : Specifies the number of
  iterations of the multiscale filter. Values larger than 2 exploit the
  "multiscale" nature of the algorithm. \param level : Specifies distribution
  of the Gaussian blurring kernel sizes for Scale division values > 2:
    - 0, tends to treat all image intensities similarly,
    - 1, enhances dark regions of the image,
    - 2, enhances the bright regions of the image.
  \param dynamic : Adjusts the color of the result. Large values produce less
  saturated images. \param kernelSize : Kernel size for the gaussian blur
  operation. If -1, the kernel size is calculated from the image size.
*/
void vp::retinex(vpImage<vpRGBa> &I, const int scale, const int scaleDiv, const int level, const double dynamic,
                 const int kernelSize)
{
  // Assert scale
  if (scale < 16 || scale > 250) {
    std::cerr << "Scale must be between the interval [16 - 250]" << std::endl;
    return;
  }

  // Assert scaleDiv
  if (scaleDiv < 1 || scaleDiv > 8) {
    std::cerr << "Scale division must be between the interval [1 - 8]" << std::endl;
    return;
  }

  if (I.getWidth() * I.getHeight() == 0) {
    return;
  }

  MSRCR(I, scale, scaleDiv, level, dynamic, kernelSize);
}

/*!
  \ingroup group_imgproc_retinex

  Apply the Retinex algorithm.
  \param I1 : The input color image.
  \param I2 : The output color image after application of the Retinex
  technique. \param scale : Specifies the depth of the retinex effect. Minimum
  value is 16, a value providing gross, unrefined filtering. Maximum value is
  250. Optimal and default value is 240. \param scaleDiv : Specifies the
  number of iterations of the multiscale filter. Values larger than 2 exploit
  the "multiscale" nature of the algorithm. \param level : Specifies
  distribution of the Gaussian blurring kernel sizes for Scale division values
  > 2:
    - 0, tends to treat all image intensities similarly,
    - 1, enhances dark regions of the image,
    - 2, enhances the bright regions of the image.
  \param dynamic : Adjusts the color of the result. Large values produce less
  saturated images. \param kernelSize : Kernel size for the gaussian blur
  operation. If -1, the kernel size is calculated from the image size.
*/
void vp::retinex(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const int scale, const int scaleDiv, const int level,
                 const double dynamic, const int kernelSize)
{
  I2 = I1;
  vp::retinex(I2, scale, scaleDiv, level, dynamic, kernelSize);
}
