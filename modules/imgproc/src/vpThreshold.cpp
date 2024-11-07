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
 * Automatic thresholding functions.
 */

/*!
  \file vpThreshold.cpp
  \brief Automatic thresholding functions.
*/

#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/imgproc/vpImgproc.h>

namespace VISP_NAMESPACE_NAME
{

bool isBimodal(const std::vector<float> &hist_float)
{
  int modes = 0;
  const int bimodalVal = 2;
  size_t hist_float_size_m_1 = hist_float.size() - 1;
  for (size_t cpt = 1; cpt < hist_float_size_m_1; ++cpt) {
    if ((hist_float[cpt - 1] < hist_float[cpt]) && (hist_float[cpt] > hist_float[cpt + 1])) {
      ++modes;
    }

    if (modes > bimodalVal) {
      return false;
    }
  }

  return (modes == bimodalVal);
}

int computeThresholdHuang(const vpHistogram &hist)
{
  // Code ported from the AutoThreshold ImageJ plugin:
  // Implements Huang's fuzzy thresholding method
  // Uses Shannon's entropy function (one can also use Yager's entropy
  // function) Huang L.-K. and Wang M.-J.J. (1995) "Image Thresholding by
  // Minimizing the Measures of Fuzziness" Pattern Recognition, 28(1): 41-51
  // Reimplemented (to handle 16-bit efficiently) by Johannes Schindelin Jan
  // 31, 2011

  // Find first and last non-empty bin
  size_t first, last;
  size_t hist_size = hist.getSize();
  for (first = 0; (first < hist_size) && (hist[static_cast<unsigned char>(first)] == 0); ++first) {
    // do nothing
  }

  for (last = (hist_size - 1); (last > first) && (hist[static_cast<unsigned char>(last)] == 0); --last) {
    // do nothing
  }

  if (first == last) {
    return 0;
  }

  // Calculate the cumulative density and the weighted cumulative density
  std::vector<float> S(last + 1);
  std::vector<float> W(last + 1);

  S[0] = static_cast<float>(hist[0]);
  W[0] = 0.0f;
  for (size_t i = std::max<size_t>(static_cast<size_t>(1), first); i <= last; ++i) {
    S[i] = S[i - 1] + hist[static_cast<unsigned char>(i)];
    W[i] = W[i - 1] + (i * static_cast<float>(hist[static_cast<unsigned char>(i)]));
  }

  // Precalculate the summands of the entropy given the absolute difference x
  // - mu (integral)
  float C = static_cast<float>(last - first);
  std::vector<float> Smu((last + 1) - first);

  size_t smu_size = Smu.size();
  for (size_t i = 1; i < smu_size; ++i) {
    float mu = 1 / (1 + (i / C));
    Smu[i] = (-mu * std::log(mu)) - ((1 - mu) * std::log(1 - mu));
  }

  // Calculate the threshold
  int bestThreshold = 0;
  float bestEntropy = std::numeric_limits<float>::max();

  for (size_t threshold = first; threshold <= last; ++threshold) {
    float entropy = 0;
    int mu = vpMath::round(W[threshold] / S[threshold]);
    for (size_t i = first; i <= threshold; ++i) {
      entropy += Smu[static_cast<size_t>(std::abs(static_cast<int>(i) - mu))] * hist[static_cast<unsigned char>(i)];
    }

    mu = vpMath::round((W[last] - W[threshold]) / (S[last] - S[threshold]));
    for (size_t i = threshold + 1; i <= last; ++i) {
      entropy += Smu[static_cast<size_t>(std::abs(static_cast<int>(i) - mu))] * hist[static_cast<unsigned char>(i)];
    }

    if (bestEntropy > entropy) {
      bestEntropy = entropy;
      bestThreshold = static_cast<int>(threshold);
    }
  }

  return bestThreshold;
}

int computeThresholdIntermodes(const vpHistogram &hist)
{
  const unsigned int minSize = 3;
  if (hist.getSize() < minSize) {
    return -1;
  }

  // Code based on the AutoThreshold ImageJ plugin:
  // J. M. S. Prewitt and M. L. Mendelsohn, "The analysis of cell images," in
  // Annals of the New York Academy of Sciences, vol. 128, pp. 1035-1053,
  // 1966. ported to ImageJ plugin by G.Landini from Antti Niemisto's Matlab
  // code (GPL) Original Matlab code Copyright (C) 2004 Antti Niemisto See
  // http://www.cs.tut.fi/~ant/histthresh/ for an excellent slide presentation
  // and the original Matlab code.
  //
  // Assumes a bimodal histogram. The histogram needs is smoothed (using a
  // running average of size 3, iteratively) until there are only two local
  // maxima. j and k Threshold t is (j+k)/2. Images with histograms having
  // extremely unequal peaks or a broad and ﬂat valley are unsuitable for this
  // method.

  std::vector<float> hist_float(hist.getSize());
  unsigned int hist_size = hist.getSize();
  for (unsigned int cpt = 0; cpt < hist_size; ++cpt) {
    hist_float[cpt] = static_cast<float>(hist[cpt]);
  }

  int iter = 0;
  const float nbPtsAverage = 3.;
  const int maxIter = 10000;
  while (!isBimodal(hist_float)) {
    // Smooth with a 3 point running mean filter
    size_t hist_float_size_m_1 = hist_float.size() - 1;
    for (size_t cpt = 1; cpt < hist_float_size_m_1; ++cpt) {
      hist_float[cpt] = (hist_float[cpt - 1] + hist_float[cpt] + hist_float[cpt + 1]) / nbPtsAverage;
    }

    // First value
    hist_float[0] = (hist_float[0] + hist_float[1]) / 2.0f;

    // Last value
    const size_t var2 = 2;
    hist_float[hist_float.size() - 1] = (((hist_float.size() - var2) + hist_float.size()) - 1) / 2.0f;

    ++iter;

    if (iter > maxIter) {
      std::cerr << "Intermodes Threshold not found after " << maxIter << " iterations!" << std::endl;
      return -1;
    }
  }

  // The threshold is the mean between the two peaks.
  int tt = 0;
  size_t hist_float_size_m_1 = hist_float.size() - 1;
  for (size_t cpt = 1; cpt < hist_float_size_m_1; ++cpt) {
    if ((hist_float[cpt - 1] < hist_float[cpt]) && (hist_float[cpt] > hist_float[cpt + 1])) {
      // Mode
      tt += static_cast<int>(cpt);
    }
  }

  return static_cast<int>(std::floor(tt / 2.0)); // vpMath round of tt div 2.0
}

int computeThresholdIsoData(const vpHistogram &hist, unsigned int imageSize)
{
  int threshold = 0;

  // Code based on BSD Matlab isodata implementation by zephyr
  // STEP 1: Compute mean intensity of image from histogram, set T=mean(I)
  std::vector<float> cumsum(hist.getSize(), 0.0f);
  std::vector<float> sum_ip(hist.getSize(), 0.0f);
  cumsum[0] = static_cast<float>(hist[0]);

  unsigned int hist_size = hist.getSize();
  for (unsigned int cpt = 1; cpt < hist_size; ++cpt) {
    sum_ip[cpt] = (cpt * static_cast<float>(hist[cpt])) + sum_ip[cpt - 1];
    cumsum[cpt] = static_cast<float>(hist[cpt]) + cumsum[cpt - 1];
  }

  int T = vpMath::round(sum_ip[255] / imageSize);

  // STEP 2: compute Mean above T (MAT) and Mean below T (MBT) using T from
  float MBT = sum_ip[static_cast<size_t>(T - 2)] / cumsum[static_cast<size_t>(T - 2)];
  float MAT = (sum_ip.back() - sum_ip[static_cast<size_t>(T - 1)]) / (cumsum.back() - cumsum[static_cast<size_t>(T - 1)]);

  int T2 = vpMath::round((MAT + MBT) / 2.0f);

  //% STEP 3 to n: repeat step 2 if T(i)~=T(i-1)
  while (std::abs(T2 - T) >= 1) {
    const int val_2 = 2;
    MBT = sum_ip[static_cast<size_t>(T2 - val_2)] / cumsum[static_cast<size_t>(T2 - val_2)];
    MAT = (sum_ip.back() - sum_ip[static_cast<size_t>(T2 - 1)]) / (cumsum.back() - cumsum[static_cast<size_t>(T2 - 1)]);

    T = T2;
    T2 = vpMath::round((MAT + MBT) / 2.0f);
    threshold = T2;
  }

  return threshold;
}

int computeThresholdMean(const vpHistogram &hist, unsigned int imageSize)
{
  // C. A. Glasbey, "An analysis of histogram-based thresholding algorithms,"
  // CVGIP: Graphical Models and Image Processing, vol. 55, pp. 532-537, 1993.
  // The threshold is the mean of the greyscale data
  float sum_ip = 0.0f;
  unsigned int hist_size = hist.getSize();
  for (unsigned int cpt = 0; cpt < hist_size; ++cpt) {
    sum_ip += cpt * static_cast<float>(hist[cpt]);
  }

  return static_cast<int>(std::floor(sum_ip / imageSize));
}

int computeThresholdOtsu(const vpHistogram &hist, unsigned int imageSize)
{
  // Otsu, N (1979), "A threshold selection method from gray-level
  // histograms",  IEEE Trans. Sys., Man., Cyber. 9: 62-66,
  // doi:10.1109/TSMC.1979.4310076

  float mu_T = 0.0f;
  float sum_ip_all[256];
  int hist_size = static_cast<int>(hist.getSize());
  for (int cpt = 0; cpt < hist_size; ++cpt) {
    mu_T += cpt * static_cast<float>(hist[cpt]);
    sum_ip_all[cpt] = mu_T;
  }

  // Weight Background / Foreground
  float w_B = 0.0f, w_F = 0.0f;

  float max_sigma_b = 0.0f;
  int threshold = 0;

  bool w_f_eq_nul = false;
  int cpt = 0;
  const int maxCpt = 256;
  while ((cpt < maxCpt) && (!w_f_eq_nul)) {
    w_B += hist[cpt];
    bool w_b_eq_nul = vpMath::nul(w_B, std::numeric_limits<float>::epsilon());
    if (!w_b_eq_nul) {

      w_F = static_cast<int>(imageSize) - w_B;
      w_f_eq_nul = vpMath::nul(w_F, std::numeric_limits<float>::epsilon());
      if (!w_f_eq_nul) {

      // Mean Background / Foreground
        float mu_B = sum_ip_all[cpt] / static_cast<float>(w_B);
        float mu_F = (mu_T - sum_ip_all[cpt]) / static_cast<float>(w_F);
        // If there is a case where (w_B * w_F) exceed FLT_MAX, normalize
        // histogram
        float sigma_b_sqr = w_B * w_F * (mu_B - mu_F) * (mu_B - mu_F);

        if (sigma_b_sqr >= max_sigma_b) {
          threshold = cpt;
          max_sigma_b = sigma_b_sqr;
        }
      }
      // else exit the loop
    }
    ++cpt;
  }

  return threshold;
}

int computeThresholdTriangle(vpHistogram &hist)
{
  int threshold = 0;

  // Zack, G. W., Rogers, W. E. and Latt, S. A., 1977,
  // Automatic Measurement of Sister Chromatid Exchange Frequency,
  // Journal of Histochemistry and Cytochemistry 25 (7), pp. 741-753

  int left_bound = -1, right_bound = -1, max_idx = -1, max_value = 0;
  // Find max value index and left / right most index
  int hist_size = static_cast<int>(hist.getSize());
  for (int cpt = 0; cpt < hist_size; ++cpt) {
    if ((left_bound == -1) && (hist[cpt] > 0)) {
      left_bound = static_cast<int>(cpt);
    }

    if ((right_bound == -1) && (hist[static_cast<int>(hist.getSize()) - 1 - cpt] > 0)) {
      right_bound = static_cast<int>(hist.getSize()) - 1 - cpt;
    }

    if (static_cast<int>(hist[cpt]) > max_value) {
      max_value = static_cast<int>(hist[cpt]);
      max_idx = cpt;
    }
  }

  // First / last index when hist(cpt) == 0
  left_bound = left_bound > 0 ? (left_bound - 1) : left_bound;
  right_bound = right_bound < (static_cast<int>(hist.getSize()) - 1) ? (right_bound + 1) : right_bound;

  // Use the largest bound
  bool flip = false;
  if ((max_idx - left_bound) < (right_bound - max_idx)) {
    // Flip histogram to get the largest bound to the left
    flip = true;

    int cpt_left = 0;
    int cpt_right = static_cast<int>(hist.getSize()) - 1;
    for (; cpt_left < cpt_right; ++cpt_left, --cpt_right) {
      unsigned int temp = hist[cpt_left];
      hist.set(cpt_left, hist[cpt_right]);
      hist.set(cpt_right, temp);
    }

    left_bound = static_cast<int>(hist.getSize()) - 1 - right_bound;
    max_idx = static_cast<int>(hist.getSize()) - 1 - max_idx;
  }

  // Distance from a point to a line defined by two points:
  //\textbf{distance} \left ( P_1, P_2, \left ( x_0,y_0 \right ) \right )
  // = \frac{ \left | \left ( y_2-y_1 \right ) x_0 - \left ( x_2-x_1 \right )
  // y_0 + x_2 y_1 - y_2 x_1 \right | }
  //        { \sqrt{ \left ( y_2 - y_1 \right )^{2} + \left ( x_2 - x_1 \right
  //        )^{2} } }
  // Constants are ignored
  float a = static_cast<float>(max_value);              // y_2 - y_1
  float b = static_cast<float>(left_bound - max_idx); //-(x_2 - x_1)
  float max_dist = 0.0f;

  for (int cpt = left_bound + 1; cpt <= max_idx; ++cpt) {
    float dist = (a * cpt) + (b * hist[cpt]);

    if (dist > max_dist) {
      max_dist = dist;
      threshold = cpt;
    }
  }
  --threshold;

  if (flip) {
    threshold = static_cast<int>(hist.getSize()) - 1 - threshold;
  }

  return threshold;
}

unsigned char autoThreshold(vpImage<unsigned char> &I, const vpAutoThresholdMethod &method,
                            const unsigned char backgroundValue, const unsigned char foregroundValue)
{
  if (I.getSize() == 0) {
    return 0;
  }

  // Compute image histogram
  vpHistogram histogram(I);
  int threshold = -1;

  switch (method) {
  case AUTO_THRESHOLD_HUANG:
    threshold = computeThresholdHuang(histogram);
    break;

  case AUTO_THRESHOLD_INTERMODES:
    threshold = computeThresholdIntermodes(histogram);
    break;

  case AUTO_THRESHOLD_ISODATA:
    threshold = computeThresholdIsoData(histogram, I.getSize());
    break;

  case AUTO_THRESHOLD_MEAN:
    threshold = computeThresholdMean(histogram, I.getSize());
    break;

  case AUTO_THRESHOLD_OTSU:
    threshold = computeThresholdOtsu(histogram, I.getSize());
    break;

  case AUTO_THRESHOLD_TRIANGLE:
    threshold = computeThresholdTriangle(histogram);
    break;

  default:
    break;
  }

  const unsigned char threshold2 = 255;
  if (threshold != -1) {
    // Threshold
    vpImageTools::binarise(I, static_cast<unsigned char>(threshold), threshold2, backgroundValue, foregroundValue,
                           foregroundValue);
  }

  return threshold;
}

} // namespace
