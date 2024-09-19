/****************************************************************************
 *
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
*****************************************************************************/

#include <visp3/rbt/vpColorHistogram.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpRect.h>

void vpColorHistogram::Builder::build(vpColorHistogram &histogram)
{
  if (histogram.getBinNumber() != m_N) {
    throw vpException(vpException::dimensionError, "Different number of bins between builder and histogram when building histogram");
  }
  unsigned int count = 0;
  for (unsigned int i = 0; i < m_counts.size(); ++i) {
    count += m_counts[i];
  }
  const float countFloat = static_cast<float>(count);
  for (unsigned int i = 0; i < m_counts.size(); ++i) {
    histogram.m_probas[i] = static_cast<float>(m_counts[i]) / countFloat;
  }
  histogram.m_numPixels = count;

}

vpColorHistogram::vpColorHistogram() : m_N(0), m_numPixels(0)
{ }

vpColorHistogram::vpColorHistogram(unsigned int N)
{
  setBinNumber(N);
}

void vpColorHistogram::setBinNumber(unsigned int N)
{
  if (N != 1 && N != 2 && N != 4 && N != 8 && N != 16 && N != 32 && N != 64 && N != 128) {
    throw vpException(vpException::badValue, "The number of bins per component should be a power of 2 (below or equal to 128)");
  }
  m_N = N;
  m_numPixels = 0;
  m_probas = std::vector<float>(N * N * N, 0.f);
}

inline unsigned int colorToIndex(const vpRGBa &p, unsigned int N, unsigned int binSize)
{
  return (p.R / binSize) * (N * N) + (p.G / binSize) * N + (p.B / binSize);
}

void vpColorHistogram::build(const vpImage<vpRGBa> &image, const vpImage<bool> &mask)
{
  std::vector<unsigned int> histo(m_N * m_N * m_N, 0);
  m_probas.resize(m_N * m_N * m_N);
  unsigned binSize = 256 / m_N;
  unsigned int pixels = 0;
  for (unsigned int i = 0; i < image.getSize(); ++i) {
    if (mask.bitmap[i]) {
      unsigned int index = colorToIndex(image.bitmap[i], m_N, binSize);
      ++histo[index];
      ++pixels;
    }
  }
  m_numPixels = pixels;
  for (unsigned int i = 0; i < histo.size(); ++i) {
    m_probas[i] = static_cast<float>(histo[i]) / pixels;
  }
}

void vpColorHistogram::build(std::vector<unsigned int> &counts)
{
  if (m_probas.size() != counts.size()) {
    throw vpException(vpException::dimensionError, "Number of bins are not the same");
  }
  m_probas.resize(m_N * m_N * m_N);
  m_numPixels = 0;
  for (unsigned int count : counts) {
    m_numPixels += count;
  }
  for (unsigned int i = 0; i < m_probas.size(); ++i) {
    m_probas[i] = static_cast<float>(counts[i]) / m_numPixels;
  }
}

void vpColorHistogram::merge(const vpColorHistogram &other, float alpha)
{
  if (other.m_N != m_N) {
    throw vpException(vpException::badValue, "Histograms should have same dimensions");
  }
  float malpha = 1.f - alpha;

  for (unsigned int i = 0; i < m_probas.size(); ++i) {
    m_probas[i] = malpha * m_probas[i] + alpha * other.m_probas[i];
  }

}

void vpColorHistogram::computeProbas(const vpImage<vpRGBa> &image, vpImage<float> &proba) const
{
  proba.resize(image.getHeight(), image.getWidth());

  unsigned int binSize = 256 / m_N;
#pragma omp parallel for
  for (unsigned int i = 0; i < image.getSize(); ++i) {
    proba.bitmap[i] = m_probas[colorToIndex(image.bitmap[i], m_N, binSize)];
  }
}

double vpColorHistogram::probability(const vpRGBa &color) const
{
  return m_probas[colorToIndex(color, m_N, 256 / m_N)];
}

double vpColorHistogram::kl(const vpColorHistogram &other) const
{
  if (other.m_N != m_N) {
    throw vpException(vpException::badValue, "Histograms should have same dimensions");
  }
  double divergence = 0.0;
  for (unsigned int i = 0; i < m_probas.size(); ++i) {
    if (other.m_probas[i] > 0.0 && m_probas[i] > 0.0) {
      divergence += m_probas[i] * log(m_probas[i] / other.m_probas[i]);
    }
  }
  return divergence;
}

double vpColorHistogram::jsd(const vpColorHistogram &other) const
{

  vpColorHistogram mixture(m_N);

  for (unsigned int i = 0; i < m_probas.size(); ++i) {
    mixture.m_probas[i] = m_probas[i] * 0.5 + other.m_probas[i] * 0.5;
  }
  // JSD = 0.5KL(P || M) + 0.5(Q||M) where M is the average mixture distrib of P and Q
  return (kl(mixture) + other.kl(mixture)) / 2.0;
}

double vpColorHistogram::hellinger(const vpColorHistogram &other) const
{
  double bcoeff = 0.0;

  for (unsigned int i = 0; i < m_probas.size(); ++i) {
    bcoeff += sqrt(m_probas[i] * other.m_probas[i]);
  }

  return sqrt(1.0 - bcoeff);
}


void vpColorHistogram::computeSplitHistograms(const vpImage<vpRGBa> &image, const vpImage<bool> &mask, vpColorHistogram &insideMask, vpColorHistogram &outsideMask)
{
  if (insideMask.m_N != outsideMask.m_N) {
    throw vpException(vpException::badValue, "Histograms should have same number of bins");
  }

  unsigned int bins = insideMask.m_probas.size();

  std::vector<unsigned int> countsIn(bins, 0), countsOut(bins, 0);
  unsigned binSize = 256 / insideMask.m_N;

#pragma omp parallel
  {
    std::vector<unsigned int>localCountsIn(bins, 0), localCountsOut(bins, 0);
#pragma omp for schedule(static, 1024)
    for (unsigned int i = 0; i < image.getSize(); ++i) {
      unsigned int index = colorToIndex(image.bitmap[i], insideMask.m_N, binSize);
      localCountsIn[index] += mask.bitmap[i] > 0;
      localCountsOut[index] += mask.bitmap[i] == 0;
    }
#pragma omp critical
    {
      for (unsigned int i = 0; i < bins; ++i) {
        countsIn[i] += localCountsIn[i];
        countsOut[i] += localCountsOut[i];
      }
    }
  }
  insideMask.build(countsIn);
  outsideMask.build(countsOut);

}

void vpColorHistogram::computeSplitHistograms(const vpImage<vpRGBa> &image, const vpImage<bool> &mask, const vpRect &bbInside, vpColorHistogram &insideMask, vpColorHistogram &outsideMask)
{
  if (insideMask.m_N != outsideMask.m_N) {
    throw vpException(vpException::badValue, "Histograms should have same number of bins");
  }

  const unsigned int bins = insideMask.m_probas.size();

  std::vector<unsigned int> countsIn(bins, 0), countsOut(bins, 0);
  const unsigned binSize = 256 / insideMask.m_N;

  const unsigned int beforeBBStart = static_cast<unsigned int>(bbInside.getTop()) * image.getWidth() + static_cast<unsigned int>(bbInside.getLeft());
  const unsigned int afterBBEnd = static_cast<unsigned int>(bbInside.getBottom()) * image.getWidth() + static_cast<unsigned int>(bbInside.getRight());


//#pragma omp parallel
  {
    std::vector<unsigned int>localCountsIn(bins, 0), localCountsOut(bins, 0);
//#pragma omp for schedule(static, 64)
    for (unsigned int i = 0; i < beforeBBStart; ++i) {
      const unsigned int index = colorToIndex(image.bitmap[i], insideMask.m_N, binSize);
      ++localCountsOut[index];
    }
//#pragma omp for schedule(static, 64)
    for (unsigned int i = afterBBEnd; i < image.getSize(); ++i) {
      const unsigned int index = colorToIndex(image.bitmap[i], insideMask.m_N, binSize);
      ++localCountsOut[index];
    }
//#pragma omp for schedule(static, 64)
    for (unsigned int i = static_cast<unsigned int>(bbInside.getTop()); i < static_cast<unsigned int>(round(bbInside.getBottom())); ++i) {
      for (unsigned int j = static_cast<unsigned int>(bbInside.getLeft()); j < static_cast<unsigned int>(round(bbInside.getRight())); ++j) {
        const unsigned int bitmapIndex = i * image.getWidth() + j;
        const unsigned int index = colorToIndex(image.bitmap[bitmapIndex], insideMask.m_N, binSize);
        const bool pixelInMask = mask.bitmap[bitmapIndex] > 0;
        localCountsIn[index] += static_cast<unsigned int>(pixelInMask);
        localCountsOut[index] += static_cast<unsigned int>(!pixelInMask);
      }
    }
//#pragma omp critical
    {
      for (unsigned int i = 0; i < bins; ++i) {
        countsIn[i] += localCountsIn[i];
        countsOut[i] += localCountsOut[i];
      }
    }
  }
  insideMask.build(countsIn);
  outsideMask.build(countsOut);

}
