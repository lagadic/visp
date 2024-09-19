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

/*!
  \file vpColorHistogram.h
  \brief Color histogram representation
*/
#ifndef VP_COLOR_HISTOGRAM_H
#define VP_COLOR_HISTOGRAM_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRGBa.h>

#include <vector>



BEGIN_VISP_NAMESPACE

template<typename T>
class vpImage;

class vpRGBa;

class vpRect;


class VISP_EXPORT vpColorHistogram
{
public:

  class Builder
  {
  public:
    Builder(unsigned int N) : m_counts(N *N *N, 0), m_N(N), m_binSize(256 / N) { }
    inline void add(const vpRGBa &color)
    {
      unsigned int index = (color.R / m_binSize) * (m_N * m_N) + (color.G / m_binSize) * m_N + (color.B / m_binSize);
      ++m_counts[index];
    }
    void build(vpColorHistogram &histogram);
  private:
    std::vector<unsigned int> m_counts;
    unsigned int m_N, m_binSize;
  };

  vpColorHistogram();
  vpColorHistogram(unsigned int N);

  /**
   * \brief Change the number of bins per color component that the histogram has
   * After calling this method, the histogram will be reset and the values will not be kept.
   * \param N the number of bins per RGB component: the histogram will have N^3 bins in total. N should be a power of 2 between 1 and 128
   */
  void setBinNumber(unsigned int N);

  unsigned int getBinNumber() const { return m_N; }

  unsigned int getNumPixels() const { return m_numPixels; }

  void build(const vpImage<vpRGBa> &image, const vpImage<bool> &mask);

  void build(std::vector<unsigned int> &counts);

  void merge(const vpColorHistogram &other, float alpha);

  void computeProbas(const vpImage<vpRGBa> &image, vpImage<float> &proba) const;

  double probability(const vpRGBa &color) const;

  double kl(const vpColorHistogram &other) const;

  double jsd(const vpColorHistogram &other) const;

  double hellinger(const vpColorHistogram &other) const;

  static void computeSplitHistograms(const vpImage<vpRGBa> &image, const vpImage<bool> &mask, vpColorHistogram &inMask, vpColorHistogram &outsideMask);
  static void computeSplitHistograms(const vpImage<vpRGBa> &image, const vpImage<bool> &mask, const vpRect &bbInside, vpColorHistogram &insideMask, vpColorHistogram &outsideMask);

private:
  unsigned int m_N;
  std::vector<float> m_probas;
  unsigned int m_numPixels;
};
END_VISP_NAMESPACE



#endif
