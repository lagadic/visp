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
 * CLAHE (Contrast Limited Adaptive Histogram Equalization) algorithm.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/**
 * License: GPL
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */
/**
 * &lsquot;Contrast Limited Adaptive Histogram Equalization&rsquot; as
 * described in
 *
 * <br />BibTeX:
 * <pre>
 * &#64;article{zuiderveld94,
 *   author    = {Zuiderveld, Karel},
 *   title     = {Contrast limited adaptive histogram equalization},
 *   book      = {Graphics gems IV},
 *   year      = {1994},
 *   isbn      = {0-12-336155-9},
 *   pages     = {474--485},
 *   publisher = {Academic Press Professional, Inc.},
 *   address   = {San Diego, CA, USA},
 * }
 * </pre>
 *
 * This version calculates the CDF for adjacent blocks and interpolates
 * the respective CDF for each pixel location in between.
 *
 * @author Stephan Saalfeld <saalfeld@mpi-cbg.de>
 * @version 0.3b
 */

/*!
  \file vpCLAHE.cpp
  \brief Contrast Limited Adaptive Histogram Equalization (CLAHE).
*/

#include <visp3/core/vpImageConvert.h>
#include <visp3/imgproc/vpImgproc.h>

namespace
{
int fastRound(const float value) { return (int)(value + 0.5f); }

void clipHistogram(const std::vector<int> &hist, std::vector<int> &clippedHist, const int limit)
{
  clippedHist = hist;
  int clippedEntries = 0, clippedEntriesBefore = 0;
  int histlength = (int)hist.size();

  do {
    clippedEntriesBefore = clippedEntries;
    clippedEntries = 0;
    for (int i = 0; i < histlength; i++) {
      int d = clippedHist[i] - limit;
      if (d > 0) {
        clippedEntries += d;
        clippedHist[i] = limit;
      }
    }

    int d = clippedEntries / (histlength);
    int m = clippedEntries % (histlength);
    for (int i = 0; i < histlength; i++) {
      clippedHist[i] += d;
    }

    if (m != 0) {
      int s = (histlength - 1) / m;
      for (int i = s / 2; i < histlength; i += s) {
        ++(clippedHist[i]);
      }
    }
  } while (clippedEntries != clippedEntriesBefore);
}

void createHistogram(const int blockRadius, const int bins, const int blockXCenter, const int blockYCenter,
                     const vpImage<unsigned char> &I, std::vector<int> &hist)
{
  std::fill(hist.begin(), hist.end(), 0);

  int xMin = std::max(0, blockXCenter - blockRadius);
  int yMin = std::max(0, blockYCenter - blockRadius);
  int xMax = std::min((int)I.getWidth(), blockXCenter + blockRadius + 1);
  int yMax = std::min((int)I.getHeight(), blockYCenter + blockRadius + 1);

  for (int y = yMin; y < yMax; ++y) {
    for (int x = xMin; x < xMax; ++x) {
      ++hist[fastRound(I[y][x] / 255.0f * bins)];
    }
  }
}

std::vector<float> createTransfer(const std::vector<int> &hist, const int limit, std::vector<int> &cdfs)
{
  clipHistogram(hist, cdfs, limit);
  int hMin = (int)hist.size() - 1;

  for (int i = 0; i < hMin; ++i) {
    if (cdfs[i] != 0) {
      hMin = i;
    }
  }
  int cdf = 0;
  for (int i = hMin; i < (int)hist.size(); ++i) {
    cdf += cdfs[i];
    cdfs[i] = cdf;
  }

  int cdfMin = cdfs[hMin];
  int cdfMax = cdfs[hist.size() - 1];

  std::vector<float> transfer(hist.size());
  for (int i = 0; i < (int)transfer.size(); ++i) {
    transfer[i] = (cdfs[i] - cdfMin) / (float)(cdfMax - cdfMin);
  }

  return transfer;
}

float transferValue(const int v, std::vector<int> &clippedHist)
{
  int clippedHistLength = (int)clippedHist.size();
  int hMin = clippedHistLength - 1;
  for (int i = 0; i < hMin; i++) {
    if (clippedHist[i] != 0) {
      hMin = i;
    }
  }

  int cdf = 0;
  for (int i = hMin; i <= v; i++) {
    cdf += clippedHist[i];
  }

  int cdfMax = cdf;
  for (int i = v + 1; i < clippedHistLength; ++i) {
    cdfMax += clippedHist[i];
  }

  int cdfMin = clippedHist[hMin];
  return (cdf - cdfMin) / (float)(cdfMax - cdfMin);
}

float transferValue(const int v, const std::vector<int> &hist, std::vector<int> &clippedHist, const int limit)
{
  clipHistogram(hist, clippedHist, limit);

  return transferValue(v, clippedHist);
}
}

/*!
  \ingroup group_imgproc_brightness

  Adjust the contrast of a grayscale image locally using the Contrast Limited
  Adaptative Histogram Equalization method. The limit parameter allows to
  limit the slope of the transformation function to prevent the
  overamplification of noise. This method is a transcription of the CLAHE
  ImageJ plugin code by Stephan Saalfeld.

  \param I1 : The first grayscale image.
  \param I2 : The second grayscale image after application of the CLAHE
  method.
  \param blockRadius : The size (2*blockRadius+1) of the local region
  around a pixel for which the histogram is equalized. This size should be
  larger than the size of features to be preserved.
  \param bins : The number
  of histogram bins used for histogram equalization (between 1 and 256). The
  number of histogram bins should be smaller than the number of pixels in a
  block.
  \param slope : Limits the contrast stretch in the intensity transfer
  function. Very large values will let the histogram equalization do whatever
  it wants to do, that is result in maximal local contrast. The value 1 will
  result in the original image.
  \param fast : Use the fast but less accurate
  version of the filter. The fast version does not evaluate the intensity
  transfer function for each pixel independently but for a grid of adjacent
  boxes of the given block size only and interpolates for locations in
  between.
*/
void vp::clahe(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, const int blockRadius, const int bins,
               const float slope, const bool fast)
{
  if (blockRadius < 0) {
    std::cerr << "Error: blockRadius < 0!" << std::endl;
    return;
  }

  if (bins < 0 || bins > 256) {
    std::cerr << "Error: (bins < 0 || bins > 256)!" << std::endl;
    return;
  }

  if ((unsigned int)(2 * blockRadius + 1) > I1.getWidth() || (unsigned int)(2 * blockRadius + 1) > I1.getHeight()) {
    std::cerr << "Error: (unsigned int) (2*blockRadius+1) > I1.getWidth() || "
                 "(unsigned int) (2*blockRadius+1) > I1.getHeight()!"
              << std::endl;
    return;
  }

  I2.resize(I1.getHeight(), I1.getWidth());

  if (fast) {
    int blockSize = 2 * blockRadius + 1;
    int limit = (int)(slope * blockSize * blockSize / bins + 0.5);

    /* div */
    int nc = I1.getWidth() / blockSize;
    int nr = I1.getHeight() / blockSize;

    /* % */
    int cm = I1.getWidth() - nc * blockSize;
    std::vector<int> cs;

    switch (cm) {
    case 0:
      cs.resize(nc);
      for (int i = 0; i < nc; ++i) {
        cs[i] = i * blockSize + blockRadius + 1;
      }
      break;

    case 1:
      cs.resize(nc + 1);
      for (int i = 0; i < nc; ++i) {
        cs[i] = i * blockSize + blockRadius + 1;
      }
      cs[nc] = I1.getWidth() - blockRadius - 1;
      break;

    default:
      cs.resize(nc + 2);
      cs[0] = blockRadius + 1;
      for (int i = 0; i < nc; ++i) {
        cs[i + 1] = i * blockSize + blockRadius + 1 + cm / 2;
      }
      cs[nc + 1] = I1.getWidth() - blockRadius - 1;
    }

    int rm = I1.getHeight() - nr * blockSize;
    std::vector<int> rs;

    switch (rm) {
    case 0:
      rs.resize((size_t)nr);
      for (int i = 0; i < nr; ++i) {
        rs[i] = i * blockSize + blockRadius + 1;
      }
      break;

    case 1:
      rs.resize((size_t)(nr + 1));
      for (int i = 0; i < nr; ++i) {
        rs[i] = i * blockSize + blockRadius + 1;
      }
      rs[nr] = I1.getHeight() - blockRadius - 1;
      break;

    default:
      rs.resize((size_t)(nr + 2));
      rs[0] = blockRadius + 1;
      for (int i = 0; i < nr; ++i) {
        rs[i + 1] = i * blockSize + blockRadius + 1 + rm / 2;
      }
      rs[nr + 1] = I1.getHeight() - blockRadius - 1;
    }

    std::vector<int> hist((size_t)(bins + 1));
    std::vector<int> cdfs((size_t)(bins + 1));
    std::vector<float> tl;
    std::vector<float> tr;
    std::vector<float> br;
    std::vector<float> bl;

    for (int r = 0; r <= (int)rs.size(); ++r) {
      int r0 = std::max(0, r - 1);
      int r1 = std::min((int)rs.size() - 1, r);
      int dr = rs[r1] - rs[r0];

      createHistogram(blockRadius, bins, cs[0], rs[r0], I1, hist);
      tr = createTransfer(hist, limit, cdfs);
      if (r0 == r1) {
        br = tr;
      } else {
        createHistogram(blockRadius, bins, cs[0], rs[r1], I1, hist);
        br = createTransfer(hist, limit, cdfs);
      }

      int yMin = (r == 0 ? 0 : rs[r0]);
      int yMax = (r < (int)rs.size() ? rs[r1] : I1.getHeight());

      for (int c = 0; c <= (int)cs.size(); ++c) {
        int c0 = std::max(0, c - 1);
        int c1 = std::min((int)cs.size() - 1, c);
        int dc = cs[c1] - cs[c0];

        tl = tr;
        bl = br;

        if (c0 != c1) {
          createHistogram(blockRadius, bins, cs[c1], rs[r0], I1, hist);
          tr = createTransfer(hist, limit, cdfs);
          if (r0 == r1) {
            br = tr;
          } else {
            createHistogram(blockRadius, bins, cs[c1], rs[r1], I1, hist);
            br = createTransfer(hist, limit, cdfs);
          }
        }

        int xMin = (c == 0 ? 0 : cs[c0]);
        int xMax = (c < (int)cs.size() ? cs[c1] : I1.getWidth());
        for (int y = yMin; y < yMax; ++y) {
          float wy = (float)(rs[r1] - y) / dr;

          for (int x = xMin; x < xMax; ++x) {
            float wx = (float)(cs[c1] - x) / dc;
            int v = fastRound(I1[y][x] / 255.0f * bins);
            float t00 = tl[v];
            float t01 = tr[v];
            float t10 = bl[v];
            float t11 = br[v];
            float t0 = 0.0f, t1 = 0.0f;

            if (c0 == c1) {
              t0 = t00;
              t1 = t10;
            } else {
              t0 = wx * t00 + (1.0f - wx) * t01;
              t1 = wx * t10 + (1.0f - wx) * t11;
            }

            float t = (r0 == r1) ? t0 : wy * t0 + (1.0f - wy) * t1;
            I2[y][x] = std::max(0, std::min(255, fastRound(t * 255.0f)));
          }
        }
      }
    }
  } else {
    std::vector<int> hist(bins + 1), prev_hist(bins + 1);
    std::vector<int> clippedHist(bins + 1);

    bool first = true;
    int xMin0 = 0;
    int xMax0 = std::min((int)I1.getWidth(), blockRadius);

    for (int y = 0; y < (int)I1.getHeight(); y++) {
      int yMin = std::max(0, y - (int)blockRadius);
      int yMax = std::min((int)I1.getHeight(), y + blockRadius + 1);
      int h = yMax - yMin;

#if 0
      std::fill(hist.begin(), hist.end(), 0);
      // Compute histogram for the current block
      for (int yi = yMin; yi < yMax; yi++) {
        for (int xi = xMin0; xi < xMax0; xi++) {
          ++hist[fastRound(I1[yi][xi] / 255.0f * bins)];
        }
      }
#else
      if (first) {
        first = false;
        // Compute histogram for the block at (0,0)
        for (int yi = yMin; yi < yMax; yi++) {
          for (int xi = xMin0; xi < xMax0; xi++) {
            ++hist[fastRound(I1[yi][xi] / 255.0f * bins)];
          }
        }
      } else {
        hist = prev_hist;

        if (yMin > 0) {
          int yMin1 = yMin - 1;
          // Sliding histogram, remove top
          for (int xi = xMin0; xi < xMax0; xi++) {
            --hist[fastRound(I1[yMin1][xi] / 255.0f * bins)];
          }
        }

        if (y + blockRadius < (int)I1.getHeight()) {
          int yMax1 = yMax - 1;
          // Sliding histogram, add bottom
          for (int xi = xMin0; xi < xMax0; xi++) {
            ++hist[fastRound(I1[yMax1][xi] / 255.0f * bins)];
          }
        }
      }
      prev_hist = hist;
#endif

      for (int x = 0; x < (int)I1.getWidth(); x++) {
        int xMin = std::max(0, x - (int)blockRadius);
        int xMax = x + blockRadius + 1;

        if (xMin > 0) {
          int xMin1 = xMin - 1;
          // Sliding histogram, remove left
          for (int yi = yMin; yi < yMax; yi++) {
            --hist[fastRound(I1[yi][xMin1] / 255.0f * bins)];
          }
        }

        if (xMax <= (int)I1.getWidth()) {
          int xMax1 = xMax - 1;
          // Sliding histogram, add right
          for (int yi = yMin; yi < yMax; yi++) {
            ++hist[fastRound(I1[yi][xMax1] / 255.0f * bins)];
          }
        }

        int v = fastRound(I1[y][x] / 255.0f * bins);
        int w = std::min((int)I1.getWidth(), xMax) - xMin;
        int n = h * w;
        int limit = (int)(slope * n / bins + 0.5f);
        I2[y][x] = fastRound(transferValue(v, hist, clippedHist, limit) * 255.0f);
      }
    }
  }
}

/*!
  \ingroup group_imgproc_brightness

  Adjust the contrast of a color image locally using the Contrast Limited
  Adaptative Histogram Equalization method. The limit parameter allows to
  limit the slope of the transformation function to prevent the
  overamplification of noise. This method is a transcription of the CLAHE
  ImageJ plugin code by Stephan Saalfeld.

  \param I1 : The first color image.
  \param I2 : The second color image after application of the CLAHE method.
  \param blockRadius : The size (2*blockRadius+1) of the local region around a
  pixel for which the histogram is equalized. This size should be larger than
  the size of features to be preserved. \param  bins : The number of histogram
  bins used for histogram equalization (between 1 and 256). The number of
  histogram bins should be smaller than the number of pixels in a block.
  \param slope : Limits the contrast stretch in the intensity transfer
  function. Very large values will let the histogram equalization do whatever
  it wants to do, that is result in maximal local contrast. The value 1 will
  result in the original image. \param fast : Use the fast but less accurate
  version of the filter. The fast version does not evaluate the intensity
  transfer function for each pixel independently but for a grid of adjacent
  boxes of the given block size only and interpolates for locations in
  between.
*/
void vp::clahe(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, const int blockRadius, const int bins, const float slope,
               const bool fast)
{
  // Split
  vpImage<unsigned char> pR(I1.getHeight(), I1.getWidth());
  vpImage<unsigned char> pG(I1.getHeight(), I1.getWidth());
  vpImage<unsigned char> pB(I1.getHeight(), I1.getWidth());
  vpImage<unsigned char> pa(I1.getHeight(), I1.getWidth());

  vpImageConvert::split(I1, &pR, &pG, &pB, &pa);

  // Apply CLAHE independently on RGB channels
  vpImage<unsigned char> resR, resG, resB;
  clahe(pR, resR, blockRadius, bins, slope, fast);
  clahe(pG, resG, blockRadius, bins, slope, fast);
  clahe(pB, resB, blockRadius, bins, slope, fast);

  I2.resize(I1.getHeight(), I1.getWidth());
  unsigned int size = I2.getWidth() * I2.getHeight();
  unsigned char *ptrStart = (unsigned char *)I2.bitmap;
  unsigned char *ptrEnd = ptrStart + size * 4;
  unsigned char *ptrCurrent = ptrStart;

  unsigned int cpt = 0;
  while (ptrCurrent != ptrEnd) {
    *ptrCurrent = resR.bitmap[cpt];
    ++ptrCurrent;

    *ptrCurrent = resG.bitmap[cpt];
    ++ptrCurrent;

    *ptrCurrent = resB.bitmap[cpt];
    ++ptrCurrent;

    *ptrCurrent = pa.bitmap[cpt];
    ++ptrCurrent;

    cpt++;
  }
}
