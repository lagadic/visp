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
 * CLAHE (Contrast Limited Adaptive Histogram Equalization) algorithm.
 */
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

namespace VISP_NAMESPACE_NAME
{

int fastRound(float value) { return static_cast<int>(value + 0.5f); }

void clipHistogram(const std::vector<int> &hist, std::vector<int> &clippedHist, int limit)
{
  clippedHist = hist;
  int clippedEntries = 0, clippedEntriesBefore = 0;
  int histlength = static_cast<int>(hist.size());

  do {
    clippedEntriesBefore = clippedEntries;
    clippedEntries = 0;
    for (int i = 0; i < histlength; ++i) {
      int d = clippedHist[i] - limit;
      if (d > 0) {
        clippedEntries += d;
        clippedHist[i] = limit;
      }
    }

    int d = clippedEntries / histlength;
    int m = clippedEntries % histlength;
    for (int i = 0; i < histlength; ++i) {
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

void createHistogram(int blockRadius, int bins, int blockXCenter, int blockYCenter, const vpImage<unsigned char> &I,
                     std::vector<int> &hist)
{
  std::fill(hist.begin(), hist.end(), 0);

  int xMin = std::max<int>(0, blockXCenter - blockRadius);
  int yMin = std::max<int>(0, blockYCenter - blockRadius);
  int xMax = std::min<int>(static_cast<int>(I.getWidth()), blockXCenter + blockRadius + 1);
  int yMax = std::min<int>(static_cast<int>(I.getHeight()), blockYCenter + blockRadius + 1);

  for (int y = yMin; y < yMax; ++y) {
    for (int x = xMin; x < xMax; ++x) {
      ++hist[fastRound((I[y][x] / 255.0f) * bins)];
    }
  }
}

std::vector<float> createTransfer(const std::vector<int> &hist, int limit, std::vector<int> &cdfs)
{
  clipHistogram(hist, cdfs, limit);
  int hMin = static_cast<int>(hist.size()) - 1;

  int stopIdx = hMin;
  bool hasNotFoundFirstNotZero = true;
  int i = 0;
  while ((i < stopIdx) && hasNotFoundFirstNotZero) {
    if (cdfs[i] != 0) {
      hMin = i;
      hasNotFoundFirstNotZero = false;
    }
    ++i;
  }
  int cdf = 0;
  int hist_size = static_cast<int>(hist.size());
  for (int i = hMin; i < hist_size; ++i) {
    cdf += cdfs[i];
    cdfs[i] = cdf;
  }

  int cdfMin = cdfs[hMin];
  int cdfMax = cdfs[hist.size() - 1];

  std::vector<float> transfer(hist.size());
  int transfer_size = static_cast<int>(transfer.size());
  for (int i = 0; i < transfer_size; ++i) {
    transfer[i] = (cdfs[i] - cdfMin) / static_cast<float>(cdfMax - cdfMin);
  }

  return transfer;
}

float transferValue(int v, std::vector<int> &clippedHist)
{
  int clippedHistLength = static_cast<int>(clippedHist.size());
  int hMin = clippedHistLength - 1;
  int idxStop = hMin;
  int i = 0;
  bool hasNotFoundFirstNotZero = true;
  while ((i<idxStop) && hasNotFoundFirstNotZero) {
    if (clippedHist[i] != 0) {
      hMin = i;
      hasNotFoundFirstNotZero = false;
    }
    ++i;
  }

  int cdf = 0;
  for (int i = hMin; i <= v; ++i) {
    cdf += clippedHist[i];
  }

  int cdfMax = cdf;
  for (int i = v + 1; i < clippedHistLength; ++i) {
    cdfMax += clippedHist[i];
  }

  int cdfMin = clippedHist[hMin];
  return (cdf - cdfMin) / static_cast<float>(cdfMax - cdfMin);
}

float transferValue(int v, const std::vector<int> &hist, std::vector<int> &clippedHist, int limit)
{
  clipHistogram(hist, clippedHist, limit);

  return transferValue(v, clippedHist);
}

bool checkClaheInputs(const int &blockRadius, const int &bins, const unsigned int &width, const unsigned int &height)
{
  if (blockRadius < 0) {
    std::cerr << "Error: blockRadius < 0!" << std::endl;
    return false;
  }

  const int maxBins = 256;
  if ((bins < 0) || (bins > maxBins)) {
    std::cerr << "Error: (bins < 0 || bins > " << maxBins << ")!" << std::endl;
    return false;
  }

  const int twice = 2;
  if ((static_cast<unsigned int>((twice * blockRadius) + 1) > width) || (static_cast<unsigned int>((twice * blockRadius) + 1) > height)) {
    std::cerr << "Error: (unsigned int) (2*blockRadius+1) > I1.getWidth() || "
      "(unsigned int) (2*blockRadius+1) > I1.getHeight()!"
      << std::endl;
    return false;
  }
  return true;
}

void clahe(const vpImage<unsigned char> &I1, vpImage<unsigned char> &I2, int blockRadius, int bins, float slope, bool fast)
{
  if (!checkClaheInputs(blockRadius, bins, I1.getWidth(), I1.getHeight())) { return; }

  I2.resize(I1.getHeight(), I1.getWidth());
  if (fast) {
    const int val_2 = 2;
    int blockSize = (val_2 * blockRadius) + 1;
    int limit = static_cast<int>(((slope * blockSize * blockSize) / bins) + 0.5);
    /* div */
    int nc = I1.getWidth() / blockSize;
    int nr = I1.getHeight() / blockSize;
    /* % */
    int cm = I1.getWidth() - (nc * blockSize);
    std::vector<int> cs;
    switch (cm) {
    case 0:
      cs.resize(nc);
      for (int i = 0; i < nc; ++i) {
        cs[i] = (i * blockSize) + blockRadius + 1;
      }
      break;
    case 1:
      cs.resize(nc + 1);
      for (int i = 0; i < nc; ++i) {
        cs[i] = (i * blockSize) + blockRadius + 1;
      }
      cs[nc] = I1.getWidth() - blockRadius - 1;
      break;
    default:
      cs.resize(nc + val_2);
      cs[0] = blockRadius + 1;
      for (int i = 0; i < nc; ++i) {
        cs[i + 1] = (i * blockSize) + blockRadius + 1 + (cm / val_2);
      }
      cs[nc + 1] = I1.getWidth() - blockRadius - 1;
    }

    int rm = I1.getHeight() - (nr * blockSize);
    std::vector<int> rs;
    switch (rm) {
    case 0:
      rs.resize(static_cast<size_t>(nr));
      for (int i = 0; i < nr; ++i) {
        rs[i] = (i * blockSize) + blockRadius + 1;
      }
      break;
    case 1:
      rs.resize(static_cast<size_t>(nr + 1));
      for (int i = 0; i < nr; ++i) {
        rs[i] = (i * blockSize) + blockRadius + 1;
      }
      rs[nr] = I1.getHeight() - blockRadius - 1;
      break;
    default:
      rs.resize(static_cast<size_t>(nr + val_2));
      rs[0] = blockRadius + 1;
      for (int i = 0; i < nr; ++i) {
        rs[i + 1] = (i * blockSize) + blockRadius + 1 + (rm / val_2);
      }
      rs[nr + 1] = I1.getHeight() - blockRadius - 1;
    }

    std::vector<int> hist(static_cast<size_t>(bins + 1)), cdfs(static_cast<size_t>(bins + 1));
    std::vector<float> tl, tr, br, bl;
    int rs_size = static_cast<int>(rs.size());
    for (int r = 0; r <= rs_size; ++r) {
      int r0 = std::max<int>(0, r - 1);
      int r1 = std::min<int>(static_cast<int>(rs.size()) - 1, r);
      int dr = rs[r1] - rs[r0];
      createHistogram(blockRadius, bins, cs[0], rs[r0], I1, hist);
      tr = createTransfer(hist, limit, cdfs);
      if (r0 == r1) {
        br = tr;
      }
      else {
        createHistogram(blockRadius, bins, cs[0], rs[r1], I1, hist);
        br = createTransfer(hist, limit, cdfs);
      }

      int yMin = (r == 0 ? 0 : rs[r0]);
      int yMax = (r < static_cast<int>(rs.size()) ? rs[r1] : I1.getHeight());
      int cs_size = static_cast<int>(cs.size());
      for (int c = 0; c <= cs_size; ++c) {
        int c0 = std::max<int>(0, c - 1);
        int c1 = std::min<int>(static_cast<int>(cs.size()) - 1, c);
        int dc = cs[c1] - cs[c0];
        tl = tr;
        bl = br;
        if (c0 != c1) {
          createHistogram(blockRadius, bins, cs[c1], rs[r0], I1, hist);
          tr = createTransfer(hist, limit, cdfs);
          if (r0 == r1) {
            br = tr;
          }
          else {
            createHistogram(blockRadius, bins, cs[c1], rs[r1], I1, hist);
            br = createTransfer(hist, limit, cdfs);
          }
        }

        int xMin = (c == 0 ? 0 : cs[c0]);
        int xMax = (c < static_cast<int>(cs.size()) ? cs[c1] : I1.getWidth());
        for (int y = yMin; y < yMax; ++y) {
          float wy = static_cast<float>(rs[r1] - y) / dr;
          for (int x = xMin; x < xMax; ++x) {
            float wx = static_cast<float>(cs[c1] - x) / dc;
            int v = fastRound((I1[y][x] / 255.0f) * bins);
            float t00 = tl[v];
            float t01 = tr[v];
            float t10 = bl[v];
            float t11 = br[v];
            float t0 = (c0 == c1) ? t00 : ((wx * t00) + ((1.0f - wx) * t01));
            float t1 = (c0 == c1) ? t10 : ((wx * t10) + ((1.0f - wx) * t11));
            float t = (r0 == r1) ? t0 : ((wy * t0) + ((1.0f - wy) * t1));
            const int maxPixelIntensity = 255;
            I2[y][x] = std::max<unsigned char>(0, std::min<unsigned char>(maxPixelIntensity, fastRound(t * 255.0f)));
          }
        }
      }
    }
  }
  else {
    std::vector<int> hist(bins + 1), prev_hist(bins + 1), clippedHist(bins + 1);
    bool first = true;
    int xMin0 = 0;
    int xMax0 = std::min<int>(static_cast<int>(I1.getWidth()), blockRadius);
    int i1_height = static_cast<int>(I1.getHeight());
    for (int y = 0; y < i1_height; ++y) {
      int yMin = std::max<int>(0, y - static_cast<int>(blockRadius));
      int yMax = std::min<int>(static_cast<int>(I1.getHeight()), y + blockRadius + 1);
      int h = yMax - yMin;

      if (first) {
        first = false;
        // Compute histogram for the block at (0,0)
        for (int yi = yMin; yi < yMax; ++yi) {
          for (int xi = xMin0; xi < xMax0; ++xi) {
            ++hist[fastRound((I1[yi][xi] / 255.0f) * bins)];
          }
        }
      }
      else {
        hist = prev_hist;

        if (yMin > 0) {
          int yMin1 = yMin - 1;
          // Sliding histogram, remove top
          for (int xi = xMin0; xi < xMax0; ++xi) {
            --hist[fastRound((I1[yMin1][xi] / 255.0f) * bins)];
          }
        }

        if ((y + blockRadius) < static_cast<int>(I1.getHeight())) {
          int yMax1 = yMax - 1;
          // Sliding histogram, add bottom
          for (int xi = xMin0; xi < xMax0; ++xi) {
            ++hist[fastRound((I1[yMax1][xi] / 255.0f) * bins)];
          }
        }
      }
      prev_hist = hist;

      int i1_width = static_cast<int>(I1.getWidth());
      for (int x = 0; x < i1_width; ++x) {
        int xMin = std::max<int>(0, x - static_cast<int>(blockRadius));
        int xMax = x + blockRadius + 1;

        if (xMin > 0) {
          int xMin1 = xMin - 1;
          // Sliding histogram, remove left
          for (int yi = yMin; yi < yMax; ++yi) {
            --hist[fastRound((I1[yi][xMin1] / 255.0f) * bins)];
          }
        }

        if (xMax <= static_cast<int>(I1.getWidth())) {
          int xMax1 = xMax - 1;
          // Sliding histogram, add right
          for (int yi = yMin; yi < yMax; ++yi) {
            ++hist[fastRound((I1[yi][xMax1] / 255.0f) * bins)];
          }
        }

        int v = fastRound((I1[y][x] / 255.0f) * bins);
        int w = std::min<int>(static_cast<int>(I1.getWidth()), xMax) - xMin;
        int n = h * w;
        int limit = static_cast<int>(((slope * n) / bins) + 0.5f);
        I2[y][x] = fastRound(transferValue(v, hist, clippedHist, limit) * 255.0f);
      }
    }
  }
}

void clahe(const vpImage<vpRGBa> &I1, vpImage<vpRGBa> &I2, int blockRadius, int bins, float slope, bool fast)
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

  const unsigned int sizeRGBa = 4;
  I2.resize(I1.getHeight(), I1.getWidth());
  unsigned int size = I2.getWidth() * I2.getHeight();
  unsigned char *ptrStart = reinterpret_cast<unsigned char *>(I2.bitmap);
  unsigned char *ptrEnd = ptrStart + (size * sizeRGBa);
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

    ++cpt;
  }
}

} // namespace
