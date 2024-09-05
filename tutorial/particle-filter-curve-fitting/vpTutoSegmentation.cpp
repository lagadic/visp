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

#include "vpTutoSegmentation.h"

#include <visp3/core/vpGaussRand.h>

namespace tutorial
{
void performSegmentationHSV(vpTutoCommonData &data)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  const unsigned int height = data.m_I_orig.getHeight(), width = data.m_I_orig.getWidth();
  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);
  vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(data.m_I_orig.bitmap),
                            H.bitmap,
                            S.bitmap,
                            V.bitmap,
                            data.m_I_orig.getSize());

  vpImageTools::inRange(H.bitmap,
                        S.bitmap,
                        V.bitmap,
                        data.m_hsv_values,
                        data.m_mask.bitmap,
                        data.m_mask.getSize());

  vpImageTools::inMask(data.m_I_orig, data.m_mask, data.m_I_segmented);
}

std::vector< VISP_NAMESPACE_ADDRESSING vpImagePoint > extractSkeletton(vpTutoCommonData &data)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  const int height = data.m_mask.getHeight();
  const int width = data.m_mask.getWidth();
  data.m_Iskeleton.resize(height, width, 0);
  std::vector<vpImagePoint> points;
  // Edge thinning along the horizontal direction
  for (int y = 0; y < height; ++y) {
    int left = -1;
    for (int x = 0; x < width - 1; ++x) {
      if ((data.m_mask[y][x] > 0) && (data.m_mask[y][x + 1] > 0)) {
        if (left < 0) {
          left = x;
        }
      }
      else if (data.m_mask[y][x] > 0) {
        int cx = x; // Case 1 pix wide
        if (left >= 0) {
          // Case more than 1 pix wide
          cx = static_cast<int>(((left + x) - 1) * 0.5f);
        }
        vpImagePoint pt(y, cx);
        points.push_back(pt);
        data.m_Iskeleton[y][cx] = 255;
        left = -1;
      }
    }
  }

  // Edge thinning along the vertical direction
  for (int x = 0; x < width; ++x) {
    int top = -1;
    for (int y = 0; y < height - 1; ++y) {
      if ((data.m_mask[y][x] > 0) && (data.m_mask[y + 1][x] > 0)) {
        if (top < 0) {
          top = y;
        }
      }
      else if (data.m_mask[y][x] > 0) {
        int cy = y; // Case 1 pix wide
        if (top >= 0) {
          cy = static_cast<int>(((top + y) - 1) * 0.5f);  // Case more than 1 pix wide
        }
        if (data.m_Iskeleton[cy][x] == 0) {
          vpImagePoint pt(cy, x);
          points.push_back(pt);
          data.m_Iskeleton[cy][x] = 255;
        }
        top = -1;
      }
    }
  }
  return points;
}

std::vector< vpImagePoint > addSaltAndPepperNoise(const std::vector< vpImagePoint > &noisefreePts, vpTutoCommonData &data)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  const unsigned int nbNoiseFreePts = noisefreePts.size();
  const unsigned int nbPtsToAdd = data.m_ratioSaltPepperNoise * nbNoiseFreePts;
  const double width = data.m_Iskeleton.getWidth();
  const double height = data.m_Iskeleton.getHeight();
  data.m_IskeletonNoisy = data.m_Iskeleton;
  vpGaussRand rngX(0.1666, 0.5, vpTime::measureTimeMicros());
  vpGaussRand rngY(0.1666, 0.5, vpTime::measureTimeMicros() + 4224);
  std::vector<vpImagePoint> noisyPts = noisefreePts;
  for (unsigned int i = 0; i < nbPtsToAdd + 1; ++i) {
    double uNormalized = rngX();
    double vNormalized = rngY();
    // Clamp to interval[0, 1[
    uNormalized = std::max(uNormalized, 0.);
    uNormalized = std::min(uNormalized, 0.99999);
    vNormalized = std::max(vNormalized, 0.);
    vNormalized = std::min(vNormalized, 0.99999);
    // Scale to image size
    double u = uNormalized * width;
    double v = vNormalized * height;
    // Create corresponding image point
    vpImagePoint pt(v, u);
    noisyPts.push_back(pt);
    data.m_IskeletonNoisy[static_cast<int>(v)][static_cast<int>(u)] = 255;
  }
  return noisyPts;
}
}
