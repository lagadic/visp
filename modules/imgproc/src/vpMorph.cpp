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
 * Additional image morphology functions.
 */

/*!
  \file vpMorph.cpp
  \brief Additional image morphology functions.
*/

#include <visp3/core/vpImageTools.h>
#include <visp3/imgproc/vpImgproc.h>

namespace VISP_NAMESPACE_NAME
{

void fillHoles(vpImage<unsigned char> &I
#if USE_OLD_FILL_HOLE
  ,
  const vpImageMorphology::vpConnexityType &connexity
#endif
)
{
  if (I.getSize() == 0) {
    return;
  }

#if USE_OLD_FILL_HOLE
  // Code similar to Matlab imfill(BW,'holes')
  // Replaced by flood fill as imfill use imreconstruct
  // and our reconstruct implementation is naive and inefficient
  // Difference between new and old implementation:
  //  - new implementation allows to set the fill value
  //  - only background==0 is required, before it was 0 (background) / 1
  //  (foreground)
  //  - no more connexity option
  vpImage<unsigned char> mask(I.getHeight() + 2, I.getWidth() + 2, 255);
  // Copy I to mask + add border padding + complement
  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      mask[i + 1][j + 1] = 255 - I[i][j];
    }
  }

  vpImage<unsigned char> marker(I.getHeight() + 2, I.getWidth() + 2, 0);
  // Create marker with 255 1-pixel border
  for (unsigned int i = 0; i < marker.getHeight(); ++i) {
    if (i == 0 || i == marker.getHeight() - 1) {
      for (unsigned int j = 0; j < marker.getWidth(); ++j) {
        marker[i][j] = 255;
      }
    }
    else {
      marker[i][0] = 255;
      marker[i][marker.getWidth() - 1] = 255;
    }
  }

  vpImage<unsigned char> I_reconstruct;
  reconstruct(marker, mask, I_reconstruct, connexity);

  for (unsigned int i = 0; i < I.getHeight(); ++i) {
    for (unsigned int j = 0; j < I.getWidth(); ++j) {
      I[i][j] = 255 - I_reconstruct[i + 1][j + 1];
    }
  }
#else
  // Create flood fill mask
  vpImage<unsigned char> flood_fill_mask(I.getHeight() + 2, I.getWidth() + 2, 0);
  // Copy I to mask + add border padding
  unsigned int i_height = I.getHeight();
  for (unsigned int i = 0; i < i_height; ++i) {
    memcpy(flood_fill_mask[i + 1] + 1, I[i], sizeof(unsigned char) * I.getWidth());
  }

  // Perform flood fill
  const unsigned char newVal = 255;
  floodFill(flood_fill_mask, vpImagePoint(0, 0), 0, newVal);

  // Get current mask
  vpImage<unsigned char> mask(I.getHeight(), I.getWidth());
  unsigned int mask_height = mask.getHeight();
  for (unsigned int i = 0; i < mask_height; ++i) {
    memcpy(mask[i], flood_fill_mask[i + 1] + 1, sizeof(unsigned char) * mask.getWidth());
  }

  // Get image with holes filled
  vpImage<unsigned char> I_white(I.getHeight(), I.getWidth(), 255), I_holes;
  vpImageTools::imageSubtract(I_white, mask, I_holes);
  vpImageTools::imageAdd(I, I_holes, I, true);
#endif
}

void reconstruct(const vpImage<unsigned char> &marker, const vpImage<unsigned char> &mask,
                 vpImage<unsigned char> &h_kp1 /*alias I */, const vpImageMorphology::vpConnexityType &connexity)
{
  if ((marker.getHeight() != mask.getHeight()) || (marker.getWidth() != mask.getWidth())) {
    std::cerr << "marker.getHeight() != mask.getHeight() || "
      "marker.getWidth() != mask.getWidth()"
      << std::endl;
    return;
  }

  if (marker.getSize() == 0) {
    std::cerr << "Input images are empty!" << std::endl;
    return;
  }

  vpImage<unsigned char> h_k = marker;
  h_kp1 = h_k;

  bool h_kp1_eq_h_k = false;
  do {
    // Dilatation
    vpImageMorphology::dilatation<unsigned char>(h_kp1, connexity);

    // Keep min
    unsigned int h_kp1_height = h_kp1.getHeight();
    unsigned int h_kp1_width = h_kp1.getWidth();
    for (unsigned int i = 0; i < h_kp1_height; ++i) {
      for (unsigned int j = 0; j < h_kp1_width; ++j) {
        h_kp1[i][j] = std::min<unsigned char>(h_kp1[i][j], mask[i][j]);
      }
    }

    if (h_kp1 == h_k) {
      h_kp1_eq_h_k = true;
      // break
    }
    else {
      h_k = h_kp1;
    }
  } while (h_kp1_eq_h_k == false);
}

} // namespace
