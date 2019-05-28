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
 * Additional image morphology functions.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpMorph.cpp
  \brief Additional image morphology functions.
*/

#include <visp3/core/vpImageTools.h>
#include <visp3/imgproc/vpImgproc.h>

/*!
  \ingroup group_imgproc_morph

  Fill the holes in a binary image.

  \param I : Input binary image (0 means background, 255 means foreground).
*/
void vp::fillHoles(vpImage<unsigned char> &I
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
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      mask[i + 1][j + 1] = 255 - I[i][j];
    }
  }

  vpImage<unsigned char> marker(I.getHeight() + 2, I.getWidth() + 2, 0);
  // Create marker with 255 1-pixel border
  for (unsigned int i = 0; i < marker.getHeight(); i++) {
    if (i == 0 || i == marker.getHeight() - 1) {
      for (unsigned int j = 0; j < marker.getWidth(); j++) {
        marker[i][j] = 255;
      }
    } else {
      marker[i][0] = 255;
      marker[i][marker.getWidth() - 1] = 255;
    }
  }

  vpImage<unsigned char> I_reconstruct;
  reconstruct(marker, mask, I_reconstruct, connexity);

  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      I[i][j] = 255 - I_reconstruct[i + 1][j + 1];
    }
  }
#else
  // Create flood fill mask
  vpImage<unsigned char> flood_fill_mask(I.getHeight() + 2, I.getWidth() + 2, 0);
  // Copy I to mask + add border padding
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    memcpy(flood_fill_mask[i + 1] + 1, I[i], sizeof(unsigned char) * I.getWidth());
  }

  // Perform flood fill
  vp::floodFill(flood_fill_mask, vpImagePoint(0, 0), 0, 255);

  // Get current mask
  vpImage<unsigned char> mask(I.getHeight(), I.getWidth());
  for (unsigned int i = 0; i < mask.getHeight(); i++) {
    memcpy(mask[i], flood_fill_mask[i + 1] + 1, sizeof(unsigned char) * mask.getWidth());
  }

  // Get image with holes filled
  vpImage<unsigned char> I_white(I.getHeight(), I.getWidth(), 255), I_holes;
  vpImageTools::imageSubtract(I_white, mask, I_holes);
  vpImageTools::imageAdd(I, I_holes, I, true);
#endif
}

/*!
  \ingroup group_imgproc_morph

  Perform morphological reconstruction of the image \a marker under the image
  \a mask. Definition from Gleb V. Tcheslavsk: > The morphological
  reconstruction by dilation of a grayscale image \f$ g \f$ by a grayscale
  marker image \f$ f \f$ > is defined as the geodesic dilation of \f$ f \f$
  with respect to \f$ g \f$ repeated (iterated) until stability is reached:
  \f[
    R_{g}^{D} \left ( f \right ) = D_{g}^{\left ( k \right )} \left ( f \right
  ) \f] with \f$ k \f$ such that: \f$ D_{g}^{\left ( k \right )} \left ( f
  \right ) = D_{g}^{\left ( k+1 \right )} \left ( f \right ) \f$

  \param marker : Grayscale image marker.
  \param mask : Grayscale image mask.
  \param h_kp1 : Image morphologically reconstructed.
  \param connexity : Type of connexity.
*/
void vp::reconstruct(const vpImage<unsigned char> &marker, const vpImage<unsigned char> &mask,
                     vpImage<unsigned char> &h_kp1 /*alias I */, const vpImageMorphology::vpConnexityType &connexity)
{
  if (marker.getHeight() != mask.getHeight() || marker.getWidth() != mask.getWidth()) {
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

  do {
    // Dilatation
    vpImageMorphology::dilatation(h_kp1, connexity);

    // Keep min
    for (unsigned int i = 0; i < h_kp1.getHeight(); i++) {
      for (unsigned int j = 0; j < h_kp1.getWidth(); j++) {
        h_kp1[i][j] = std::min(h_kp1[i][j], mask[i][j]);
      }
    }

    if (h_kp1 == h_k) {
      break;
    }

    h_k = h_kp1;
  } while (true);
}
