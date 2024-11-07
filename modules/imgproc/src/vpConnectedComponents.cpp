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
 * Connected components.
 */

/*!
  \file vpConnectedComponents.cpp
  \brief Basic connected components.
*/

#include <queue>
#include <visp3/imgproc/vpImgproc.h>

namespace VISP_NAMESPACE_NAME
{

void getNeighbors(const vpImage<unsigned char> &I, std::queue<vpImagePoint> &listOfNeighbors, unsigned int i,
                  unsigned int j, const vpImageMorphology::vpConnexityType &connexity)
{
  unsigned char currValue = I[i][j];

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    // Top
    if (I[i - 1][j] == currValue) {
      listOfNeighbors.push(vpImagePoint(i - 1, j));
    }

    // Left
    if (I[i][j - 1] == currValue) {
      listOfNeighbors.push(vpImagePoint(i, j - 1));
    }

    // Right
    if (I[i][j + 1] == currValue) {
      listOfNeighbors.push(vpImagePoint(i, j + 1));
    }

    // Bottom
    if (I[i + 1][j] == currValue) {
      listOfNeighbors.push(vpImagePoint(i + 1, j));
    }
  }
  else {
    for (int cpt1 = -1; cpt1 <= 1; ++cpt1) {
      for (int cpt2 = -1; cpt2 <= 1; ++cpt2) {
        // Everything except the current position
        if ((cpt1 != 0) || (cpt2 != 0)) {
          if (I[static_cast<int>(i) + cpt1][static_cast<int>(j) + cpt2] == currValue) {
            listOfNeighbors.push(vpImagePoint(static_cast<int>(i) + cpt1, static_cast<int>(j) + cpt2));
          }
        }
      }
    }
  }
}

void visitNeighbors(vpImage<unsigned char> &I_copy, std::queue<vpImagePoint> &listOfNeighbors, vpImage<int> &labels,
                    int current_label, const vpImageMorphology::vpConnexityType &connexity)
{
  // Visit the neighbors
  while (!listOfNeighbors.empty()) {
    vpImagePoint imPt = listOfNeighbors.front();
    unsigned int i = static_cast<unsigned int>(imPt.get_i());
    unsigned int j = static_cast<unsigned int>(imPt.get_j());
    listOfNeighbors.pop();

    if (I_copy[i][j]) {
      getNeighbors(I_copy, listOfNeighbors, i, j, connexity);

      // Reset current position and set label
      I_copy[i][j] = 0;
      labels[i][j] = current_label;
    }
  }
}

void connectedComponents(const vpImage<unsigned char> &I, vpImage<int> &labels, int &nbComponents, const vpImageMorphology::vpConnexityType &connexity)
{
  if (I.getSize() == 0) {
    return;
  }

  labels.resize(I.getHeight(), I.getWidth());

  vpImage<unsigned char> I_copy(I.getHeight() + 2, I.getWidth() + 2);
  // Copy and add border
  unsigned int i_copy_height = I_copy.getHeight();
  for (unsigned int i = 0; i < i_copy_height; ++i) {
    if ((i == 0) || (i == (I_copy.getHeight() - 1))) {
      memset(I_copy[i], 0, sizeof(unsigned char) * I_copy.getWidth());
    }
    else {
      I_copy[i][0] = 0;
      memcpy(I_copy[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      I_copy[i][I_copy.getWidth() - 1] = 0;
    }
  }

  vpImage<int> labels_copy(I.getHeight() + 2, I.getWidth() + 2, 0);

  int current_label = 1;
  std::queue<vpImagePoint> listOfNeighbors;

  unsigned int i_height = I.getHeight();
  for (unsigned int cpt1 = 0; cpt1 < i_height; ++cpt1) {
    unsigned int i = cpt1 + 1;

    unsigned int i_width = I.getWidth();
    for (unsigned int cpt2 = 0; cpt2 < i_width; ++cpt2) {
      unsigned int j = cpt2 + 1;

      if (I_copy[i][j] && (labels_copy[i][j] == 0)) {
        // Get all the neighbors relative to the current position
        getNeighbors(I_copy, listOfNeighbors, i, j, connexity);

        // Reset current position and set label
        I_copy[i][j] = 0;
        labels_copy[i][j] = current_label;

        visitNeighbors(I_copy, listOfNeighbors, labels_copy, current_label, connexity);

        // Increment label
        ++current_label;
      }
    }
  }
  unsigned int labels_height = labels.getHeight();
  for (unsigned int i = 0; i < labels_height; ++i) {
    memcpy(labels[i], labels_copy[i + 1] + 1, sizeof(int) * labels.getWidth());
  }

  nbComponents = current_label - 1;
}

} // namespace
