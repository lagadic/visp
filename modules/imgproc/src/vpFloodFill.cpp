/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2016 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Flood fill algorithm.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!
  \file vpFloodFill.cpp
  \brief Flood fill algorithm.
*/

#include <queue>
#include <visp3/imgproc/vpImgproc.h>


/*!
  \ingroup group_imgproc_connected_components

  Perform the flood fill algorithm.

  \param I : Input image to flood fill.
  \param seedPoint : Seed position in the image.
  \param oldValue : Old value to replace.
  \param newValue : New value to flood fill.
  \param connexity : Type of connexity.
*/
void vp::floodFill(vpImage<unsigned char> &I, const vpImagePoint &seedPoint, const unsigned char oldValue, const unsigned char newValue,
                   const vpImageMorphology::vpConnexityType &connexity) {
  //Code from Lode Vandevenne tutorial.
  //Naive modification for 8-connexity implementation
  if (oldValue == newValue || I.getSize() == 0) {
    return;
  }

  std::queue<vpImagePoint> seed_queue;

  //Add initial seed point
  seed_queue.push(seedPoint);

  while ( !seed_queue.empty() ) {
    vpImagePoint current_seed = seed_queue.front();
    seed_queue.pop();

    unsigned int x = current_seed.get_j();
    unsigned int y = current_seed.get_i();
    int x1 = (int) x;

    //Find most left pixel
    while (x1 >= 0 && I[y][x1] == oldValue) {
      x1--;
    }
    x1++;

    bool spanAbove = false, spanBelow = false;

    while (x1 < (int) I.getWidth() && I[y][x1] == oldValue) {
      I[y][x1] = newValue;

      if (!spanAbove && y > 0) {
        if (I[y-1][x1] == oldValue) {
          //North
          spanAbove = true;
          seed_queue.push( vpImagePoint(y-1, x1) );
        }

        if (connexity != vpImageMorphology::CONNEXITY_4) {
          if (x1 > 0 && I[y-1][x1-1] == oldValue) {
            //North west
            spanAbove = true;
            seed_queue.push( vpImagePoint(y-1, x1-1) );
          }
          if (x1 < (int) I.getWidth()-1 && I[y-1][x1+1] == oldValue) {
            //North east
            spanAbove = true;
            seed_queue.push( vpImagePoint(y-1, x1+1) );
          }
        }
      } else if (spanAbove && y > 0 && I[y-1][x1] != oldValue) {
        spanAbove = false;
      }

      if (!spanBelow && y < I.getHeight()-1) {
        if (I[y+1][x1] == oldValue) {
          //South
          seed_queue.push( vpImagePoint(y+1, x1) );
          spanBelow = true;
        }

        if (connexity != vpImageMorphology::CONNEXITY_4) {
          if (x1 > 0 && I[y+1][x1-1] == oldValue) {
            //South west
            seed_queue.push( vpImagePoint(y+1, x1-1) );
            spanBelow = true;
          }
          if (x1 < (int) I.getWidth()-1 && I[y+1][x1+1] == oldValue) {
            //South east
            seed_queue.push( vpImagePoint(y+1, x1+1) );
            spanBelow = true;
          }
        }
      } else if (spanBelow && y < I.getHeight()-1 && I[y+1][x1] != oldValue) {
        spanBelow = false;
      }

      //TODO: improve 8-connexity
      if (connexity != vpImageMorphology::CONNEXITY_4) {
        spanBelow = false;
        spanAbove = false;
      }

      x1++;
    }
  }
}
