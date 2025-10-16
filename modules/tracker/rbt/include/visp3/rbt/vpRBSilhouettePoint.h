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
 */

/*!
  \file vpRBSilhouettePoint.h
  \brief Silhouette point simple candidate representation
*/
#ifndef VP_RB_SILHOUETTE_POINT_H
#define VP_RB_SILHOUETTE_POINT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>

BEGIN_VISP_NAMESPACE
/*!
  \brief Silhouette point simple candidate representation.
  \ingroup group_rbt_core
*/
class VISP_EXPORT vpRBSilhouettePoint
{
public:
  unsigned int i, j; //! Pixel coordinates of the silhouette point
  vpColVector normal; //! Normal to the silhouette at point i,j, in world frame
  double orientation; //! angle of the normal in the image.
  double Z; //! Point depth
  bool isSilhouette;

  vpRBSilhouettePoint(unsigned int a_i, unsigned int a_j, const vpColVector &a_normal, double a_orientation, double a_Z) :
    i(a_i), j(a_j), normal(a_normal), orientation(a_orientation), Z(a_Z)
  { }

  void detectSilhouette(const vpImage<float> &I)
  {
    int range = 4;
    unsigned int urange = static_cast<unsigned int>(range);
    unsigned int k = 0;
    if (i < urange || j < urange || i >= (I.getHeight() - urange) || j >= (I.getWidth() - urange)) {
      isSilhouette = false;
      return;
    }
    double c = cos(orientation);
    double s = sin(orientation);
    for (int n = -range; n <= range; n++) {
      unsigned int ii = static_cast<unsigned int>(round(i + s * n));
      unsigned int jj = static_cast<unsigned int>(round(j + c * n));
      unsigned int isBg = static_cast<unsigned int>(I[ii][jj] == 0.f);
      k += isBg;
    }
    isSilhouette = k > 2;
  }

};

END_VISP_NAMESPACE

#endif
