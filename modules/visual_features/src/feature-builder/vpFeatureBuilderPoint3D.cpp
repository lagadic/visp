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
 * Conversion between tracker and visual feature 3D Point.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderPoint3D.cpp
  \brief  conversion between tracker
  and visual feature 3D Point
*/
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureException.h>

/*!

  Initialize a 3D point feature using the coordinates of the point
  \f$(X,Y,Z)\f$ in the camera frame. The values of \f$(X,Y,Z)\f$ are
  expressed in meters.

  \warning To be sure that the vpFeaturePoint is well initialized, you have to
  be sure that at least the point coordinates in the camera frame are computed
  and stored in the vpPoint.

  \param s : Visual feature to initialize.

  \param t : The vpPoint used to create the vpFeaturePoint.
*/
void vpFeatureBuilder::create(vpFeaturePoint3D &s, const vpPoint &t)
{
  try {

    s.set_X(t.cP[0] / t.cP[3]);
    s.set_Y(t.cP[1] / t.cP[3]);
    s.set_Z(t.cP[2] / t.cP[3]);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
