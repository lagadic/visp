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

#include <visp3/rbt/vpDepthMask.h>

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

void vpDepthMask::updateMask(const vpRBFeatureTrackerInput &frame,
                                      const vpRBFeatureTrackerInput &previousFrame,
                                      vpImage<float> &mask)
{
  mask.resize(frame.depth.getHeight(), frame.depth.getWidth());
  // const float radius = frame.renders.objectDiameter / 2.f;
  const float radius = (frame.renders.zFar - frame.renders.zNear) / 2.f;

  const float Zo = frame.renders.cMo[2][3];
  const float Zmi = Zo - radius;
  const float Zma = Zo + radius;
  const float tol = radius / 3.f;
  const float fac = 1.f / (tol * (1.f / sqrtf(2.f)));
#if defined(VISP_HAVE_OPENMP)
#pragma omp parallel for
#endif
  for (unsigned int i = 0; i < frame.depth.getSize(); ++i) {
    const float Z = frame.depth.bitmap[i];
    if (Z == 0.f) {
      mask.bitmap[i] = 1.f;
    }
    else {
      if (Z > Zmi && Z < Zma) {
        mask.bitmap[i] = 1.f;
      }
      else {
        if (Z <= Zmi) {
          mask.bitmap[i] = erfc((Zmi - Z) * fac) / 2.f;

        }
        else {
          mask.bitmap[i] = erfc((Z - Zma) * fac) / 2.f;
        }
      }
    }
  }
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpDepthMask::loadJsonConfiguration(const nlohmann::json &json)
{

}
#endif

END_VISP_NAMESPACE
