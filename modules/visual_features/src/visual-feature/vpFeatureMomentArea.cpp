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
 * Implementation of vpFeatureMomentArea associated to vpMomentArea
 *
 * Authors:
 * Manikandan Bakthavatchalam
 *
 *****************************************************************************/
#include <cassert>
#include <limits>
#include <limits> // numeric_limits
#include <vector>
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpMomentArea.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentArea.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>

/*!
  Computes interaction matrix for the normalized surface moment. Called
  internally. The moment primitives must be computed before calling this. This
  feature depends on:
  - vpMomentGravityCenter
  - vpMomentArea
*/
void vpFeatureMomentArea::compute_interaction()
{

  interaction_matrices.resize(1);
  interaction_matrices[0].resize(1, 6);

  // Retreive the moment object associated with this feature
  vpMomentObject mobj = moment->getObject();
  if (mobj.getType() == vpMomentObject::DISCRETE) {
    /*
     *  The interaction matrix for the discrete case is zero
     *  since the feature m00 is constant.
     *  Refer thesis of Omar Tahri 2005 [Section 3.4.22]
     */
    interaction_matrices[0][0][0] = 0.;
    interaction_matrices[0][0][1] = 0.;
    interaction_matrices[0][0][2] = 0.;
    interaction_matrices[0][0][3] = 0.;
    interaction_matrices[0][0][4] = 0.;
    interaction_matrices[0][0][5] = 0.;
  } else {
    // Get Xg and Yg
    bool found_xgyg;
    const vpMomentGravityCenter &momentGravity =
        static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_xgyg));
    if (!found_xgyg)
      throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

    bool found_m00;
    const vpMomentArea &areamoment = static_cast<const vpMomentArea &>(moments.get("vpMomentArea", found_m00));
    if (!found_m00)
      throw vpException(vpException::notInitialized, "vpMomentArea not found");

    double Xg = momentGravity.getXg();
    double Yg = momentGravity.getYg();

    double a = areamoment.get()[0]; // Area scalar

    assert(std::fabs(a - mobj.get(0, 0)) < a * std::numeric_limits<double>::epsilon());

    interaction_matrices[0][0][0] = -a * A;
    interaction_matrices[0][0][1] = -a * B;
    interaction_matrices[0][0][2] = (3 * a) * (A * Xg + B * Yg) + (2 * C * a);
    interaction_matrices[0][0][3] = 3 * a * Yg;
    interaction_matrices[0][0][4] = -3 * a * Xg;
    interaction_matrices[0][0][5] = 0.;
  }
}
