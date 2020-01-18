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
 * Implementation for all supported moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_MOMENTS_COMBINE_MATRICES

#include <limits>
#include <vector>

#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenter.h>

/*!
  Computes interaction matrix for gravity center moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpFeatureMomentBasic

  Minimum vpMomentObject order needed to compute this feature: 2.
*/
void vpFeatureMomentGravityCenter::compute_interaction()
{
  bool found_featuremoment_basic;

  vpFeatureMomentBasic &featureMomentBasic = (static_cast<vpFeatureMomentBasic &>(
      featureMomentsDataBase->get("vpFeatureMomentBasic", found_featuremoment_basic)));
  const vpMomentObject &momentObject = moment->getObject();

  if (!found_featuremoment_basic)
    throw vpException(vpException::notInitialized, "vpFeatureMomentBasic not found");

  interaction_matrices[0].resize(1, 6);
  interaction_matrices[1].resize(1, 6);

  interaction_matrices[0] =
      featureMomentBasic.interaction(1, 0) / momentObject.get(0, 0) -
      momentObject.get(1, 0) * pow(momentObject.get(0, 0), -0.2e1) * featureMomentBasic.interaction(0, 0);
  interaction_matrices[1] =
      featureMomentBasic.interaction(0, 1) / momentObject.get(0, 0) -
      momentObject.get(0, 1) * pow(momentObject.get(0, 0), -0.2e1) * featureMomentBasic.interaction(0, 0);
}

#else

#include <limits>
#include <vector>

#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentGravityCenter.h>

/*!
  Computes interaction matrix for gravity center moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpMomentCentered
  - vpMomentGravityCenter

  Minimum vpMomentObject order needed to compute this feature: 2.
*/
void vpFeatureMomentGravityCenter::compute_interaction()
{
  bool found_moment_centered;
  bool found_moment_gravity;

  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered)));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_moment_gravity));

  const vpMomentObject &momentObject = moment->getObject();

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

  interaction_matrices[0].resize(1, 6);
  interaction_matrices[1].resize(1, 6);
  int epsilon;
  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    epsilon = 1;
  } else {
    epsilon = 4;
  }
  double n11 = momentCentered.get(1, 1) / momentObject.get(0, 0);
  double n20 = momentCentered.get(2, 0) / momentObject.get(0, 0);
  double n02 = momentCentered.get(0, 2) / momentObject.get(0, 0);
  double Xg = momentGravity.getXg();
  double Yg = momentGravity.getYg();
  double Zg_inv = A * Xg + B * Yg + C;
  double Xgvz = Xg * Zg_inv + A * epsilon * n20 + B * epsilon * n11;
  double Ygvz = Yg * Zg_inv + A * epsilon * n11 + B * epsilon * n02;
  double Xgwx = Xg * Yg + epsilon * n11;
  double Ygwy = -Xgwx;
  double Xgwy = -(1 + Xg * Xg + epsilon * n20);
  double Ygwx = 1 + Yg * Yg + epsilon * n02;

  int VX = 0;
  int VY = 1;
  int VZ = 2;
  int WX = 3;
  int WY = 4;
  int WZ = 5;

  interaction_matrices[0][0][VX] = -Zg_inv;
  interaction_matrices[0][0][VY] = 0;
  interaction_matrices[0][0][VZ] = Xgvz;

  interaction_matrices[0][0][WX] = Xgwx;
  interaction_matrices[0][0][WY] = Xgwy;
  interaction_matrices[0][0][WZ] = Yg;

  interaction_matrices[1][0][VX] = 0;
  interaction_matrices[1][0][VY] = -Zg_inv;
  interaction_matrices[1][0][VZ] = Ygvz;

  interaction_matrices[1][0][WX] = Ygwx;
  interaction_matrices[1][0][WY] = Ygwy;
  interaction_matrices[1][0][WZ] = -Xg;
}

#endif
