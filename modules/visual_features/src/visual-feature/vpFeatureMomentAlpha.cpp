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
 * Implementation for alpha moment features.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentAlpha.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>

#include <limits>
#include <vector>

#ifdef VISP_MOMENTS_COMBINE_MATRICES

/*!
  Computes interaction matrix for alpha moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpMomentCentered
  - vpFeatureMomentCentered
*/
void vpFeatureMomentAlpha::compute_interaction()
{
  bool found_moment_centered;
  bool found_FeatureMoment_centered;

  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered)));
  vpFeatureMomentCentered &featureMomentCentered = (static_cast<vpFeatureMomentCentered &>(
      featureMomentsDataBase->get("vpFeatureMomentCentered", found_FeatureMoment_centered)));

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_FeatureMoment_centered)
    throw vpException(vpException::notInitialized, "vpFeatureMomentCentered not found");

  double multiplier =
      -1. /
      (momentCentered.get(2, 0) * momentCentered.get(2, 0) - 2 * momentCentered.get(0, 2) * momentCentered.get(2, 0) +
       4 * momentCentered.get(1, 1) * momentCentered.get(1, 1) + momentCentered.get(0, 2) * momentCentered.get(0, 2));

  interaction_matrices[0].resize(1, 6);
  interaction_matrices[0] =
      multiplier * (momentCentered.get(1, 1) * featureMomentCentered.interaction(2, 0) +
                    (momentCentered.get(0, 2) - momentCentered.get(2, 0)) * featureMomentCentered.interaction(1, 1) -
                    momentCentered.get(1, 1) * featureMomentCentered.interaction(0, 2));
}

#else

/*!
  Computes interaction matrix for alpha moment. Called internally.
  The moment primitives must be computed before calling this.
  This feature depends on:
  - vpMomentCentered
  - vpMomentGravityCenter
*/
void vpFeatureMomentAlpha::compute_interaction()
{
  bool found_moment_centered;
  bool found_moment_gravity;

  const vpMomentCentered &momentCentered =
      static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_moment_gravity));
  const vpMomentObject &momentObject = moment->getObject();

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");

  double mu11 = momentCentered.get(1, 1);
  double mu20 = momentCentered.get(2, 0);
  double mu02 = momentCentered.get(0, 2);
  double mu12 = momentCentered.get(1, 2);
  double mu21 = momentCentered.get(2, 1);
  double mu03 = momentCentered.get(0, 3);
  double mu30 = momentCentered.get(3, 0);

  double Xg = momentGravity.getXg();
  double Yg = momentGravity.getYg();

  double Avx, Avy, Avz, Awx, Awy;
  double beta, gamma;

  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    beta = 4;
    gamma = 2;
  } else {
    beta = 5;
    gamma = 1;
  }

  double d = (mu20 - mu02) * (mu20 - mu02) + 4 * mu11 * mu11;
  double DA = mu20 + mu02;
  double DA_2 = DA * DA;
  double mu11_2 = mu11 * mu11;

  Avx = mu11 * DA * A / d + (DA * mu02 + (0.5) * d - (0.5) * DA_2) * B / d;
  Avy = (DA * mu02 - (0.5) * d - (.5) * DA_2) * A / d - B * mu11 * DA / d;

  Awx = (beta * (mu12 * (mu20 - mu02) + mu11 * (mu03 - mu21)) + gamma * Xg * (mu02 * (mu20 - mu02) - 2 * mu11_2) +
         gamma * Yg * mu11 * (mu20 + mu02)) /
        d;
  Awy = (beta * (mu21 * (mu02 - mu20) + mu11 * (mu30 - mu12)) + gamma * Xg * mu11 * (mu20 + mu02) +
         gamma * Yg * (mu20 * (mu02 - mu20) - 2 * mu11_2)) /
        d;

  Avz = B * Awx - A * Awy;
  interaction_matrices.resize(1);
  interaction_matrices[0].resize(1, 6);

  int VX = 0;
  int VY = 1;
  int VZ = 2;
  int WX = 3;
  int WY = 4;
  int WZ = 5;

  interaction_matrices[0][0][VX] = Avx;
  interaction_matrices[0][0][VY] = Avy;
  interaction_matrices[0][0][VZ] = Avz;

  interaction_matrices[0][0][WX] = Awx;
  interaction_matrices[0][0][WY] = Awy;
  interaction_matrices[0][0][WZ] = -1.;
}

vpColVector vpFeatureMomentAlpha::error(const vpBasicFeature &s_star, const unsigned int /* select */)
{
  vpColVector e(0);
  double err = s[0] - s_star[0];

  if (err < -M_PI)
    err += 2 * M_PI;
  if (err > M_PI)
    err -= 2 * M_PI;

  vpColVector ecv(1);
  ecv[0] = err;
  e = vpColVector::stack(e, ecv);

  return e;
}
#endif
