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

#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>

/*!
  Computes interaction matrix for the normalized surface moment. Called
  internally. The moment primitives must be computed before calling this. This
  feature depends on:
  - vpMomentCentered
  - vpFeatureMomentCentered
  - vpMomentAreaNormalized
  - vpFeatureMomentBasic
*/
void vpFeatureMomentAreaNormalized::compute_interaction()
{
  bool found_moment_centered;
  bool found_moment_surface_normalized;
  bool found_FeatureMoment_centered;

  bool found_featuremoment_basic;
  vpFeatureMomentBasic &featureMomentBasic = (static_cast<vpFeatureMomentBasic &>(
      featureMomentsDataBase->get("vpFeatureMomentBasic", found_featuremoment_basic)));

  const vpMomentCentered &momentCentered =
      static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered));
  const vpMomentObject &momentObject = moment->getObject();
  const vpMomentAreaNormalized &momentSurfaceNormalized = static_cast<const vpMomentAreaNormalized &>(
      moments.get("vpMomentAreaNormalized", found_moment_surface_normalized));
  vpFeatureMomentCentered &featureMomentCentered = (static_cast<vpFeatureMomentCentered &>(
      featureMomentsDataBase->get("vpFeatureMomentCentered", found_FeatureMoment_centered)));

  if (!found_FeatureMoment_centered)
    throw vpException(vpException::notInitialized, "vpFeatureMomentCentered not found");
  if (!found_moment_surface_normalized)
    throw vpException(vpException::notInitialized, "vpMomentAreaNormalized not found");
  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_featuremoment_basic)
    throw vpException(vpException::notInitialized, "vpFeatureMomentBasic not found");
  interaction_matrices.resize(1);
  interaction_matrices[0].resize(1, 6);
  double normalized_multiplier;
  double a;
  vpMatrix La;
  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
    La = featureMomentCentered.interaction(2, 0) + featureMomentCentered.interaction(0, 2);
  } else {
    a = momentObject.get(0, 0);
    La = featureMomentBasic.interaction(0, 0);
  }

  normalized_multiplier =
      (-momentSurfaceNormalized.getDesiredDepth() / (2 * a)) * sqrt(momentSurfaceNormalized.getDesiredArea() / a);
  interaction_matrices[0] = normalized_multiplier * La;
}

#else

#include <limits>
#include <vector>

#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/visual_features/vpFeatureMomentDatabase.h>

/*!
  Computes interaction matrix for the normalized surface moment. Called
  internally. The moment primitives must be computed before calling this. This
  feature depends on:
  - vpMomentCentered
  - vpMomentAreaNormalized
  - vpMomentGravityCenter
*/
void vpFeatureMomentAreaNormalized::compute_interaction()
{
  bool found_moment_centered;
  bool found_moment_surface_normalized;
  bool found_moment_gravity;

  const vpMomentCentered &momentCentered =
      static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered));
  const vpMomentGravityCenter &momentGravity =
      static_cast<const vpMomentGravityCenter &>(moments.get("vpMomentGravityCenter", found_moment_gravity));
  const vpMomentObject &momentObject = moment->getObject();
  const vpMomentAreaNormalized &momentSurfaceNormalized = static_cast<const vpMomentAreaNormalized &>(
      moments.get("vpMomentAreaNormalized", found_moment_surface_normalized));

  if (!found_moment_surface_normalized)
    throw vpException(vpException::notInitialized, "vpMomentAreaNormalized not found");
  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_gravity)
    throw vpException(vpException::notInitialized, "vpMomentGravityCenter not found");
  interaction_matrices.resize(1);
  interaction_matrices[0].resize(1, 6);
  double n11 = momentCentered.get(1, 1) / momentObject.get(0, 0);
  double n20 = momentCentered.get(2, 0) / momentObject.get(0, 0);
  double n02 = momentCentered.get(0, 2) / momentObject.get(0, 0);
  double Xg = momentGravity.getXg();
  double Yg = momentGravity.getYg();

  double An = momentSurfaceNormalized.get()[0];

  double Xn = An * Xg;
  double Yn = An * Yg;

  double Anvx, Anvy, Anvz, Anwx, Anwy;

  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    double a = momentCentered.get(2, 0) + momentCentered.get(0, 2);

    double e01 = momentCentered.get(0, 1) / a;
    double e10 = momentCentered.get(1, 0) / a;
    double e11 = momentCentered.get(1, 1) / a;
    double e02 = momentCentered.get(0, 2) / a;
    double e20 = momentCentered.get(2, 0) / a;
    double e12 = momentCentered.get(1, 2) / a;
    double e21 = momentCentered.get(2, 1) / a;
    double e03 = momentCentered.get(0, 3) / a;
    double e30 = momentCentered.get(3, 0) / a;

    Anvx = An * A * e20 + An * B * e11;
    Anvy = An * A * e11 + An * B * e02;

    Anwx = (n02 * e01 + n11 * e10 - e03 - e21) * An - Xn * e11 + (-1 - e02) * Yn;
    Anwy = (e12 + e30 - n11 * e01 - n20 * e10) * An + (2 - e02) * Xn + Yn * e11;

    Anvz = -An * C + B * Anwx - A * Anwy;

  } else {
    Anvx = A * An / 2.;
    Anvy = B * An / 2.;
    Anvz = -An * C - (3. / 2.) * A * Xn - (3. / 2.) * B * Yn;

    Anwx = -(3. / 2.) * Yn;
    Anwy = (3. / 2.) * Xn;
  }

  int VX = 0;
  int VY = 1;
  int VZ = 2;
  int WX = 3;
  int WY = 4;
  int WZ = 5;

  interaction_matrices[0][0][VX] = Anvx;
  interaction_matrices[0][0][VY] = Anvy;
  interaction_matrices[0][0][VZ] = Anvz;

  interaction_matrices[0][0][WX] = Anwx;
  interaction_matrices[0][0][WY] = Anwy;
  interaction_matrices[0][0][WZ] = 0.;
}

#endif
