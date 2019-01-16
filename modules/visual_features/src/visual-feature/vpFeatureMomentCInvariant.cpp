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
#include <visp3/core/vpMomentObject.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCInvariant.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>

#include <visp3/visual_features/vpFeatureMomentDatabase.h>

#include <limits>
#include <vector>

/*!
  Computes interaction matrix for space-scale-rotation invariants. Called
  internally. The moment primitives must be computed before calling this. This
  feature depends on:
  - vpMomentCentered
  - vpFeatureMomentCentered
  - vpMomentCInvariant
  - vpFeatureMomentBasic
*/
void vpFeatureMomentCInvariant::compute_interaction()
{
  std::vector<vpMatrix> LI(16);
  bool found_moment_centered;
  bool found_moment_cinvariant;
  bool found_FeatureMoment_centered;
  bool found_featuremoment_basic;

  const vpMomentObject &momentObject = moment->getObject();
  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered)));
  const vpMomentCInvariant &momentCInvariant =
      (static_cast<const vpMomentCInvariant &>(moments.get("vpMomentCInvariant", found_moment_cinvariant)));
  vpFeatureMomentCentered &featureMomentCentered = (static_cast<vpFeatureMomentCentered &>(
      featureMomentsDataBase->get("vpFeatureMomentCentered", found_FeatureMoment_centered)));

  vpFeatureMomentBasic &featureMomentBasic = (static_cast<vpFeatureMomentBasic &>(
      featureMomentsDataBase->get("vpFeatureMomentBasic", found_featuremoment_basic)));

  if (!found_featuremoment_basic)
    throw vpException(vpException::notInitialized, "vpFeatureMomentBasic not found");

  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_cinvariant)
    throw vpException(vpException::notInitialized, "vpMomentCInvariant not found");
  if (!found_FeatureMoment_centered)
    throw vpException(vpException::notInitialized, "vpFeatureMomentCentered not found");

  vpMatrix zeros(1, 6);
  for (int i = 0; i < 6; i++)
    zeros[0][i] = 0;

  LI[1] = -featureMomentCentered.interaction(2, 0) * momentCentered.get(0, 2) -
          momentCentered.get(2, 0) * featureMomentCentered.interaction(0, 2) +
          2 * momentCentered.get(1, 1) * featureMomentCentered.interaction(1, 1);

  LI[2] = 2 * (momentCentered.get(2, 0) - momentCentered.get(0, 2)) *
              (featureMomentCentered.interaction(2, 0) - featureMomentCentered.interaction(0, 2)) +
          8 * momentCentered.get(1, 1) * featureMomentCentered.interaction(1, 1);

  LI[3] = 2 * (momentCentered.get(3, 0) - 3 * momentCentered.get(1, 2)) *
              (featureMomentCentered.interaction(3, 0) - 3 * featureMomentCentered.interaction(1, 2)) +
          2 * (3 * momentCentered.get(2, 1) - momentCentered.get(0, 3)) *
              (3 * featureMomentCentered.interaction(2, 1) - featureMomentCentered.interaction(0, 3));

  LI[4] = 2 * (momentCentered.get(3, 0) + momentCentered.get(1, 2)) *
              (featureMomentCentered.interaction(3, 0) + featureMomentCentered.interaction(1, 2)) +
          2 * (momentCentered.get(2, 1) + momentCentered.get(0, 3)) *
              (featureMomentCentered.interaction(2, 1) + featureMomentCentered.interaction(0, 3));

  LI[5] = -2 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(3, 0) +
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(3, 0) -
          4 * pow(momentCentered.get(1, 2), 3) * featureMomentCentered.interaction(3, 0) +
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(2, 1) -
          12 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(2, 1) +
          6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(2, 1) +
          6 * momentCentered.get(0, 3) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(1, 2) -
          12 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(1, 2) +
          6 * momentCentered.get(1, 2) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(1, 2) -
          2 * momentCentered.get(0, 3) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(0, 3) +
          6 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(0, 3) -
          4 * pow(momentCentered.get(2, 1), 3) * featureMomentCentered.interaction(0, 3);

  LI[6] = 6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(3, 0) +
          4 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(3, 0) -
          6 * momentCentered.get(1, 2) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(3, 0) -
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(3, 0) +
          2 * pow(momentCentered.get(1, 2), 3) * featureMomentCentered.interaction(3, 0) -
          12 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(2, 1) -
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(2, 1) +
          12 * pow(momentCentered.get(2, 1), 3) * featureMomentCentered.interaction(2, 1) +
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(2, 1) +
          6 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(2, 1) -
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(1, 2), 2) * featureMomentCentered.interaction(2, 1) +
          6 * momentCentered.get(1, 2) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(1, 2) -
          6 * pow(momentCentered.get(2, 1), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(1, 2) -
          6 * momentCentered.get(0, 3) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(1, 2) +
          6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(1, 2) -
          12 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(1, 2) +
          12 * pow(momentCentered.get(1, 2), 3) * featureMomentCentered.interaction(1, 2) +
          4 * momentCentered.get(0, 3) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(0, 3) -
          6 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(0, 3) +
          2 * pow(momentCentered.get(2, 1), 3) * featureMomentCentered.interaction(0, 3) +
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(0, 3) -
          6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(0, 3);

  LI[7] = -3 * momentCentered.get(0, 3) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(3, 0) +
          6 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(3, 0) -
          2 * pow(momentCentered.get(2, 1), 3) * featureMomentCentered.interaction(3, 0) -
          3 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(3, 0) +
          6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(3, 0) +
          3 * momentCentered.get(0, 3) * pow(momentCentered.get(1, 2), 2) * featureMomentCentered.interaction(3, 0) +
          pow(momentCentered.get(0, 3), 3) * featureMomentCentered.interaction(3, 0) +
          3 * momentCentered.get(1, 2) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(2, 1) -
          6 * pow(momentCentered.get(2, 1), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(2, 1) -
          6 * momentCentered.get(0, 3) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(2, 1) +
          6 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(2, 1) -
          9 * momentCentered.get(1, 2) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(2, 1) -
          12 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(2, 1) +
          3 * pow(momentCentered.get(1, 2), 3) * featureMomentCentered.interaction(2, 1) -
          3 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(1, 2) * featureMomentCentered.interaction(2, 1) +
          3 * momentCentered.get(2, 1) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(1, 2) +
          12 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(1, 2) +
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(1, 2) -
          3 * pow(momentCentered.get(2, 1), 3) * featureMomentCentered.interaction(1, 2) -
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(1, 2) +
          9 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(1, 2) -
          3 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(1, 2) +
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(1, 2), 2) * featureMomentCentered.interaction(1, 2) -
          pow(momentCentered.get(3, 0), 3) * featureMomentCentered.interaction(0, 3) -
          3 * pow(momentCentered.get(2, 1), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(0, 3) +
          3 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(0, 3) +
          3 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(0, 3) -
          6 * momentCentered.get(1, 2) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(0, 3) -
          6 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(0, 3) +
          2 * pow(momentCentered.get(1, 2), 3) * featureMomentCentered.interaction(0, 3);

  LI[8] = -2 * momentCentered.get(3, 0) * momentCentered.get(2, 1) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(3, 0) +
          6 * momentCentered.get(3, 0) * momentCentered.get(2, 1) * momentCentered.get(1, 2) *
              featureMomentCentered.interaction(2, 1) -
          6 * featureMomentCentered.interaction(3, 0) * momentCentered.get(2, 1) * momentCentered.get(0, 3) *
              momentCentered.get(1, 2) -
          6 * momentCentered.get(3, 0) * featureMomentCentered.interaction(2, 1) * momentCentered.get(0, 3) *
              momentCentered.get(1, 2) -
          6 * momentCentered.get(3, 0) * momentCentered.get(2, 1) * featureMomentCentered.interaction(0, 3) *
              momentCentered.get(1, 2) -
          6 * momentCentered.get(3, 0) * momentCentered.get(2, 1) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(1, 2) -
          2 * momentCentered.get(3, 0) * momentCentered.get(1, 2) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(0, 3) +
          6 * momentCentered.get(2, 1) * momentCentered.get(1, 2) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(1, 2) -
          pow((double)momentCentered.get(3, 0), (double)3) * featureMomentCentered.interaction(1, 2) +
          3 * featureMomentCentered.interaction(3, 0) * pow((double)momentCentered.get(1, 2), (double)3) +
          6 * pow((double)momentCentered.get(2, 1), (double)3) * featureMomentCentered.interaction(0, 3) -
          featureMomentCentered.interaction(2, 1) * pow((double)momentCentered.get(0, 3), (double)3) +
          3 * featureMomentCentered.interaction(2, 1) * pow((double)momentCentered.get(1, 2), (double)2) *
              momentCentered.get(0, 3) +
          18 * pow((double)momentCentered.get(2, 1), (double)2) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(2, 1) -
          pow((double)momentCentered.get(3, 0), (double)2) * featureMomentCentered.interaction(2, 1) *
              momentCentered.get(0, 3) +
          9 * momentCentered.get(3, 0) * pow((double)momentCentered.get(1, 2), (double)2) *
              featureMomentCentered.interaction(1, 2) -
          4 * pow((double)momentCentered.get(3, 0), (double)2) * momentCentered.get(1, 2) *
              featureMomentCentered.interaction(1, 2) +
          2 * pow((double)momentCentered.get(1, 2), (double)2) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(0, 3) -
          4 * momentCentered.get(3, 0) * pow((double)momentCentered.get(1, 2), (double)2) *
              featureMomentCentered.interaction(3, 0) +
          2 * momentCentered.get(1, 2) * pow((double)momentCentered.get(0, 3), (double)2) *
              featureMomentCentered.interaction(1, 2) -
          4 * momentCentered.get(2, 1) * pow((double)momentCentered.get(0, 3), (double)2) *
              featureMomentCentered.interaction(2, 1) +
          3 * momentCentered.get(3, 0) * pow((double)momentCentered.get(2, 1), (double)2) *
              featureMomentCentered.interaction(1, 2) -
          3 * pow((double)momentCentered.get(3, 0), (double)2) * momentCentered.get(1, 2) *
              featureMomentCentered.interaction(3, 0) -
          momentCentered.get(3, 0) * featureMomentCentered.interaction(1, 2) *
              pow((double)momentCentered.get(0, 3), (double)2) -
          4 * pow((double)momentCentered.get(2, 1), (double)2) * momentCentered.get(0, 3) *
              featureMomentCentered.interaction(0, 3) -
          3 * momentCentered.get(2, 1) * pow((double)momentCentered.get(0, 3), (double)2) *
              featureMomentCentered.interaction(0, 3) +
          2 * momentCentered.get(3, 0) * pow((double)momentCentered.get(2, 1), (double)2) *
              featureMomentCentered.interaction(3, 0) +
          2 * pow((double)momentCentered.get(3, 0), (double)2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(2, 1) +
          3 * featureMomentCentered.interaction(3, 0) * pow((double)momentCentered.get(2, 1), (double)2) *
              momentCentered.get(1, 2) -
          pow((double)momentCentered.get(3, 0), (double)2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(0, 3) +
          3 * momentCentered.get(2, 1) * pow((double)momentCentered.get(1, 2), (double)2) *
              featureMomentCentered.interaction(0, 3) -
          featureMomentCentered.interaction(3, 0) * momentCentered.get(1, 2) *
              pow((double)momentCentered.get(0, 3), (double)2);

  LI[9] = 4 * pow(momentCentered.get(3, 0), 3) * featureMomentCentered.interaction(3, 0) +
          18 * momentCentered.get(1, 2) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(3, 0) +
          12 * momentCentered.get(0, 3) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(3, 0) +
          18 * pow(momentCentered.get(1, 2), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(3, 0) +
          4 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(3, 0) +
          18 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(2, 1) *
              featureMomentCentered.interaction(3, 0) +
          6 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(1, 2) * featureMomentCentered.interaction(3, 0) +
          6 * momentCentered.get(0, 3) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(2, 1) +
          18 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(2, 1) +
          18 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(2, 1) +
          6 * pow(momentCentered.get(0, 3), 3) * featureMomentCentered.interaction(2, 1) +
          6 * pow(momentCentered.get(3, 0), 3) * featureMomentCentered.interaction(1, 2) +
          18 * momentCentered.get(1, 2) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(1, 2) +
          18 * momentCentered.get(0, 3) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(1, 2) +
          6 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(3, 0) * featureMomentCentered.interaction(1, 2) +
          6 * momentCentered.get(2, 1) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(0, 3) +
          4 * momentCentered.get(0, 3) * pow(momentCentered.get(3, 0), 2) * featureMomentCentered.interaction(0, 3) +
          18 * momentCentered.get(1, 2) * momentCentered.get(2, 1) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(0, 3) +
          12 * momentCentered.get(0, 3) * momentCentered.get(1, 2) * momentCentered.get(3, 0) *
              featureMomentCentered.interaction(0, 3) +
          18 * momentCentered.get(0, 3) * pow(momentCentered.get(2, 1), 2) * featureMomentCentered.interaction(0, 3) +
          18 * pow(momentCentered.get(0, 3), 2) * momentCentered.get(2, 1) * featureMomentCentered.interaction(0, 3) +
          4 * pow(momentCentered.get(0, 3), 3) * featureMomentCentered.interaction(0, 3);

  LI[10] = featureMomentCentered.interaction(4, 0) * momentCentered.get(0, 4) +
           momentCentered.get(4, 0) * featureMomentCentered.interaction(0, 4) -
           4 * featureMomentCentered.interaction(3, 1) * momentCentered.get(1, 3) -
           4 * momentCentered.get(3, 1) * featureMomentCentered.interaction(1, 3) +
           6 * momentCentered.get(2, 2) * featureMomentCentered.interaction(2, 2);

  LI[11] = -3 * featureMomentCentered.interaction(4, 0) * momentCentered.get(2, 2) -
           3 * momentCentered.get(4, 0) * featureMomentCentered.interaction(2, 2) -
           2 * featureMomentCentered.interaction(4, 0) * momentCentered.get(0, 4) -
           2 * momentCentered.get(4, 0) * featureMomentCentered.interaction(0, 4) +
           6 * momentCentered.get(3, 1) * featureMomentCentered.interaction(3, 1) +
           2 * featureMomentCentered.interaction(3, 1) * momentCentered.get(1, 3) +
           2 * momentCentered.get(3, 1) * featureMomentCentered.interaction(1, 3) -
           3 * featureMomentCentered.interaction(2, 2) * momentCentered.get(0, 4) -
           3 * momentCentered.get(2, 2) * featureMomentCentered.interaction(0, 4) +
           6 * momentCentered.get(1, 3) * featureMomentCentered.interaction(1, 3);

  LI[12] = 6 * momentCentered.get(4, 0) * featureMomentCentered.interaction(4, 0) +
           12 * featureMomentCentered.interaction(4, 0) * momentCentered.get(2, 2) +
           12 * momentCentered.get(4, 0) * featureMomentCentered.interaction(2, 2) +
           2 * featureMomentCentered.interaction(4, 0) * momentCentered.get(0, 4) +
           2 * momentCentered.get(4, 0) * featureMomentCentered.interaction(0, 4) +
           16 * featureMomentCentered.interaction(3, 1) * momentCentered.get(1, 3) +
           16 * momentCentered.get(3, 1) * featureMomentCentered.interaction(1, 3) +
           12 * featureMomentCentered.interaction(2, 2) * momentCentered.get(0, 4) +
           12 * momentCentered.get(2, 2) * featureMomentCentered.interaction(0, 4) +
           6 * momentCentered.get(0, 4) * featureMomentCentered.interaction(0, 4);

  LI[13] = 2 * (momentCentered.get(5, 0) + 2 * momentCentered.get(3, 2) + momentCentered.get(1, 4)) *
               (featureMomentCentered.interaction(5, 0) + 2 * featureMomentCentered.interaction(3, 2) +
                featureMomentCentered.interaction(1, 4)) +
           2 * (momentCentered.get(0, 5) + 2 * momentCentered.get(2, 3) + momentCentered.get(4, 1)) *
               (featureMomentCentered.interaction(0, 5) + 2 * featureMomentCentered.interaction(2, 3) +
                featureMomentCentered.interaction(4, 1));

  LI[14] = 2 * (momentCentered.get(5, 0) - 2 * momentCentered.get(3, 2) - 3 * momentCentered.get(1, 4)) *
               (featureMomentCentered.interaction(5, 0) - 2 * featureMomentCentered.interaction(3, 2) -
                3 * featureMomentCentered.interaction(1, 4)) +
           2 * (momentCentered.get(0, 5) - 2 * momentCentered.get(2, 3) - 3 * momentCentered.get(4, 1)) *
               (featureMomentCentered.interaction(0, 5) - 2 * featureMomentCentered.interaction(2, 3) -
                3 * featureMomentCentered.interaction(4, 1));

  LI[15] = 2 * (momentCentered.get(5, 0) - 10 * momentCentered.get(3, 2) + 5 * momentCentered.get(1, 4)) *
               (featureMomentCentered.interaction(5, 0) - 10 * featureMomentCentered.interaction(3, 2) +
                5 * featureMomentCentered.interaction(1, 4)) +
           2 * (momentCentered.get(0, 5) - 10 * momentCentered.get(2, 3) + 5 * momentCentered.get(4, 1)) *
               (featureMomentCentered.interaction(0, 5) - 10 * featureMomentCentered.interaction(2, 3) +
                5 * featureMomentCentered.interaction(4, 1));

  double s3 = momentCInvariant.getS(3);
  double s2 = momentCInvariant.getS(2);
  double c3 = momentCInvariant.getC(3);
  double c2 = momentCInvariant.getC(2);
  double I1 = momentCInvariant.getII(1);
  double I2 = momentCInvariant.getII(2);
  double I3 = momentCInvariant.getII(3);

  vpMatrix Lc2 = featureMomentCentered.interaction(0, 3) - 3 * featureMomentCentered.interaction(2, 1);
  vpMatrix Ls2 = featureMomentCentered.interaction(3, 0) - 3 * featureMomentCentered.interaction(1, 2);
  vpMatrix Lc3 = 2 * (momentCentered.get(2, 0) - momentCentered.get(0, 2)) *
                     (featureMomentCentered.interaction(2, 0) - featureMomentCentered.interaction(0, 2)) -
                 8 * momentCentered.get(1, 1) * featureMomentCentered.interaction(1, 1);
  vpMatrix Ls3 = 4 * featureMomentCentered.interaction(1, 1) * (momentCentered.get(2, 0) - momentCentered.get(0, 2)) +
                 4 * momentCentered.get(1, 1) *
                     (featureMomentCentered.interaction(2, 0) - featureMomentCentered.interaction(0, 2));
  vpMatrix LI1 = 2 * (momentCentered.get(2, 0) - momentCentered.get(0, 2)) *
                     (featureMomentCentered.interaction(2, 0) - featureMomentCentered.interaction(0, 2)) +
                 8 * momentCentered.get(1, 1) * featureMomentCentered.interaction(1, 1);
  vpMatrix LI2 = 2 * (momentCentered.get(0, 3) - 3 * momentCentered.get(2, 1)) *
                     (featureMomentCentered.interaction(0, 3) - 3 * featureMomentCentered.interaction(2, 1)) +
                 2 * (momentCentered.get(3, 0) - 3 * momentCentered.get(1, 2)) *
                     (featureMomentCentered.interaction(3, 0) - 3 * featureMomentCentered.interaction(1, 2));
  vpMatrix LI3 = featureMomentCentered.interaction(2, 0) + featureMomentCentered.interaction(0, 2);

  vpMatrix La(1, 6);
  double a;
  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
    La = (featureMomentCentered.interaction(2, 0) + featureMomentCentered.interaction(0, 2));
  } else {
    a = momentObject.get(0, 0);
    La = featureMomentBasic.interaction(0, 0);
  }
  interaction_matrices.resize(14);

  interaction_matrices[0] = (1. / (momentCInvariant.getI(2) * momentCInvariant.getI(2))) *
                            (momentCInvariant.getI(2) * LI[1] - momentCInvariant.getI(1) * LI[2]);
  interaction_matrices[1] = (1. / (momentCInvariant.getI(4) * momentCInvariant.getI(4))) *
                            (momentCInvariant.getI(4) * LI[3] - momentCInvariant.getI(3) * LI[4]);

  interaction_matrices[2] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[5] - momentCInvariant.getI(5) * LI[6]);

  interaction_matrices[3] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[7] - momentCInvariant.getI(7) * LI[6]);

  interaction_matrices[4] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[8] - momentCInvariant.getI(8) * LI[6]);

  interaction_matrices[5] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[9] - momentCInvariant.getI(9) * LI[6]);

  interaction_matrices[6] = (1. / (momentCInvariant.getI(10) * momentCInvariant.getI(10))) *
                            (momentCInvariant.getI(10) * LI[11] - momentCInvariant.getI(11) * LI[10]);

  interaction_matrices[7] = (1. / (momentCInvariant.getI(10) * momentCInvariant.getI(10))) *
                            (momentCInvariant.getI(10) * LI[12] - momentCInvariant.getI(12) * LI[10]);

  interaction_matrices[8] = (1. / (momentCInvariant.getI(15) * momentCInvariant.getI(15))) *
                            (momentCInvariant.getI(15) * LI[13] - momentCInvariant.getI(13) * LI[15]);

  interaction_matrices[9] = (1. / (momentCInvariant.getI(15) * momentCInvariant.getI(15))) *
                            (momentCInvariant.getI(15) * LI[14] - momentCInvariant.getI(14) * LI[15]);

  interaction_matrices[10] = (Lc2 * c3 + c2 * Lc3 + Ls2 * s3 + s2 * Ls3) * sqrt(a) / I1 * pow(I3, -0.3e1 / 0.2e1) +
                             (c2 * c3 + s2 * s3) * pow(a, -0.1e1 / 0.2e1) / I1 * pow(I3, -0.3e1 / 0.2e1) * La / 0.2e1 -
                             (c2 * c3 + s2 * s3) * sqrt(a) * pow(I1, -0.2e1) * pow(I3, -0.3e1 / 0.2e1) * LI1 -
                             0.3e1 / 0.2e1 * (c2 * c3 + s2 * s3) * sqrt(a) / I1 * pow(I3, -0.5e1 / 0.2e1) * LI3;

  interaction_matrices[11] = (Ls2 * c3 + s2 * Lc3 - Lc2 * s3 - c2 * Ls3) * sqrt(a) / I1 * pow(I3, -0.3e1 / 0.2e1) +
                             (s2 * c3 - c2 * s3) * pow(a, -0.1e1 / 0.2e1) / I1 * pow(I3, -0.3e1 / 0.2e1) * La / 0.2e1 -
                             (s2 * c3 - c2 * s3) * sqrt(a) * pow(I1, -0.2e1) * pow(I3, -0.3e1 / 0.2e1) * LI1 -
                             0.3e1 / 0.2e1 * (s2 * c3 - c2 * s3) * sqrt(a) / I1 * pow(I3, -0.5e1 / 0.2e1) * LI3;

  interaction_matrices[12] = (1 / (I3 * I3)) * LI1 - (2 * I1 / (I3 * I3 * I3)) * LI3;
  interaction_matrices[13] =
      (I2 / (I3 * I3 * I3)) * La + (a / (I3 * I3 * I3)) * LI2 - (3 * a * I2 / (I3 * I3 * I3 * I3)) * LI3;
}

#else
#include <visp3/core/vpMomentCInvariant.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/visual_features/vpFeatureMomentBasic.h>
#include <visp3/visual_features/vpFeatureMomentCInvariant.h>
#include <visp3/visual_features/vpFeatureMomentCentered.h>

#include <visp3/visual_features/vpFeatureMomentDatabase.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

/*!
  Computes interaction matrix for space-scale-rotation invariants. Called
  internally. The moment primitives must be computed before calling this. This
  feature depends on:
  - vpMomentCentered
  - vpFeatureMomentCentered
  - vpMomentCInvariant
  - vpFeatureMomentBasic
*/
void vpFeatureMomentCInvariant::compute_interaction()
{

  // std::vector<vpMatrix> LI(16);
  LI.resize(16); // LI made class member

  bool found_moment_centered;
  bool found_moment_cinvariant;
  bool found_FeatureMoment_centered;
  bool found_featuremoment_basic;

  const vpMomentObject &momentObject = moment->getObject();
  const vpMomentCentered &momentCentered =
      (static_cast<const vpMomentCentered &>(moments.get("vpMomentCentered", found_moment_centered)));
  const vpMomentCInvariant &momentCInvariant =
      (static_cast<const vpMomentCInvariant &>(moments.get("vpMomentCInvariant", found_moment_cinvariant)));

  vpFeatureMomentCentered &featureMomentCentered = (static_cast<vpFeatureMomentCentered &>(
      featureMomentsDataBase->get("vpFeatureMomentCentered", found_FeatureMoment_centered)));

  vpFeatureMomentBasic &featureMomentBasic = (static_cast<vpFeatureMomentBasic &>(
      featureMomentsDataBase->get("vpFeatureMomentBasic", found_featuremoment_basic)));

  if (!found_featuremoment_basic)
    throw vpException(vpException::notInitialized, "vpFeatureMomentBasic not found");
  if (!found_moment_centered)
    throw vpException(vpException::notInitialized, "vpMomentCentered not found");
  if (!found_moment_cinvariant)
    throw vpException(vpException::notInitialized, "vpMomentCInvariant not found");
  if (!found_FeatureMoment_centered)
    throw vpException(vpException::notInitialized, "vpFeatureMomentCentered not found");

  vpMatrix zeros(1, 6);
  for (int i = 0; i < 6; i++)
    zeros[0][i] = 0;

  double mu30 = momentCentered.get(3, 0);
  double mu30_2 = mu30 * mu30;
  double mu30_3 = mu30_2 * mu30;

  double mu03 = momentCentered.get(0, 3);
  double mu03_2 = mu03 * mu03;
  double mu03_3 = mu03 * mu03_2;

  double mu20 = momentCentered.get(2, 0);
  double mu02 = momentCentered.get(0, 2);
  double mu50 = momentCentered.get(5, 0);
  double mu32 = momentCentered.get(3, 2);
  double mu14 = momentCentered.get(1, 4);
  double mu05 = momentCentered.get(0, 5);
  double mu23 = momentCentered.get(2, 3);
  double mu41 = momentCentered.get(4, 1);
  double mu40 = momentCentered.get(4, 0);
  double mu04 = momentCentered.get(0, 4);
  double mu31 = momentCentered.get(3, 1);
  double mu13 = momentCentered.get(1, 3);
  double mu22 = momentCentered.get(2, 2);
  double mu21 = momentCentered.get(2, 1);
  double mu12 = momentCentered.get(1, 2);
  double mu11 = momentCentered.get(1, 1);

  double mu12_2 = mu12 * mu12;
  double mu21_2 = mu21 * mu21;
  double mu21_3 = mu21 * mu21_2;
  double mu12_3 = mu12_2 * mu12;

  vpMatrix Lmu20 = featureMomentCentered.interaction(2, 0);
  vpMatrix Lmu02 = featureMomentCentered.interaction(0, 2);
  vpMatrix Lmu11 = featureMomentCentered.interaction(1, 1);
  vpMatrix Lmu12 = featureMomentCentered.interaction(1, 2);
  vpMatrix Lmu03 = featureMomentCentered.interaction(0, 3);
  vpMatrix Lmu21 = featureMomentCentered.interaction(2, 1);
  vpMatrix Lmu30 = featureMomentCentered.interaction(3, 0);
  vpMatrix Lmu40 = featureMomentCentered.interaction(4, 0);
  vpMatrix Lmu04 = featureMomentCentered.interaction(0, 4);
  vpMatrix Lmu31 = featureMomentCentered.interaction(3, 1);
  vpMatrix Lmu13 = featureMomentCentered.interaction(1, 3);
  vpMatrix Lmu22 = featureMomentCentered.interaction(2, 2);
  vpMatrix Lmu05 = featureMomentCentered.interaction(0, 5);
  vpMatrix Lmu14 = featureMomentCentered.interaction(1, 4);
  vpMatrix Lmu23 = featureMomentCentered.interaction(2, 3);
  vpMatrix Lmu32 = featureMomentCentered.interaction(3, 2);
  vpMatrix Lmu41 = featureMomentCentered.interaction(4, 1);
  vpMatrix Lmu50 = featureMomentCentered.interaction(5, 0);

  LI[1] = -Lmu20 * mu02 - mu20 * Lmu02 + 2 * mu11 * Lmu11;

  LI[2] = (-2 * mu20 + 2 * mu02) * Lmu02 + 8 * mu11 * Lmu11 + (2 * mu20 - 2 * mu02) * Lmu20;

  LI[3] = (-6 * mu21 + 2 * mu03) * Lmu03 + (-6 * mu30 + 18 * mu12) * Lmu12 + (18 * mu21 - 6 * mu03) * Lmu21 +
          (2 * mu30 - 6 * mu12) * Lmu30;

  LI[4] = (2 * mu21 + 2 * mu03) * Lmu03 + (2 * mu30 + 2 * mu12) * Lmu12 + (2 * mu21 + 2 * mu03) * Lmu21 +
          (2 * mu30 + 2 * mu12) * Lmu30;

  LI[5] = (-2 * mu30_2 * mu03 + 6 * mu30 * mu21 * mu12 - 4 * mu21_3) * Lmu03 +
          (6 * mu30 * mu21 * mu03 - 12 * mu30 * mu12_2 + 6 * mu21_2 * mu12) * Lmu12 +
          (6 * mu30 * mu12 * mu03 - 12 * mu21_2 * mu03 + 6 * mu21 * mu12_2) * Lmu21 +
          (-2 * mu30 * mu03_2 - 4 * mu12_3 + 6 * mu21 * mu12 * mu03) * Lmu30;

  LI[6] = (-6 * mu30 * mu21 * mu12 - 6 * mu21 * mu12_2 + 6 * mu21_2 * mu03 + 2 * mu21_3 + 4 * mu30_2 * mu03) * Lmu03 +
          (-6 * mu30 * mu21_2 - 6 * mu30 * mu21 * mu03 + 12 * mu12_3 + 6 * mu30_2 * mu12 - 12 * mu21 * mu12 * mu03 +
           6 * mu30 * mu12_2) *
              Lmu12 +
          (6 * mu21 * mu03_2 + 6 * mu21_2 * mu03 - 6 * mu30 * mu12 * mu03 + 12 * mu21_3 - 12 * mu30 * mu21 * mu12 -
           6 * mu12_2 * mu03) *
              Lmu21 +
          (6 * mu30 * mu12_2 + 2 * mu12_3 + 4 * mu30 * mu03_2 - 6 * mu21_2 * mu12 - 6 * mu21 * mu12 * mu03) * Lmu30;

  LI[7] = (-6 * mu21_2 * mu12 + 3 * mu30 * mu03_2 - mu30_3 - 3 * mu30 * mu21_2 - 6 * mu21 * mu12 * mu03 +
           3 * mu30 * mu12_2 + 2 * mu12_3) *
              Lmu03 +
          (-3 * mu21 * mu03_2 + 12 * mu30 * mu21 * mu12 + 6 * mu30 * mu12 * mu03 + 3 * mu30_2 * mu21 +
           9 * mu21 * mu12_2 - 6 * mu21_2 * mu03 - 3 * mu21_3 + 6 * mu12_2 * mu03) *
              Lmu12 +
          (3 * mu30_2 * mu12 - 9 * mu21_2 * mu12 - 12 * mu21 * mu12 * mu03 - 6 * mu30 * mu21 * mu03 -
           6 * mu30 * mu21_2 + 6 * mu30 * mu12_2 + 3 * mu12_3 - 3 * mu12 * mu03_2) *
              Lmu21 +
          (6 * mu21 * mu12_2 + 6 * mu30 * mu21 * mu12 - 3 * mu30_2 * mu03 + 3 * mu12_2 * mu03 - 3 * mu21_2 * mu03 -
           2 * mu21_3 + mu03_3) *
              Lmu30;

  LI[8] = (6 * mu21_3 - 2 * mu30 * mu12 * mu03 + 2 * mu12_2 * mu03 + 3 * mu21 * mu12_2 - 6 * mu30 * mu21 * mu12 -
           mu30_2 * mu21 - 4 * mu21_2 * mu03 - 3 * mu21 * mu03_2) *
              Lmu03 +
          (2 * mu12 * mu03_2 - 4 * mu30_2 * mu12 + 9 * mu30 * mu12_2 - mu30 * mu03_2 - 6 * mu30 * mu21 * mu03 +
           3 * mu30 * mu21_2 + 6 * mu21 * mu12 * mu03 - mu30_3) *
              Lmu12 +
          (18 * mu21_2 * mu03 + 6 * mu30 * mu21 * mu12 - 4 * mu21 * mu03_2 - mu03_3 - mu30_2 * mu03 -
           6 * mu30 * mu12 * mu03 + 3 * mu12_2 * mu03 + 2 * mu30_2 * mu21) *
              Lmu21 +
          (-6 * mu21 * mu12 * mu03 - 4 * mu30 * mu12_2 - 2 * mu30 * mu21 * mu03 + 2 * mu30 * mu21_2 + 3 * mu12_3 +
           3 * mu21_2 * mu12 - 3 * mu30_2 * mu12 - mu12 * mu03_2) *
              Lmu30;

  LI[9] = (2 * (2 * mu03 + 3 * mu21)) * (3 * mu03 * mu21 + 3 * mu30 * mu12 + mu30_2 + mu03_2) * Lmu03 +
          6 * mu30 * (3 * mu03 * mu21 + 3 * mu30 * mu12 + mu30_2 + mu03_2) * Lmu12 +
          6 * mu03 * (3 * mu03 * mu21 + 3 * mu30 * mu12 + mu30_2 + mu03_2) * Lmu21 +
          (2 * (2 * mu30 + 3 * mu12)) * (3 * mu03 * mu21 + 3 * mu30 * mu12 + mu30_2 + mu03_2) * Lmu30;

  LI[10] = Lmu40 * mu04 + mu40 * Lmu04 - 4 * Lmu31 * mu13 - 4 * mu31 * Lmu13 + 6 * mu22 * Lmu22;

  LI[11] = (-2 * mu40 - 3 * mu22) * Lmu04 + (2 * mu31 + 6 * mu13) * Lmu13 + (-3 * mu04 - 3 * mu40) * Lmu22 +
           (2 * mu13 + 6 * mu31) * Lmu31 + (-3 * mu22 - 2 * mu04) * Lmu40;

  LI[12] = (2 * mu40 + 12 * mu22 + 6 * mu04) * Lmu04 + 16 * mu31 * Lmu13 + (12 * mu40 + 12 * mu04) * Lmu22 +
           16 * Lmu31 * mu13 + (6 * mu40 + 12 * mu22 + 2 * mu04) * Lmu40;

  LI[13] = (2 * mu05 + 4 * mu23 + 2 * mu41) * Lmu05 + (2 * mu50 + 4 * mu32 + 2 * mu14) * Lmu14 +
           (4 * mu05 + 8 * mu23 + 4 * mu41) * Lmu23 + (4 * mu50 + 8 * mu32 + 4 * mu14) * Lmu32 +
           (2 * mu05 + 4 * mu23 + 2 * mu41) * Lmu41 + (2 * mu50 + 4 * mu32 + 2 * mu14) * Lmu50;

  LI[14] = (2 * mu05 - 4 * mu23 - 6 * mu41) * Lmu05 + (-6 * mu50 + 12 * mu32 + 18 * mu14) * Lmu14 +
           (-4 * mu05 + 8 * mu23 + 12 * mu41) * Lmu23 + (-4 * mu50 + 8 * mu32 + 12 * mu14) * Lmu32 +
           (-6 * mu05 + 12 * mu23 + 18 * mu41) * Lmu41 + (2 * mu50 - 4 * mu32 - 6 * mu14) * Lmu50;

  LI[15] = (2 * mu05 - 20 * mu23 + 10 * mu41) * Lmu05 + (10 * mu50 - 100 * mu32 + 50 * mu14) * Lmu14 +
           (-20 * mu05 + 200 * mu23 - 100 * mu41) * Lmu23 + (-20 * mu50 + 200 * mu32 - 100 * mu14) * Lmu32 +
           (10 * mu05 - 100 * mu23 + 50 * mu41) * Lmu41 + (2 * mu50 - 20 * mu32 + 10 * mu14) * Lmu50;

  double s3 = momentCInvariant.getS(3);
  double s2 = momentCInvariant.getS(2);
  double c3 = momentCInvariant.getC(3);
  double c2 = momentCInvariant.getC(2);

  double I1 = momentCInvariant.getII(1);
  double I2 = momentCInvariant.getII(2);
  double I3 = momentCInvariant.getII(3);

  vpMatrix Lmu20__Lmu02 = Lmu20 - Lmu02;
  double mu20__mu02 = mu20 - mu02;
  vpMatrix Lc2 = Lmu03 - 3 * Lmu21;
  vpMatrix Ls2 = Lmu30 - 3 * Lmu12;
  vpMatrix Lc3 = 2 * (mu20__mu02) * (Lmu20__Lmu02)-8. * mu11 * Lmu11;
  vpMatrix Ls3 = 4 * Lmu11 * (mu20__mu02) + 4 * mu11 * (Lmu20__Lmu02);
  vpMatrix LI1 = 2 * (mu20__mu02) * (Lmu20__Lmu02) + 8 * mu11 * Lmu11;
  vpMatrix LI2 = 2 * (mu03 - 3 * mu21) * (Lc2) + 2 * (mu30 - 3 * mu12) * (Ls2);
  vpMatrix LI3 = Lmu20 + Lmu02;

  vpMatrix La(1, 6);
  double a;
  if (momentObject.getType() == vpMomentObject::DISCRETE) {
    a = momentCentered.get(2, 0) + momentCentered.get(0, 2);
    La = (featureMomentCentered.interaction(2, 0) + featureMomentCentered.interaction(0, 2));
  } else {
    a = momentObject.get(0, 0);
    La = featureMomentBasic.interaction(0, 0);
  }

  interaction_matrices.resize(14);

  /*
  momentCInvariant.printInvariants(std::cout);
  printLsofInvariants(std::cout);
  */

  interaction_matrices[0] = (1. / (momentCInvariant.getI(2) * momentCInvariant.getI(2))) *
                            (momentCInvariant.getI(2) * LI[1] - momentCInvariant.getI(1) * LI[2]);

  interaction_matrices[1] = (1. / (momentCInvariant.getI(4) * momentCInvariant.getI(4))) *
                            (momentCInvariant.getI(4) * LI[3] - momentCInvariant.getI(3) * LI[4]);

  interaction_matrices[2] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[5] - momentCInvariant.getI(5) * LI[6]);

  interaction_matrices[3] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[7] - momentCInvariant.getI(7) * LI[6]);

  interaction_matrices[4] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[8] - momentCInvariant.getI(8) * LI[6]);

  interaction_matrices[5] = (1. / (momentCInvariant.getI(6) * momentCInvariant.getI(6))) *
                            (momentCInvariant.getI(6) * LI[9] - momentCInvariant.getI(9) * LI[6]);

  interaction_matrices[6] = (1. / (momentCInvariant.getI(10) * momentCInvariant.getI(10))) *
                            (momentCInvariant.getI(10) * LI[11] - momentCInvariant.getI(11) * LI[10]);

  interaction_matrices[7] = (1. / (momentCInvariant.getI(10) * momentCInvariant.getI(10))) *
                            (momentCInvariant.getI(10) * LI[12] - momentCInvariant.getI(12) * LI[10]);

  interaction_matrices[8] = (1. / (momentCInvariant.getI(15) * momentCInvariant.getI(15))) *
                            (momentCInvariant.getI(15) * LI[13] - momentCInvariant.getI(13) * LI[15]);

  interaction_matrices[9] = (1. / (momentCInvariant.getI(15) * momentCInvariant.getI(15))) *
                            (momentCInvariant.getI(15) * LI[14] - momentCInvariant.getI(14) * LI[15]);

  interaction_matrices[10] = (Lc2 * c3 + c2 * Lc3 + Ls2 * s3 + s2 * Ls3) * sqrt(a) / I1 * pow(I3, -0.3e1 / 0.2e1) +
                             (c2 * c3 + s2 * s3) * pow(a, -0.1e1 / 0.2e1) / I1 * pow(I3, -0.3e1 / 0.2e1) * La / 0.2e1 -
                             (c2 * c3 + s2 * s3) * sqrt(a) * pow(I1, -0.2e1) * pow(I3, -0.3e1 / 0.2e1) * LI1 -
                             0.3e1 / 0.2e1 * (c2 * c3 + s2 * s3) * sqrt(a) / I1 * pow(I3, -0.5e1 / 0.2e1) * LI3;

  interaction_matrices[11] = (Ls2 * c3 + s2 * Lc3 - Lc2 * s3 - c2 * Ls3) * sqrt(a) / I1 * pow(I3, -0.3e1 / 0.2e1) +
                             (s2 * c3 - c2 * s3) * pow(a, -0.1e1 / 0.2e1) / I1 * pow(I3, -0.3e1 / 0.2e1) * La / 0.2e1 -
                             (s2 * c3 - c2 * s3) * sqrt(a) * pow(I1, -0.2e1) * pow(I3, -0.3e1 / 0.2e1) * LI1 -
                             0.3e1 / 0.2e1 * (s2 * c3 - c2 * s3) * sqrt(a) / I1 * pow(I3, -0.5e1 / 0.2e1) * LI3;

  interaction_matrices[12] = (1 / (I3 * I3)) * LI1 - (2 * I1 / (I3 * I3 * I3)) * LI3;

  interaction_matrices[13] =
      (I2 / (I3 * I3 * I3)) * La + (a / (I3 * I3 * I3)) * LI2 - (3 * a * I2 / (I3 * I3 * I3 * I3)) * LI3;

  /*
  std::cout << (*this);
  vpTRACE("Done.");
  std::exit(-1);
  */
}

/*!
  Print out all invariants that were computed
  There are 15 of them, as in [Point-based and region based.ITRO05]
  \cite Tahri05z
 */
void vpFeatureMomentCInvariant::printLsofInvariants(std::ostream &os) const
{
  for (unsigned int i = 1; i < 15; ++i) {
    os << "LI[" << i << "] = ";
    LI[i].matlabPrint(os);
    os << std::endl;
  }
}

/*!
  \relates vpFeatureMomentCInvariant
  Print all the interaction matrices of visual features
 */
std::ostream &operator<<(std::ostream &os, const vpFeatureMomentCInvariant &featcinv)
{
  // Print L for c1 .. c10
  for (unsigned int i = 0; i < 10; ++i) {
    os << "L_c[" << i << "] = ";
    featcinv.interaction_matrices[i].matlabPrint(os);
    os << std::endl;
  }

  // sx, sy
  os << "L_sx = ";
  featcinv.interaction_matrices[10].matlabPrint(os);
  os << std::endl;
  os << "L_sy = ";
  featcinv.interaction_matrices[11].matlabPrint(os);
  os << std::endl;
  // Px,Py
  os << "L_Px = ";
  featcinv.interaction_matrices[12].matlabPrint(os);
  os << std::endl;
  os << "L_Py = ";
  featcinv.interaction_matrices[13].matlabPrint(os);
  os << std::endl;

  return os;
}
#endif
