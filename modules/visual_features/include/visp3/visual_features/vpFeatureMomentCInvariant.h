/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
/*!
  \file vpFeatureMomentCInvariant.h
  \brief Implementation of the interaction matrix computation for
  vpMomentCInvariant.
*/
#ifndef __FEATUREMOMENTCINVARIANT_H__
#define __FEATUREMOMENTCINVARIANT_H__
#include <visp3/visual_features/vpFeatureMoment.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
/*!
  \class vpFeatureMomentCInvariant

  \ingroup group_visual_features

  \brief Functionality computation for 2D rotation/translation/scale
  non-symmetric invariant moment feature. Computes the interaction matrix
  associated with vpMomentCInvariant.

  The interaction matrix for the moment feature can be deduced from  \cite
  Tahri05z, equations (9). To do so, one must derive them and obtain a
  combination of interaction matrices by using (1). It allows to compute the
  interaction matrix for \f$ c_i, i \in [1..10] \f$.

  These interaction matrices may be selected afterwards by calling
  vpFeatureMomentCInvariant::interaction(). The selection by the
  vpFeatureMomentCInvariant::selectCi method for \f$ L_{c_i} \f$. For example,
  to select \f$ L_{c_1} \f$ you should input
  vpFeatureMomentCInvariant::selectC1() into ViSP's selector. Special matrices
  for features \f$ S_x \f$ and \f$ S_y \f$ are selected by
  vpFeatureMomentCInvariant::selectSx() and
  vpFeatureMomentCInvariant::selectSy() respectively. Special matrices for
  features \f$ P_x \f$ and \f$ P_y \f$ are selected by
  vpFeatureMomentCInvariant::selectPx() and
  vpFeatureMomentCInvariant::selectPy() respectively.

  These features are often used in moment-based visual servoing to control the
  two out-of-plane rotations.

  Be careful about the nature of your object when selecting the right
  features. Use \f$ L_{S_{x}} \f$ and \f$ L_{S_{y}} \f$ when you're dealing
  with a symmetric object all other features otherwise.

  Minimum vpMomentObject order needed to compute this feature: 6. This is the
  highest ordrer required by classic features.

  This feature depends on:
    - vpMomentCentered
    - vpFeatureMomentCentered
    - vpMomentCInvariant
    - vpFeatureMomentBasic

  An example of how to use vpFeatureMomentCInvariant in a complete visual
  servoing example is given in vpFeatureMomentCommon.

*/
class VISP_EXPORT vpFeatureMomentCInvariant : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param moments : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.

  */
  vpFeatureMomentCInvariant(vpMomentDatabase &moments, double A, double B, double C,
                            vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(moments, A, B, C, featureMoments, 16)
  {
  }
  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentCInvariant"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentCInvariant"; }

  /*!
    Shortcut selector for \f$C_1\f$.
    */
  static unsigned int selectC1() { return 1 << 0; }
  /*!
    Shortcut selector for \f$C_2\f$.
    */
  static unsigned int selectC2() { return 1 << 1; }
  /*!
    Shortcut selector for \f$C_3\f$.
    */
  static unsigned int selectC3() { return 1 << 2; }
  /*!
    Shortcut selector for \f$C_4\f$.
    */
  static unsigned int selectC4() { return 1 << 3; }
  /*!
    Shortcut selector for \f$C_5\f$.
    */
  static unsigned int selectC5() { return 1 << 4; }
  /*!
    Shortcut selector for \f$C_6\f$.
    */
  static unsigned int selectC6() { return 1 << 5; }
  /*!
    Shortcut selector for \f$C_7\f$.
    */
  static unsigned int selectC7() { return 1 << 6; }
  /*!
    Shortcut selector for \f$C_8\f$.
    */
  static unsigned int selectC8() { return 1 << 7; }
  /*!
    Shortcut selector for \f$C_9\f$.
    */
  static unsigned int selectC9() { return 1 << 8; }
  /*!
    Shortcut selector for \f$C_{10}\f$.
    */
  static unsigned int selectC10() { return 1 << 9; }
  /*!
    Shortcut selector for \f$S_x\f$.
    */
  static unsigned int selectSx() { return 1 << 10; }
  /*!
    Shortcut selector for \f$S_y\f$.
    */
  static unsigned int selectSy() { return 1 << 11; }
  /*!
    Shortcut selector for \f$P_x\f$.
    */
  static unsigned int selectPx() { return 1 << 12; }
  /*!
    Shortcut selector for \f$P_y\f$.
    */
  static unsigned int selectPy() { return 1 << 13; }
};

#else
class vpMomentDatabase;

/*!
  \class vpFeatureMomentCInvariant

  \ingroup group_visual_features

  \brief Functionality computation for 2D rotation/translation/scale
  non-symmetric invariant moment feature. Computes the interaction matrix
  associated with vpMomentCInvariant.

  The interaction matrix for the moment feature can be deduced from
  \cite Tahri05z, equations (9). To do so, one must derive them and obtain a
  combination of interaction matrices by using (1). It allows to compute the
  interaction matrix for \f$ c_i, i \in [1..10] \f$.

  These interaction matrices may be selected afterwards by calling
  vpFeatureMomentCInvariant::interaction(). The selection by the
  vpFeatureMomentCInvariant::selectCi method for \f$ L_{c_i} \f$. For example,
  to select \f$ L_{c_1} \f$ you should input
  vpFeatureMomentCInvariant::selectC1() into ViSP's selector. Special matrices
  for features \f$ S_x \f$ and \f$ S_y \f$ are selected by
  vpFeatureMomentCInvariant::selectSx() and
  vpFeatureMomentCInvariant::selectSy() respectively. Special matrices for
  features \f$ P_x \f$ and \f$ P_y \f$ are selected by
  vpFeatureMomentCInvariant::selectPx() and
  vpFeatureMomentCInvariant::selectPy() respectively.

  These features are often used in moment-based visual servoing to control the
  two out-of-plane rotations.

  Be careful about the nature of your object when selecting the right
  features. Use \f$ L_{S_{x}} \f$ and \f$ L_{S_{y}} \f$ when you're dealing
  with a symmetric object all other features otherwise.

  Minimum vpMomentObject order needed to compute this feature: 6. This is the
  highest ordrer required by classic features.

  This feature depends on:
    - vpMomentCentered
    - vpFeatureMomentCentered
    - vpMomentCInvariant
    - vpFeatureMomentBasic

  An example of how to use vpFeatureMomentCInvariant in a complete visual
  servoing example is given in vpFeatureMomentCommon.

*/
class VISP_EXPORT vpFeatureMomentCInvariant : public vpFeatureMoment
{
private:
  std::vector<vpMatrix> LI;

public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param data_base : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B_ : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C_ : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.

  */
  vpFeatureMomentCInvariant(vpMomentDatabase &data_base, double A_, double B_, double C_,
                            vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 16), LI(16)
  {
  }

  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentCInvariant"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentCInvariant"; }

  /*!
    Shortcut selector for \f$C_1\f$.
    */
  static unsigned int selectC1() { return 1 << 0; }
  /*!
    Shortcut selector for \f$C_2\f$.
    */
  static unsigned int selectC2() { return 1 << 1; }
  /*!
    Shortcut selector for \f$C_3\f$.
    */
  static unsigned int selectC3() { return 1 << 2; }
  /*!
    Shortcut selector for \f$C_4\f$.
    */
  static unsigned int selectC4() { return 1 << 3; }
  /*!
    Shortcut selector for \f$C_5\f$.
    */
  static unsigned int selectC5() { return 1 << 4; }
  /*!
    Shortcut selector for \f$C_6\f$.
    */
  static unsigned int selectC6() { return 1 << 5; }
  /*!
    Shortcut selector for \f$C_7\f$.
    */
  static unsigned int selectC7() { return 1 << 6; }
  /*!
    Shortcut selector for \f$C_8\f$.
    */
  static unsigned int selectC8() { return 1 << 7; }
  /*!
    Shortcut selector for \f$C_9\f$.
    */
  static unsigned int selectC9() { return 1 << 8; }
  /*!
    Shortcut selector for \f$C_{10}\f$.
    */
  static unsigned int selectC10() { return 1 << 9; }
  /*!
    Shortcut selector for \f$S_x\f$.
    */
  static unsigned int selectSx() { return 1 << 10; }
  /*!
    Shortcut selector for \f$S_y\f$.
    */
  static unsigned int selectSy() { return 1 << 11; }
  /*!
    Shortcut selector for \f$P_x\f$.
    */
  static unsigned int selectPx() { return 1 << 12; }
  /*!
    Shortcut selector for \f$P_y\f$.
    */
  static unsigned int selectPy() { return 1 << 13; }

  /*!
    Print all the interaction matrices of the moment invariants
   */
  void printLsofInvariants(std::ostream &os) const;

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpFeatureMomentCInvariant &featcinv);
};
#endif
#endif
