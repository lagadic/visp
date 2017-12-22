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
  \file vpFeatureMomentAreaNormalized.h
  \brief Implementation of the interaction matrix computation for
  vpMomentAreaNormalized.
*/
#ifndef __FEATUREMOMENTAREANORMALIZED_H__
#define __FEATUREMOMENTAREANORMALIZED_H__
#include <visp3/visual_features/vpFeatureMoment.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
class vpMomentDatabase;

/*!
  \class vpFeatureMomentAreaNormalized

  \ingroup group_visual_features

  \brief Functionality computation for normalized surface moment feature.
  Computes the interaction matrix associated with vpMomentAreaNormalized.

  The interaction matrix for the moment feature can be deduced from \cite
  Tahri05z.

  To do so, one must derive it and obtain a combination of interaction
  matrices by using (1). It allows to compute the interaction matrix for \f$
  a_n \f$.

  The interaction matrix computed is single-dimension (no selection possible)
  and can be obtained by calling
  vpFeatureMomentGravityCenterNormalized::interaction.

  This feature is often used in moment-based visual servoing to control the
  depth parameter.

  Minimum vpMomentObject order needed to compute this feature: 1 in dense mode
  and 3 in discrete mode.

  This feature depends on:
    - vpMomentCentered
    - vpFeatureMomentCentered
    - vpMomentAreaNormalized
    - vpFeatureMomentBasic

*/
class VISP_EXPORT vpFeatureMomentAreaNormalized : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param database : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.

  */
  vpFeatureMomentAreaNormalized(vpMomentDatabase &database, double A_, double B_, double C_,
                                vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(database, A_, B_, C_, featureMoments, 1)
  {
  }
  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentAreaNormalized"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentAreaNormalized"; }
};

#else
class vpMomentDatabase;

/*!
  \class vpFeatureMomentAreaNormalized

  \ingroup group_visual_features

  \brief Functionality computation for normalized surface moment feature.
Computes the interaction matrix associated with vpMomentAreaNormalized.

  The interaction matrix for the moment has the following form:
  - In the discrete case:
  \f[
  L_{a_n} =
{
 \left[ \begin {array}{c} a_{{n}}Ae_{{2,0}}+a_{{n}}Be_{{1,1}}
\\ \noalign{\medskip}a_{{n}}Ae_{{1,1}}+a_{{n}}Be_{{0,2}}
\\ \noalign{\medskip}-a_{{n}}C+Bw_{{x}}-Aw_{{y}}\\ \noalign{\medskip}-
 \left( e_{{2,0}}+2\,e_{{0,2}} \right) y_{{g}}-e_{{2,1}}-x_{{g}}e_{{1,
1}}+\eta_{{1,1}}e_{{1,0}}-e_{{0,3}}+\eta_{{0,2}}e_{{0,1}}
\\ \noalign{\medskip} \left( 2\,e_{{2,0}}+e_{{0,2}} \right) x_{{g}}+e_
{{3,0}}+y_{{g}}e_{{1,1}}-\eta_{{2,0}}e_{{1,0}}+e_{{1,2}}-\eta_{{1,1}}e
_{{0,1}}\\ \noalign{\medskip}0\end {array} \right]

}^t

  \f]
  - In the dense case:
  \f[
  L_{a_n} =
{
 \left[ \begin {array}{c} 1/2\,a_{{n}}A\\ \noalign{\medskip}1/2\,a_{{n
}}B\\ \noalign{\medskip}-a_{{n}}C-3/2\,Ax_{{n}}-3/2\,By_{{n}}
\\ \noalign{\medskip}-3/2\,y_{{n}}\\ \noalign{\medskip}3/2\,x_{{n}}
\\ \noalign{\medskip}0\end {array} \right]

}^t

  \f]
with:
    - \f$e_{i,j}=\frac{\mu_{i,j}}{NA}\f$
    - \f$NA=\mu_{2,0}+\mu_{0,2}\f$
    - \f$\eta\f$ is the centered and normalized moment.
  To do so, one must derive it and obtain a combination of interaction
matrices by using (1). It allows to compute the interaction matrix for \f$ a_n
\f$.

  The interaction matrix computed is single-dimension (no selection possible)
and can be obtained by calling
vpFeatureMomentGravityCenterNormalized::interaction.

  This feature is often used in moment-based visual servoing to control the
depth parameter.

  Minimum vpMomentObject order needed to compute this feature: 1 in dense mode
and 3 in discrete mode.

  This feature depends on:
  - vpMomentCentered
  - vpMomentAreaNormalized
  - vpMomentGravityCenter

*/
class VISP_EXPORT vpFeatureMomentAreaNormalized : public vpFeatureMoment
{
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
  vpFeatureMomentAreaNormalized(vpMomentDatabase &data_base, double A_, double B_, double C_,
                                vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 1)
  {
  }
  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentAreaNormalized"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentAreaNormalized"; }
};
#endif
#endif
