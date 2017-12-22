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
  \file vpFeatureMomentAlpha.h
  \brief Implementation of the interaction matrix computation for
  vpMomentAlpha.
*/

#ifndef __FEATUREMOMENTALPHA_H__
#define __FEATUREMOMENTALPHA_H__
#include <visp3/core/vpColVector.h>
#include <visp3/visual_features/vpFeatureMoment.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES

class vpMomentDatabase;
/*!
  \class vpFeatureMomentAlpha

  \ingroup group_visual_features

  \brief Functionality computation for in-plane rotation moment feature \f$
  \alpha \f$. Computes the interaction matrix associated with vpMomentAlpha.

  The interaction matrix for the feature can be deduced from \cite Tahri05z.

  This class allows to compute the interaction matrix associated to \f$ \alpha
  = \frac{1}{2} arctan(\frac{2\mu_{11}}{\mu_{20}-\mu_{02}}) \f$ moment
  primitive.

  The interaction matrix computed is single-dimension (no selection possible)
  and can be obtained by calling vpFeatureMomentAlpha::interaction().

  This feature is often used in moment-based visual servoing to control the
  planar rotation parameter.

  Minimum vpMomentObject order needed to compute this feature: 4.

  This feature depends on:
  - vpMomentCentered
  - vpFeatureMomentCentered.
*/
class VISP_EXPORT vpFeatureMomentAlpha : public vpFeatureMoment
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
  vpFeatureMomentAlpha(vpMomentDatabase &moments, double A, double B, double C,
                       vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(moments, A, B, C, featureMoments, 1)
  {
  }

  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentAlpha"; }
  /*!
    feature name
    */
  const char *name() const { return "vpFeatureMomentAlpha"; }
};
#else
class vpMomentDatabase;
/*!
  \class vpFeatureMomentAlpha

  \ingroup group_visual_features

  \brief Functionality computation for in-plane rotation moment feature \f$
\alpha \f$: computes the interaction matrix associated with vpMomentAlpha.

  This class computes the interaction matrix associated to \f$ \alpha =
\frac{1}{2} arctan(\frac{2\mu_{11}}{\mu_{20}-\mu_{02}}) \f$ moment primitive.

  The interaction matrix for the feature has the following form:
  \f[{
 \left[ \begin {array}{c} {\frac {\mu_{{1,1}}{\it DA}\,A}{d}}+{\frac {
 \left( {\it DA}\,\mu_{{0,2}}+1/2\,d-1/2\,{{\it DA}}^{2} \right) B}{d}
}\\ \noalign{\medskip}{\frac { \left( {\it DA}\,\mu_{{0,2}}-1/2\,d-1/2
\,{{\it DA}}^{2} \right) A}{d}}-{\frac {B\mu_{{1,1}}{\it DA}}{d}}
\\ \noalign{\medskip}Bw_{{x}}-Aw_{{y}}\\ \noalign{\medskip}{\frac {
\beta\, \left( \mu_{{1,2}} \left( \mu_{{2,0}}-\mu_{{0,2}} \right) +\mu
_{{1,1}} \left( \mu_{{0,3}}-\mu_{{2,1}} \right)  \right) +\gamma\,x_{{
g}} \left( \mu_{{0,2}} \left( \mu_{{2,0}}-\mu_{{0,2}} \right) -2\,{\mu
_{{1,1}}}^{2} \right) +\gamma\,y_{{g}}\mu_{{1,1}} \left( \mu_{{2,0}}+
\mu_{{0,2}} \right) }{d}}\\ \noalign{\medskip}{\frac {\beta\, \left(
\mu_{{2,1}} \left( \mu_{{0,2}}-\mu_{{2,0}} \right) +\mu_{{1,1}}
 \left( \mu_{{3,0}}-\mu_{{1,2}} \right)  \right) +\gamma\,x_{{g}}\mu_{
{1,1}} \left( \mu_{{2,0}}+\mu_{{0,2}} \right) +\gamma\,y_{{g}} \left(
\mu_{{2,0}} \left( \mu_{{0,2}}-\mu_{{2,0}} \right) -2\,{\mu_{{1,1}}}^{
2} \right) }{d}}\\ \noalign{\medskip}-1\end {array} \right]
}^t
\f]
with \f${\it DA} = \mu_{{2,0}}-\mu_{{0,2}}\f$ and \f${\it d} =
DA^2+4{\mu_{1,1}}^2\f$.

  - In the discrete case:
  \f$beta = 4\f$,\f$gamma = 2\f$.
  - In the dense case:
  \f$beta = 5\f$,\f$gamma = 1\f$.


  The interaction matrix computed is single-dimension (no selection possible)
and can be obtained by calling vpFeatureMomentAlpha::interaction().

  This feature is often used in moment-based visual servoing to control the
planar rotation parameter.

  Minimum vpMomentObject order needed to compute this feature: 4.

  This feature depends on:
  - vpMomentCentered
  - vpMomentGravityCenter
*/
class VISP_EXPORT vpFeatureMomentAlpha : public vpFeatureMoment
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
  vpFeatureMomentAlpha(vpMomentDatabase &data_base, double A_, double B_, double C_,
                       vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 1)
  {
  }

  void compute_interaction();
  /*!
    Associated moment name.
    */
  const char *momentName() const { return "vpMomentAlpha"; }
  /*!
    Feature name.
    */
  const char *name() const { return "vpFeatureMomentAlpha"; }

  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);
};
#endif
#endif
