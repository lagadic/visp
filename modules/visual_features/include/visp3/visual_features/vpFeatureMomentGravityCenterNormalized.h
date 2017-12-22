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
  \file vpFeatureMomentGravityCenterNormalized.h
  \brief Implementation of the interaction matrix computation for
  vpMomentGravityCenterNormalized.
*/
#ifndef __FEATUREMOMENTGRAVITYCENTERNORMALIZED_H__
#define __FEATUREMOMENTGRAVITYCENTERNORMALIZED_H__
#include <visp3/visual_features/vpFeatureMoment.h>
#ifdef VISP_MOMENTS_COMBINE_MATRICES
class vpMomentDatabase;
/*!
  \class vpFeatureMomentGravityCenterNormalized

  \ingroup group_visual_features

  \brief Functionality computation for centered and normalized moment feature.
  Computes the interaction matrix associated with
  vpMomentGravityCenterNormalized.

  The interaction matrix for the moment feature can be deduced from \cite
  Tahri05z, equation (19). To do so, one must derive it and obtain a
  combination of interaction matrices by using (1). It allows to compute the
  interaction matrices for \f$ (x_n,y_n) \f$.

  These interaction matrices may be selected afterwards by calling
  vpFeatureMomentGravityCenterNormalized::interaction. The selection is done
  by the following methods: vpFeatureMomentGravityCenterNormalized::selectXn
  for \f$ L_{x_{n}} \f$ and vpFeatureMomentGravityCenterNormalized::selectYn
  for \f$ L_{y_{n}} \f$. You can use these shortcut selectors as follows:

  \code
  task.addFeature(db_src.getFeatureGravityNormalized(), db_dst.getFeatureGravityNormalized(),
                  vpFeatureMomentGravityCenterNormalized::selectXn() | vpFeatureMomentGravityCenterNormalized::selectYn());
  \endcode

  The behaviour of this feature is very similar to
  vpFeatureMomentGravityCenter which also contains a sample code demonstrating
  a selection.

  This feature is often used in moment-based visual servoing to control the
  planar translation parameters.

  Minimum vpMomentObject order needed to compute this feature: 2 in dense mode
  and 3 in discrete mode.

  This feature depends on:
    - vpFeatureMomentGravityCenter
    - vpMomentGravityCenter
    - vpMomentAreaNormalized
    - vpFeatureMomentAreaNormalized

*/
class VISP_EXPORT vpFeatureMomentGravityCenterNormalized : public vpFeatureMoment
{
public:
  /*!
  Initializes the feature with information about the database of moment
  primitives, the object plane and feature database. \param database : Moment
  database. The database of moment primitives (first parameter) is mandatory.
  It is used to access different moment values later used to compute the final
  matrix. \param A_ : Plane coefficient in a \f$ A \times x+B \times y + C =
  \frac{1}{Z} \f$ plane. \param B_ : Plane coefficient in a \f$ A \times x+B
  \times y + C = \frac{1}{Z} \f$ plane. \param C_ : Plane coefficient in a \f$
  A \times x+B \times y + C = \frac{1}{Z} \f$ plane. \param featureMoments :
  Feature database.

  */
  vpFeatureMomentGravityCenterNormalized(vpMomentDatabase &database, double A_, double B_, double C_,
                                         vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(database, A_, B_, C_, featureMoments, 2)
  {
  }
  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentGravityCenterNormalized"; }
  /*!
      feature name
    */
  const char *name() const { return "vpFeatureMomentGravityCenterNormalized"; }

  /*!
    Shortcut selector for \f$x_n\f$.
    */
  static unsigned int selectXn() { return 1 << 0; }

  /*!
    Shortcut selector for \f$y_n\f$.
    */
  static unsigned int selectYn() { return 1 << 1; }
};

#else
class vpMomentDatabase;
/*!
  \class vpFeatureMomentGravityCenterNormalized

  \ingroup group_visual_features

  \brief Functionality computation for centered and normalized moment feature.
Computes the interaction matrix associated with
vpMomentGravityCenterNormalized.

  It computes the interaction matrices for \f$ (x_n,y_n) \f$.
  The interaction matrix for the moment feature has the following expression:
  - In the discrete case:
  \f[
  L_{x_n} =
{
 \left[ \begin {array}{c} -Ax_{{n}}\theta+ \left( x_{{n}}e_{{1,1}}-y_{
{n}} \right) B-a_{{n}}C\\ \noalign{\medskip}Ax_{{n}}e_{{1,1}}+Bx_{{n}}
\theta\\ \noalign{\medskip} \left( -a_{{n}}-w_{{y}} \right) A+Bw_{{x}}
\\ \noalign{\medskip}a_{{n}}e_{{1,1}}{\it NA}+ \left( \eta_{{1,0}}e_{{
1,1}}+\eta_{{0,1}}-e_{{2,1}}-x_{{g}}e_{{1,1}}+\eta_{{0,1}}\theta
 \right) x_{{n}}+ \left( \eta_{{1,0}}-x_{{g}}\theta \right) y_{{n}}-{
\frac {x_{{n}}\eta_{{0,3}}}{{\it NA}}}\\ \noalign{\medskip} \left( -1+
\theta \right) a_{{n}}{\it NA}+ \left( e_{{1,2}}+x_{{g}}-\eta_{{0,1}}e
_{{1,1}}-2\,\eta_{{1,0}}+e_{{3,0}}+ \left( -x_{{g}}+\eta_{{1,0}}
 \right) \theta \right) x_{{n}}+e_{{1,1}}x_{{g}}y_{{n}}-a_{{n}}
\\ \noalign{\medskip}y_{{n}}\end {array} \right]
}^t

  L_{y_n} =


{
 \left[ \begin {array}{c}  \left( 1-\theta \right) y_{{n}}A+y_{{n}}e_{
{1,1}}B\\ \noalign{\medskip} \left( -x_{{n}}+y_{{n}}e_{{1,1}} \right)
A+ \left( -1+\theta \right) y_{{n}}B-a_{{n}}C\\ \noalign{\medskip}-Aw_
{{y}}+ \left( -a_{{n}}+w_{{x}} \right) B\\ \noalign{\medskip}\theta\,a
_{{n}}{\it NA}+ \left( -e_{{2,1}}+\eta_{{1,0}}e_{{1,1}}+\eta_{{0,1}}-x
_{{g}}e_{{1,1}}+ \left( \eta_{{0,1}}-y_{{g}} \right) \theta \right) y_
{{n}}+a_{{n}}-{\frac {y_{{n}}\eta_{{0,3}}}{{\it NA}}}
\\ \noalign{\medskip}-a_{{n}}e_{{1,1}}{\it NA}-x_{{n}}\eta_{{0,1}}+
 \left( e_{{1,2}}+y_{{g}}e_{{1,1}}-\eta_{{0,1}}e_{{1,1}}+x_{{g}}+e_{{3
,0}}-2\,\eta_{{1,0}}+ \left( -x_{{g}}+\eta_{{1,0}} \right) \theta
 \right) y_{{n}}\\ \noalign{\medskip}-x_{{n}}\end {array} \right]

}^t
  \f]
  - In the dense case:
  \f[
  L_{x_n} =
{
\left[ \begin {array}{c} -a_{{n}}C-1/2\,Ax_{{n}}-By_{{n}}
\\ \noalign{\medskip}1/2\,Bx_{{n}}\\ \noalign{\medskip} \left( -a_{{n}
}-w_{{y}} \right) A+Bw_{{x}}\\ \noalign{\medskip} \left( 4\,\eta_{{1,0
}}-1/2\,x_{{g}} \right) y_{{n}}+4\,a_{{n}}\eta_{{1,1}}+4\,x_{{n}}\eta_
{{0,1}}\\ \noalign{\medskip} \left( -4\,\eta_{{1,0}}+1/2\,x_{{g}}
 \right) x_{{n}}+ \left( -1-4\,\eta_{{2,0}} \right) a_{{n}}
\\ \noalign{\medskip}y_{{n}}\end {array} \right]


}^t

  L_{y_n} =
{
 \left[ \begin {array}{c} 1/2\,Ay_{{n}}\\ \noalign{\medskip}-1/2\,By_{
{n}}-a_{{n}}C-Ax_{{n}}\\ \noalign{\medskip}-Aw_{{y}}+ \left( -a_{{n}}+
w_{{x}} \right) B\\ \noalign{\medskip}4\,\theta\,a_{{n}}{\it NA}+
 \left( 4\,\eta_{{0,1}}-1/2\,y_{{g}} \right) y_{{n}}+a_{{n}}
\\ \noalign{\medskip} \left( -4\,\eta_{{1,0}}+1/2\,x_{{g}} \right) y_{
{n}}-4\,a_{{n}}\eta_{{1,1}}-4\,x_{{n}}\eta_{{0,1}}
\\ \noalign{\medskip}-x_{{n}}\end {array} \right]

}^t
  \f]
with:
    - \f$e_{i,j}=\frac{\mu_{i,j}}{NA}\f$
    - \f$NA=\mu_{2,0}+\mu_{0,2}\f$
    - \f$\theta=\frac{\eta_{0,2}}{NA}\f$
    - \f$\eta\f$ is the centered and normalized moment.

  These interaction matrices may be selected afterwards by calling
vpFeatureMomentGravityCenterNormalized::interaction. The selection is done by
the following methods: vpFeatureMomentGravityCenterNormalized::selectXn for
\f$ L_{x_{n}} \f$ and vpFeatureMomentGravityCenterNormalized::selectYn for \f$
L_{y_{n}} \f$. You can use these shortcut selectors as follows:

  \code
  task.addFeature(db_src.getFeatureGravityNormalized(),db_dst.getFeatureGravityNormalized(),
                  vpFeatureMomentGravityCenterNormalized::selectXn() |
                  vpFeatureMomentGravityCenterNormalized::selectYn());
  \endcode

  The behaviour of this feature is very similar to
vpFeatureMomentGravityCenter which also contains a sample code demonstrating a
selection.

  This feature is often used in moment-based visual servoing to control the
planar translation parameters.

  Minimum vpMomentObject order needed to compute this feature: 2 in dense mode
and 3 in discrete mode.

  This feature depends on:
    - vpFeatureMomentGravityCenter
    - vpMomentGravityCenter
    - vpMomentAreaNormalized
    - vpFeatureMomentAreaNormalized

*/
class VISP_EXPORT vpFeatureMomentGravityCenterNormalized : public vpFeatureMoment
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
  vpFeatureMomentGravityCenterNormalized(vpMomentDatabase &data_base, double A_, double B_, double C_,
                                         vpFeatureMomentDatabase *featureMoments = NULL)
    : vpFeatureMoment(data_base, A_, B_, C_, featureMoments, 2)
  {
  }
  void compute_interaction();
  /*!
    associated moment name
    */
  const char *momentName() const { return "vpMomentGravityCenterNormalized"; }
  /*!
      feature name
    */
  const char *name() const { return "vpFeatureMomentGravityCenterNormalized"; }

  /*!
    Shortcut selector for \f$x_n\f$.
    */
  static unsigned int selectXn() { return 1 << 0; }

  /*!
    Shortcut selector for \f$y_n\f$.
    */
  static unsigned int selectYn() { return 1 << 1; }
};
#endif
#endif
