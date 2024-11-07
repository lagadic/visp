/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 *
 * Description:
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/
/*!
 \file vpTemplateTrackerWarpAffine.h
 \brief Affine warping function: w(X)=AX+b with: A=[[1+p0, p2], [p1, 1+p3]]
 and b= [p4, p5]]
*/

#ifndef vpTemplateTrackerWarpAffine_hh
#define vpTemplateTrackerWarpAffine_hh

#include <visp3/core/vpConfig.h>
#include <visp3/tt/vpTemplateTrackerWarp.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpTemplateTrackerWarpAffine
  \ingroup group_tt_warp

  This class consider the affine warping model \f$M\f$ with parameters \f$p=(a_1, a_2, a_3, a_4, a_5, a_6)\f$ such as
  \f[M(p) = \left[
  \begin{array}{ccc}
  a_1 + 1 & a_3 & a_5 \\
  a_2 & a_4 + 1 & a_6
  \end{array}
  \right]
  \f]

  We recall that u axis is the image horizontal axis, and v axis is the image vertical axis. A point (u,v) with
  coordinates (0,0) is located in the top left image corner.

*/
class VISP_EXPORT vpTemplateTrackerWarpAffine : public vpTemplateTrackerWarp
{
public:
  vpTemplateTrackerWarpAffine();

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void computeCoeff(const vpColVector &) { }
  void computeDenom(vpColVector &, const vpColVector &) { }
#endif

  void dWarp(const vpColVector &X, const vpColVector &, const vpColVector &, vpMatrix &dM);
  void dWarpCompo(const vpColVector &, const vpColVector &, const vpColVector &p, const double *dwdp0, vpMatrix &dM);

  void getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW);
  void getdWdp0(const int &v, const int &u, double *dIdW);

  void getParamInverse(const vpColVector &p, vpColVector &p_inv) const;
  void getParamPyramidDown(const vpColVector &p, vpColVector &p_down);
  void getParamPyramidUp(const vpColVector &p, vpColVector &p_up);

  /*!
   * Tells if the warping function is ESM compatible.
   * \return false. Affine model is not compatible with ESM.
   */
  bool isESMcompatible() const { return false; }

  void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const;

  void warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p);
  void warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p);
  void warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p);
};
END_VISP_NAMESPACE
#endif
