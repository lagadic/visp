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
 \file vpTemplateTrackerWarpHomography.h
 \brief warping function of an homography: the homography is directly defined
 by the diplacement parameter: H=[[1+p0, p3, p6], [p1, 1+p4, p7], [p2, p5, 1]]
*/

#ifndef vpTemplateTrackerWarpHomography_hh
#define vpTemplateTrackerWarpHomography_hh

#include <visp3/core/vpConfig.h>
#include <visp3/tt/vpTemplateTrackerWarp.h>
#include <visp3/vision/vpHomography.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpTemplateTrackerWarpHomography
  \ingroup group_tt_warp

  This class consider the homography warping model \f$M\f$ with parameters \f$p=(h_1, h_2, h_3, h_4, h_5, h_6, h_7,
  h_8)\f$ such as \f[M(p) = \left[ \begin{array}{ccc}
  h_1 + 1 & h_4 & h_7 \\
  h_2 & h_5 + 1 & h_8 \\
  h_3 & h_6 & 1
  \end{array}
  \right]
  \f]

  We recall that u axis is the image horizontal axis, and v axis is the image vertical axis. A point (u,v) with
  coordinates (0,0) is located in the top left image corner.

*/
class VISP_EXPORT vpTemplateTrackerWarpHomography : public vpTemplateTrackerWarp
{
public:
  vpTemplateTrackerWarpHomography();

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void computeCoeff(const vpColVector &) { }
#endif

  void computeDenom(vpColVector &X, const vpColVector &p);

  void dWarp(const vpColVector &, const vpColVector &X, const vpColVector &, vpMatrix &dW);
  void dWarpCompo(const vpColVector &X, const vpColVector &, const vpColVector &p, const double *dwdp0, vpMatrix &dW);

  void getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW);
  void getdWdp0(const int &v, const int &u, double *dIdW);

  vpHomography getHomography(const vpColVector &ParamM) const;

  void getParam(const vpHomography &H, vpColVector &p) const;
  void getParam(const vpMatrix &H, vpColVector &p) const;

  void getParamInverse(const vpColVector &p, vpColVector &p_inv) const;
  void getParamPyramidDown(const vpColVector &p, vpColVector &p_down);
  void getParamPyramidUp(const vpColVector &p, vpColVector &p_up);

  /*!
   * Tells if the warping function is ESM compatible.
   * \return false. Homography model is not compatible with ESM.
   */
  bool isESMcompatible() const { return false; }

  void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const;

  void warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p);
  void warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p);
  void warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p);
};
END_VISP_NAMESPACE
#endif
