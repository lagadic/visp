/*
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
 */

/*!
 *\file vpTemplateTrackerWarpHomographySL3.h
 *\brief warping function of an homography: the homography is defined on the
 *sl3 lie algebra  H=exp(Sum(p[i]* A_i)) A_i is the basis of the SL3 Algebra
 */

#ifndef vpTemplateTrackerWarpHomographySL3_hh
#define vpTemplateTrackerWarpHomographySL3_hh

#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/tt/vpTemplateTrackerWarp.h>
#include <visp3/vision/vpHomography.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpTemplateTrackerWarpHomographySL3
 * \ingroup group_tt_warp
*/
class VISP_EXPORT vpTemplateTrackerWarpHomographySL3 : public vpTemplateTrackerWarp
{
protected:
  vpMatrix G;
  vpMatrix dGx;
  std::vector<vpMatrix> A;

public:
  vpTemplateTrackerWarpHomographySL3();

  void computeCoeff(const vpColVector &p);
  void computeDenom(vpColVector &X, const vpColVector &);

  void dWarp(const vpColVector &X1, const vpColVector &X2, const vpColVector &, vpMatrix &dW);
  void dWarpCompo(const vpColVector &, const vpColVector &X, const vpColVector &, const double *dwdp0, vpMatrix &dW);

  void findWarp(const double *ut0, const double *vt0, const double *u, const double *v, int nb_pt, vpColVector &p);

  void getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW);
  void getdWdp0(const int &v, const int &u, double *dIdW);
  void getdWdp0(const double &v, const double &u, double *dIdW);

  vpHomography getHomography() const;

  void getParamInverse(const vpColVector &p, vpColVector &p_inv) const;
  void getParamPyramidDown(const vpColVector &p, vpColVector &p_down);
  void getParamPyramidUp(const vpColVector &p, vpColVector &p_up);

  /*!
   * Tells if the warping function is ESM compatible.
   * \return true. Homography SL3 model is compatible with ESM.
   */
  bool isESMcompatible() const { return true; }

  void pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const;

  void warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &);
  void warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void warpXInv(const vpColVector &, vpColVector &, const vpColVector &) { }
#endif
};
END_VISP_NAMESPACE
#endif
