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
#include <visp3/tt/vpTemplateTrackerWarpTranslation.h>

BEGIN_VISP_NAMESPACE
/*!
 * Construct a model with 2 parameters for translation initialized to zero.
 */
vpTemplateTrackerWarpTranslation::vpTemplateTrackerWarpTranslation() { nbParam = 2; }

/*!
 * Get the parameters of the warping function one level down
 * where image size is divided by two along the lines and the columns.
 * \param p : 2-dim vector that contains the current parameters of the warping function.
 * \param p_down : 2-dim vector that contains the resulting parameters one level down.
 */
void vpTemplateTrackerWarpTranslation::getParamPyramidDown(const vpColVector &p, vpColVector &p_down)
{
  p_down[0] = p[0] / 2.;
  p_down[1] = p[1] / 2.;
}

/*!
 * Get the parameters of the warping function one level up
 * where image size is multiplied by two along the lines and the columns.
 * \param p : 2-dim vector that contains the current parameters of the warping function.
 * \param p_up : 2-dim vector that contains the resulting parameters one level up.
 */
void vpTemplateTrackerWarpTranslation::getParamPyramidUp(const vpColVector &p, vpColVector &p_up)
{
  p_up[0] = p[0] * 2.;
  p_up[1] = p[1] * 2.;
}

/*!
 * Compute the derivative of the image with relation to the warping function parameters.
 * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
 * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
 * \param dIdW : Resulting derivative matrix (image according to the warping function).
 */
void vpTemplateTrackerWarpTranslation::getdW0(const int &, const int &, const double &dv, const double &du,
                                              double *dIdW)
{
  dIdW[0] = du;
  dIdW[1] = dv;
}

/*!
 * Compute the derivative of the warping model \f$M\f$ according to the initial parameters \f$p_0\f$.
 * \f[
 * \frac{\partial M}{\partial p}(p_0)
 * \f]
 *
 * \param dIdW : Resulting 2-by-2 derivative matrix.
 */
void vpTemplateTrackerWarpTranslation::getdWdp0(const int &, const int &, double *dIdW)
{
  dIdW[0] = 1.;
  dIdW[1] = 0;

  dIdW[2] = 0;
  dIdW[3] = 1.;
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
 * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
 * \param p : 2-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpTranslation::warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p)
{
  u2 = u1 + p[0];
  v2 = v1 + p[1];
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
 * \param p : 2-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpTranslation::warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  X2[0] = X1[0] + p[0];
  X2[1] = X1[1] + p[1];
}

/*!
 * Compute the derivative matrix of the warping function:
 * \f[
 * \frac{\partial M}{\partial p}(p)
 * \f]
 * \param dM : Resulting warping model derivative returned as a 2-by-2 matrix.
 */
void vpTemplateTrackerWarpTranslation::dWarp(const vpColVector &, const vpColVector &, const vpColVector &,
                                             vpMatrix &dM)
{
  dM[0][0] = 1;
  dM[0][1] = 0;
  dM[1][0] = 0;
  dM[1][1] = 1;
}

/*!
 * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
 * \param dwdp0 : Derivative matrix of the warping function according to
 * the initial warping function parameters (p=0).
 * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-2 matrix.
 */
void vpTemplateTrackerWarpTranslation::dWarpCompo(const vpColVector &, const vpColVector &, const vpColVector &,
                                                  const double *dwdp0, vpMatrix &dM)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dM[0][i] = dwdp0[i];
    dM[1][i] = dwdp0[i + nbParam];
  }
}

/*!
 * Warp a point X1 with the inverse transformation \f$M\f$.
 * \f[ X_2 = {\left({^1}M_2\right)}^{-1} \; X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates (u,v) of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates (u,v) of the warped point.
 * \param p : Parameters corresponding to the warping model \f${^1}M_2\f$.
 */
void vpTemplateTrackerWarpTranslation::warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  X2[0] = X1[0] + p[0];
  X2[1] = X1[1] + p[1];
}

/*!
 * Compute inverse of the warping transformation.
 * \param p : 2-dim vector that contains the translation parameters corresponding
 * to the transformation to inverse.
 * \param p_inv : 2-dim vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
 */
void vpTemplateTrackerWarpTranslation::getParamInverse(const vpColVector &p, vpColVector &p_inv) const
{
  p_inv[0] = -p[0];
  p_inv[1] = -p[1];
}

/*!
 * Compute the transformation resulting from the composition of two other transformations.
 * \param p1 : 2-dim vector that contains the translation parameters corresponding
 * to first transformation.
 * \param p2 : 2-dim vector that contains the translation parameters corresponding
 * to second transformation.
 * \param p12 : 2-dim vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
 */
void vpTemplateTrackerWarpTranslation::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const
{
  p12[0] = p1[0] + p2[0];
  p12[1] = p1[1] + p2[1];
}
END_VISP_NAMESPACE
