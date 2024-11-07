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
#include <visp3/tt/vpTemplateTrackerWarpRT.h>

BEGIN_VISP_NAMESPACE
/*!
 * Construct a model with 3 parameters for rotation and translation initialized to zero.
 */
vpTemplateTrackerWarpRT::vpTemplateTrackerWarpRT() { nbParam = 3; }

/*!
 * Get the parameters of the warping function one level down
 * where image size is divided by two along the lines and the columns.
 * \param p : 3-dim vector that contains the current parameters of the warping function.
 * \param p_down : 3-dim vector that contains the resulting parameters one level down.
 */
void vpTemplateTrackerWarpRT::getParamPyramidDown(const vpColVector &p, vpColVector &p_down)
{
  p_down[0] = p[0];
  p_down[1] = p[1] / 2.;
  p_down[2] = p[2] / 2.;
}

/*!
 * Get the parameters of the warping function one level up
 * where image size is multiplied by two along the lines and the columns.
 * \param p : 3-dim vector that contains the current parameters of the warping function.
 * \param p_up : 3-dim vector that contains the resulting parameters one level up.
 */
void vpTemplateTrackerWarpRT::getParamPyramidUp(const vpColVector &p, vpColVector &p_up)
{
  p_up[0] = p[0];
  p_up[1] = p[1] * 2.;
  p_up[2] = p[2] * 2.;
}

/*!
 * Compute the derivative of the image with relation to the warping function parameters.
 * \param v : Coordinate (along the image rows axis) of the point to consider in the image.
 * \param u : Coordinate (along the image columns axis) of the point to consider in the image.
 * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
 * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
 * \param dIdW : Resulting derivative matrix (image according to the warping function).
 */
void vpTemplateTrackerWarpRT::getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW)
{
  dIdW[0] = -v * du + u * dv;
  dIdW[1] = du;
  dIdW[2] = dv;
}

/*!
 * Compute the derivative of the warping model \f$M\f$ according to the initial parameters \f$p_0\f$
 * at point \f$X=(u,v)\f$:
 * \f[
 * \frac{\partial M}{\partial p}(X, p_0)
 * \f]
 *
 * \param v : Coordinate (along the image rows axis) of the point X(u,v) to consider in the image.
 * \param u : Coordinate (along the image columns axis) of the point X(u,v) to consider in the image.
 * \param dIdW : Resulting 2-by-3 derivative matrix.
 */
void vpTemplateTrackerWarpRT::getdWdp0(const int &v, const int &u, double *dIdW)
{
  dIdW[0] = -v;
  dIdW[1] = 1.;
  dIdW[2] = 0;

  dIdW[3] = u;
  dIdW[4] = 0;
  dIdW[5] = 1.;
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
 * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
 * \param p : 3-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpRT::warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p)
{
  double c = cos(p[0]);
  double s = sin(p[0]);

  u2 = (c * u1) - (s * v1) + p[1];
  v2 = (s * u1) + (c * v1) + p[2];
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
 * \param p : 3-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpRT::warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  double c = cos(p[0]);
  double s = sin(p[0]);

  X2[0] = (c * X1[0]) - (s * X1[1]) + p[1];
  X2[1] = (s * X1[0]) + (c * X1[1]) + p[2];
}

/*!
 * Compute the derivative matrix of the warping function at point \f$X=(u,v)\f$ according to the model parameters:
 * \f[
 * \frac{\partial M}{\partial p}(X, p)
 * \f]
 * \param X : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param p : 3-dim vector that contains the parameters of the warping function.
 * \param dM : Resulting warping model derivative returned as a 2-by-3 matrix.
 */
void vpTemplateTrackerWarpRT::dWarp(const vpColVector &X, const vpColVector &, const vpColVector &p, vpMatrix &dM)
{
  double u = X[0];
  double v = X[1];
  double c = cos(p[0]);
  double s = sin(p[0]);

  dM[0][0] = -s * u - c * v;
  dM[0][1] = 1;
  dM[0][2] = 0;

  dM[1][0] = c * u - s * v;
  dM[1][1] = 0;
  dM[1][2] = 1;
}

/*!
 * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
 * \param p : 3-dim vector that contains the parameters of the warping function.
 * \param dwdp0 : 2-by-3 derivative matrix of the warping function according to
 * the initial warping function parameters (p=0).
 * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-3 matrix.
 */
void vpTemplateTrackerWarpRT::dWarpCompo(const vpColVector &, const vpColVector &, const vpColVector &p,
                                         const double *dwdp0, vpMatrix &dM)
{
  double c = cos(p[0]);
  double s = sin(p[0]);

  for (unsigned int i = 0; i < nbParam; i++) {
    dM[0][i] = (c * dwdp0[i]) - (s * dwdp0[i + nbParam]);
    dM[1][i] = (s * dwdp0[i]) + (c * dwdp0[i + nbParam]);
  }
}

/*!
 * Warp a point X1 with the inverse transformation \f$M\f$.
 * \f[ X_2 = {\left({^1}M_2\right)}^{-1} \; X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates (u,v) of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates (u,v) of the warped point.
 * \param p : Parameters corresponding to the warping model \f${^1}M_2\f$.
 */
void vpTemplateTrackerWarpRT::warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  double c = cos(p[0]);
  double s = sin(p[0]);

  X2[0] = (c * X1[0]) - (s * X1[1]) + p[1];
  X2[1] = (s * X1[0]) + (c * X1[1]) + p[2];
}

/*!
 * Compute inverse of the warping transformation.
 * \param p : 3-dim vector that contains the parameters corresponding
 * to the transformation to inverse.
 * \param p_inv : 3-dim vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
 */
void vpTemplateTrackerWarpRT::getParamInverse(const vpColVector &p, vpColVector &p_inv) const
{
  double c = cos(p[0]);
  double s = sin(p[0]);
  double u = p[1];
  double v = p[2];

  p_inv[0] = atan2(-s, c);
  p_inv[1] = -(c * u + s * v);
  p_inv[2] = s * u - c * v;
}

/*!
 * Compute the transformation resulting from the composition of two other transformations.
 * \param p1 : 3-dim vector that contains the parameters corresponding
 * to first transformation.
 * \param p2 : 3-dim vector that contains the parameters corresponding
 * to second transformation.
 * \param p12 : 3-dim vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
 */
void vpTemplateTrackerWarpRT::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const
{
  double c1 = cos(p1[0]);
  double s1 = sin(p1[0]);
  double c2 = cos(p2[0]);
  double s2 = sin(p2[0]);
  double u1 = p1[1];
  double v1 = p1[2];
  double u2 = p2[1];
  double v2 = p2[2];

  p12[0] = atan2(s1 * c2 + c1 * s2, c1 * c2 - s1 * s2);
  p12[1] = c1 * u2 - s1 * v2 + u1;
  p12[2] = s1 * u2 + c1 * v2 + v1;
}
END_VISP_NAMESPACE
