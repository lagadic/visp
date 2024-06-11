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
#include <visp3/tt/vpTemplateTrackerWarpAffine.h>

BEGIN_VISP_NAMESPACE
/*!
 * Construct a model with 6 affine parameters initialized to zero.
 */
vpTemplateTrackerWarpAffine::vpTemplateTrackerWarpAffine() { nbParam = 6; }

/*!
 * Get the parameters of the warping function one level down
 * where image size is divided by two along the lines and the columns.
 * \param p : 6-dim vector that contains the current parameters of the warping function.
 * \param p_down : 6-dim vector that contains the resulting parameters one level down.
 */
void vpTemplateTrackerWarpAffine::getParamPyramidDown(const vpColVector &p, vpColVector &p_down)
{
  p_down[0] = p[0];
  p_down[1] = p[1];
  p_down[2] = p[2];
  p_down[3] = p[3];
  p_down[4] = p[4] / 2.;
  p_down[5] = p[5] / 2.;
}

/*!
 * Get the parameters of the warping function one level up
 * where image size is multiplied by two along the lines and the columns.
 * \param p : 6-dim vector that contains the current parameters of the warping function.
 * \param p_up : 6-dim vector that contains the resulting parameters one level up.
 */
void vpTemplateTrackerWarpAffine::getParamPyramidUp(const vpColVector &p, vpColVector &p_up)
{
  p_up[0] = p[0];
  p_up[1] = p[1];
  p_up[2] = p[2];
  p_up[3] = p[3];
  p_up[4] = p[4] * 2.;
  p_up[5] = p[5] * 2.;
}

/*!
 * Compute the derivative of the image with relation to the warping function parameters.
 * \param v : Coordinate (along the image rows axis) of the point to consider in the image.
 * \param u : Coordinate (along the image columns axis) of the point to consider in the image.
 * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
 * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
 * \param dIdW : Resulting derivative matrix (image according to the warping function).
 */
void vpTemplateTrackerWarpAffine::getdW0(const int &v, const int &u, const double &dv, const double &du, double *dIdW)
{
  dIdW[0] = u * du;
  dIdW[1] = u * dv;
  dIdW[2] = v * du;
  dIdW[3] = v * dv;
  dIdW[4] = du;
  dIdW[5] = dv;
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
 * \param dIdW : Resulting 2-by-6 derivative matrix.
 */
void vpTemplateTrackerWarpAffine::getdWdp0(const int &v, const int &u, double *dIdW)
{
  dIdW[0] = u;
  dIdW[1] = 0;
  dIdW[2] = v;
  dIdW[3] = 0;
  dIdW[4] = 1.;
  dIdW[5] = 0;

  dIdW[6] = 0;
  dIdW[7] = u;
  dIdW[8] = 0;
  dIdW[9] = v;
  dIdW[10] = 0;
  dIdW[11] = 1.;
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
 * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
 * \param p : 6-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpAffine::warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p)
{
  u2 = (1 + p[0]) * u1 + p[2] * v1 + p[4];
  v2 = p[1] * u1 + (1 + p[3]) * v1 + p[5];
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
 * \param p : 6-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpAffine::warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  X2[0] = (1.0 + p[0]) * X1[0] + p[2] * X1[1] + p[4];
  X2[1] = p[1] * X1[0] + (1.0 + p[3]) * X1[1] + p[5];
}

/*!
 * Compute the derivative matrix of the warping function at point \f$X=(u,v)\f$ according to the model parameters:
 * \f[
 * \frac{\partial M}{\partial p}(X, p)
 * \f]
 * \param X : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param dM : Resulting warping model derivative returned as a 2-by-6 matrix.
 */
void vpTemplateTrackerWarpAffine::dWarp(const vpColVector &X, const vpColVector &, const vpColVector &, vpMatrix &dM)
{
  double u = X[0];
  double v = X[1];

  dM[0][0] = u;
  dM[0][1] = 0;
  dM[0][2] = v;
  dM[0][3] = 0;
  dM[0][4] = 1;
  dM[0][5] = 0;
  dM[1][0] = 0;
  dM[1][1] = u;
  dM[1][2] = 0;
  dM[1][3] = v;
  dM[1][4] = 0;
  dM[1][5] = 1;
}

/*!
 * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
 * \param p : 6-dim vector that contains the parameters of the warping function.
 * \param dwdp0 : Derivative matrix of the warping function according to
 * the initial warping function parameters (p=0).
 * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-6 matrix.
 */
void vpTemplateTrackerWarpAffine::dWarpCompo(const vpColVector &, const vpColVector &, const vpColVector &p,
                                             const double *dwdp0, vpMatrix &dM)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dM[0][i] = (1. + p[0]) * dwdp0[i] + p[2] * dwdp0[i + nbParam];
    dM[1][i] = p[1] * dwdp0[i] + (1. + p[3]) * dwdp0[i + nbParam];
  }
}

/*!
 * Warp a point X1 with the inverse transformation \f$M\f$.
 * \f[ X_2 = {\left({^1}M_2\right)}^{-1} \; X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates (u,v) of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates (u,v) of the warped point.
 * \param p : Parameters corresponding to the warping model \f${^1}M_2\f$.
 */
void vpTemplateTrackerWarpAffine::warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  X2[0] = (1 + p[0]) * X1[0] + p[2] * X1[1] + p[4];
  X2[1] = p[1] * X1[0] + (1 + p[3]) * X1[1] + p[5];
}

/*!
 * Compute inverse of the warping transformation.
 * \param p : 6-dim vector that contains the parameters corresponding
 * to the transformation to inverse.
 * \param p_inv : 6-dim vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
 */
void vpTemplateTrackerWarpAffine::getParamInverse(const vpColVector &p, vpColVector &p_inv) const
{
  double u = p[4];
  double v = p[5];
  double r_00 = 1 + p[0], r_01 = p[2];
  double r_10 = p[1], r_11 = 1 + p[3];
  double det = r_00 * r_11 - r_01 * r_10;
  if (std::fabs(det) < std::numeric_limits<double>::epsilon()) {
    throw(vpException(vpException::fatalError, "In vpTemplateTrackerWarpAffine::getParamInverse() "
                      "cannot inverse 2-by-2 matrix. Matrix determinant is 0."));
  }

  double ri_11 = r_00 / det;
  double ri_00 = r_11 / det;
  double ri_01 = -r_01 / det;
  double ri_10 = -r_10 / det;

  p_inv[0] = ri_00 - 1;
  p_inv[1] = ri_10;
  p_inv[2] = ri_01;
  p_inv[3] = ri_11 - 1;
  p_inv[4] = -(ri_00 * u + ri_01 * v);
  p_inv[5] = -(ri_10 * u + ri_11 * v);
}

/*!
 * Compute the transformation resulting from the composition of two other transformations.
 * \param p1 : 6-dim vector that contains the parameters corresponding
 * to first transformation.
 * \param p2 : 6-dim vector that contains the parameters corresponding
 * to second transformation.
 * \param p12 : 6-dim vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
 */
void vpTemplateTrackerWarpAffine::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const
{
  double r1_00 = 1 + p1[0], r1_01 = p1[2];
  double r1_10 = p1[1], r1_11 = 1 + p1[3];
  double r2_00 = 1 + p2[0], r2_01 = p2[2];
  double r2_10 = p2[1], r2_11 = 1 + p2[3];
  double u1 = p1[4];
  double v1 = p1[5];
  double u2 = p2[4];
  double v2 = p2[5];

  p12[0] = r1_00 * r2_00 + r1_01 * r2_10 - 1.;
  p12[1] = r1_10 * r2_00 + r1_11 * r2_10;
  p12[2] = r1_00 * r2_01 + r1_01 * r2_11;
  p12[3] = r1_10 * r2_01 + r1_11 * r2_11 - 1.;
  p12[4] = r1_00 * u2 + r1_01 * v2 + u1;
  p12[5] = r1_10 * u2 + r1_11 * v2 + v1;
}
END_VISP_NAMESPACE
