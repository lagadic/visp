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
#include <visp3/core/vpTrackingException.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>

BEGIN_VISP_NAMESPACE
/*!
 * Construct an homography model with 8 parameters initialized to zero.
 */
vpTemplateTrackerWarpHomography::vpTemplateTrackerWarpHomography() { nbParam = 8; }

/*!
 * Get the parameters of the warping function one level down
 * where image size is divided by two along the lines and the columns.
 * \param p : 8-dim vector that contains the current parameters of the warping function.
 * \param p_down : 8-dim vector that contains the resulting parameters one level down.
 */
void vpTemplateTrackerWarpHomography::getParamPyramidDown(const vpColVector &p, vpColVector &p_down)
{
  p_down[0] = p[0];
  p_down[1] = p[1];
  p_down[2] = p[2] * 2.;
  p_down[3] = p[3];
  p_down[4] = p[4];
  p_down[5] = p[5] * 2.;
  p_down[6] = p[6] / 2.;
  p_down[7] = p[7] / 2.;
}

/*!
 * Get the parameters of the warping function one level up
 * where image size is multiplied by two along the lines and the columns.
 * \param p : 8-dim vector that contains the current parameters of the warping function.
 * \param p_up : 8-dim vector that contains the resulting parameters one level up.
 */
void vpTemplateTrackerWarpHomography::getParamPyramidUp(const vpColVector &p, vpColVector &p_up)
{
  p_up[0] = p[0];
  p_up[1] = p[1];
  p_up[2] = p[2] / 2.;
  p_up[3] = p[3];
  p_up[4] = p[4];
  p_up[5] = p[5] / 2.;
  p_up[6] = p[6] * 2.;
  p_up[7] = p[7] * 2.;
}

/*!
 * Compute the derivative of the image with relation to the warping function parameters.
 * \param v : Coordinate (along the image rows axis) of the point to consider in the image.
 * \param u : Coordinate (along the image columns axis) of the point to consider in the image.
 * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
 * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
 * \param dIdW : Resulting derivative matrix (image according to the warping function).
 */
void vpTemplateTrackerWarpHomography::getdW0(const int &v, const int &u, const double &dv, const double &du,
                                             double *dIdW)
{
  double u_du_ = u * du;
  double v_dv_ = v * dv;
  dIdW[0] = u_du_;
  dIdW[1] = u * dv;
  dIdW[2] = -u * (u_du_ + v_dv_);
  dIdW[3] = v * du;
  dIdW[4] = v_dv_;
  dIdW[5] = -v * (u_du_ + v_dv_);
  dIdW[6] = du;
  dIdW[7] = dv;
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
 * \param dIdW : Resulting 2-by-8 derivative matrix.
 */
void vpTemplateTrackerWarpHomography::getdWdp0(const int &v, const int &u, double *dIdW)
{
  double uv_ = u * v;
  dIdW[0] = u;
  dIdW[1] = 0;
  dIdW[2] = -u * u;
  dIdW[3] = v;
  dIdW[4] = 0;
  dIdW[5] = -uv_;
  dIdW[6] = 1.;
  dIdW[7] = 0;

  dIdW[8] = 0;
  dIdW[9] = u;
  dIdW[10] = -uv_;
  dIdW[11] = 0;
  dIdW[12] = v;
  dIdW[13] = -v * v;
  dIdW[14] = 0;
  dIdW[15] = 1.;
}

/*!
 * Compute the projection denominator (Z) used in x = X/Z and y = Y/Z.
 * \param X : Point with coordinates (u, v) to consider.
 * \param p : 8-dim vector that contains the current parameters of the warping function.
 *
 * \sa warpX(const vpColVector &, vpColVector &, const vpColVector &)
 * \sa warpX(const int &, const int &, double &, double &, const vpColVector &)
 * \sa dWarp(), dWarpCompo()
 */
void vpTemplateTrackerWarpHomography::computeDenom(vpColVector &X, const vpColVector &p)
{
  double value = (p[2] * X[0] + p[5] * X[1] + 1.);

  if (std::fabs(value) > std::numeric_limits<double>::epsilon()) {
    denom = (1. / value);
  }
  else {
    throw(vpTrackingException(vpTrackingException::fatalError,
                              "Division by zero in vpTemplateTrackerWarpHomography::computeDenom()"));
  }
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
 * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
 * \param p : 8-dim vector that contains the parameters of the transformation.
 */
void vpTemplateTrackerWarpHomography::warpX(const int &v1, const int &u1, double &v2, double &u2, const vpColVector &p)
{
  u2 = ((1. + p[0]) * u1 + p[3] * v1 + p[6]) * denom;
  v2 = (p[1] * u1 + (1. + p[4]) * v1 + p[7]) * denom;
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
 * \param p : 8-dim vector that contains the parameters of the transformation.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomography::warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  X2[0] = ((1 + p[0]) * X1[0] + p[3] * X1[1] + p[6]) * denom;
  X2[1] = (p[1] * X1[0] + (1 + p[4]) * X1[1] + p[7]) * denom;
}

/*!
 * Compute the derivative matrix of the warping function at point \f$X=(u,v)\f$ according to the model parameters:
 * \f[
 * \frac{\partial M}{\partial p}(X, p)
 * \f]
 * \param X : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param dM : Resulting warping model derivative returned as a 2-by-8 matrix.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomography::dWarp(const vpColVector &X, const vpColVector &, const vpColVector &,
                                            vpMatrix &dM)
{
  double u = X[0];
  double v = X[1];
  dM[0][0] = u * denom;
  dM[0][1] = 0;
  dM[0][2] = -u * X[0] * denom;
  dM[0][3] = v * denom;
  dM[0][4] = 0;
  dM[0][5] = -v * X[0] * denom;
  dM[0][6] = denom;

  dM[1][1] = u * denom;
  dM[1][2] = -u * X[1] * denom;
  dM[1][4] = v * denom;
  dM[1][5] = -v * X[1] * denom;
  dM[1][7] = denom;
}

/*!
 * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
 * \param X : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param p : 8-dim vector that contains the parameters of the warping function.
 * \param dwdp0 : 2-by-8 derivative matrix of the warping function according to
 * the initial warping function parameters (p=0).
 * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-8 matrix.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomography::dWarpCompo(const vpColVector &, const vpColVector &X, const vpColVector &p,
                                                 const double *dwdp0, vpMatrix &dM)
{
  double dwdx0, dwdx1;
  double dwdy0, dwdy1;

  dwdx0 = ((1. + p[0]) - X[0] * p[2]) * denom;
  dwdx1 = (p[1] - X[1] * p[2]) * denom;
  dwdy0 = (p[3] - X[0] * p[5]) * denom;
  dwdy1 = ((1. + p[4]) - X[1] * p[5]) * denom;
  for (unsigned int i = 0; i < nbParam; i++) {
    dM[0][i] = dwdx0 * dwdp0[i] + dwdy0 * dwdp0[i + nbParam];
    dM[1][i] = dwdx1 * dwdp0[i] + dwdy1 * dwdp0[i + nbParam];
  }
}

/*!
 * Warp a point X1 with the inverse transformation \f$M\f$.
 * \f[ X_2 = {\left({^1}M_2\right)}^{-1} \; X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates (u,v) of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates (u,v) of the warped point.
 * \param p : Parameters corresponding to the warping model \f${^1}M_2\f$.
 */
void vpTemplateTrackerWarpHomography::warpXInv(const vpColVector &X1, vpColVector &X2, const vpColVector &p)
{
  double value = (p[2] * X1[0] + p[5] * X1[1] + 1.);

  if (std::fabs(value) > std::numeric_limits<double>::epsilon()) {
    X2[0] = ((1 + p[0]) * X1[0] + p[3] * X1[1] + p[6]) / value;
    X2[1] = (p[1] * X1[0] + (1 + p[4]) * X1[1] + p[7]) / value;
  }
  else {
    throw(vpTrackingException(vpTrackingException::fatalError, "Division by zero in "
                              "vpTemplateTrackerWarpHomography::"
                              "warpXInv()"));
  }
}

/*!
 * Compute inverse of the warping transformation.
 * \param p : 8-dim vector that contains the parameters corresponding
 * to the transformation to inverse.
 * \param p_inv : 8-dim vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
 */
void vpTemplateTrackerWarpHomography::getParamInverse(const vpColVector &p, vpColVector &p_inv) const
{
  double h_00 = 1. + p[0];
  double h_10 = p[1];
  double h_20 = p[2];
  double h_01 = p[3];
  double h_11 = 1. + p[4];
  double h_21 = p[5];
  double h_02 = p[6];
  double h_12 = p[7];

  double h_inv_22 = (h_00 * h_11 - h_01 * h_10);

  if (std::fabs(h_inv_22) < std::numeric_limits<double>::epsilon()) {
    throw(vpException(vpException::fatalError, "Cannot get homography inverse parameters. Matrix determinant is 0."));
  }

  p_inv[0] = (h_11 - h_12 * h_21) / h_inv_22 - 1.;
  p_inv[3] = (h_02 * h_21 - h_01) / h_inv_22;
  p_inv[6] = (h_01 * h_12 - h_02 * h_11) / h_inv_22;

  p_inv[1] = (h_12 * h_20 - h_10) / h_inv_22;
  p_inv[4] = (h_00 - h_02 * h_20) / h_inv_22 - 1.;
  p_inv[7] = (h_02 * h_10 - h_00 * h_12) / h_inv_22;

  p_inv[2] = (h_10 * h_21 - h_11 * h_20) / h_inv_22;
  p_inv[5] = (h_01 * h_20 - h_00 * h_21) / h_inv_22;
}

/*!
 * Return the homography corresponding to the parameters.
 * \param p : 8-dim vector that contains the parameters corresponding
 * to the transformation to inverse.
 * \return Corresponding homography.
 */
vpHomography vpTemplateTrackerWarpHomography::getHomography(const vpColVector &p) const
{
  vpHomography H;
  H[0][0] = 1. + p[0];
  H[1][0] = p[1];
  H[2][0] = p[2];
  H[0][1] = p[3];
  H[1][1] = 1. + p[4];
  H[2][1] = p[5];
  H[0][2] = p[6];
  H[1][2] = p[7];
  H[2][2] = 1.;

  return H;
}

/*!
 * Compute the parameters corresponding to an homography.
 * \param[in] H : Homography
 * \param[out] p : 8-dim vector corresponding to the homography.
 */
void vpTemplateTrackerWarpHomography::getParam(const vpHomography &H, vpColVector &p) const
{
  p.resize(getNbParam(), false);
  p[0] = H[0][0] / H[2][2] - 1.;
  p[1] = H[1][0] / H[2][2];
  p[2] = H[2][0] / H[2][2];
  p[3] = H[0][1] / H[2][2];
  p[4] = H[1][1] / H[2][2] - 1.;
  p[5] = H[2][1] / H[2][2];
  p[6] = H[0][2] / H[2][2];
  p[7] = H[1][2] / H[2][2];
}

/*!
 * Compute the parameters corresponding to an homography as a 3-by-3 matrix.
 * \param[in] H : 3-by-3 matrix corresponding to an homography
 * \param[out] p : 8-dim vector corresponding to the homography.
 */
void vpTemplateTrackerWarpHomography::getParam(const vpMatrix &H, vpColVector &p) const
{
  p.resize(getNbParam(), false);
  p[0] = H[0][0] / H[2][2] - 1.;
  p[1] = H[1][0] / H[2][2];
  p[2] = H[2][0] / H[2][2];
  p[3] = H[0][1] / H[2][2];
  p[4] = H[1][1] / H[2][2] - 1.;
  p[5] = H[2][1] / H[2][2];
  p[6] = H[0][2] / H[2][2];
  p[7] = H[1][2] / H[2][2];
}

/*!
 * Compute the transformation resulting from the composition of two other transformations.
 * \param p1 : 8-dim vector that contains the parameters corresponding
 * to first transformation.
 * \param p2 : 8-dim vector that contains the parameters corresponding
 * to second transformation.
 * \param p12 : 8-dim vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
 */
void vpTemplateTrackerWarpHomography::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const
{
  double h1_00 = 1. + p1[0];
  double h1_10 = p1[1];
  double h1_20 = p1[2];
  double h1_01 = p1[3];
  double h1_11 = 1. + p1[4];
  double h1_21 = p1[5];
  double h1_02 = p1[6];
  double h1_12 = p1[7];

  double h2_00 = 1. + p2[0];
  double h2_10 = p2[1];
  double h2_20 = p2[2];
  double h2_01 = p2[3];
  double h2_11 = 1. + p2[4];
  double h2_21 = p2[5];
  double h2_02 = p2[6];
  double h2_12 = p2[7];

  double h12_22 = h1_20 * h2_02 + h1_21 * h2_12 + 1.;

  p12[0] = (h1_00 * h2_00 + h1_01 * h2_10 + h1_02 * h2_20) / h12_22 - 1.;
  p12[3] = (h1_00 * h2_01 + h1_01 * h2_11 + h1_02 * h2_21) / h12_22;
  p12[6] = (h1_00 * h2_02 + h1_01 * h2_12 + h1_02) / h12_22;

  p12[1] = (h1_10 * h2_00 + h1_11 * h2_10 + h1_12 * h2_20) / h12_22;
  p12[4] = (h1_10 * h2_01 + h1_11 * h2_11 + h1_12 * h2_21) / h12_22 - 1.;
  p12[7] = (h1_10 * h2_02 + h1_11 * h2_12 + h1_12) / h12_22;

  p12[2] = (h1_20 * h2_00 + h1_21 * h2_10 + h2_20) / h12_22;
  p12[5] = (h1_20 * h2_01 + h1_21 * h2_11 + h2_21) / h12_22;
}
END_VISP_NAMESPACE
