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
#include <visp3/tt/vpTemplateTrackerWarpHomographySL3.h>

BEGIN_VISP_NAMESPACE
// findWarp special a SL3 car methode additionnelle ne marche pas (la derivee
// n est calculable qu en p=0)
// => resout le probleme de maniere compositionnelle
/*!
 * Find the displacement/warping function parameters from a list of points.
 *
 * \param ut0 : Original u coordinates.
 * \param vt0 : Original v coordinates.
 * \param u : Warped u coordinates.
 * \param v : Warped v coordinates.
 * \param nb_pt : Number of points.
 * \param p : Resulting warping function parameters.
 */
void vpTemplateTrackerWarpHomographySL3::findWarp(const double *ut0, const double *vt0, const double *u,
                                                  const double *v, int nb_pt, vpColVector &p)
{
  vpColVector dp(nbParam);
  vpMatrix dW_(2, nbParam);
  vpMatrix dX(2, 1);
  vpMatrix H(nbParam, nbParam), HLM(nbParam, nbParam);
  vpMatrix G_(nbParam, 1);

  // vpMatrix *dW_ddp0=new vpMatrix[nb_pt];
  double **dW_ddp0 = new double *[(unsigned int)nb_pt];
  for (int i = 0; i < nb_pt; i++) {
    // dW_ddp0[i].resize(2,nbParam);
    dW_ddp0[i] = new double[2 * nbParam];
    // getdWdp0(vt0[i],ut0[i],dW_ddp0[i]);
    // std::cout<<"findWarp"<<v[i]<<","<<u[i]<<std::endl;
    getdWdp0(v[i], u[i], dW_ddp0[i]);
  }

  int cpt = 0;
  vpColVector X1(2);
  vpColVector fX1(2);
  vpColVector X2(2);
  double erreur = 0;
  double erreur_prec;
  double lambda = 0.00001;
  do {
    erreur_prec = erreur;
    H = 0;
    G_ = 0;
    erreur = 0;
    computeCoeff(p);
    for (int i = 0; i < nb_pt; i++) {
      X1[0] = ut0[i];
      X1[1] = vt0[i];
      computeDenom(X1, p);
      warpX(X1, fX1, p);
      // dWarpCompo(X1,fX1,p,dW_ddp0[i],dW);
      // dWarp(X1,fX1,p,dW);
      for (unsigned int ip = 0; ip < nbParam; ip++) {
        dW_[0][ip] = dW_ddp0[i][ip];
        dW_[1][ip] = dW_ddp0[i][ip + nbParam];
      }

      H += dW_.AtA();

      X2[0] = u[i];
      X2[1] = v[i];

      dX = X2 - fX1;
      G_ += dW_.t() * dX;

      erreur += ((u[i] - fX1[0]) * (u[i] - fX1[0]) + (v[i] - fX1[1]) * (v[i] - fX1[1]));
    }

    vpMatrix::computeHLM(H, lambda, HLM);
    try {
      dp = HLM.inverseByLU() * G_;
    }
    catch (const vpException &e) {
   // std::cout<<"Cannot inverse the matrix by LU "<<std::endl;
      throw(e);
    }
    pRondp(p, dp, p);

    cpt++;
    //  std::cout<<"erreur ="<<erreur<<std::endl;
  }
  // while((cpt<1500));
  while ((cpt < 150) && (sqrt((erreur_prec - erreur) * (erreur_prec - erreur)) > 1e-20));

  // std::cout<<"erreur apres transformation="<<erreur<<std::endl;
  for (int i = 0; i < nb_pt; i++)
    delete[] dW_ddp0[i];
  delete[] dW_ddp0;
}

/*!
 * Construct an homography SL3 model with 8 parameters initialized to zero.
 */
vpTemplateTrackerWarpHomographySL3::vpTemplateTrackerWarpHomographySL3() : G(), dGx(), A()
{
  nbParam = 8;
  G.resize(3, 3);
  dGx.resize(3, nbParam);

  A.resize(8);
  for (unsigned int i = 0; i < 8; i++) {
    A[i].resize(3, 3);
    A[i] = 0;
  }
  A[0][0][2] = 1;
  A[1][1][2] = 1;
  A[2][0][1] = 1;
  A[3][1][0] = 1;
  A[4][0][0] = 1;
  A[4][1][1] = -1;
  A[5][1][1] = -1;
  A[5][2][2] = 1;
  A[6][2][0] = 1;
  A[7][2][1] = 1;
}

// get the parameter corresponding to the lower level of a gaussian pyramid
// a refaire de facon analytique
/*!
 * Get the parameters of the warping function one level down
 * where image size is divided by two along the lines and the columns.
 * \param p : 8-dim vector that contains the current parameters of the warping function.
 * \param p_down : 8-dim vector that contains the resulting parameters one level down.
 */
void vpTemplateTrackerWarpHomographySL3::getParamPyramidDown(const vpColVector &p, vpColVector &p_down)
{
  double *u, *v;
  u = new double[4];
  v = new double[4];
  // u[0]=0;v[0]=0;u[1]=640;v[1]=0;u[2]=640;v[2]=480;u[3]=0;v[3]=480;
  u[0] = 0;
  v[0] = 0;
  u[1] = 160;
  v[1] = 0;
  u[2] = 160;
  v[2] = 120;
  u[3] = 0;
  v[3] = 120;
  double *u2, *v2;
  u2 = new double[4];
  v2 = new double[4];
  warp(u, v, 4, p, u2, v2);
  // p=0;findWarp(u,v,u2,v2,4,p);
  for (int i = 0; i < 4; i++) {
    u[i] = u[i] / 2.;
    v[i] = v[i] / 2.;
    u2[i] = u2[i] / 2.;
    v2[i] = v2[i] / 2.;
    // std::cout<<"recherche "<<u2[i]<<","<<v2[i]<<std::endl;
  }
  p_down = p;
  findWarp(u, v, u2, v2, 4, p_down);
  delete[] u;
  delete[] v;
  delete[] u2;
  delete[] v2;
}

/*!
 * Get the parameters of the warping function one level up
 * where image size is multiplied by two along the lines and the columns.
 * \param p : 8-dim vector that contains the current parameters of the warping function.
 * \param p_up : 8-dim vector that contains the resulting parameters one level up.
 */
void vpTemplateTrackerWarpHomographySL3::getParamPyramidUp(const vpColVector &p, vpColVector &p_up)
{
  double *u, *v;
  u = new double[4];
  v = new double[4];
  // u[0]=0;v[0]=0;u[1]=640;v[1]=0;u[2]=640;v[2]=480;u[3]=0;v[3]=480;
  u[0] = 0;
  v[0] = 0;
  u[1] = 160;
  v[1] = 0;
  u[2] = 160;
  v[2] = 120;
  u[3] = 0;
  v[3] = 120;
  // u[0]=40;v[0]=30;u[1]=160;v[1]=30;u[2]=160;v[2]=120;u[3]=40;v[3]=120;
  double *u2, *v2;
  u2 = new double[4];
  v2 = new double[4];

  // p_up=p;

  /*vpColVector ptest=pup;
  warp(u,v,4,ptest,u2,v2);
  for(int i=0;i<4;i++)
    std::cout<<"test "<<u2[i]<<","<<v2[i]<<std::endl;*/

  warp(u, v, 4, p, u2, v2);
  // p=0;findWarp(u,v,u2,v2,4,p);

  for (int i = 0; i < 4; i++) {
    u[i] = u[i] * 2.;
    v[i] = v[i] * 2.;
    u2[i] = u2[i] * 2.;
    v2[i] = v2[i] * 2.;
    /*std::cout<<"#########################################################################################"<<std::endl;
    std::cout<<"#########################################################################################"<<std::endl;
    std::cout<<"#########################################################################################"<<std::endl;
    std::cout<<"recherche "<<u2[i]<<","<<v2[i]<<std::endl;*/
  }
  findWarp(u, v, u2, v2, 4, p_up);

  delete[] u;
  delete[] v;
  delete[] u2;
  delete[] v2;
}

/*!
 * Compute the projection denominator (Z) used in x = X/Z and y = Y/Z.
 * \param X : Point with coordinates (u, v) to consider.
 *
 * \sa warpX(const vpColVector &, vpColVector &, const vpColVector &)
 * \sa warpX(const int &, const int &, double &, double &, const vpColVector &)
 * \sa dWarp(), dWarpCompo()
 */
void vpTemplateTrackerWarpHomographySL3::computeDenom(vpColVector &X, const vpColVector &)
{
  denom = X[0] * G[2][0] + X[1] * G[2][1] + G[2][2];
}

/*!
 * Compute the exponential of the homography matrix defined by the given
 * parameters.
 * \param p : Parameters of the SL3 homography warping function.
 */
void vpTemplateTrackerWarpHomographySL3::computeCoeff(const vpColVector &p)
{
  vpMatrix pA(3, 3);
  pA[0][0] = p[4];
  pA[0][1] = p[2];
  pA[0][2] = p[0];

  pA[1][0] = p[3];
  pA[1][1] = -p[4] - p[5];
  pA[1][2] = p[1];

  pA[2][0] = p[6];
  pA[2][1] = p[7];
  pA[2][2] = p[5];

  G = pA.expm();
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to warp.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the warped point.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomographySL3::warpX(const vpColVector &X1, vpColVector &X2, const vpColVector &)
{
  double u = X1[0], v = X1[1];
  X2[0] = (u * G[0][0] + v * G[0][1] + G[0][2]) / denom;
  X2[1] = (u * G[1][0] + v * G[1][1] + G[1][2]) / denom;
}

/*!
 * Warp point \f$X_1=(u_1,v_1)\f$ using the transformation model with parameters \f$p\f$.
 * \f[X_2 = {^2}M_1(p) * X_1\f]
 * \param v1 : Coordinate (along the image rows axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param u1 : Coordinate (along the image columns axis) of the point \f$X_1=(u_1,v_1)\f$ to warp.
 * \param v2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image rows axis.
 * \param u2 : Coordinate of the warped point \f$X_2=(u_2,v_2)\f$ along the image column axis.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomographySL3::warpX(const int &v1, const int &u1, double &v2, double &u2,
                                               const vpColVector &)
{
  u2 = (u1 * G[0][0] + v1 * G[0][1] + G[0][2]) / denom;
  v2 = (u1 * G[1][0] + v1 * G[1][1] + G[1][2]) / denom;
}

/*!
 * Return the homography corresponding to the parameters.
 * \return Corresponding homography.
 */
vpHomography vpTemplateTrackerWarpHomographySL3::getHomography() const
{
  vpHomography H;
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      H[i][j] = G[i][j];
  return H;
}

/*!
 * Compute the derivative matrix of the warping function at point \f$X=(u,v)\f$ according to the model parameters:
 * \f[
 * \frac{\partial M}{\partial p}(X, p)
 * \f]
 * \param X1 : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param X2 : 2-dim vector corresponding to the coordinates \f$(u_2, v_2)\f$ of the point to
 * consider in the derivative computation.
 * \param dM : Resulting warping model derivative returned as a 2-by-8 matrix.
 *
 * \sa computeDenom()
 */
void vpTemplateTrackerWarpHomographySL3::dWarp(const vpColVector &X1, const vpColVector &X2, const vpColVector &,
                                               vpMatrix &dM)
{
  vpMatrix dhdx(2, 3);
  dhdx = 0;
  dhdx[0][0] = 1. / denom;
  dhdx[1][1] = 1. / denom;
  dhdx[0][2] = -X2[0] / (denom);
  dhdx[1][2] = -X2[1] / (denom);
  dGx = 0;
  for (unsigned int i = 0; i < 3; i++) {
    dGx[i][0] = G[i][0];
    dGx[i][1] = G[i][1];
    dGx[i][2] = G[i][0] * X1[1];
    dGx[i][3] = G[i][1] * X1[0];
    dGx[i][4] = G[i][0] * X1[0] - G[i][1] * X1[1];
    dGx[i][5] = G[i][2] - G[i][1] * X1[1];
    dGx[i][6] = G[i][2] * X1[0];
    dGx[i][7] = G[i][2] * X1[1];
  }
  dM = dhdx * dGx;
}

/*!
 * Compute the derivative of the image with relation to the warping function parameters.
 * \param v : Coordinate (along the image rows axis) of the point to consider in the image.
 * \param u : Coordinate (along the image columns axis) of the point to consider in the image.
 * \param dv : Derivative on the v-axis (along the rows) of the point (u,v).
 * \param du : Derivative on the u-axis (along the columns) of the point (u,v).
 * \param dIdW : Resulting derivative matrix (image according to the warping function).
 */
void vpTemplateTrackerWarpHomographySL3::getdW0(const int &v, const int &u, const double &dv, const double &du,
                                                double *dIdW)
{
  vpMatrix dhdx(1, 3);
  dhdx = 0;
  dhdx[0][0] = du;
  dhdx[0][1] = dv;
  dhdx[0][2] = -u * du - v * dv;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * v;
    dGx[par][3] = G[par][1] * u;
    dGx[par][4] = G[par][0] * u - G[par][1] * v;
    dGx[par][5] = G[par][2] - G[par][1] * v;
    dGx[par][6] = G[par][2] * u;
    dGx[par][7] = G[par][2] * v;
  }

  for (unsigned int par = 0; par < nbParam; par++) {
    double res = 0;
    for (unsigned int par2 = 0; par2 < 3; par2++)
      res += dhdx[0][par2] * dGx[par2][par];
    dIdW[par] = res;
  }
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
void vpTemplateTrackerWarpHomographySL3::getdWdp0(const int &v, const int &u, double *dIdW)
{
  vpMatrix dhdx(2, 3);
  dhdx = 0;
  dhdx[0][0] = 1.;
  dhdx[1][1] = 1.;
  dhdx[0][2] = -u;
  dhdx[1][2] = -v;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * v;
    dGx[par][3] = G[par][1] * u;
    dGx[par][4] = G[par][0] * u - G[par][1] * v;
    dGx[par][5] = G[par][2] - G[par][1] * v;
    dGx[par][6] = G[par][2] * u;
    dGx[par][7] = G[par][2] * v;
  }
  vpMatrix dIdW_temp(2, nbParam);
  dIdW_temp = dhdx * dGx;

  for (unsigned int par = 0; par < nbParam; par++) {
    dIdW[par] = dIdW_temp[0][par];
    dIdW[par + nbParam] = dIdW_temp[1][par];
  }
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
void vpTemplateTrackerWarpHomographySL3::getdWdp0(const double &v, const double &u, double *dIdW)
{
  vpMatrix dhdx(2, 3);
  dhdx = 0;
  dhdx[0][0] = 1.;
  dhdx[1][1] = 1.;
  dhdx[0][2] = -u;
  dhdx[1][2] = -v;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * v;
    dGx[par][3] = G[par][1] * u;
    dGx[par][4] = G[par][0] * u - G[par][1] * v;
    dGx[par][5] = G[par][2] - G[par][1] * v;
    dGx[par][6] = G[par][2] * u;
    dGx[par][7] = G[par][2] * v;
  }
  vpMatrix dIdW_temp(2, nbParam);
  dIdW_temp = dhdx * dGx;

  for (unsigned int par = 0; par < nbParam; par++) {
    dIdW[par] = dIdW_temp[0][par];
    dIdW[par + nbParam] = dIdW_temp[1][par];
  }
}

/*!
 * Compute the compositionnal derivative matrix of the warping function according to the model parameters.
 * \param X : 2-dim vector corresponding to the coordinates \f$(u_1, v_1)\f$ of the point to
 * consider in the derivative computation.
 * \param dwdp0 : 2-by-8 derivative matrix of the warping function according to
 * the initial warping function parameters (p=0).
 * \param dM : Resulting warping model compositionnal derivative returned as a 2-by-8 matrix.
 *
 * \sa computeDenom()
 */

void vpTemplateTrackerWarpHomographySL3::dWarpCompo(const vpColVector &, const vpColVector &X, const vpColVector &,
                                                    const double *dwdp0, vpMatrix &dM)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dM[0][i] = denom * ((G[0][0] - X[0] * G[2][0]) * dwdp0[i] + (G[0][1] - X[0] * G[2][1]) * dwdp0[i + nbParam]);
    dM[1][i] = denom * ((G[1][0] - X[1] * G[2][0]) * dwdp0[i] + (G[1][1] - X[1] * G[2][1]) * dwdp0[i + nbParam]);
  }
}

/*!
 * Compute inverse of the warping transformation.
 * \param p : 8-dim vector that contains the parameters corresponding
 * to the transformation to inverse.
 * \param p_inv : 8-dim vector that contains the parameters of the inverse transformation \f$ {M(p)}^{-1}\f$.
 */
void vpTemplateTrackerWarpHomographySL3::getParamInverse(const vpColVector &p, vpColVector &p_inv) const { p_inv = -p; }

/*!
 * Compute the transformation resulting from the composition of two other transformations.
 * \param p1 : 8-dim vector that contains the parameters corresponding
 * to first transformation.
 * \param p2 : 8-dim vector that contains the parameters corresponding
 * to second transformation.
 * \param p12 : 8-dim vector that contains the resulting transformation \f$ p_{12} = p_1 \circ p_2\f$.
 */
void vpTemplateTrackerWarpHomographySL3::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &p12) const
{
  // vrai que si commutatif ...
  p12 = p1 + p2;
}
END_VISP_NAMESPACE
