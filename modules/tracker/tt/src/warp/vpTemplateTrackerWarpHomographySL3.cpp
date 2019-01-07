/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/tt/vpTemplateTrackerWarpHomographySL3.h>

// findWarp special a SL3 car methode additionnelle ne marche pas (la derivee
// n est calculable qu en p=0)
// => resout le probleme de maniere compositionnelle
void vpTemplateTrackerWarpHomographySL3::findWarp(const double *ut0, const double *vt0, const double *u,
                                                  const double *v, int nb_pt, vpColVector &p)
{
  // std::cout<<"findWarp OVERLOADE"<<std::endl;
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
    } catch (const vpException &e) {
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

vpTemplateTrackerWarpHomographySL3::~vpTemplateTrackerWarpHomographySL3() {}

// get the parameter corresponding to the lower level of a gaussian pyramid
// a refaire de facon analytique
void vpTemplateTrackerWarpHomographySL3::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
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
  pdown = p;
  findWarp(u, v, u2, v2, 4, pdown);
  delete[] u;
  delete[] v;
  delete[] u2;
  delete[] v2;
}

void vpTemplateTrackerWarpHomographySL3::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
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

  // pup=p;

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
  findWarp(u, v, u2, v2, 4, pup);

  delete[] u;
  delete[] v;
  delete[] u2;
  delete[] v2;
}

void vpTemplateTrackerWarpHomographySL3::computeDenom(vpColVector &vX, const vpColVector & /*ParamM*/)
{
  denom = vX[0] * G[2][0] + vX[1] * G[2][1] + G[2][2];
}

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

void vpTemplateTrackerWarpHomographySL3::warpX(const vpColVector &vX, vpColVector &vXres,
                                               const vpColVector & /*ParamM*/)
{
  double i = vX[1], j = vX[0];
  vXres[0] = (j * G[0][0] + i * G[0][1] + G[0][2]) / denom;
  vXres[1] = (j * G[1][0] + i * G[1][1] + G[1][2]) / denom;
}
void vpTemplateTrackerWarpHomographySL3::warpX(const int &i, const int &j, double &i2, double &j2,
                                               const vpColVector & /*ParamM*/)
{
  j2 = (j * G[0][0] + i * G[0][1] + G[0][2]) / denom;
  i2 = (j * G[1][0] + i * G[1][1] + G[1][2]) / denom;
}

vpHomography vpTemplateTrackerWarpHomographySL3::getHomography() const
{
  vpHomography H;
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      H[i][j] = G[i][j];
  return H;
}

void vpTemplateTrackerWarpHomographySL3::dWarp(const vpColVector &X1, const vpColVector &X2,
                                               const vpColVector & /*ParamM*/, vpMatrix &dW_)
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
  dW_ = dhdx * dGx;
}

/*calcul de di*dw(x,p0)/dp
 */
void vpTemplateTrackerWarpHomographySL3::getdW0(const int &i, const int &j, const double &dy, const double &dx,
                                                double *dIdW)
{
  vpMatrix dhdx(1, 3);
  dhdx = 0;
  dhdx[0][0] = dx;
  dhdx[0][1] = dy;
  dhdx[0][2] = -j * dx - i * dy;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * i;
    dGx[par][3] = G[par][1] * j;
    dGx[par][4] = G[par][0] * j - G[par][1] * i;
    dGx[par][5] = G[par][2] - G[par][1] * i;
    dGx[par][6] = G[par][2] * j;
    dGx[par][7] = G[par][2] * i;
  }

  for (unsigned int par = 0; par < nbParam; par++) {
    double res = 0;
    for (unsigned int par2 = 0; par2 < 3; par2++)
      res += dhdx[0][par2] * dGx[par2][par];
    dIdW[par] = res;
  }
}
/*calcul de dw(x,p0)/dp
 */

void vpTemplateTrackerWarpHomographySL3::getdWdp0(const int &i, const int &j, double *dIdW)
{
  vpMatrix dhdx(2, 3);
  dhdx = 0;
  dhdx[0][0] = 1.;
  dhdx[1][1] = 1.;
  dhdx[0][2] = -j;
  dhdx[1][2] = -i;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * i;
    dGx[par][3] = G[par][1] * j;
    dGx[par][4] = G[par][0] * j - G[par][1] * i;
    dGx[par][5] = G[par][2] - G[par][1] * i;
    dGx[par][6] = G[par][2] * j;
    dGx[par][7] = G[par][2] * i;
  }
  vpMatrix dIdW_temp(2, nbParam);
  dIdW_temp = dhdx * dGx;

  for (unsigned int par = 0; par < nbParam; par++) {
    dIdW[par] = dIdW_temp[0][par];
    dIdW[par + nbParam] = dIdW_temp[1][par];
  }
}
void vpTemplateTrackerWarpHomographySL3::getdWdp0(const double &i, const double &j, double *dIdW)
{
  vpMatrix dhdx(2, 3);
  dhdx = 0;
  dhdx[0][0] = 1.;
  dhdx[1][1] = 1.;
  dhdx[0][2] = -j;
  dhdx[1][2] = -i;
  G.eye();

  dGx = 0;
  for (unsigned int par = 0; par < 3; par++) {
    dGx[par][0] = G[par][0];
    dGx[par][1] = G[par][1];
    dGx[par][2] = G[par][0] * i;
    dGx[par][3] = G[par][1] * j;
    dGx[par][4] = G[par][0] * j - G[par][1] * i;
    dGx[par][5] = G[par][2] - G[par][1] * i;
    dGx[par][6] = G[par][2] * j;
    dGx[par][7] = G[par][2] * i;
  }
  vpMatrix dIdW_temp(2, nbParam);
  dIdW_temp = dhdx * dGx;

  for (unsigned int par = 0; par < nbParam; par++) {
    dIdW[par] = dIdW_temp[0][par];
    dIdW[par + nbParam] = dIdW_temp[1][par];
  }
}
/*compute dw=dw/dx*dw/dp
 */

void vpTemplateTrackerWarpHomographySL3::dWarpCompo(const vpColVector & /*X1*/, const vpColVector &X2,
                                                    const vpColVector & /*ParamM*/, const double *dwdp0, vpMatrix &dW_)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] = denom * ((G[0][0] - X2[0] * G[2][0]) * dwdp0[i] + (G[0][1] - X2[0] * G[2][1]) * dwdp0[i + nbParam]);
    dW_[1][i] = denom * ((G[1][0] - X2[1] * G[2][0]) * dwdp0[i] + (G[1][1] - X2[1] * G[2][1]) * dwdp0[i + nbParam]);
  }
}

void vpTemplateTrackerWarpHomographySL3::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  ParamMinv = -ParamM;
}
void vpTemplateTrackerWarpHomographySL3::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  // vrai que si commutatif ...
  pres = p1 + p2;
}
