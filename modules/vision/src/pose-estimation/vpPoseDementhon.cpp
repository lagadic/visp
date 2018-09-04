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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/

#include <visp3/core/vpMath.h>
#include <visp3/vision/vpPose.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0

#define SEUIL_RESIDUAL 0.001 /* avant 0.01 dans while du calculArbreDementhon */
#define ITER_MAX 50  /* max number of iterations for Demonthon's loop */  
/*!
\brief  Compute the pose using Dementhon approach for non planar objects
this is a direct implementation of the algorithm proposed by
Dementhon and Davis in their 1995 paper \cite Dementhon95.

*/

void vpPose::poseDementhonNonPlan(vpHomogeneousMatrix &cMo)
{
  vpPoint p0 = listP.front();

  c3d.clear();
  vpPoint P;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = (*it);
    P.set_oX(P.get_oX() - p0.get_oX());
    P.set_oY(P.get_oY() - p0.get_oY());
    P.set_oZ(P.get_oZ() - p0.get_oZ());
    c3d.push_back(P);
  }

  vpMatrix a(npt, 3);

  for (unsigned int i = 0; i < npt; i++) {
    a[i][0] = c3d[i].get_oX();
    a[i][1] = c3d[i].get_oY();
    a[i][2] = c3d[i].get_oZ();
  }

  // std::cout << a << std::endl ;
  // calcul a^T a
  vpMatrix ata;
  ata = a.t() * a;

  // calcul (a^T a)^-1 par decomposition LU
  vpMatrix ata1;
  ata1 = ata.pseudoInverse(1e-6); // InverseByLU() ;

  vpMatrix b;
  b = (a * ata1).t();

#if (DEBUG_LEVEL2)
  {
    std::cout << "a" << std::endl << a << std::endl;
    std::cout << "ata" << std::endl << ata << std::endl;
    std::cout << "ata1" << std::endl << ata1 << std::endl;
    std::cout << " ata*ata1" << std::endl << ata * ata1;
    std::cout << " b" << std::endl << (a * ata1).t();
  }
#endif

  int cpt = 0;
  double diff_err = 1.0;
  double err1 = 0.0;
  double Z0;
  vpColVector I, J, K;

  I.resize(3);
  J.resize(3);
  K.resize(3);

  vpColVector eps(npt);
  eps = 0;

  while ((cpt < ITER_MAX) && (diff_err > SEUIL_RESIDUAL)) {
    vpColVector xprim(npt);
    vpColVector yprim(npt);
    for (unsigned int i = 0; i < npt; i++) {
      xprim[i] = (1 + eps[i]) * c3d[i].get_x() - c3d[0].get_x();
      yprim[i] = (1 + eps[i]) * c3d[i].get_y() - c3d[0].get_y();
    }
    I = b * xprim;
    J = b * yprim;
    double normI = sqrt(I.sumSquare());
    double normJ = sqrt(J.sumSquare());
    I = I / normI;
    J = J / normJ;

    if (normI + normJ < 1e-10) {
      // vpERROR_TRACE(" normI+normJ = 0, division par zero " ) ;
      throw(vpException(vpException::divideByZeroError,
                        "Division by zero in Dementhon pose computation: normI+normJ = 0"));
    }
    K = vpColVector::cross(I, J);
    K.normalize(); // ajout FC

    Z0 = 2.0 / (normI + normJ);
    double err = 0.0;
    for (unsigned int i = 0; i < npt; i++) {
      eps[i] = (c3d[i].get_oX() * K[0] + c3d[i].get_oY() * K[1] + c3d[i].get_oZ() * K[2]) / Z0;
      err += eps[i]*eps[i];
    }
    err = sqrt(err/npt);
    diff_err = fabs(err - err1);
    err1 = err;
    //    printf("Dementhon it %d, diff erreur %lf\n",cpt,diff_err);
    cpt++;
  }
  //  K.normalize();  // FC inutile car mis plus haut
  J = vpColVector::cross(K, I);
  /*matrice de passage*/

  cMo[0][0] = I[0];
  cMo[0][1] = I[1];
  cMo[0][2] = I[2];
  cMo[0][3] = c3d[0].get_x() * Z0;

  cMo[1][0] = J[0];
  cMo[1][1] = J[1];
  cMo[1][2] = J[2];
  cMo[1][3] = c3d[0].get_y() * Z0;

  cMo[2][0] = K[0];
  cMo[2][1] = K[1];
  cMo[2][2] = K[2];
  cMo[2][3] = Z0;

  cMo[0][3] -= (p0.get_oX() * cMo[0][0] + p0.get_oY() * cMo[0][1] + p0.get_oZ() * cMo[0][2]);
  cMo[1][3] -= (p0.get_oX() * cMo[1][0] + p0.get_oY() * cMo[1][1] + p0.get_oZ() * cMo[1][2]);
  cMo[2][3] -= (p0.get_oX() * cMo[2][0] + p0.get_oY() * cMo[2][1] + p0.get_oZ() * cMo[2][2]);
}

#define EPS_DEM 0.001

static void calculRTheta(double s, double c, double &r, double &theta)
{
  if ((fabs(c) > EPS_DEM) || (fabs(s) > EPS_DEM)) {
    r = sqrt(sqrt(s * s + c * c));
    theta = atan2(s, c) / 2.0;
  } else {
    if (fabs(c) > fabs(s)) {
      r = fabs(c);
      if (c >= 0.0)
        theta = M_PI / 2;
      else
        theta = -M_PI / 2;
    } else {
      r = fabs(s);
      if (s >= 0.0)
        theta = M_PI / 4.0;
      else
        theta = -M_PI / 4.0;
    }
  }
}

static void calculSolutionDementhon(double xi0, double yi0, vpColVector &I, vpColVector &J, vpHomogeneousMatrix &cMo)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin (Dementhon.cc)CalculSolutionDementhon() " << std::endl;
#endif

  double normI, normJ, Z0;
  vpColVector k(3);

  // normalisation de I et J
  normI = sqrt(I.sumSquare());
  normJ = sqrt(J.sumSquare());

  Z0 = 2.0 / (normI + normJ);

  I /= normI;
  J /= normJ;

  k = vpColVector::cross(I, J); // k = I x J
  k.normalize();

  J = vpColVector::cross(k, I);

  // calcul de la matrice de passage
  cMo[0][0] = I[0];
  cMo[0][1] = I[1];
  cMo[0][2] = I[2];
  cMo[0][3] = xi0 * Z0;

  cMo[1][0] = J[0];
  cMo[1][1] = J[1];
  cMo[1][2] = J[2];
  cMo[1][3] = yi0 * Z0;

  cMo[2][0] = k[0];
  cMo[2][1] = k[1];
  cMo[2][2] = k[2];
  cMo[2][3] = Z0;

#if (DEBUG_LEVEL1)
  std::cout << "end (Dementhon.cc)CalculSolutionDementhon() " << std::endl;
#endif
}

int vpPose::calculArbreDementhon(vpMatrix &b, vpColVector &U, vpHomogeneousMatrix &cMo)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::CalculArbreDementhon() " << std::endl;
#endif

  int erreur = 0;

  // test if all points are in front of the camera
  for (unsigned int i = 0; i < npt; i++) {
    double z;
    z = cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ() + cMo[2][3];
    if (z <= 0.0) {
      erreur = -1;
      /* FC et il faut sortir de la fonction... rajout return. OK ?*/
      return erreur;
    }
  }
  if (erreur == 0) {  /* FC du coup test inutile */
 
   vpColVector xi(npt);
   vpColVector yi(npt);
   for (unsigned int i = 0; i < npt; i++) {
      xi[i] = c3d[i].get_x();
      yi[i] = c3d[i].get_y();
    }

    unsigned int cpt = 0;
    double smin = sqrt(computeResidualDementhon(cMo) / npt);
    double smin_old = vpMath::maximum((double) (2.0 *SEUIL_RESIDUAL),2.0*smin);

    /* FC modif seuil 0.01 à 0.001 */
    while ((cpt < ITER_MAX) && (smin_old > SEUIL_RESIDUAL) && (smin < smin_old)) {
      double r, theta, s1, s2;
      vpColVector eps(npt);
      vpColVector I0(3);
      vpColVector J0(3);
      vpColVector I(3);
      vpColVector J(3);
      vpHomogeneousMatrix cMo1, cMo2, cMo_old;

      smin_old = smin;
      cMo_old = cMo;

      unsigned int k = 0;
      for (unsigned int i = 0; i < npt; i++) {
        if (k != 0) { // On ne prend pas le 1er point
          eps[k] =
            (cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ()) / cMo[2][3];
        }
        k++;
      }
      I0 = 0;
      J0 = 0;

      for (unsigned int i = 1; i < npt; i++) {
        double s = (1.0 + eps[i]) * xi[i] - xi[0];
        I0[0] += b[0][i - 1] * s;
        I0[1] += b[1][i - 1] * s;
        I0[2] += b[2][i - 1] * s;
        s = (1.0 + eps[i]) * yi[i] - yi[0];
        J0[0] += b[0][i - 1] * s;
        J0[1] += b[1][i - 1] * s;
        J0[2] += b[2][i - 1] * s;
      }

      double s = -2.0 * (vpColVector::dotProd(I0, J0));
      double c = J0.sumSquare() - I0.sumSquare();

      calculRTheta(s, c, r, theta);
      double co = cos(theta);
      double si = sin(theta);

      /* 1ere branche	*/
      I = I0 + U * r * co;
      J = J0 + U * r * si;

#if (DEBUG_LEVEL3)
      {
        std::cout << "I " << I.t();
        std::cout << "J " << J.t();
      }
#endif

      calculSolutionDementhon(xi[0], yi[0], I, J, cMo1);
      s1 = sqrt(computeResidualDementhon(cMo1) / npt);
#if (DEBUG_LEVEL3)
      std::cout << "cMo1 " << std::endl << cMo1 << std::endl;
#endif

      /* 2eme branche	*/
      I = I0 - U * r * co;
      J = J0 - U * r * si;
#if (DEBUG_LEVEL3)
      {
        std::cout << "I " << I.t();
        std::cout << "J " << J.t();
      }
#endif

      calculSolutionDementhon(xi[0], yi[0], I, J, cMo2);
      s2 = sqrt(computeResidualDementhon(cMo2) / npt);
#if (DEBUG_LEVEL3)
      std::cout << "cMo2 " << std::endl << cMo2 << std::endl;
#endif

      if (s1 <= s2) {
        smin = s1;
        cMo = cMo1;
      } else {
        smin = s2;
        cMo = cMo2;        
      }
      if (smin > smin_old) {
#if (DEBUG_LEVEL2)
        std::cout << "Divergence " << std::endl;
#endif
        cMo = cMo_old;
      }
#if (DEBUG_LEVEL2)
      {
        std::cout << "s1 = " << s1 << std::endl;
        std::cout << "s2 = " << s2 << std::endl;
        std::cout << "smin = " << smin << std::endl;
        std::cout << "smin_old = " << smin_old << std::endl;
      }
#endif
      cpt++;
    }
  }
#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::CalculArbreDementhon() return " << erreur << std::endl;
#endif

  return erreur;
}

/*!
\brief  Compute the pose using Dementhon approach for planar objects
this is a direct implementation of the algorithm proposed by
Dementhon in his PhD.

\author Francois Chaumette (simplified by Eric Marchand)
*/

void vpPose::poseDementhonPlan(vpHomogeneousMatrix &cMo)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin CCalculPose::PoseDementhonPlan()" << std::endl;
#endif

  unsigned int i, j, k;

  vpPoint p0 = listP.front();

  vpPoint P;
  c3d.clear();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;
    P.set_oX(P.get_oX() - p0.get_oX());
    P.set_oY(P.get_oY() - p0.get_oY());
    P.set_oZ(P.get_oZ() - p0.get_oZ());
    c3d.push_back(P);
  }

  vpMatrix a;
  try {
    a.resize(npt - 1, 3);
  } catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }

  for (i = 1; i < npt; i++) {
    a[i - 1][0] = c3d[i].get_oX();
    a[i - 1][1] = c3d[i].get_oY();
    a[i - 1][2] = c3d[i].get_oZ();
  }

  // calcul a^T a
  vpMatrix ata;
  ata = a.t() * a;

  vpMatrix ata_sav;
  ata_sav = ata;

#if (DEBUG_LEVEL2)
  {
    std::cout << "a" << std::endl << a << std::endl;
    std::cout << "ata" << std::endl << ata << std::endl;
  }
#endif

  // calcul (a^T a)^-1
  vpMatrix ata1(ata.getRows(), ata.getCols());
  vpMatrix v(ata.getRows(), ata.getCols());
  vpColVector sv(ata.getRows());
  //  ata1 = ata.i() ;
  unsigned int imin = 0;
  double s = 0.0;

  // calcul de ata^-1
  ata.svd(sv, v);

  unsigned int nc = sv.getRows();
  for (i = 0; i < nc; i++)
    if (sv[i] > s)
      s = sv[i];

  s *= 0.0002;
  int irank = 0;
  for (i = 0; i < nc; i++)
    if (sv[i] > s)
      irank++;

  double svm = 100.0;
  for (i = 0; i < nc; i++) {
    if (sv[i] < svm) {
      imin = i;
      svm = sv[i];
    }
  }

#if (DEBUG_LEVEL2)
  {
    std::cout << "rang: " << irank << std::endl;
    ;
    std::cout << "imin = " << imin << std::endl;
    std::cout << "sv " << sv.t() << std::endl;
  }
#endif

  for (i = 0; i < ata.getRows(); i++) {
    for (j = 0; j < ata.getCols(); j++) {
      ata1[i][j] = 0.0;
      for (k = 0; k < nc; k++)
        if (sv[k] > s)
          ata1[i][j] += ((v[i][k] * ata[j][k]) / sv[k]);
    }
  }

  vpMatrix b; // b=(at a)^-1*at
  b = ata1 * a.t();

  // calcul de U
  vpColVector U(3);
  U = ata.getCol(imin);

#if (DEBUG_LEVEL2)
  {
    std::cout << "a" << std::endl << a << std::endl;
    std::cout << "ata" << std::endl << ata_sav << std::endl;
    std::cout << "ata1" << std::endl << ata1 << std::endl;
    std::cout << "ata1*ata" << std::endl << ata1 * ata_sav;
    std::cout << "b" << std::endl << b;
    std::cout << "U " << U.t() << std::endl;
  }
#endif

  vpColVector xi(npt);
  vpColVector yi(npt);
  // calcul de la premiere solution
  for (i = 0; i < npt; i++) {
    xi[i] = c3d[i].get_x();
    yi[i] = c3d[i].get_y();
  }

  vpColVector I0(3);
  I0 = 0;
  vpColVector J0(3);
  J0 = 0;
  vpColVector I(3);
  vpColVector J(3);

  for (i = 1; i < npt; i++) {
    I0[0] += b[0][i - 1] * (xi[i] - xi[0]);
    I0[1] += b[1][i - 1] * (xi[i] - xi[0]);
    I0[2] += b[2][i - 1] * (xi[i] - xi[0]);

    J0[0] += b[0][i - 1] * (yi[i] - yi[0]);
    J0[1] += b[1][i - 1] * (yi[i] - yi[0]);
    J0[2] += b[2][i - 1] * (yi[i] - yi[0]);
  }

#if (DEBUG_LEVEL2)
  {
    std::cout << "I0 " << I0.t();
    std::cout << "J0 " << J0.t();
  }
#endif

  s = -2.0 * vpColVector::dotProd(I0, J0);
  double c = J0.sumSquare() - I0.sumSquare();

  double r, theta, si, co;
  calculRTheta(s, c, r, theta);
  co = cos(theta);
  si = sin(theta);

  // calcul de la premiere solution
  I = I0 + U * r * co;
  J = J0 + U * r * si;

  vpHomogeneousMatrix cMo1f;
  calculSolutionDementhon(xi[0], yi[0], I, J, cMo1f);
#ifdef DEBUG_LEVEL2
  /* ajout FC */
   double res1 = sqrt(computeResidualDementhon(cMo1f)/npt);
   std::cout << "cMo Dementhon 1 : res " << res1  << std::endl << cMo1f  << std::endl; 
   vpThetaUVector erc;
    cMo1f.extract(erc);
    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
  /* fin ajout FC */
#endif

  int erreur1 = calculArbreDementhon(b, U, cMo1f);
/* ajout FC */
#ifdef DEBUG_LEVEL2
   res1 = sqrt(computeResidualDementhon(cMo1f)/npt);
   std::cout << "cMo Dementhon 1f : erreur1 " << erreur1 << " res 1 " << res1 << std::endl << cMo1f  << std::endl; 
    cMo1f.extract(erc);
    std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
#endif
  /* fin ajout FC */

  // calcul de la deuxieme solution
  I = I0 - U * r * co;
  J = J0 - U * r * si;

  vpHomogeneousMatrix cMo2f;
  calculSolutionDementhon(xi[0], yi[0], I, J, cMo2f);
  /* ajout FC */
  // double res2 = sqrt(computeResidualDementhon(cMo2f)/npt);
  // std::cout << "cMo Dementhon 2 res : " << res2 << std::endl << cMo2f  << std::endl; 
  //  cMo2f.extract(erc);
  //  std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl; 
 /* fin ajout FC */

  int erreur2 = calculArbreDementhon(b, U, cMo2f);
  /* ajout FC */
  // res2 = sqrt(computeResidualDementhon(cMo2f)/npt);
  // std::cout << "cMo Dementhon 2f : : erreur2 " << erreur2 << " res 2 " << res2 << std::endl << cMo2f  << std::endl; 
  //    cMo2f.extract(erc);
  //  std::cout << "Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl; 
/* fin ajout FC */

  if ((erreur1 == 0) && (erreur2 == -1))
    cMo = cMo1f;
  if ((erreur1 == -1) && (erreur2 == 0))
    cMo = cMo2f;
  if ((erreur1 == 0) && (erreur2 == 0)) {
    double s1 = computeResidualDementhon(cMo1f);
    double s2 = computeResidualDementhon(cMo2f);

    if (s1 <= s2)
      cMo = cMo1f;
    else
      cMo = cMo2f;
  }

  cMo[0][3] -= p0.get_oX() * cMo[0][0] + p0.get_oY() * cMo[0][1] + p0.get_oZ() * cMo[0][2];
  cMo[1][3] -= p0.get_oX() * cMo[1][0] + p0.get_oY() * cMo[1][1] + p0.get_oZ() * cMo[1][2];
  cMo[2][3] -= p0.get_oX() * cMo[2][0] + p0.get_oY() * cMo[2][1] + p0.get_oZ() * cMo[2][2];

#if (DEBUG_LEVEL1)
  std::cout << "end CCalculPose::PoseDementhonPlan()" << std::endl;
#endif
}

#undef EPS_DEM

/*!
\brief Compute and return the residual expressed in meter for the pose matrix
'pose'.

\param cMo : the matrix that defines the pose to be tested.

\return the value of the residual in meter^2
*/
double vpPose::computeResidualDementhon(const vpHomogeneousMatrix &cMo)
{
  double residual_ = 0;

  residual_ = 0;
  for (unsigned int i = 0; i < npt; i++) {

    double X = c3d[i].get_oX() * cMo[0][0] + c3d[i].get_oY() * cMo[0][1] + c3d[i].get_oZ() * cMo[0][2] + cMo[0][3];
    double Y = c3d[i].get_oX() * cMo[1][0] + c3d[i].get_oY() * cMo[1][1] + c3d[i].get_oZ() * cMo[1][2] + cMo[1][3];
    double Z = c3d[i].get_oX() * cMo[2][0] + c3d[i].get_oY() * cMo[2][1] + c3d[i].get_oZ() * cMo[2][2] + cMo[2][3];

    double x = X / Z;
    double y = Y / Z;

    residual_ += vpMath::sqr(x - c3d[i].get_x()) + vpMath::sqr(y - c3d[i].get_y());
  }
  return residual_;
}

#undef SEUIL_RESIDUAL
#undef ITER_MAX
#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3
