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

#define SEUIL_RESIDUAL 0.0001 /* avant 0.01 dans while du calculArbreDementhon */
#define EPS_DEM 0.001

#define ITER_MAX 30  /* max number of iterations for Demonthon's loop */

static void calculSolutionDementhon(vpColVector &I4, vpColVector &J4, vpHomogeneousMatrix &cMo)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin (Dementhon.cc)CalculSolutionDementhon() " << std::endl;
#endif

  // norm of the 3 first components of I4 and J4
  double normI3 = sqrt(I4[0]*I4[0] + I4[1]*I4[1] + I4[2]*I4[2]);
  double normJ3 = sqrt(J4[0]*J4[0] + J4[1]*J4[1] + J4[2]*J4[2]);

  if ((normI3 < 1e-10) || (normJ3 < 1e-10)) {
    // vpERROR_TRACE(" normI+normJ = 0, division par zero " ) ;
    throw(vpException(vpException::divideByZeroError,
                      "Division by zero in Dementhon pose computation: normI or normJ = 0"));
  }

  double  Z0 = 2.0 / (normI3 + normJ3);

  vpColVector I3(3), J3(3), K3(3);
  for (unsigned int i=0; i<3; i++) {
    I3[i] = I4[i] / normI3;
    J3[i] = J4[i] / normJ3;
  }

  K3 = vpColVector::cross(I3, J3); // k = I x J
  K3.normalize();

  J3 = vpColVector::cross(K3, I3);

  // calcul de la matrice de passage
  cMo[0][0] = I3[0];
  cMo[0][1] = I3[1];
  cMo[0][2] = I3[2];
  cMo[0][3] = I4[3] * Z0;

  cMo[1][0] = J3[0];
  cMo[1][1] = J3[1];
  cMo[1][2] = J3[2];
  cMo[1][3] = J4[3] * Z0;

  cMo[2][0] = K3[0];
  cMo[2][1] = K3[1];
  cMo[2][2] = K3[2];
  cMo[2][3] = Z0;

#if (DEBUG_LEVEL1)
  std::cout << "end (Dementhon.cc)CalculSolutionDementhon() " << std::endl;
#endif
}

/*!
  Compute the pose using Dementhon approach for non planar objects.
  This is a direct implementation of the algorithm proposed by
  Dementhon and Davis in their 1995 paper \cite Dementhon95.
*/
void vpPose::poseDementhonNonPlan(vpHomogeneousMatrix &cMo)
{
  vpPoint P;
  double cdg[3];
  /* compute the cog of the 3D points */
  cdg[0] = cdg[1] = cdg[2] = 0.0;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = (*it);
    cdg[0] += P.get_oX();
    cdg[1] += P.get_oY();
    cdg[2] += P.get_oZ();
  }
  for (unsigned int i=0; i<3;i++) cdg[i] /= npt;
  //  printf("cdg : %lf %lf %lf\n", cdg[0], cdg[1],cdg[2]);

  c3d.clear();
  /* translate the 3D points wrt cog */
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = (*it);
    P.set_oX(P.get_oX() - cdg[0]);
    P.set_oY(P.get_oY() - cdg[1]);
    P.set_oZ(P.get_oZ() - cdg[2]);
    c3d.push_back(P);
  }

  vpMatrix A(npt, 4), Ap;

  for (unsigned int i = 0; i < npt; i++) {
    A[i][0] = c3d[i].get_oX();
    A[i][1] = c3d[i].get_oY();
    A[i][2] = c3d[i].get_oZ();
    A[i][3] = 1.0;
  }
  Ap = A.pseudoInverse();

#if (DEBUG_LEVEL2)
  {
    std::cout << "A" << std::endl << A << std::endl;
    std::cout << "A^+" << std::endl << Ap << std::endl;
  }
#endif

  vpColVector xprim(npt);
  vpColVector yprim(npt);

  for (unsigned int i = 0; i < npt; i++) {
    xprim[i] = c3d[i].get_x();
    yprim[i] = c3d[i].get_y();
  }
  vpColVector I4(4), J4(4);

  I4 = Ap * xprim;
  J4 = Ap * yprim;

  calculSolutionDementhon(I4, J4, cMo);

  int erreur = 0;
  for (unsigned int i = 0; i < npt; i++) {
    double z;
    z = cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ() + cMo[2][3];
    if (z <= 0.0)  erreur = -1;
  }
  if (erreur == -1) {
    throw(vpException(vpException::fatalError, "End of Dementhon since z < 0 for both solutions at the beginning"));
  }
  int cpt = 0;
  double res = sqrt(computeResidualDementhon(cMo) / npt);
  double res_old = 2.0*res;

  // In his paper, Dementhon suggests to use the difference
  // between 2 successive norm of eps. We prefer to use the mean of the
  // residuals in the image
  while ((cpt < ITER_MAX) && (res > SEUIL_RESIDUAL) && (res < res_old)) {

    vpHomogeneousMatrix cMo_old;
    res_old = res;
    cMo_old = cMo;

    for (unsigned int i = 0; i < npt; i++) {
      double eps = (cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ()) / cMo[2][3];

      xprim[i] = (1.0 + eps) * c3d[i].get_x();
      yprim[i] = (1.0 + eps) * c3d[i].get_y();
    }
    I4 = Ap * xprim;
    J4 = Ap * yprim;

    calculSolutionDementhon(I4, J4, cMo);
    res = sqrt(computeResidualDementhon(cMo) / npt);
    for (unsigned int i = 0; i < npt; i++) {
      double z;
      z = cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ() + cMo[2][3];
      if (z <= 0.0)  erreur = -1;
    }
    if (erreur == -1) {
      cMo = cMo_old;
      res = res_old; // to leave the while loop
#if (DEBUG_LEVEL3)
      std::cout  << "Pb z < 0 with cMo in Dementhon's loop" << std::endl;
#endif
    }
#if (DEBUG_LEVEL3)
    vpThetaUVector erc;
    cMo.extract(erc);
    std::cout << "it = " << cpt << " residu = " << res << " Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
#endif
    if (res > res_old) {
#if (DEBUG_LEVEL3)
      std::cout << "Divergence : res = " << res << " res_old = " << res_old << std::endl;
#endif
      cMo = cMo_old;
    }
    cpt++;
  }
  // go back to the initial frame
  cMo[0][3] -= (cdg[0] * cMo[0][0] + cdg[1] * cMo[0][1] + cdg[2] * cMo[0][2]);
  cMo[1][3] -= (cdg[0] * cMo[1][0] + cdg[1] * cMo[1][1] + cdg[2] * cMo[1][2]);
  cMo[2][3] -= (cdg[0] * cMo[2][0] + cdg[1] * cMo[2][1] + cdg[2] * cMo[2][2]);
}


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


static void calculTwoSolutionsDementhonPlan(vpColVector &I04, vpColVector &J04, vpColVector &U, vpHomogeneousMatrix &cMo1, vpHomogeneousMatrix &cMo2)
{
  vpColVector I0(3), J0(3);
  for (unsigned int i=0; i<3; i++) {
    I0[i] = I04[i];
    J0[i] = J04[i];
  }
  double s = -2.0 * vpColVector::dotProd(I0, J0);
  double c = J0.sumSquare() - I0.sumSquare();

  double r, theta;
  calculRTheta(s, c, r, theta);
  double co = cos(theta);
  double si = sin(theta);

  // calcul de la premiere solution
  vpColVector I(4),J(4);
  I = I04 + U * r * co;
  J = J04 + U * r * si;

#if (DEBUG_LEVEL2)
  {
    std::cout << "I0 " << I04.t() << std::endl;
    std::cout << "J0 " << J04.t() << std::endl;
    std::cout << "I1 " << I.t() << std::endl;
    std::cout << "J1 " << J.t() << std::endl;
  }
#endif
  calculSolutionDementhon(I, J, cMo1);

  // calcul de la deuxieme solution
  I = I04 - U * r * co;
  J = J04 - U * r * si;
#if (DEBUG_LEVEL2)
  {
    std::cout << "I2 " << I.t() << std::endl;
    std::cout << "J2 " << J.t() << std::endl;
  }
#endif
  calculSolutionDementhon(I, J, cMo2);
}


/*!
  Return 0 if success, -1 if failure.
 */
int vpPose::calculArbreDementhon(vpMatrix &Ap, vpColVector &U, vpHomogeneousMatrix &cMo)
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
      return erreur;
    }
  }

  unsigned int cpt = 0;
  double res_min = sqrt(computeResidualDementhon(cMo)/npt);
  double res_old = 2.0*res_min;

  /* FC modif SEUIL_RESIDUAL 0.01 a 0.001 */
  while ((cpt < ITER_MAX) && (res_min > SEUIL_RESIDUAL) && (res_min < res_old)) {
    vpHomogeneousMatrix cMo1, cMo2, cMo_old;

    res_old = res_min;
    cMo_old = cMo;

    vpColVector xprim(npt),yprim(npt);
    for (unsigned int i = 0; i < npt; i++) {
      double eps = (cMo[2][0] * c3d[i].get_oX() + cMo[2][1] * c3d[i].get_oY() + cMo[2][2] * c3d[i].get_oZ()) / cMo[2][3];

      xprim[i] = (1.0 + eps)*c3d[i].get_x();
      yprim[i] = (1.0 + eps)*c3d[i].get_y();
    }

    vpColVector I04(4), J04(4);
    I04 = Ap * xprim;
    J04 = Ap * yprim;

    calculTwoSolutionsDementhonPlan(I04,J04,U,cMo1,cMo2);

    // test if all points are in front of the camera for cMo1 and cMo2
    int erreur1 = 0;
    int erreur2 = 0;
    for (unsigned int i = 0; i < npt; i++) {
      double z;
      z = cMo1[2][0] * c3d[i].get_oX() + cMo1[2][1] * c3d[i].get_oY() + cMo1[2][2] * c3d[i].get_oZ() + cMo1[2][3];
      if (z <= 0.0)  erreur1 = -1;
      z = cMo2[2][0] * c3d[i].get_oX() + cMo2[2][1] * c3d[i].get_oY() + cMo2[2][2] * c3d[i].get_oZ() + cMo2[2][3];
      if (z <= 0.0)  erreur2 = -1;
    }

    if ((erreur1 == -1) && (erreur2 == -1)) {
      cMo = cMo_old;
#if (DEBUG_LEVEL3)
      std::cout  << " End of loop since z < 0 for both solutions" << std::endl;
#endif
      break; // outside of while due to z < 0
    }
    if ((erreur1 == 0) && (erreur2 == -1)) {
      cMo = cMo1;
      res_min = sqrt(computeResidualDementhon(cMo)/npt);
    }
    if ((erreur1 == -1) && (erreur2 == 0)) {
      cMo = cMo2;
      res_min = sqrt(computeResidualDementhon(cMo)/npt);
    }
    if ((erreur1 == 0) && (erreur2 == 0)) {
      double res1 = sqrt(computeResidualDementhon(cMo1)/npt);
      double res2 = sqrt(computeResidualDementhon(cMo2)/npt);
      if (res1 <= res2) {
        res_min = res1;
        cMo = cMo1;
      } else {
        res_min = res2;
        cMo = cMo2;
      }
    }

#if (DEBUG_LEVEL3)
    if (erreur1 == 0) {
      double s = sqrt(computeResidualDementhon(cMo1)/npt);
      vpThetaUVector erc;
      cMo1.extract(erc);
      std::cout  << "it = " << cpt << " cMo1 : residu: " << s << " Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
    }
    else std::cout  << "Pb z < 0 with cMo1" << std::endl;

    if (erreur2 == 0) {
      double s = sqrt(computeResidualDementhon(cMo2)/npt);
      vpThetaUVector erc;
      cMo2.extract(erc);
      std::cout << "it = " << cpt << " cMo2 : residu: " << s << " Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
    }
    else std::cout  << "Pb z < 0 with cMo2" << std::endl;
#endif

    if (res_min > res_old) {
#if (DEBUG_LEVEL3)
      std::cout << "Divergence : res_min = " << res_min << " res_old = " << res_old << std::endl;
#endif
      cMo = cMo_old;
    }
    cpt++;
  }   /* end of while */

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

  vpPoint P;
  double cdg[3];
  /* compute the cog of the 3D points */
  cdg[0] = cdg[1] = cdg[2] = 0.0;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = (*it);
    cdg[0] += P.get_oX();
    cdg[1] += P.get_oY();
    cdg[2] += P.get_oZ();
  }
  for (unsigned int i=0; i<3;i++) cdg[i] /= npt;
  //  printf("cdg : %lf %lf %lf\n", cdg[0], cdg[1],cdg[2]);

  c3d.clear();
  /* translate the 3D points wrt cog */
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = (*it);
    P.set_oX(P.get_oX() - cdg[0]);
    P.set_oY(P.get_oY() - cdg[1]);
    P.set_oZ(P.get_oZ() - cdg[2]);
    c3d.push_back(P);
  }

  vpMatrix A(npt,4);

  for (unsigned int i = 0; i < npt; i++) {
    A[i][0] = c3d[i].get_oX();
    A[i][1] = c3d[i].get_oY();
    A[i][2] = c3d[i].get_oZ();
    A[i][3] = 1.0;
  }
  vpColVector sv;
  vpMatrix Ap,imA, imAt, kAt;
  int irank = A.pseudoInverse(Ap, sv, 1.e-6, imA, imAt, kAt);
  if (irank != 3) {
    throw(vpException(vpException::fatalError, "In Dementhon planar, rank is not 3"));
  }
  // calcul de U
  vpColVector U(4);
  for (unsigned int i = 0; i<4; i++) {
    U[i] = kAt[0][i];
  }
#if (DEBUG_LEVEL2)
  {
    std::cout << "A" << std::endl << A << std::endl;
    std::cout << "A^+" << std::endl << Ap << std::endl;
    std::cout << "U^T = " << U.t() << std::endl;
  }
#endif

  vpColVector xi(npt);
  vpColVector yi(npt);

  for (unsigned int i = 0; i < npt; i++) {
    xi[i] = c3d[i].get_x();
    yi[i] = c3d[i].get_y();
  }

  vpColVector I04(4), J04(4);
  I04 = Ap * xi;
  J04 = Ap * yi;

  vpHomogeneousMatrix cMo1,cMo2;
  calculTwoSolutionsDementhonPlan(I04,J04,U,cMo1,cMo2);

#if DEBUG_LEVEL3
  double res = sqrt(computeResidualDementhon(cMo1)/npt);
  vpThetaUVector erc;
  cMo1.extract(erc);
  std::cout << "cMo Start Tree 1 : res " << res << " Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
  res = sqrt(computeResidualDementhon(cMo2)/npt);
  cMo2.extract(erc);
  std::cout << "cMo Start Tree 2 : res " << res << " Theta U rotation: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1]) << " " << vpMath::deg(erc[2]) << std::endl;
#endif

  int erreur1 = calculArbreDementhon(Ap, U, cMo1);
  int erreur2 = calculArbreDementhon(Ap, U, cMo2);

  if ((erreur1 == -1)  && (erreur2 == -1)) {
    throw(vpException(vpException::fatalError, "Error in Dementhon planar: z < 0 with Start Tree 1 and Start Tree 2..."));
  }
  if ((erreur1 == 0) && (erreur2 == -1))
    cMo = cMo1;
  if ((erreur1 == -1) && (erreur2 == 0))
    cMo = cMo2;
  if ((erreur1 == 0) && (erreur2 == 0)) {
    double s1 = computeResidualDementhon(cMo1);
    double s2 = computeResidualDementhon(cMo2);

    if (s1 <= s2)
      cMo = cMo1;
    else
      cMo = cMo2;

#if DEBUG_LEVEL3
    if (erreur1 == -1) std::cout << "Pb z < 0 with Start Tree 1" << std::endl;
    if (erreur2 == -1) std::cout << "Pb z < 0 with Start Tree 2" << std::endl;
    if (s1 <= s2) std::cout << " Tree 1 chosen  " << std::endl;
    else std::cout << " Tree 2 chosen  " << std::endl;
#endif
  }

  cMo[0][3] -= (cdg[0] * cMo[0][0] + cdg[1] * cMo[0][1] + cdg[2] * cMo[0][2]);
  cMo[1][3] -= (cdg[0] * cMo[1][0] + cdg[1] * cMo[1][1] + cdg[2] * cMo[1][2]);
  cMo[2][3] -= (cdg[0] * cMo[2][0] + cdg[1] * cMo[2][1] + cdg[2] * cMo[2][2]);

#if (DEBUG_LEVEL1)
  std::cout << "end CCalculPose::PoseDementhonPlan()" << std::endl;
#endif
}

/*!
  \brief Compute and return the residual corresponding to the sum of squared residuals
  in meter^2 for the pose matrix \e cMo.

  \param cMo : the matrix that defines the pose to be tested.

  \return the value of the sum of squared residuals in meter^2.
*/
double vpPose::computeResidualDementhon(const vpHomogeneousMatrix &cMo)
{
  double squared_error = 0;

  // Be careful: since c3d has been translated so that point0 is at the cdg,
  // cMo here corresponds to object frame at that point, i.e, only the one used
  // internally to Dementhon's method

  for (unsigned int i = 0; i < npt; i++) {

    double X = c3d[i].get_oX() * cMo[0][0] + c3d[i].get_oY() * cMo[0][1] + c3d[i].get_oZ() * cMo[0][2] + cMo[0][3];
    double Y = c3d[i].get_oX() * cMo[1][0] + c3d[i].get_oY() * cMo[1][1] + c3d[i].get_oZ() * cMo[1][2] + cMo[1][3];
    double Z = c3d[i].get_oX() * cMo[2][0] + c3d[i].get_oY() * cMo[2][1] + c3d[i].get_oZ() * cMo[2][2] + cMo[2][3];

    double x = X / Z;
    double y = Y / Z;

    squared_error += vpMath::sqr(x - c3d[i].get_x()) + vpMath::sqr(y - c3d[i].get_y());
  }
  return squared_error;
}

#undef EPS_DEM
#undef SEUIL_RESIDUAL
#undef ITER_MAX
#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3
