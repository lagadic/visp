/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Pose computation.
 */

#include <visp3/core/vpMath.h>
#include <visp3/vision/vpPose.h>

#define SEUIL_RESIDUAL 0.0001 /* avant 0.01 dans while du calculArbreDementhon */
#define EPS_DEM 0.001
#define ITER_MAX 30 /* max number of iterations for Demonthon's loop */

BEGIN_VISP_NAMESPACE

static void calculSolutionDementhon(vpColVector &I4, vpColVector &J4, vpHomogeneousMatrix &cMo)
{
  // norm of the 3 first components of I4 and J4
  const int id0 = 0, id1 = 1, id2 = 2;
  double normI3 = sqrt((I4[id0] * I4[id0]) + (I4[id1] * I4[id1]) + (I4[id2] * I4[id2]));
  double normJ3 = sqrt((J4[id0] * J4[id0]) + (J4[id1] * J4[id1]) + (J4[id2] * J4[id2]));

  if ((normI3 < 1e-10) || (normJ3 < 1e-10)) {
    throw(vpException(vpException::divideByZeroError,
                      "Division by zero in Dementhon pose computation: normI or normJ = 0"));
  }

  double Z0 = 2.0 / (normI3 + normJ3);

  const unsigned int sizeVectors = 3;
  vpColVector I3(sizeVectors), J3(sizeVectors), K3(sizeVectors);
  for (unsigned int i = 0; i < sizeVectors; ++i) {
    I3[i] = I4[i] / normI3;
    J3[i] = J4[i] / normJ3;
  }

  K3 = vpColVector::cross(I3, J3); // k = I x J
  K3.normalize();

  J3 = vpColVector::cross(K3, I3);

  // calcul de la matrice de passage
  const unsigned int idX = 0, idY = 1, idZ = 2, idTranslation = 3;
  cMo[idX][idX] = I3[idX];
  cMo[idX][idY] = I3[idY];
  cMo[idX][idZ] = I3[idZ];
  cMo[idX][idTranslation] = I4[idTranslation] * Z0;

  cMo[idY][idX] = J3[idX];
  cMo[idY][idY] = J3[idY];
  cMo[idY][idZ] = J3[idZ];
  cMo[idY][idTranslation] = J4[idTranslation] * Z0;

  cMo[idZ][idX] = K3[idX];
  cMo[idZ][idY] = K3[idY];
  cMo[idZ][idZ] = K3[idZ];
  cMo[idZ][idTranslation] = Z0;
}

void vpPose::poseDementhonNonPlan(vpHomogeneousMatrix &cMo)
{
  vpPoint P;
  const unsigned int idX = 0, idY = 1, idZ = 2, size = 3;
  double cdg[size];
  /* compute the cog of the 3D points */
  cdg[idX] = 0.0;
  cdg[idY] = 0.0;
  cdg[idZ] = 0.0;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = (*it);
    cdg[idX] += P.get_oX();
    cdg[idY] += P.get_oY();
    cdg[idZ] += P.get_oZ();
  }

  for (unsigned int i = 0; i < size; ++i) {
    cdg[i] /= npt;
  }
  // --comment: print cdg cdg of 0 cdg of 1 cdg of 2

  c3d.clear();
  /* translate the 3D points wrt cog */
  std::list<vpPoint>::const_iterator listp_end_s = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end_s; ++it) {
    P = (*it);
    P.set_oX(P.get_oX() - cdg[idX]);
    P.set_oY(P.get_oY() - cdg[idY]);
    P.set_oZ(P.get_oZ() - cdg[idZ]);
    c3d.push_back(P);
  }

  const unsigned int idHomogeneousCoord = 3;
  const unsigned int homogeneousCoordSize = 4;
  vpMatrix A(npt, homogeneousCoordSize), Ap;

  for (unsigned int i = 0; i < npt; ++i) {
    A[i][idX] = c3d[i].get_oX();
    A[i][idY] = c3d[i].get_oY();
    A[i][idZ] = c3d[i].get_oZ();
    A[i][idHomogeneousCoord] = 1.0;
  }
  Ap = A.pseudoInverse(m_dementhonSvThresh);

  vpColVector xprim(npt);
  vpColVector yprim(npt);

  for (unsigned int i = 0; i < npt; ++i) {
    xprim[i] = c3d[i].get_x();
    yprim[i] = c3d[i].get_y();
  }
  vpColVector I4(homogeneousCoordSize), J4(homogeneousCoordSize);

  I4 = Ap * xprim;
  J4 = Ap * yprim;

  calculSolutionDementhon(I4, J4, cMo);

  const unsigned int idTranslation = 3;
  int erreur = 0;
  for (unsigned int i = 0; i < npt; ++i) {
    double z;
    z = (cMo[idZ][idX] * c3d[i].get_oX()) + (cMo[idZ][idY] * c3d[i].get_oY()) + (cMo[idZ][idZ] * c3d[i].get_oZ()) + cMo[idZ][idTranslation];
    if (z <= 0.0) {
      erreur = -1;
    }
  }
  if (erreur == -1) {
    throw(vpException(vpException::fatalError, "End of Dementhon since z < 0 for both solutions at the beginning"));
  }
  int cpt = 0;
  double res = sqrt(computeResidualDementhon(cMo) / npt);
  double res_old = 2.0 * res;

  // In his paper, Dementhon suggests to use the difference
  // between 2 successive norm of eps. We prefer to use the mean of the
  // residuals in the image
  while ((cpt < ITER_MAX) && (res > SEUIL_RESIDUAL) && (res < res_old)) {

    vpHomogeneousMatrix cMo_old;
    res_old = res;
    cMo_old = cMo;

    for (unsigned int i = 0; i < npt; ++i) {
      double eps =
        ((cMo[idZ][idX] * c3d[i].get_oX()) + (cMo[idZ][idY] * c3d[i].get_oY()) + (cMo[idZ][idZ] * c3d[i].get_oZ())) / cMo[idZ][idTranslation];

      xprim[i] = (1.0 + eps) * c3d[i].get_x();
      yprim[i] = (1.0 + eps) * c3d[i].get_y();
    }
    I4 = Ap * xprim;
    J4 = Ap * yprim;

    calculSolutionDementhon(I4, J4, cMo);
    res = sqrt(computeResidualDementhon(cMo) / npt);
    for (unsigned int i = 0; i < npt; ++i) {
      double z;
      z = (cMo[idZ][idX] * c3d[i].get_oX()) + (cMo[idZ][idY] * c3d[i].get_oY()) + (cMo[idZ][idZ] * c3d[i].get_oZ()) + cMo[idZ][idTranslation];
      if (z <= 0.0) {
        erreur = -1;
      }
    }
    if (erreur == -1) {
      cMo = cMo_old;
      res = res_old; // to leave the while loop
    }

    if (res > res_old) {
      cMo = cMo_old;
    }
    ++cpt;
  }
  // go back to the initial frame
  cMo[idX][3] -= ((cdg[idX] * cMo[idX][idX]) + (cdg[idY] * cMo[idX][idY]) + (cdg[idZ] * cMo[idX][idZ]));
  cMo[idY][3] -= ((cdg[idX] * cMo[idY][idX]) + (cdg[idY] * cMo[idY][idY]) + (cdg[idZ] * cMo[idY][idZ]));
  cMo[idZ][3] -= ((cdg[idX] * cMo[idZ][idX]) + (cdg[idY] * cMo[idZ][idY]) + (cdg[idZ] * cMo[idZ][idZ]));
}

static void calculRTheta(double s, double c, double &r, double &theta)
{
  if ((fabs(c) > EPS_DEM) || (fabs(s) > EPS_DEM)) {
    r = sqrt(sqrt((s * s) + (c * c)));
    theta = atan2(s, c) / 2.0;
  }
  else {
    if (fabs(c) > fabs(s)) {
      r = fabs(c);
      if (c >= 0.0) {
        theta = M_PI / 2.;
      }
      else {
        theta = -M_PI / 2.;
      }
    }
    else {
      r = fabs(s);
      if (s >= 0.0) {
        theta = M_PI / 4.0;
      }
      else {
        theta = -M_PI / 4.0;
      }
    }
  }
}

static void calculTwoSolutionsDementhonPlan(vpColVector &I04, vpColVector &J04, vpColVector &U,
                                            vpHomogeneousMatrix &cMo1, vpHomogeneousMatrix &cMo2)
{
  const unsigned int size = 3;
  vpColVector I0(size), J0(size);
  for (unsigned int i = 0; i < size; ++i) {
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
  const unsigned int sizeHomogeneous = 4;
  vpColVector I(sizeHomogeneous), J(sizeHomogeneous);
  I = I04 + (U * r * co);
  J = J04 + (U * r * si);

  calculSolutionDementhon(I, J, cMo1);

  // calcul de la deuxieme solution
  I = I04 - (U * r * co);
  J = J04 - (U * r * si);

  calculSolutionDementhon(I, J, cMo2);
}

int vpPose::calculArbreDementhon(vpMatrix &Ap, vpColVector &U, vpHomogeneousMatrix &cMo)
{
  int erreur = 0;

  // test if all points are in front of the camera
  const unsigned int idX = 0, idY = 1, idZ = 2, idTranslation = 3;
  for (unsigned int i = 0; i < npt; ++i) {
    double z;
    z = (cMo[idZ][idX] * c3d[i].get_oX()) + (cMo[idZ][idY] * c3d[i].get_oY()) + (cMo[idZ][idZ] * c3d[i].get_oZ()) + cMo[idZ][idTranslation];
    if (z <= 0.0) {
      erreur = -1;
      return erreur;
    }
  }

  unsigned int cpt = 0;
  double res_min = sqrt(computeResidualDementhon(cMo) / npt);
  double res_old = 2.0 * res_min;

  /* FC modif SEUIL_RESIDUAL 0.01 a 0.001 */
  while ((cpt < ITER_MAX) && (res_min > SEUIL_RESIDUAL) && (res_min < res_old)) {
    vpHomogeneousMatrix cMo1, cMo2, cMo_old;

    res_old = res_min;
    cMo_old = cMo;

    vpColVector xprim(npt), yprim(npt);
    for (unsigned int i = 0; i < npt; ++i) {
      double eps =
        ((cMo[idZ][idX] * c3d[i].get_oX()) + (cMo[idZ][idY] * c3d[i].get_oY()) + (cMo[idZ][idZ] * c3d[i].get_oZ())) / cMo[idZ][idTranslation];

      xprim[i] = (1.0 + eps) * c3d[i].get_x();
      yprim[i] = (1.0 + eps) * c3d[i].get_y();
    }

    const unsigned int sizeHomogeneous = 4;
    vpColVector I04(sizeHomogeneous), J04(sizeHomogeneous);
    I04 = Ap * xprim;
    J04 = Ap * yprim;

    calculTwoSolutionsDementhonPlan(I04, J04, U, cMo1, cMo2);

    // test if all points are in front of the camera for cMo1 and cMo2
    int erreur1 = 0;
    int erreur2 = 0;
    for (unsigned int i = 0; i < npt; ++i) {
      double z;
      z = (cMo1[idZ][idX] * c3d[i].get_oX()) + (cMo1[idZ][idY] * c3d[i].get_oY()) + (cMo1[idZ][idZ] * c3d[i].get_oZ()) + cMo1[idZ][idTranslation];
      if (z <= 0.0) {
        erreur1 = -1;
      }
      z = (cMo2[idZ][idX] * c3d[i].get_oX()) + (cMo2[idZ][idY] * c3d[i].get_oY()) + (cMo2[idZ][idZ] * c3d[i].get_oZ()) + cMo2[idZ][idTranslation];
      if (z <= 0.0) {
        erreur2 = -1;
      }
    }

    if ((erreur1 == -1) && (erreur2 == -1)) {
      cMo = cMo_old;
      break; // outside of while due to z < 0
    }
    if ((erreur1 == 0) && (erreur2 == -1)) {
      cMo = cMo1;
      res_min = sqrt(computeResidualDementhon(cMo) / npt);
    }
    if ((erreur1 == -1) && (erreur2 == 0)) {
      cMo = cMo2;
      res_min = sqrt(computeResidualDementhon(cMo) / npt);
    }
    if ((erreur1 == 0) && (erreur2 == 0)) {
      double res1 = sqrt(computeResidualDementhon(cMo1) / npt);
      double res2 = sqrt(computeResidualDementhon(cMo2) / npt);
      if (res1 <= res2) {
        res_min = res1;
        cMo = cMo1;
      }
      else {
        res_min = res2;
        cMo = cMo2;
      }
    }

    if (res_min > res_old) {
      cMo = cMo_old;
    }
    ++cpt;
  } /* end of while */

  return erreur;
}

void vpPose::poseDementhonPlan(vpHomogeneousMatrix &cMo)
{
  const double svdFactorUsedWhenFailure = 10.; // Factor by which is multipled m_dementhonSvThresh each time the svdDecomposition fails
  const double svdThresholdLimit = 1e-2; // The svd decomposition will be tested with a threshold up to this value. If with this threshold, the rank of A is still !=3, an exception is thrown
  const double lnOfSvdFactorUsed = std::log(svdFactorUsedWhenFailure);
  const double logNOfSvdThresholdLimit = std::log(svdThresholdLimit)/lnOfSvdFactorUsed;
  vpPoint P;
  const unsigned int idX = 0, idY = 1, idZ = 2, size3Dpt = 3;
  double cdg[size3Dpt];
  /* compute the cog of the 3D points */
  cdg[idX] = 0.0;
  cdg[idY] = 0.0;
  cdg[idZ] = 0.0;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = (*it);
    cdg[idX] += P.get_oX();
    cdg[idY] += P.get_oY();
    cdg[idZ] += P.get_oZ();
  }

  for (unsigned int i = 0; i < size3Dpt; ++i) {
    cdg[i] /= npt;
  }
  // --comment: print cdg  cdg of 0 of 1 and of 2

  c3d.clear();
  /* translate the 3D points wrt cog */
  std::list<vpPoint>::const_iterator listp_end_decl2 = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end_decl2; ++it) {
    P = (*it);
    P.set_oX(P.get_oX() - cdg[0]);
    P.set_oY(P.get_oY() - cdg[1]);
    P.set_oZ(P.get_oZ() - cdg[2]);
    c3d.push_back(P);
  }

  const unsigned int sizeHomogeneous = 4, idHomogeneous = 3;
  vpMatrix A(npt, sizeHomogeneous);

  for (unsigned int i = 0; i < npt; ++i) {
    A[i][idX] = c3d[i].get_oX();
    A[i][idY] = c3d[i].get_oY();
    A[i][idZ] = c3d[i].get_oZ();
    A[i][idHomogeneous] = 1.0;
  }
  vpColVector sv;
  vpMatrix Ap, imA, imAt, kAt;
  bool isRankEqualTo3 = false; // Indicates if the rank of A is the expected one
  // Get the log_n(m_dementhonSvThresh), where n is the factor by which we will multiply it if the svd decomposition fails.
  double logNofSvdThresh = std::log(m_dementhonSvThresh)/lnOfSvdFactorUsed;
  // Ensure that if the user chose a threshold > svdThresholdLimit, at least 1 iteration of svd decomposition is performed
  int nbMaxIter = static_cast<int>(std::max<double>(std::ceil(logNOfSvdThresholdLimit - logNofSvdThresh), 1.));
  double svdThreshold = m_dementhonSvThresh;
  int irank = 0;
  int i = 0;
  const unsigned int expectedRank = 3;
  while ((i < nbMaxIter) && (!isRankEqualTo3)) {
    irank = A.pseudoInverse(Ap, sv, svdThreshold, imA, imAt, kAt);
    if (irank == expectedRank) {
      isRankEqualTo3 = true;
    }
    else {
      isRankEqualTo3 = false;
      svdThreshold *= svdFactorUsedWhenFailure;
    }
    ++i;
  }

  if (!isRankEqualTo3) {
    std::stringstream errorMsg;
    errorMsg << "In Dementhon planar, after ";
    errorMsg << nbMaxIter;
    errorMsg << " trials multiplying the svd threshold by ";
    errorMsg << svdFactorUsedWhenFailure;
    errorMsg << ", rank (";
    errorMsg << irank;
    errorMsg << ") is still not 3";
    throw(vpException(vpException::fatalError, errorMsg.str()));
  }
  // calcul de U
  vpColVector U(sizeHomogeneous);
  for (unsigned int i = 0; i < sizeHomogeneous; ++i) {
    U[i] = kAt[0][i];
  }

  vpColVector xi(npt);
  vpColVector yi(npt);

  for (unsigned int i = 0; i < npt; ++i) {
    xi[i] = c3d[i].get_x();
    yi[i] = c3d[i].get_y();
  }

  vpColVector I04(sizeHomogeneous), J04(sizeHomogeneous);
  I04 = Ap * xi;
  J04 = Ap * yi;

  vpHomogeneousMatrix cMo1, cMo2;
  calculTwoSolutionsDementhonPlan(I04, J04, U, cMo1, cMo2);

  int erreur1 = calculArbreDementhon(Ap, U, cMo1);
  int erreur2 = calculArbreDementhon(Ap, U, cMo2);

  if ((erreur1 == -1) && (erreur2 == -1)) {
    throw(
        vpException(vpException::fatalError, "Error in Dementhon planar: z < 0 with Start Tree 1 and Start Tree 2..."));
  }
  if ((erreur1 == 0) && (erreur2 == -1)) {
    cMo = cMo1;
  }
  if ((erreur1 == -1) && (erreur2 == 0)) {
    cMo = cMo2;
  }
  if ((erreur1 == 0) && (erreur2 == 0)) {
    double s1 = computeResidualDementhon(cMo1);
    double s2 = computeResidualDementhon(cMo2);

    if (s1 <= s2) {
      cMo = cMo1;
    }
    else {
      cMo = cMo2;
    }
  }
  const unsigned int idTranslation = 3;
  cMo[idX][idTranslation] -= ((cdg[idX] * cMo[idX][idX]) + (cdg[idY] * cMo[idX][idY]) + (cdg[idZ] * cMo[idX][idZ]));
  cMo[idY][idTranslation] -= ((cdg[idX] * cMo[idY][idX]) + (cdg[idY] * cMo[idY][idY]) + (cdg[idZ] * cMo[idY][idZ]));
  cMo[idZ][idTranslation] -= ((cdg[idX] * cMo[idZ][idX]) + (cdg[idY] * cMo[idZ][idY]) + (cdg[idZ] * cMo[idZ][idZ]));
}

double vpPose::computeResidualDementhon(const vpHomogeneousMatrix &cMo)
{
  double squared_error = 0;

  // Be careful: since c3d has been translated so that point0 is at the cdg,
  // cMo here corresponds to object frame at that point, i.e, only the one used
  // internally to Dementhon's method
  const unsigned int idX = 0, idY = 1, idZ = 2, idTranslation = 3;
  for (unsigned int i = 0; i < npt; ++i) {

    double X = (c3d[i].get_oX() * cMo[idX][idX]) + (c3d[i].get_oY() * cMo[idX][idY]) + (c3d[i].get_oZ() * cMo[idX][idZ]) + cMo[idX][idTranslation];
    double Y = (c3d[i].get_oX() * cMo[idY][idX]) + (c3d[i].get_oY() * cMo[idY][idY]) + (c3d[i].get_oZ() * cMo[idY][idZ]) + cMo[idY][idTranslation];
    double Z = (c3d[i].get_oX() * cMo[idZ][idX]) + (c3d[i].get_oY() * cMo[idZ][idY]) + (c3d[i].get_oZ() * cMo[idZ][idZ]) + cMo[idZ][idTranslation];

    double x = X / Z;
    double y = Y / Z;

    squared_error += vpMath::sqr(x - c3d[i].get_x()) + vpMath::sqr(y - c3d[i].get_y());
  }
  return squared_error;
}

END_VISP_NAMESPACE

#undef EPS_DEM
#undef SEUIL_RESIDUAL
#undef ITER_MAX
