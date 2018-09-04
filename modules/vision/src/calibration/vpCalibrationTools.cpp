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
 * Camera calibration.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/

#include <visp3/core/vpMath.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpCalibration.h>
#include <visp3/vision/vpPose.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

#undef MAX   /* FC unused anywhere */
#undef MIN   /* FC unused anywhere */

void vpCalibration::calibLagrange(vpCameraParameters &cam_est, vpHomogeneousMatrix &cMo_est)
{

  vpMatrix A(2 * npt, 3);
  vpMatrix B(2 * npt, 9);

  std::list<double>::const_iterator it_LoX = LoX.begin();
  std::list<double>::const_iterator it_LoY = LoY.begin();
  std::list<double>::const_iterator it_LoZ = LoZ.begin();
  std::list<vpImagePoint>::const_iterator it_Lip = Lip.begin();

  vpImagePoint ip;

  for (unsigned int i = 0; i < npt; i++) {

    double x0 = *it_LoX;
    double y0 = *it_LoY;
    double z0 = *it_LoZ;

    ip = *it_Lip;

    double xi = ip.get_u();
    double yi = ip.get_v();

    A[2 * i][0] = x0 * xi;
    A[2 * i][1] = y0 * xi;
    A[2 * i][2] = z0 * xi;
    B[2 * i][0] = -x0;
    B[2 * i][1] = -y0;
    B[2 * i][2] = -z0;
    B[2 * i][3] = 0.0;
    B[2 * i][4] = 0.0;
    B[2 * i][5] = 0.0;
    B[2 * i][6] = -1.0;
    B[2 * i][7] = 0.0;
    B[2 * i][8] = xi;
    A[2 * i + 1][0] = x0 * yi;
    A[2 * i + 1][1] = y0 * yi;
    A[2 * i + 1][2] = z0 * yi;
    B[2 * i + 1][0] = 0.0;
    B[2 * i + 1][1] = 0.0;
    B[2 * i + 1][2] = 0.0;
    B[2 * i + 1][3] = -x0;
    B[2 * i + 1][4] = -y0;
    B[2 * i + 1][5] = -z0;
    B[2 * i + 1][6] = 0.0;
    B[2 * i + 1][7] = -1.0;
    B[2 * i + 1][8] = yi;

    ++it_LoX;
    ++it_LoY;
    ++it_LoZ;
    ++it_Lip;
  }

  vpMatrix BtB; /* compute B^T B  */
  BtB = B.t() * B;

  /* compute (B^T B)^(-1)         */
  /* input : btb    (dimension 9 x 9) = (B^T B)     */
  /* output : btbinv  (dimension 9 x 9) = (B^T B)^(-1)  */

  vpMatrix BtBinv;
  BtBinv = BtB.pseudoInverse(1e-16);

  vpMatrix BtA;
  BtA = B.t() * A; /* compute B^T A  */

  vpMatrix r;
  r = BtBinv * BtA; /* compute (B^T B)^(-1) B^T A */

  vpMatrix e; /* compute - A^T B (B^T B)^(-1) B^T A*/
  e = -(A.t() * B) * r;

  vpMatrix AtA; /* compute A^T A */
  AtA = A.AtA();

  e += AtA; /* compute E = A^T A - A^T B (B^T B)^(-1) B^T A */

  vpColVector x1(3);
  vpColVector x2;

  e.svd(x1, AtA); // destructive on e
  // eigenvector computation of E corresponding to the min eigenvalue.
  /* SVmax  computation*/
  double svm = 0.0;
  unsigned int imin = 1;
  for (unsigned int i = 0; i < x1.getRows(); i++) {
    if (x1[i] > svm) {
      svm = x1[i];
      imin = i;
    }
  }

  // svm *= 0.1; /* for the rank */

  for (unsigned int i = 0; i < x1.getRows(); i++) {
    if (x1[i] < x1[imin])
      imin = i;
  }

  for (unsigned int i = 0; i < x1.getRows(); i++)
    x1[i] = AtA[i][imin];

  x2 = -(r * x1); // X_2 = - (B^T B)^(-1) B^T A X_1

  vpColVector sol(12);
  vpColVector resul(7);
  for (unsigned int i = 0; i < 3; i++)
    sol[i] = x1[i];                    /* X_1  */
  for (unsigned int i = 0; i < 9; i++) /* X_2 = - (B^T B)^(-1) B^T A X_1 */
  {
    sol[i + 3] = x2[i];
  }

  if (sol[11] < 0.0)
    for (unsigned int i = 0; i < 12; i++)
      sol[i] = -sol[i]; /* since Z0 > 0 */

  resul[0] = sol[3] * sol[0] + sol[4] * sol[1] + sol[5] * sol[2]; /* u0 */

  resul[1] = sol[6] * sol[0] + sol[7] * sol[1] + sol[8] * sol[2]; /* v0 */

  resul[2] = sqrt(sol[3] * sol[3] + sol[4] * sol[4] + sol[5] * sol[5] /* px */
                  - resul[0] * resul[0]);
  resul[3] = sqrt(sol[6] * sol[6] + sol[7] * sol[7] + sol[8] * sol[8] /* py */
                  - resul[1] * resul[1]);

  cam_est.initPersProjWithoutDistortion(resul[2], resul[3], resul[0], resul[1]);

  resul[4] = (sol[9] - sol[11] * resul[0]) / resul[2];  /* X0 */
  resul[5] = (sol[10] - sol[11] * resul[1]) / resul[3]; /* Y0 */
  resul[6] = sol[11];                                   /* Z0 */

  vpMatrix rd(3, 3);
  /* fill rotation matrix */
  for (unsigned int i = 0; i < 3; i++)
    rd[0][i] = (sol[i + 3] - sol[i] * resul[0]) / resul[2];
  for (unsigned int i = 0; i < 3; i++)
    rd[1][i] = (sol[i + 6] - sol[i] * resul[1]) / resul[3];
  for (unsigned int i = 0; i < 3; i++)
    rd[2][i] = sol[i];

  //  std::cout << "norme X1 " << x1.sumSquare() <<std::endl;
  //  std::cout << rd*rd.t() ;

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++)
      cMo_est[i][j] = rd[i][j];
  }
  for (unsigned int i = 0; i < 3; i++)
    cMo_est[i][3] = resul[i + 4];

  this->cMo = cMo_est;
  this->cMo_dist = cMo_est;

  double deviation, deviation_dist;
  this->computeStdDeviation(deviation, deviation_dist);
}

void vpCalibration::calibVVS(vpCameraParameters &cam_est, vpHomogeneousMatrix &cMo_est, bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int n_points = npt;

  vpColVector oX(n_points), cX(n_points);
  vpColVector oY(n_points), cY(n_points);
  vpColVector oZ(n_points), cZ(n_points);
  vpColVector u(n_points);
  vpColVector v(n_points);

  vpColVector P(2 * n_points);
  vpColVector Pd(2 * n_points);

  vpImagePoint ip;

  std::list<double>::const_iterator it_LoX = LoX.begin();
  std::list<double>::const_iterator it_LoY = LoY.begin();
  std::list<double>::const_iterator it_LoZ = LoZ.begin();
  std::list<vpImagePoint>::const_iterator it_Lip = Lip.begin();

  for (unsigned int i = 0; i < n_points; i++) {
    oX[i] = *it_LoX;
    oY[i] = *it_LoY;
    oZ[i] = *it_LoZ;

    ip = *it_Lip;

    u[i] = ip.get_u();
    v[i] = ip.get_v();

    ++it_LoX;
    ++it_LoY;
    ++it_LoZ;
    ++it_Lip;
  }

  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {
    iter++;
    residu_1 = r;

    double px = cam_est.get_px();
    double py = cam_est.get_py();
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    r = 0;

    for (unsigned int i = 0; i < n_points; i++) {
      cX[i] = oX[i] * cMo_est[0][0] + oY[i] * cMo_est[0][1] + oZ[i] * cMo_est[0][2] + cMo_est[0][3];
      cY[i] = oX[i] * cMo_est[1][0] + oY[i] * cMo_est[1][1] + oZ[i] * cMo_est[1][2] + cMo_est[1][3];
      cZ[i] = oX[i] * cMo_est[2][0] + oY[i] * cMo_est[2][1] + oZ[i] * cMo_est[2][2] + cMo_est[2][3];

      Pd[2 * i] = u[i];
      Pd[2 * i + 1] = v[i];

      P[2 * i] = cX[i] / cZ[i] * px + u0;
      P[2 * i + 1] = cY[i] / cZ[i] * py + v0;

      r += ((vpMath::sqr(P[2 * i] - Pd[2 * i]) + vpMath::sqr(P[2 * i + 1] - Pd[2 * i + 1])));
    }

    vpColVector error;
    error = P - Pd;
    // r = r/n_points ;

    vpMatrix L(n_points * 2, 10);
    for (unsigned int i = 0; i < n_points; i++) {
      double x = cX[i];
      double y = cY[i];
      double z = cZ[i];
      double inv_z = 1 / z;

      double X = x * inv_z;
      double Y = y * inv_z;

      //---------------
      {
        L[2 * i][0] = px * (-inv_z);
        L[2 * i][1] = 0;
        L[2 * i][2] = px * X * inv_z;
        L[2 * i][3] = px * X * Y;
        L[2 * i][4] = -px * (1 + X * X);
        L[2 * i][5] = px * Y;
      }
      {
        L[2 * i][6] = 1;
        L[2 * i][7] = 0;
        L[2 * i][8] = X;
        L[2 * i][9] = 0;
      }
      {
        L[2 * i + 1][0] = 0;
        L[2 * i + 1][1] = py * (-inv_z);
        L[2 * i + 1][2] = py * (Y * inv_z);
        L[2 * i + 1][3] = py * (1 + Y * Y);
        L[2 * i + 1][4] = -py * X * Y;
        L[2 * i + 1][5] = -py * X;
      }
      {
        L[2 * i + 1][6] = 0;
        L[2 * i + 1][7] = 1;
        L[2 * i + 1][8] = 0;
        L[2 * i + 1][9] = Y;
      }
    } // end interaction
    vpMatrix Lp;
    Lp = L.pseudoInverse(1e-10);

    vpColVector e;
    e = Lp * error;

    vpColVector Tc, Tc_v(6);
    Tc = -e * gain;

    //   Tc_v =0 ;
    for (unsigned int i = 0; i < 6; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithoutDistortion(px + Tc[8], py + Tc[9], u0 + Tc[6], v0 + Tc[7]);

    cMo_est = vpExponentialMap::direct(Tc_v).inverse() * cMo_est;
    if (verbose)
      std::cout << " std dev " << sqrt(r / n_points) << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }
  this->cMo = cMo_est;
  this->cMo_dist = cMo_est;
  this->residual = r;
  this->residual_dist = r;
  if (verbose)
    std::cout << " std dev " << sqrt(r / n_points) << std::endl;
  // Restore ostream format
  std::cout.flags(original_flags);
}

void vpCalibration::calibVVSMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam_est,
                                  double &globalReprojectionError, bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int nbPoint[256];     // number of points by image
  unsigned int nbPointTotal = 0; // total number of points
  unsigned int nbPose = (unsigned int)table_cal.size();
  unsigned int nbPose6 = 6 * nbPose;

  for (unsigned int i = 0; i < nbPose; i++) {
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  if (nbPointTotal < 4) {
    // vpERROR_TRACE("Not enough point to calibrate");
    throw(vpCalibrationException(vpCalibrationException::notInitializedError, "Not enough point to calibrate"));
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal);
  vpColVector oY(nbPointTotal), cY(nbPointTotal);
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal);
  vpColVector u(nbPointTotal);
  vpColVector v(nbPointTotal);

  vpColVector P(2 * nbPointTotal);
  vpColVector Pd(2 * nbPointTotal);
  vpImagePoint ip;

  unsigned int curPoint = 0; // current point indice
  for (unsigned int p = 0; p < nbPose; p++) {
    std::list<double>::const_iterator it_LoX = table_cal[p].LoX.begin();
    std::list<double>::const_iterator it_LoY = table_cal[p].LoY.begin();
    std::list<double>::const_iterator it_LoZ = table_cal[p].LoZ.begin();
    std::list<vpImagePoint>::const_iterator it_Lip = table_cal[p].Lip.begin();

    for (unsigned int i = 0; i < nbPoint[p]; i++) {
      oX[curPoint] = *it_LoX;
      oY[curPoint] = *it_LoY;
      oZ[curPoint] = *it_LoZ;

      ip = *it_Lip;
      u[curPoint] = ip.get_u();
      v[curPoint] = ip.get_v();

      ++it_LoX;
      ++it_LoY;
      ++it_LoZ;
      ++it_Lip;

      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {

    iter++;
    residu_1 = r;

    double px = cam_est.get_px();
    double py = cam_est.get_py();
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    r = 0;
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint2 = 2 * curPoint;

        cX[curPoint] =
            oX[curPoint] * cMoTmp[0][0] + oY[curPoint] * cMoTmp[0][1] + oZ[curPoint] * cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] =
            oX[curPoint] * cMoTmp[1][0] + oY[curPoint] * cMoTmp[1][1] + oZ[curPoint] * cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] =
            oX[curPoint] * cMoTmp[2][0] + oY[curPoint] * cMoTmp[2][1] + oZ[curPoint] * cMoTmp[2][2] + cMoTmp[2][3];

        Pd[curPoint2] = u[curPoint];
        Pd[curPoint2 + 1] = v[curPoint];

        P[curPoint2] = cX[curPoint] / cZ[curPoint] * px + u0;
        P[curPoint2 + 1] = cY[curPoint] / cZ[curPoint] * py + v0;

        r += (vpMath::sqr(P[curPoint2] - Pd[curPoint2]) + vpMath::sqr(P[curPoint2 + 1] - Pd[curPoint2 + 1]));
        curPoint++;
      }
    }

    vpColVector error;
    error = P - Pd;
    // r = r/nbPointTotal ;

    vpMatrix L(nbPointTotal * 2, nbPose6 + 4);
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      unsigned int q = 6 * p;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint2 = 2 * curPoint;
        unsigned int curPoint21 = curPoint2 + 1;

        double x = cX[curPoint];
        double y = cY[curPoint];
        double z = cZ[curPoint];

        double inv_z = 1 / z;

        double X = x * inv_z;
        double Y = y * inv_z;

        //---------------
        {
          {
            L[curPoint2][q] = px * (-inv_z);
            L[curPoint2][q + 1] = 0;
            L[curPoint2][q + 2] = px * (X * inv_z);
            L[curPoint2][q + 3] = px * X * Y;
            L[curPoint2][q + 4] = -px * (1 + X * X);
            L[curPoint2][q + 5] = px * Y;
          }
          {
            L[curPoint2][nbPose6] = 1;
            L[curPoint2][nbPose6 + 1] = 0;
            L[curPoint2][nbPose6 + 2] = X;
            L[curPoint2][nbPose6 + 3] = 0;
          }
          {
            L[curPoint21][q] = 0;
            L[curPoint21][q + 1] = py * (-inv_z);
            L[curPoint21][q + 2] = py * (Y * inv_z);
            L[curPoint21][q + 3] = py * (1 + Y * Y);
            L[curPoint21][q + 4] = -py * X * Y;
            L[curPoint21][q + 5] = -py * X;
          }
          {
            L[curPoint21][nbPose6] = 0;
            L[curPoint21][nbPose6 + 1] = 1;
            L[curPoint21][nbPose6 + 2] = 0;
            L[curPoint21][nbPose6 + 3] = Y;
          }
        }
        curPoint++;
      } // end interaction
    }
    vpMatrix Lp;
    Lp = L.pseudoInverse(1e-10);

    vpColVector e;
    e = Lp * error;

    vpColVector Tc, Tc_v(nbPose6);
    Tc = -e * gain;

    //   Tc_v =0 ;
    for (unsigned int i = 0; i < nbPose6; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithoutDistortion(px + Tc[nbPose6 + 2], py + Tc[nbPose6 + 3], u0 + Tc[nbPose6],
                                          v0 + Tc[nbPose6 + 1]);

    //    cam.setKd(get_kd() + Tc[10]) ;
    vpColVector Tc_v_Tmp(6);

    for (unsigned int p = 0; p < nbPose; p++) {
      for (unsigned int i = 0; i < 6; i++)
        Tc_v_Tmp[i] = Tc_v[6 * p + i];

      table_cal[p].cMo = vpExponentialMap::direct(Tc_v_Tmp, 1).inverse() * table_cal[p].cMo;
    }

    if (verbose)
      std::cout << " std dev " << sqrt(r / nbPointTotal) << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }
  for (unsigned int p = 0; p < nbPose; p++) {
    table_cal[p].cMo_dist = table_cal[p].cMo;
    table_cal[p].cam = cam_est;
    table_cal[p].cam_dist = cam_est;
    double deviation, deviation_dist;
    table_cal[p].computeStdDeviation(deviation, deviation_dist);
  }
  globalReprojectionError = sqrt(r / nbPointTotal);
  // Restore ostream format
  std::cout.flags(original_flags);
}

void vpCalibration::calibVVSWithDistortion(vpCameraParameters &cam_est, vpHomogeneousMatrix &cMo_est, bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int n_points = npt;

  vpColVector oX(n_points), cX(n_points);
  vpColVector oY(n_points), cY(n_points);
  vpColVector oZ(n_points), cZ(n_points);
  vpColVector u(n_points);
  vpColVector v(n_points);

  vpColVector P(4 * n_points);
  vpColVector Pd(4 * n_points);

  std::list<double>::const_iterator it_LoX = LoX.begin();
  std::list<double>::const_iterator it_LoY = LoY.begin();
  std::list<double>::const_iterator it_LoZ = LoZ.begin();
  std::list<vpImagePoint>::const_iterator it_Lip = Lip.begin();

  vpImagePoint ip;

  for (unsigned int i = 0; i < n_points; i++) {
    oX[i] = *it_LoX;
    oY[i] = *it_LoY;
    oZ[i] = *it_LoZ;

    ip = *it_Lip;
    u[i] = ip.get_u();
    v[i] = ip.get_v();

    ++it_LoX;
    ++it_LoY;
    ++it_LoZ;
    ++it_Lip;
  }

  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {
    iter++;
    residu_1 = r;

    r = 0;
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    double px = cam_est.get_px();
    double py = cam_est.get_py();

    double inv_px = 1 / px;
    double inv_py = 1 / py;

    double kud = cam_est.get_kud();
    double kdu = cam_est.get_kdu();

    double k2ud = 2 * kud;
    double k2du = 2 * kdu;
    vpMatrix L(n_points * 4, 12);

    for (unsigned int i = 0; i < n_points; i++) {
      unsigned int i4 = 4 * i;
      unsigned int i41 = 4 * i + 1;
      unsigned int i42 = 4 * i + 2;
      unsigned int i43 = 4 * i + 3;

      cX[i] = oX[i] * cMo_est[0][0] + oY[i] * cMo_est[0][1] + oZ[i] * cMo_est[0][2] + cMo_est[0][3];
      cY[i] = oX[i] * cMo_est[1][0] + oY[i] * cMo_est[1][1] + oZ[i] * cMo_est[1][2] + cMo_est[1][3];
      cZ[i] = oX[i] * cMo_est[2][0] + oY[i] * cMo_est[2][1] + oZ[i] * cMo_est[2][2] + cMo_est[2][3];

      double x = cX[i];
      double y = cY[i];
      double z = cZ[i];
      double inv_z = 1 / z;

      double X = x * inv_z;
      double Y = y * inv_z;

      double X2 = X * X;
      double Y2 = Y * Y;
      double XY = X * Y;

      double up = u[i];
      double vp = v[i];

      Pd[i4] = up;
      Pd[i41] = vp;

      double up0 = up - u0;
      double vp0 = vp - v0;

      double xp0 = up0 * inv_px;
      double xp02 = xp0 * xp0;

      double yp0 = vp0 * inv_py;
      double yp02 = yp0 * yp0;

      double r2du = xp02 + yp02;
      double kr2du = kdu * r2du;

      P[i4] = u0 + px * X - kr2du * (up0);
      P[i41] = v0 + py * Y - kr2du * (vp0);

      double r2ud = X2 + Y2;
      double kr2ud = 1 + kud * r2ud;

      double Axx = px * (kr2ud + k2ud * X2);
      double Axy = px * k2ud * XY;
      double Ayy = py * (kr2ud + k2ud * Y2);
      double Ayx = py * k2ud * XY;

      Pd[i42] = up;
      Pd[i43] = vp;

      P[i42] = u0 + px * X * kr2ud;
      P[i43] = v0 + py * Y * kr2ud;

      r += (vpMath::sqr(P[i4] - Pd[i4]) + vpMath::sqr(P[i41] - Pd[i41]) + vpMath::sqr(P[i42] - Pd[i42]) +
            vpMath::sqr(P[i43] - Pd[i43])) *
           0.5;

      //--distorted to undistorted
      {
        {
          L[i4][0] = px * (-inv_z);
          L[i4][1] = 0;
          L[i4][2] = px * X * inv_z;
          L[i4][3] = px * X * Y;
          L[i4][4] = -px * (1 + X2);
          L[i4][5] = px * Y;
        }
        {
          L[i4][6] = 1 + kr2du + k2du * xp02;
          L[i4][7] = k2du * up0 * yp0 * inv_py;
          L[i4][8] = X + k2du * xp02 * xp0;
          L[i4][9] = k2du * up0 * yp02 * inv_py;
          L[i4][10] = -(up0) * (r2du);
          L[i4][11] = 0;
        }
        {
          L[i41][0] = 0;
          L[i41][1] = py * (-inv_z);
          L[i41][2] = py * Y * inv_z;
          L[i41][3] = py * (1 + Y2);
          L[i41][4] = -py * XY;
          L[i41][5] = -py * X;
        }
        {
          L[i41][6] = k2du * xp0 * vp0 * inv_px;
          L[i41][7] = 1 + kr2du + k2du * yp02;
          L[i41][8] = k2du * vp0 * xp02 * inv_px;
          L[i41][9] = Y + k2du * yp02 * yp0;
          L[i41][10] = -vp0 * r2du;
          L[i41][11] = 0;
        }
        //---undistorted to distorted
        {
          L[i42][0] = Axx * (-inv_z);
          L[i42][1] = Axy * (-inv_z);
          L[i42][2] = Axx * (X * inv_z) + Axy * (Y * inv_z);
          L[i42][3] = Axx * X * Y + Axy * (1 + Y2);
          L[i42][4] = -Axx * (1 + X2) - Axy * XY;
          L[i42][5] = Axx * Y - Axy * X;
        }
        {
          L[i42][6] = 1;
          L[i42][7] = 0;
          L[i42][8] = X * kr2ud;
          L[i42][9] = 0;
          L[i42][10] = 0;
          L[i42][11] = px * X * r2ud;
        }
        {
          L[i43][0] = Ayx * (-inv_z);
          L[i43][1] = Ayy * (-inv_z);
          L[i43][2] = Ayx * (X * inv_z) + Ayy * (Y * inv_z);
          L[i43][3] = Ayx * XY + Ayy * (1 + Y2);
          L[i43][4] = -Ayx * (1 + X2) - Ayy * XY;
          L[i43][5] = Ayx * Y - Ayy * X;
        }
        {
          L[i43][6] = 0;
          L[i43][7] = 1;
          L[i43][8] = 0;
          L[i43][9] = Y * kr2ud;
          L[i43][10] = 0;
          L[i43][11] = py * Y * r2ud;
        }
      } // end interaction
    }   // end interaction

    vpColVector error;
    error = P - Pd;
    // r = r/n_points ;

    vpMatrix Lp;
    Lp = L.pseudoInverse(1e-10);

    vpColVector e;
    e = Lp * error;

    vpColVector Tc, Tc_v(6);
    Tc = -e * gain;

    for (unsigned int i = 0; i < 6; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithDistortion(px + Tc[8], py + Tc[9], u0 + Tc[6], v0 + Tc[7], kud + Tc[11], kdu + Tc[10]);

    cMo_est = vpExponentialMap::direct(Tc_v).inverse() * cMo_est;
    if (verbose)
      std::cout << " std dev " << sqrt(r / n_points) << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }
  this->residual_dist = r;
  this->cMo_dist = cMo_est;
  this->cam_dist = cam_est;

  if (verbose)
    std::cout << " std dev " << sqrt(r / n_points) << std::endl;

  // Restore ostream format
  std::cout.flags(original_flags);
}

void vpCalibration::calibVVSWithDistortionMulti(std::vector<vpCalibration> &table_cal, vpCameraParameters &cam_est,
                                                double &globalReprojectionError, bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int nbPoint[1024];    // number of points by image
  unsigned int nbPointTotal = 0; // total number of points
  unsigned int nbPose = (unsigned int)table_cal.size();
  unsigned int nbPose6 = 6 * nbPose;
  for (unsigned int i = 0; i < nbPose; i++) {
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  if (nbPointTotal < 4) {
    // vpERROR_TRACE("Not enough point to calibrate");
    throw(vpCalibrationException(vpCalibrationException::notInitializedError, "Not enough point to calibrate"));
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal);
  vpColVector oY(nbPointTotal), cY(nbPointTotal);
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal);
  vpColVector u(nbPointTotal);
  vpColVector v(nbPointTotal);

  vpColVector P(4 * nbPointTotal);
  vpColVector Pd(4 * nbPointTotal);
  vpImagePoint ip;

  unsigned int curPoint = 0; // current point indice
  for (unsigned int p = 0; p < nbPose; p++) {
    std::list<double>::const_iterator it_LoX = table_cal[p].LoX.begin();
    std::list<double>::const_iterator it_LoY = table_cal[p].LoY.begin();
    std::list<double>::const_iterator it_LoZ = table_cal[p].LoZ.begin();
    std::list<vpImagePoint>::const_iterator it_Lip = table_cal[p].Lip.begin();

    for (unsigned int i = 0; i < nbPoint[p]; i++) {
      oX[curPoint] = *it_LoX;
      oY[curPoint] = *it_LoY;
      oZ[curPoint] = *it_LoZ;

      ip = *it_Lip;
      u[curPoint] = ip.get_u();
      v[curPoint] = ip.get_v();

      ++it_LoX;
      ++it_LoY;
      ++it_LoZ;
      ++it_Lip;
      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {
    iter++;
    residu_1 = r;

    r = 0;
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo_dist;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        cX[curPoint] =
            oX[curPoint] * cMoTmp[0][0] + oY[curPoint] * cMoTmp[0][1] + oZ[curPoint] * cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] =
            oX[curPoint] * cMoTmp[1][0] + oY[curPoint] * cMoTmp[1][1] + oZ[curPoint] * cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] =
            oX[curPoint] * cMoTmp[2][0] + oY[curPoint] * cMoTmp[2][1] + oZ[curPoint] * cMoTmp[2][2] + cMoTmp[2][3];

        curPoint++;
      }
    }

    vpMatrix L(nbPointTotal * 4, nbPose6 + 6);
    curPoint = 0; // current point indice
    double px = cam_est.get_px();
    double py = cam_est.get_py();
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    double inv_px = 1 / px;
    double inv_py = 1 / py;

    double kud = cam_est.get_kud();
    double kdu = cam_est.get_kdu();

    double k2ud = 2 * kud;
    double k2du = 2 * kdu;

    for (unsigned int p = 0; p < nbPose; p++) {
      unsigned int q = 6 * p;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint4 = 4 * curPoint;
        double x = cX[curPoint];
        double y = cY[curPoint];
        double z = cZ[curPoint];

        double inv_z = 1 / z;
        double X = x * inv_z;
        double Y = y * inv_z;

        double X2 = X * X;
        double Y2 = Y * Y;
        double XY = X * Y;

        double up = u[curPoint];
        double vp = v[curPoint];

        Pd[curPoint4] = up;
        Pd[curPoint4 + 1] = vp;

        double up0 = up - u0;
        double vp0 = vp - v0;

        double xp0 = up0 * inv_px;
        double xp02 = xp0 * xp0;

        double yp0 = vp0 * inv_py;
        double yp02 = yp0 * yp0;

        double r2du = xp02 + yp02;
        double kr2du = kdu * r2du;

        P[curPoint4] = u0 + px * X - kr2du * (up0);
        P[curPoint4 + 1] = v0 + py * Y - kr2du * (vp0);

        double r2ud = X2 + Y2;
        double kr2ud = 1 + kud * r2ud;

        double Axx = px * (kr2ud + k2ud * X2);
        double Axy = px * k2ud * XY;
        double Ayy = py * (kr2ud + k2ud * Y2);
        double Ayx = py * k2ud * XY;

        Pd[curPoint4 + 2] = up;
        Pd[curPoint4 + 3] = vp;

        P[curPoint4 + 2] = u0 + px * X * kr2ud;
        P[curPoint4 + 3] = v0 + py * Y * kr2ud;

        r += (vpMath::sqr(P[curPoint4] - Pd[curPoint4]) + vpMath::sqr(P[curPoint4 + 1] - Pd[curPoint4 + 1]) +
              vpMath::sqr(P[curPoint4 + 2] - Pd[curPoint4 + 2]) + vpMath::sqr(P[curPoint4 + 3] - Pd[curPoint4 + 3])) *
             0.5;

        unsigned int curInd = curPoint4;
        //---------------
        {
          {
            L[curInd][q] = px * (-inv_z);
            L[curInd][q + 1] = 0;
            L[curInd][q + 2] = px * X * inv_z;
            L[curInd][q + 3] = px * X * Y;
            L[curInd][q + 4] = -px * (1 + X2);
            L[curInd][q + 5] = px * Y;
          }
          {
            L[curInd][nbPose6] = 1 + kr2du + k2du * xp02;
            L[curInd][nbPose6 + 1] = k2du * up0 * yp0 * inv_py;
            L[curInd][nbPose6 + 2] = X + k2du * xp02 * xp0;
            L[curInd][nbPose6 + 3] = k2du * up0 * yp02 * inv_py;
            L[curInd][nbPose6 + 4] = -(up0) * (r2du);
            L[curInd][nbPose6 + 5] = 0;
          }
          curInd++;
          {
            L[curInd][q] = 0;
            L[curInd][q + 1] = py * (-inv_z);
            L[curInd][q + 2] = py * Y * inv_z;
            L[curInd][q + 3] = py * (1 + Y2);
            L[curInd][q + 4] = -py * XY;
            L[curInd][q + 5] = -py * X;
          }
          {
            L[curInd][nbPose6] = k2du * xp0 * vp0 * inv_px;
            L[curInd][nbPose6 + 1] = 1 + kr2du + k2du * yp02;
            L[curInd][nbPose6 + 2] = k2du * vp0 * xp02 * inv_px;
            L[curInd][nbPose6 + 3] = Y + k2du * yp02 * yp0;
            L[curInd][nbPose6 + 4] = -vp0 * r2du;
            L[curInd][nbPose6 + 5] = 0;
          }
          curInd++;
          //---undistorted to distorted
          {
            L[curInd][q] = Axx * (-inv_z);
            L[curInd][q + 1] = Axy * (-inv_z);
            L[curInd][q + 2] = Axx * (X * inv_z) + Axy * (Y * inv_z);
            L[curInd][q + 3] = Axx * X * Y + Axy * (1 + Y2);
            L[curInd][q + 4] = -Axx * (1 + X2) - Axy * XY;
            L[curInd][q + 5] = Axx * Y - Axy * X;
          }
          {
            L[curInd][nbPose6] = 1;
            L[curInd][nbPose6 + 1] = 0;
            L[curInd][nbPose6 + 2] = X * kr2ud;
            L[curInd][nbPose6 + 3] = 0;
            L[curInd][nbPose6 + 4] = 0;
            L[curInd][nbPose6 + 5] = px * X * r2ud;
          }
          curInd++;
          {
            L[curInd][q] = Ayx * (-inv_z);
            L[curInd][q + 1] = Ayy * (-inv_z);
            L[curInd][q + 2] = Ayx * (X * inv_z) + Ayy * (Y * inv_z);
            L[curInd][q + 3] = Ayx * XY + Ayy * (1 + Y2);
            L[curInd][q + 4] = -Ayx * (1 + X2) - Ayy * XY;
            L[curInd][q + 5] = Ayx * Y - Ayy * X;
          }
          {
            L[curInd][nbPose6] = 0;
            L[curInd][nbPose6 + 1] = 1;
            L[curInd][nbPose6 + 2] = 0;
            L[curInd][nbPose6 + 3] = Y * kr2ud;
            L[curInd][nbPose6 + 4] = 0;
            L[curInd][nbPose6 + 5] = py * Y * r2ud;
          }
        } // end interaction
        curPoint++;
      } // end interaction
    }

    vpColVector error;
    error = P - Pd;
    // r = r/nbPointTotal ;

    vpMatrix Lp;
    /*double rank =*/
    L.pseudoInverse(Lp, 1e-10);
    vpColVector e;
    e = Lp * error;
    vpColVector Tc, Tc_v(6 * nbPose);
    Tc = -e * gain;
    for (unsigned int i = 0; i < 6 * nbPose; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithDistortion(px + Tc[nbPose6 + 2], py + Tc[nbPose6 + 3], u0 + Tc[nbPose6],
                                       v0 + Tc[nbPose6 + 1], kud + Tc[nbPose6 + 5], kdu + Tc[nbPose6 + 4]);

    vpColVector Tc_v_Tmp(6);
    for (unsigned int p = 0; p < nbPose; p++) {
      for (unsigned int i = 0; i < 6; i++)
        Tc_v_Tmp[i] = Tc_v[6 * p + i];

      table_cal[p].cMo_dist = vpExponentialMap::direct(Tc_v_Tmp).inverse() * table_cal[p].cMo_dist;
    }
    if (verbose)
      std::cout << " std dev: " << sqrt(r / nbPointTotal) << std::endl;
    // std::cout <<  "   residual: " << r << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }

  // double perViewError;
  // double totalError = 0;
  // int totalPoints = 0;
  for (unsigned int p = 0; p < nbPose; p++) {
    table_cal[p].cam_dist = cam_est;
    // perViewError =
    // table_cal[p].computeStdDeviation_dist(table_cal[p].cMo_dist, cam_est);
    // totalError += perViewError*perViewError * table_cal[p].npt;
    // totalPoints += (int)table_cal[p].npt;
  }
  globalReprojectionError = sqrt(r / (nbPointTotal));

  // Restore ostream format
  std::cout.flags(original_flags);
}
/*!
  \brief compute the distances of the data to the mean obtained.

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eMc : homogeneous matrix between the effector and the camera (input)
*/
void vpCalibration::HandEyeCalibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpHomogeneousMatrix &eMc)
{
  unsigned int nbPose = (unsigned int) cMo.size();
  std::vector<vpTranslationVector> rTo(nbPose); 
  std::vector<vpRotationMatrix> rRo(nbPose); 
  
  // mean  
  vpMatrix meanM(3,3); 
  vpColVector meanTrans(3);
  meanM = 0.0;
  meanTrans = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) 
  {
    vpHomogeneousMatrix rMo = rMe[i] * eMc * cMo[i];
    rMo.extract(rRo[i]);
    meanM += (vpMatrix) rRo[i];
    rMo.extract(rTo[i]);
    meanTrans += (vpColVector) rTo[i];
#if DEBUG_LEVEL2
{
    std::cout << "Pose  " << i << std::endl;
    std::cout << "Translation: " << rTo[i].t() << std::endl;  
    vpThetaUVector rPo(rRo[i]);
    std::cout << "Rotation : theta " << vpMath::deg(sqrt(rPo.sumSquare())) << std::endl << rMo  << std::endl;
    std::cout << "theta U (deg): " << vpMath::deg(rPo[0]) << " " << vpMath::deg(rPo[1]) << " " << vpMath::deg(rPo[2]) << std::endl;
}
#endif
  } 
  meanM /= nbPose;
  meanTrans /= nbPose;
  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;  
  vpColVector sv;
  meanM.pseudoInverse(M, sv, 1e-6, U, V);
  double det = sv[0]*sv[1]*sv[2];
  if (det > 0)  meanM = U * V.t();
  else 
  {
    vpMatrix D(3,3);
    D = 0.0;
    D[0][0] = D[1][1] = 1.0; D[2][2] = -1;
    meanM = U * D * V.t();
  }
  vpRotationMatrix meanRot;
  meanRot = meanM;
#if DEBUG_LEVEL2
  {
  std::cout << "Mean  " << std::endl;
  std::cout << "Translation: " << meanTrans.t() << std::endl;  
  vpThetaUVector P(meanRot);   
  std::cout << "Rotation : theta (deg) = " << vpMath::deg(sqrt(P.sumSquare())) << " Matrice : " << std::endl << meanRot  << std::endl;
  std::cout << "theta U (deg): " << vpMath::deg(P[0]) << " " << vpMath::deg(P[1]) << " " << vpMath::deg(P[2]) << std::endl;
  }
#endif

  // standard deviation, rotational part
  double resRot = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) 
  {
    vpRotationMatrix R = meanRot.t() * rRo[i]; // Rm^T  Ri
    vpThetaUVector P(R); 
    // theta = Riemannian distance d(Rm,Ri)
    double theta = sqrt(P.sumSquare());
    // #if DEBUG_LEVEL2
    {
    printf("Distance theta between rMo(%d) and mean (deg) = %lf\n",i,vpMath::deg(theta));  
    }
  // #endif
    // Euclidean distance d(Rm,Ri) not used
    // theta = 2.0*sqrt(2.0)*sin(theta/2.0);
    resRot += theta*theta;
  }
  resRot = sqrt(resRot/nbPose);
  printf("Mean residual rMo(%d) - rotation (deg) = %lf\n",nbPose,vpMath::deg(resRot));
  // standard deviation, translational part
  double resTrans = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) 
  {
    vpColVector errTrans = ((vpColVector) rTo[i]) - meanTrans;
    resTrans += errTrans.sumSquare();
    // #if DEBUG_LEVEL2
    {
    printf("Distance d between rMo(%d) and mean (m) = %lf\n",i,sqrt(errTrans.sumSquare()));  
    }
  // #endif  
  } 
  resTrans = sqrt(resTrans/nbPose);
  printf("Mean residual rMo(%d) - translation (m) = %lf\n",nbPose,resTrans);
  double resPos = (resRot*resRot + resTrans*resTrans)*nbPose;
  resPos = sqrt(resPos/(2*nbPose));
  printf("Mean residual rMo(%d) - global = %lf\n",nbPose,resPos);
}
/*!
  \brief compute the rotation part (eRc) of hand-eye pose by solving a 
  Procrustes problem [... (theta u)_e ...] = eRc [ ... (theta u)_c ...]

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eRc : rotation matrix  between the effector and the camera (output)
*/
int vpCalibration::HandEyeCalibrationRotationProcrustes(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
{
    // Method by solving the orthogonal Procrustes problem 
    // [... (theta u)_e ...] = eRc [ ... (theta u)_c ...]
    // similar to E^T = eRc C^T below
 
    vpMatrix Et,Ct;
    vpMatrix A;
    unsigned int k = 0;
    unsigned int nbPose = (unsigned int) cMo.size();

    // for all couples ij
    for (unsigned int i = 0; i < nbPose; i++) {
      vpRotationMatrix rRei, ciRo;
      rMe[i].extract(rRei);
      cMo[i].extract(ciRo);
      // std::cout << "rMei: " << std::endl << rMe[i] << std::endl;

      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {
          vpRotationMatrix rRej, cjRo;
          rMe[j].extract(rRej);
          cMo[j].extract(cjRo);
          // std::cout << "rMej: " << std::endl << rMe[j] << std::endl;

          vpRotationMatrix ejRei = rRej.t() * rRei;
          vpThetaUVector ejPei(ejRei);
	  vpColVector xe = ejPei;

          vpRotationMatrix cjRci = cjRo * ciRo.t();
          vpThetaUVector cjPci(cjRci);
	  vpColVector xc = cjPci;
        
          if (k == 0) {
            Et = xe.t(); 
            Ct = xc.t();
          } else {
            Et.stack(xe.t());
            Ct.stack(xc.t());
          }
          k++;
        }
      }
    }
    // std::cout << "Et "  << std::endl << Et << std::endl;
    // std::cout << "Ct "  << std::endl << Ct << std::endl;

    // R obtained from the SVD of (E C^T) with all singular values equal to 1
    A = Et.t() * Ct;
    vpMatrix M, U, V;
    vpColVector sv;
    int rank = A.pseudoInverse(M, sv, 1e-6, U, V);
    if (rank != 3) return -1;
    A = U * V.t();
    eRc = A;  // FC no cast problem here? vpRotationMatrix(A) not accepted...

#if DEBUG_LEVEL2
    {
      vpThetaUVector ePc(eRc);
      std::cout << "Rotation from Procrustes method " << std::endl;
      std::cout << "theta U (deg): " << vpMath::deg(ePc[0]) << " " << vpMath::deg(ePc[1]) << " " << vpMath::deg(ePc[2]) << std::endl;
      // Residual
      vpMatrix residual;
      residual = A * Ct.t() - Et.t();
      //  std::cout << "Residual: " << std::endl << residual << std::endl;
      double res = sqrt(residual.sumSquare()/(residual.getRows()*residual.getCols()));
      printf("Mean residual (rotation) = %lf\n",res);
    }
#endif
    return 0;
}
/*!
  \brief compute the rotation part (eRc) of hand-eye pose by solving a 
  linear system using R. Tsai and R. Lorenz method
  \cite Tsai89a.

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eRc : rotation matrix  between the effector and the camera (output)
*/
int vpCalibration::HandEyeCalibrationRotationTsai(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
{
    vpMatrix A;
    vpColVector B;
    unsigned int nbPose = (unsigned int) cMo.size();
    unsigned int k = 0;
    // for all couples ij
    for (unsigned int i = 0; i < nbPose; i++) {
      vpRotationMatrix rRei, ciRo;
      rMe[i].extract(rRei);
      cMo[i].extract(ciRo);
      // std::cout << "rMei: " << std::endl << rMe[i] << std::endl;

      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {
          vpRotationMatrix rRej, cjRo;
          rMe[j].extract(rRej);
          cMo[j].extract(cjRo);
          // std::cout << "rMej: " << std::endl << rMe[j] << std::endl;

          vpRotationMatrix ejRei = rRej.t() * rRei;
          vpThetaUVector ejPei(ejRei);

          vpRotationMatrix cjRci = cjRo * ciRo.t();
          vpThetaUVector cjPci(cjRci);
	  // std::cout << "theta U (camera) " << cjPci.t() << std::endl;

          vpMatrix As;
          vpColVector b(3);

          As = vpColVector::skew(vpColVector(ejPei) + vpColVector(cjPci));

          b =  (vpColVector)cjPci - (vpColVector) ejPei; // A.40

          if (k == 0) {
            A = As;
            B = b;
          } else {
            A = vpMatrix::stack(A, As);
            B = vpColVector::stack(B, b);
          }
          k++;
        }
      }
    }
#if DEBUG_LEVEL2
    {	  
    std::cout << "Tsai method: system A X = B "  << std::endl;
    std::cout << "A "  << std::endl << A << std::endl;
    std::cout << "B "  << std::endl << B << std::endl;
    }
#endif
    vpMatrix Ap;
    // the linear system A x = B is solved
    // using x = A^+ B 

    int rank = A.pseudoInverse(Ap);
    if (rank != 3) return -1;

    vpColVector x = Ap * B;

    // extraction of theta U
  
    // x = tan(theta/2) U
    double norm =  x.sumSquare();
    double c = 1 / sqrt(1 + norm);  // cos(theta/2)
    double alpha = acos(c);         // theta/2
    norm = 2.0*c/vpMath::sinc(alpha);  // theta / tan(theta/2)
    for (unsigned int i = 0; i < 3; i++) x[i] *= norm;

   // Building of the rotation matrix eRc
    vpThetaUVector xP(x[0], x[1], x[2]);
    eRc = vpRotationMatrix(xP);

#if DEBUG_LEVEL2
    {
       std::cout << "Rotation from Tsai method" << std::endl;
       std::cout << "theta U (deg): " << vpMath::deg(x[0]) << " " << vpMath::deg(x[1]) << " " << vpMath::deg(x[2]) << std::endl;
       // Residual
       for (unsigned int i = 0; i < 3; i++) x[i] /= norm; /* original x */
       vpColVector residual;
       residual = A*x-B;
       // std::cout << "Residual: " << std::endl << residual << std::endl;
       double res = sqrt(residual.sumSquare()/residual.getRows());
       printf("Mean residual (rotation) = %lf\n",res);
    }
#endif
    return 0;
}
/*!
  \brief Old ViSP implementation for computing the rotation part (eRc) 
  of hand-eye pose by solving a linear system using R. Tsai and R. Lorenz method
  \cite Tsai89a.

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eRc : rotation matrix  between the effector and the camera (output)
*/
int vpCalibration::OldHandEyeCalibrationRotationTsai(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
{
    unsigned int nbPose = (unsigned int) cMo.size();
    vpMatrix A;
    vpColVector B;
    vpColVector x;
    unsigned int k = 0;
    // for all couples ij
    for (unsigned int i = 0; i < nbPose; i++) {
      vpRotationMatrix rRei, ciRo;
      rMe[i].extract(rRei);
      cMo[i].extract(ciRo);
      // std::cout << "rMei: " << std::endl << rMe[i] << std::endl;

      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {
          vpRotationMatrix rRej, cjRo;
          rMe[j].extract(rRej);
          cMo[j].extract(cjRo);
          // std::cout << "rMej: " << std::endl << rMe[j] << std::endl;

          vpRotationMatrix rReij = rRej.t() * rRei;

          vpRotationMatrix cijRo = cjRo * ciRo.t();

          vpThetaUVector rPeij(rReij);

          double theta = sqrt(rPeij[0] * rPeij[0] + rPeij[1] * rPeij[1] + rPeij[2] * rPeij[2]);

	  // std::cout << i << " " << j << " " << "ejRei: " << std::endl << rReij << std::endl;
	  // std::cout << "theta (robot) " << theta << std::endl;
	  // std::cout << "theta U (robot) " << rPeij << std::endl;
	  // std::cout << "cjRci: " << std::endl << cijRo.t() << std::endl;

          for (unsigned int m = 0; m < 3; m++)
            rPeij[m] = rPeij[m] * vpMath::sinc(theta / 2);

          vpThetaUVector cijPo(cijRo);
          theta = sqrt(cijPo[0] * cijPo[0] + cijPo[1] * cijPo[1] + cijPo[2] * cijPo[2]);
          for (unsigned int m = 0; m < 3; m++)
            cijPo[m] = cijPo[m] * vpMath::sinc(theta / 2);

	  // std::cout << "theta (camera) " << theta << std::endl;
	  // std::cout << "theta U (camera) " << cijPo.t() << std::endl;

          vpMatrix As;
          vpColVector b(3);

          As = vpColVector::skew(vpColVector(rPeij) + vpColVector(cijPo));

          b = (vpColVector)cijPo - (vpColVector)rPeij; // A.40

          if (k == 0) {
            A = As;
            B = b;
          } else {
            A = vpMatrix::stack(A, As);
            B = vpColVector::stack(B, b);
          }
          k++;
        }
      }
    }
	  
    // std::cout << "A "  << std::endl << A << std::endl;
    // std::cout << "B "  << std::endl << B << std::endl;

    // the linear system is defined
    // x = AtA^-1AtB is solved
    vpMatrix AtA = A.AtA();

    vpMatrix Ap;
    int rank = AtA.pseudoInverse(Ap, 1e-6); 
    if (rank != 3) return -1;

    x = Ap * A.t() * B;
    vpColVector x2 = x; /* pour calcul residu */

    //     {
    //       // Residual
    //       vpColVector residual;
    //       residual = A*x-B;
    //       std::cout << "Residual: " << std::endl << residual << std::endl;

    //       double res = 0;
    //       for (int i=0; i < residual.getRows(); i++)
    // 	res += residual[i]*residual[i];
    //       res = sqrt(res/residual.getRows());
    //       printf("Mean residual = %lf\n",res);
    //     }

    // extraction of theta and U
    double theta;
    double d = x.sumSquare();
    for (unsigned int i = 0; i < 3; i++)
      x[i] = 2 * x[i] / sqrt(1 + d);
    theta = sqrt(x.sumSquare()) / 2;
    theta = 2 * asin(theta);
    // if (theta !=0)
    if (std::fabs(theta) > std::numeric_limits<double>::epsilon()) {
      for (unsigned int i = 0; i < 3; i++)
        x[i] *= theta / (2 * sin(theta / 2));
    } else
      x = 0;
    
   // Building of the rotation matrix eRc
    vpThetaUVector xP(x[0], x[1], x[2]);
    eRc = vpRotationMatrix(xP);

#if DEBUG_LEVEL2
    {
       std::cout << "Rotation from Old Tsai method" << std::endl;
       std::cout << "theta U (deg): " << vpMath::deg(x[0]) << " " << vpMath::deg(x[1]) << " " << vpMath::deg(x[2]) << std::endl;
       // Residual
       vpColVector residual;
       residual = A*x2-B;
       // std::cout << "Residual: " << std::endl << residual << std::endl;
       double res = sqrt(residual.sumSquare()/residual.getRows());
       printf("Mean residual (rotation) = %lf\n",res);
    }
#endif
    return 0;
}
/*!
  \brief compute the translation part (eTc) of hand-eye pose by solving a 
  linear system (see for instance R. Tsai and R. Lorenz method)
  \cite Tsai89a.

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eRc : rotation matrix  between the effector and the camera (input)
  \param eTc : translation  between the effector and the camera (output)
*/
int vpCalibration::HandEyeCalibrationTranslation(const std::vector<vpHomogeneousMatrix> &cMo, 
                                  const std::vector<vpHomogeneousMatrix> &rMe,
				  vpRotationMatrix &eRc, 
                                  vpTranslationVector &eTc)
{
    vpMatrix I3(3,3);
    I3.eye();
    unsigned int k = 0;
    unsigned int nbPose = (unsigned int)cMo.size();
    vpMatrix A(3*nbPose,3);
    vpColVector B(3*nbPose);
    // Building of the system for the translation estimation
    // for all couples ij
    for (unsigned int i = 0; i < nbPose; i++) {
      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {

	  vpHomogeneousMatrix ejMei = rMe[j].inverse() * rMe[i];
	  vpHomogeneousMatrix cjMci = cMo[j] * cMo[i].inverse();

          vpRotationMatrix ejRei, cjRci;
          vpTranslationVector ejTei, cjTci;

	  ejMei.extract(ejRei);
	  ejMei.extract(ejTei);

	  cjMci.extract(cjRci);
	  cjMci.extract(cjTci);

          vpMatrix a = vpMatrix(ejRei) - I3;
          vpTranslationVector b = eRc * cjTci - ejTei;

          if (k == 0) {
            A = a;
            B = b;
          } else {
            A = vpMatrix::stack(A, a);
            B = vpColVector::stack(B, b);
          }
          k++;
        }
      }
    }

    // the linear system A x = B is solved
    // using x = A^+ B  
    vpMatrix Ap;
    int rank = A.pseudoInverse(Ap);  
    if (rank != 3) return -1;

    vpColVector x = Ap * B;
    eTc = (vpTranslationVector) x;

#if DEBUG_LEVEL2     
{
    printf("New Hand-eye calibration : ");
    std::cout << "Translation: " << eTc[0] << " " << eTc[1] << " " << eTc[2] << std::endl;
    // residual
    vpColVector residual;
    residual = A*x-B;
    // std::cout << "Residual: " << std::endl << residual << std::endl;
    double res = sqrt(residual.sumSquare()/residual.getRows());
    printf("Mean residual (translation) = %lf\n",res);
}
#endif   
 return 0;
}

/*!
  \brief Old method to compute the translation part (eTc) of hand-eye pose 
  by solving a linear system (see for instance R. Tsai and R. Lorenz method)
  \cite Tsai89a.

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eRc : rotation matrix  between the effector and the camera (input)
  \param eTc : translation  between the effector and the camera (output)
*/
int vpCalibration::OldHandEyeCalibrationTranslation(const std::vector<vpHomogeneousMatrix> &cMo, 
                                  const std::vector<vpHomogeneousMatrix> &rMe,
				  vpRotationMatrix &eRc, 
                                  vpTranslationVector &eTc)
{
    vpMatrix A;
    vpColVector B;
    // Building of the system for the translation estimation
    // for all couples ij
    vpRotationMatrix I3;
    I3.eye();
    int k = 0;
    unsigned int nbPose = (unsigned int)cMo.size();

    for (unsigned int i = 0; i < nbPose; i++) {
      vpRotationMatrix rRei, ciRo;
      vpTranslationVector rTei, ciTo;
      rMe[i].extract(rRei);
      cMo[i].extract(ciRo);
      rMe[i].extract(rTei);
      cMo[i].extract(ciTo);

      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {

          vpRotationMatrix rRej, cjRo;
          rMe[j].extract(rRej);
          cMo[j].extract(cjRo);

          vpTranslationVector rTej, cjTo;
          rMe[j].extract(rTej);
          cMo[j].extract(cjTo);

          vpRotationMatrix rReij = rRej.t() * rRei;

          vpTranslationVector rTeij = rTej + (-rTei);

          rTeij = rRej.t() * rTeij;

          vpMatrix a = vpMatrix(rReij) - vpMatrix(I3);

          vpTranslationVector b;
          b = eRc * cjTo - rReij * eRc * ciTo + rTeij;

          if (k == 0) {
            A = a;
            B = b;
          } else {
            A = vpMatrix::stack(A, a);
            B = vpColVector::stack(B, b);
          }
          k++;
        }
      }
    }

    // the linear system is solved
    // x = AtA^-1AtB is solved
    vpMatrix AtA = A.AtA();
    vpMatrix Ap;
    vpColVector AeTc;
    int rank = AtA.pseudoInverse(Ap, 1e-6);
    if (rank != 3) return -1;

    AeTc = Ap * A.t() * B;
    eTc = (vpTranslationVector) AeTc;    

#if DEBUG_LEVEL2     
{ 
    printf("Old Hand-eye calibration : ");
    std::cout << "Translation: " << eTc[0] << " " << eTc[1] << " " << eTc[2] << std::endl;

    // residual
    vpColVector residual;
    residual = A*AeTc-B;
    // std::cout << "Residual: " << std::endl << residual << std::endl;
    double res = 0;
    for (unsigned int i=0; i < residual.getRows(); i++)
     	res += residual[i]*residual[i];
    res = sqrt(res/residual.getRows());
    printf("Mean residual (translation) = %lf\n",res);
}
#endif
 return 0;
}
/*!
  \brief Compute the set of errors minimised by VSS

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eMc : homogeneous matrix between the effector and the camera (input)
  \param err: set of errors minimised by VVS (3 for rotation, 3 for translation, etc.) (output)
  \param resRot : mean of the rotation part (output)
  \param resTrans : mean of the translation part (output)
  \param resPos : mean of the global error (output)
*/

double vpCalibration::HandEyeCalibrationErrVVS(const std::vector<vpHomogeneousMatrix> &cMo,  const std::vector<vpHomogeneousMatrix> &rMe,
	   vpHomogeneousMatrix &eMc, vpColVector &errVVS)
{
  unsigned int nbPose = (unsigned int) cMo.size();
  vpMatrix I3(3,3);
  I3.eye();
  vpRotationMatrix eRc;
  vpTranslationVector eTc;
  eMc.extract(eRc);
  eMc.extract(eTc);

  unsigned int k = 0;
  for (unsigned int i = 0; i < nbPose; i++) {
    for (unsigned int j = 0; j < nbPose; j++) {
      if (j > i) // we don't use two times same couples...
      {
        vpColVector s(3);

        vpHomogeneousMatrix ejMei = rMe[j].inverse() * rMe[i];
        vpHomogeneousMatrix cjMci = cMo[j] * cMo[i].inverse();

        vpRotationMatrix ejRei, cjRci;
        vpTranslationVector ejTei, cjTci;

        ejMei.extract(ejRei);
        vpThetaUVector ejPei(ejRei);
        ejMei.extract(ejTei);

        cjMci.extract(cjRci);
        vpThetaUVector cjPci(cjRci);
        cjMci.extract(cjTci);
        // terms due to rotation
        s = vpMatrix(eRc) * vpColVector(cjPci) - vpColVector(ejPei);
        if (k == 0) {
          errVVS = s;
        } else {
          errVVS = vpColVector::stack(errVVS, s);
        }
        k++;
        // terms due to translation
        s = (vpMatrix(ejRei) - I3) * eTc - eRc * cjTci + ejTei; 
        errVVS = vpColVector::stack(errVVS, s);	  
      } // enf if i > j
    } // end for j
  } // end for i
  
  double resRot, resTrans, resPos; 
  resRot = resTrans = resPos = 0.0;
  for (unsigned int i=0; i < (unsigned int) errVVS.size() ; i += 6)
  {
    resRot += errVVS[i]*errVVS[i];
    resRot += errVVS[i+1]*errVVS[i+1];
    resRot += errVVS[i+2]*errVVS[i+2];
    resTrans += errVVS[i+3]*errVVS[i+3];
    resTrans += errVVS[i+4]*errVVS[i+4];
    resTrans += errVVS[i+5]*errVVS[i+5];
  }
  resPos = resRot + resTrans;
  resRot = sqrt(resRot*2/errVVS.size());
  resTrans = sqrt(resTrans*2/errVVS.size());
  resPos = sqrt(resPos/errVVS.size());
#if DEBUG_LEVEL1
  {
     printf("Mean VVS residual - rotation (deg) = %lf\n",vpMath::deg(resRot));
     printf("Mean VVS residual - translation = %lf\n",resTrans);
     printf("Mean VVS residual - global = %lf\n",resPos);
  }
#endif
  return resPos; 
}

/*!
  \brief Hand-Eye Calibration by VVS

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eMc : homogeneous matrix between the effector and the camera (input and output)
*/
#define NB_ITER_MAX 30

int vpCalibration::HandEyeCalibrationVVS(const std::vector<vpHomogeneousMatrix> &cMo, 
                                  const std::vector<vpHomogeneousMatrix> &rMe,
				  vpHomogeneousMatrix &eMc)
{
  unsigned int it = 0;
  double res = 1.0;
  unsigned int nbPose = (unsigned int) cMo.size();
  vpColVector err; 
  vpMatrix L;
  vpMatrix I3(3,3);
  I3.eye();
  vpRotationMatrix eRc;
  vpTranslationVector eTc;
  eMc.extract(eRc);
  eMc.extract(eTc);

  /* FC : on recalcule 2 fois tous les ejMei et cjMci à chaque itération 
    alors qu'ils sont constants. Ce serait sans doute mieux de les 
    calculer une seule fois et de les stocker. Pourraient alors servir 
    dans les autres fonctions HandEye. A voir si vraiment intéressant vu la 
    combinatoire. Idem pour les theta u */
  while ((res > 1e-7) && (it < NB_ITER_MAX))
  {
    /* compute s - s^* */
    vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, err);
    /* compute L_s */
    unsigned int k = 0;
    for (unsigned int i = 0; i < nbPose; i++) {
      for (unsigned int j = 0; j < nbPose; j++) {
        if (j > i) // we don't use two times same couples...
        {
	  vpMatrix Ls(3,6),Lv(3,3),Lw(3,3);

	  vpHomogeneousMatrix ejMei = rMe[j].inverse() * rMe[i];
	  vpHomogeneousMatrix cjMci = cMo[j] * cMo[i].inverse();

          vpRotationMatrix ejRei;
  	  ejMei.extract(ejRei);
          vpThetaUVector cjPci(cjMci);
 
          vpTranslationVector cjTci;

	  cjMci.extract(cjTci);
	  // terms due to rotation
	  //Lv.diag(0.0); // 
	  Lv = 0.0;
	  Lw = -vpMatrix(eRc) * vpColVector::skew(vpColVector(cjPci));
	  for (unsigned int m=0;m<3;m++)
	    for (unsigned int n=0;n<3;n++) 
	    {
	      Ls[m][n] = Lv[m][n];
	      Ls[m][n+3] = Lw[m][n];
	    }
          if (k == 0) {
            L = Ls;
          } else {
            L = vpMatrix::stack(L,Ls);
          }
          k++;
	  // terms due to translation
	  Lv = (vpMatrix(ejRei) - I3) * vpMatrix(eRc);
	  Lw =  vpMatrix(eRc) * vpColVector::skew(vpColVector(cjTci));
	  for (unsigned int m=0;m<3;m++)
	    for (unsigned int n=0;n<3;n++) 
	    {
	      Ls[m][n] = Lv[m][n];
	      Ls[m][n+3] = Lw[m][n];
	    }
          L = vpMatrix::stack(L,Ls);

	} // enf if i > j
      } // end for j
     } // end for i
     double lambda = 0.9;
     vpMatrix Lp;
     int rank = L.pseudoInverse(Lp);
     if (rank != 6) return -1;

     vpColVector e = Lp * err;
     vpColVector v = - e * lambda;
     //  std::cout << "e: "  << e.t() << std::endl;
     eMc = eMc * vpExponentialMap::direct(v);
     eMc.extract(eRc);
     eMc.extract(eTc);
     res = sqrt(v.sumSquare()/v.getRows());
     it++;
   } // end while
#if DEBUG_LEVEL2
   {
      printf(" Iteration number for NL hand-eye minimisation : %d\n",it);
      vpThetaUVector ePc(eRc);
      std::cout << "theta U (deg): " << vpMath::deg(ePc[0]) << " " << vpMath::deg(ePc[1]) << " " << vpMath::deg(ePc[2]) << std::endl;
       std::cout << "Translation: " << eTc[0] << " " << eTc[1] << " " << eTc[2] << std::endl;
       // Residual
       double res = err.sumSquare();
       res = sqrt(res/err.getRows());
       printf("Mean residual (rotation+translation) = %lf\n",res);
   }
#endif
   if (it == NB_ITER_MAX) return 1;  // VVS has not converged before NB_ITER_MAX
   else return 0;
}

#undef NB_ITER_MAX

#define HE_I 0
#define HE_TSAI_OROT 1
#define HE_TSAI_ORNT 2
#define HE_TSAI_NROT 3
#define HE_TSAI_NRNT 4
#define HE_PROCRUSTES_OT 5
#define HE_PROCRUSTES_NT 6

/*!
  \brief calibration method of effector-camera from R. Tsai and R. Lorenz
  \cite Tsai89a.

  Compute extrinsic camera parameters : the constant transformation from
  the effector to the camera frames (eMc).

  \param cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param eMc : homogeneous matrix representing the transformation
  between the effector and the camera (output)
*/
void vpCalibration::calibrationTsai(const std::vector<vpHomogeneousMatrix> &cMo,
                                    const std::vector<vpHomogeneousMatrix> &rMe, vpHomogeneousMatrix &eMc)
{
  if (cMo.size() != rMe.size())
    throw vpCalibrationException(vpCalibrationException::dimensionError, "cMo and rMe have different sizes");

  vpRotationMatrix eRc;
  vpTranslationVector eTc;
  vpColVector errVVS;
  double resPos;

  /* initialisation of eMc to I in case all other methods fail */
  eMc.eye();
  resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
  double vmin = resPos;  // will serve to determine the best method
  int He_method = HE_I;  // will serve to know which is the best method
  vpHomogeneousMatrix eMcMin = eMc;  // best initial estimation for VSS 
  // Method using Old Tsai implementation 
  int err = -1;
  err = vpCalibration::OldHandEyeCalibrationRotationTsai(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Old Tsai method \n");
  else   
  {
    eMc.insert(eRc);
    err = vpCalibration::OldHandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Old Tsai method for Rotation\n");
    else     
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by (old) Tsai, old implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_TSAI_OROT;
      }
    }    
    err = vpCalibration::HandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Old Tsai method for Rotation\n");
    else     
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by (old) Tsai, new implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_TSAI_ORNT;
      }
    }    
  }
  // First method using Tsai formulation 
  err = vpCalibration::HandEyeCalibrationRotationTsai(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Tsai method \n");
  else   
  {
    eMc.insert(eRc);
    err = vpCalibration::OldHandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Tsai method for Rotation\n");
    else     
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by Tsai, old implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_TSAI_NROT;
      }
    }    
    err = vpCalibration::HandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Tsai method for Rotation \n");
    else     
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by Tsai, new implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_TSAI_NRNT;
      }
    }
  }
  // Second method by solving the orthogonal Procrustes problem 
  err = vpCalibration::HandEyeCalibrationRotationProcrustes(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Procrustes method \n");
  else
  {
    eMc.insert(eRc);
    err = vpCalibration::OldHandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Procrustes method for Rotation\n");
    else     
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by Procrustes, old implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif    
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_PROCRUSTES_OT;
      }
    }
    err = vpCalibration::HandEyeCalibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Procrustes method for Rotation\n");
    else         
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
{
      printf("\nRotation by Procrustes, new implementation for translation\n");
      vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
}
#endif    
      resPos = vpCalibration::HandEyeCalibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
        He_method = HE_PROCRUSTES_NT;
      }
    }
  }
    /* For tests in pure simulation without noisy data. Can be removed.
    printf("NE PAS OUBLIER D'ENLEVER LES ERREURS !!!!\n");
    eTc[0] += 0.01;
    eTc[1] -= -0.1; 
    eTc[2] -= 0.05;
    vpThetaUVector afac(eRc);
    afac[0] += 0.05;
    afac[1] -= 0.1;
    afac[2] -= 0.1;
    eRc.buildFrom(afac);
    // FIN PARTIE A SUPPRIMER
    */

  /* determination of the best method in case at least one succeeds */
  eMc = eMcMin;
#if DEBUG_LEVEL1
{
  if (He_method == HE_I) printf("Best method : I !!!, vmin = %lf\n",vmin);
  if (He_method == HE_TSAI_OROT) printf("Best method : TSAI_OROT, vmin = %lf\n",vmin);
  if (He_method == HE_TSAI_ORNT) printf("Best method : TSAI_ORNT, vmin = %lf\n",vmin);
  if (He_method == HE_TSAI_NROT) printf("Best method : TSAI_NROT, vmin = %lf\n",vmin);
  if (He_method == HE_TSAI_NRNT) printf("Best method : TSAI_NRNT, vmin = %lf\n",vmin);
  if (He_method == HE_PROCRUSTES_OT) printf("Best method : PROCRUSTES_OT, vmin = %lf\n",vmin);
  if (He_method == HE_PROCRUSTES_NT) printf("Best method : PROCRUSTES_NT, vmin = %lf\n",vmin);
  vpThetaUVector ePc(eMc);
  std::cout << "theta U (deg): " << vpMath::deg(ePc[0]) << " " << vpMath::deg(ePc[1]) << " " << vpMath::deg(ePc[2]) << std::endl;
  std::cout << "Translation: " << eMc[0][3] << " " << eMc[1][3] << " " << eMc[2][3] << std::endl;
}
#endif

  // Non linear iterative minimization to estimate simultaneouslty eRc and eTc
  err = vpCalibration::HandEyeCalibrationVVS(cMo, rMe, eMc);
  // FC : err : 0 si tout OK, -1 si pb de rang, 1 si pas convergence 
  // FC : if faudrait que la fonction renvoie err 
  if (err != 0) printf("\n Problem in solving Hand-Eye Calibration by VVS \n");
  else
  {
    printf("\nRotation and translation after VVS\n");
    vpCalibration::HandEyeCalibrationVerifrMo(cMo, rMe, eMc);
  }
}

#undef HE_I
#undef HE_TSAI_OROT 
#undef HE_TSAI_ORNT 
#undef HE_TSAI_NROT 
#undef HE_TSAI_NRNT 
#undef HE_PROCRUSTES_OT 
#undef HE_PROCRUSTES_NT 

void vpCalibration::calibVVSMulti(unsigned int nbPose, vpCalibration table_cal[], vpCameraParameters &cam_est,
                                  bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int nbPoint[256];     // number of points by image
  unsigned int nbPointTotal = 0; // total number of points

  unsigned int nbPose6 = 6 * nbPose;

  for (unsigned int i = 0; i < nbPose; i++) {
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  if (nbPointTotal < 4) {
    // vpERROR_TRACE("Not enough point to calibrate");
    throw(vpCalibrationException(vpCalibrationException::notInitializedError, "Not enough point to calibrate"));
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal);
  vpColVector oY(nbPointTotal), cY(nbPointTotal);
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal);
  vpColVector u(nbPointTotal);
  vpColVector v(nbPointTotal);

  vpColVector P(2 * nbPointTotal);
  vpColVector Pd(2 * nbPointTotal);
  vpImagePoint ip;

  unsigned int curPoint = 0; // current point indice
  for (unsigned int p = 0; p < nbPose; p++) {
    std::list<double>::const_iterator it_LoX = table_cal[p].LoX.begin();
    std::list<double>::const_iterator it_LoY = table_cal[p].LoY.begin();
    std::list<double>::const_iterator it_LoZ = table_cal[p].LoZ.begin();
    std::list<vpImagePoint>::const_iterator it_Lip = table_cal[p].Lip.begin();

    for (unsigned int i = 0; i < nbPoint[p]; i++) {
      oX[curPoint] = *it_LoX;
      oY[curPoint] = *it_LoY;
      oZ[curPoint] = *it_LoZ;

      ip = *it_Lip;
      u[curPoint] = ip.get_u();
      v[curPoint] = ip.get_v();

      ++it_LoX;
      ++it_LoY;
      ++it_LoZ;
      ++it_Lip;

      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {

    iter++;
    residu_1 = r;

    double px = cam_est.get_px();
    double py = cam_est.get_py();
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    r = 0;
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint2 = 2 * curPoint;

        cX[curPoint] =
            oX[curPoint] * cMoTmp[0][0] + oY[curPoint] * cMoTmp[0][1] + oZ[curPoint] * cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] =
            oX[curPoint] * cMoTmp[1][0] + oY[curPoint] * cMoTmp[1][1] + oZ[curPoint] * cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] =
            oX[curPoint] * cMoTmp[2][0] + oY[curPoint] * cMoTmp[2][1] + oZ[curPoint] * cMoTmp[2][2] + cMoTmp[2][3];

        Pd[curPoint2] = u[curPoint];
        Pd[curPoint2 + 1] = v[curPoint];

        P[curPoint2] = cX[curPoint] / cZ[curPoint] * px + u0;
        P[curPoint2 + 1] = cY[curPoint] / cZ[curPoint] * py + v0;

        r += (vpMath::sqr(P[curPoint2] - Pd[curPoint2]) + vpMath::sqr(P[curPoint2 + 1] - Pd[curPoint2 + 1]));
        curPoint++;
      }
    }

    vpColVector error;
    error = P - Pd;
    // r = r/nbPointTotal ;

    vpMatrix L(nbPointTotal * 2, nbPose6 + 4);
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      unsigned int q = 6 * p;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint2 = 2 * curPoint;
        unsigned int curPoint21 = curPoint2 + 1;

        double x = cX[curPoint];
        double y = cY[curPoint];
        double z = cZ[curPoint];

        double inv_z = 1 / z;

        double X = x * inv_z;
        double Y = y * inv_z;

        //---------------
        {
          {
            L[curPoint2][q] = px * (-inv_z);
            L[curPoint2][q + 1] = 0;
            L[curPoint2][q + 2] = px * (X * inv_z);
            L[curPoint2][q + 3] = px * X * Y;
            L[curPoint2][q + 4] = -px * (1 + X * X);
            L[curPoint2][q + 5] = px * Y;
          }
          {
            L[curPoint2][nbPose6] = 1;
            L[curPoint2][nbPose6 + 1] = 0;
            L[curPoint2][nbPose6 + 2] = X;
            L[curPoint2][nbPose6 + 3] = 0;
          }
          {
            L[curPoint21][q] = 0;
            L[curPoint21][q + 1] = py * (-inv_z);
            L[curPoint21][q + 2] = py * (Y * inv_z);
            L[curPoint21][q + 3] = py * (1 + Y * Y);
            L[curPoint21][q + 4] = -py * X * Y;
            L[curPoint21][q + 5] = -py * X;
          }
          {
            L[curPoint21][nbPose6] = 0;
            L[curPoint21][nbPose6 + 1] = 1;
            L[curPoint21][nbPose6 + 2] = 0;
            L[curPoint21][nbPose6 + 3] = Y;
          }
        }
        curPoint++;
      } // end interaction
    }
    vpMatrix Lp;
    Lp = L.pseudoInverse(1e-10);

    vpColVector e;
    e = Lp * error;

    vpColVector Tc, Tc_v(nbPose6);
    Tc = -e * gain;

    //   Tc_v =0 ;
    for (unsigned int i = 0; i < nbPose6; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithoutDistortion(px + Tc[nbPose6 + 2], py + Tc[nbPose6 + 3], u0 + Tc[nbPose6],
                                          v0 + Tc[nbPose6 + 1]);

    //    cam.setKd(get_kd() + Tc[10]) ;
    vpColVector Tc_v_Tmp(6);

    for (unsigned int p = 0; p < nbPose; p++) {
      for (unsigned int i = 0; i < 6; i++)
        Tc_v_Tmp[i] = Tc_v[6 * p + i];

      table_cal[p].cMo = vpExponentialMap::direct(Tc_v_Tmp, 1).inverse() * table_cal[p].cMo;
    }
    if (verbose)
      std::cout << " std dev " << sqrt(r / nbPointTotal) << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }
  for (unsigned int p = 0; p < nbPose; p++) {
    table_cal[p].cMo_dist = table_cal[p].cMo;
    table_cal[p].cam = cam_est;
    table_cal[p].cam_dist = cam_est;
    double deviation, deviation_dist;
    table_cal[p].computeStdDeviation(deviation, deviation_dist);
  }
  if (verbose)
    std::cout << " Global std dev " << sqrt(r / nbPointTotal) << std::endl;

  // Restore ostream format
  std::cout.flags(original_flags);
}

void vpCalibration::calibVVSWithDistortionMulti(unsigned int nbPose, vpCalibration table_cal[],
                                                vpCameraParameters &cam_est, bool verbose)
{
  std::ios::fmtflags original_flags(std::cout.flags());
  std::cout.precision(10);
  unsigned int nbPoint[1024];    // number of points by image
  unsigned int nbPointTotal = 0; // total number of points

  unsigned int nbPose6 = 6 * nbPose;
  for (unsigned int i = 0; i < nbPose; i++) {
    nbPoint[i] = table_cal[i].npt;
    nbPointTotal += nbPoint[i];
  }

  if (nbPointTotal < 4) {
    // vpERROR_TRACE("Not enough point to calibrate");
    throw(vpCalibrationException(vpCalibrationException::notInitializedError, "Not enough point to calibrate"));
  }

  vpColVector oX(nbPointTotal), cX(nbPointTotal);
  vpColVector oY(nbPointTotal), cY(nbPointTotal);
  vpColVector oZ(nbPointTotal), cZ(nbPointTotal);
  vpColVector u(nbPointTotal);
  vpColVector v(nbPointTotal);

  vpColVector P(4 * nbPointTotal);
  vpColVector Pd(4 * nbPointTotal);
  vpImagePoint ip;

  unsigned int curPoint = 0; // current point indice
  for (unsigned int p = 0; p < nbPose; p++) {
    std::list<double>::const_iterator it_LoX = table_cal[p].LoX.begin();
    std::list<double>::const_iterator it_LoY = table_cal[p].LoY.begin();
    std::list<double>::const_iterator it_LoZ = table_cal[p].LoZ.begin();
    std::list<vpImagePoint>::const_iterator it_Lip = table_cal[p].Lip.begin();

    for (unsigned int i = 0; i < nbPoint[p]; i++) {
      oX[curPoint] = *it_LoX;
      oY[curPoint] = *it_LoY;
      oZ[curPoint] = *it_LoZ;

      ip = *it_Lip;
      u[curPoint] = ip.get_u();
      v[curPoint] = ip.get_v();

      ++it_LoX;
      ++it_LoY;
      ++it_LoZ;
      ++it_Lip;
      curPoint++;
    }
  }
  //  double lambda = 0.1 ;
  unsigned int iter = 0;

  double residu_1 = 1e12;
  double r = 1e12 - 1;
  while (vpMath::equal(residu_1, r, threshold) == false && iter < nbIterMax) {
    iter++;
    residu_1 = r;

    r = 0;
    curPoint = 0; // current point indice
    for (unsigned int p = 0; p < nbPose; p++) {
      vpHomogeneousMatrix cMoTmp = table_cal[p].cMo_dist;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        cX[curPoint] =
            oX[curPoint] * cMoTmp[0][0] + oY[curPoint] * cMoTmp[0][1] + oZ[curPoint] * cMoTmp[0][2] + cMoTmp[0][3];
        cY[curPoint] =
            oX[curPoint] * cMoTmp[1][0] + oY[curPoint] * cMoTmp[1][1] + oZ[curPoint] * cMoTmp[1][2] + cMoTmp[1][3];
        cZ[curPoint] =
            oX[curPoint] * cMoTmp[2][0] + oY[curPoint] * cMoTmp[2][1] + oZ[curPoint] * cMoTmp[2][2] + cMoTmp[2][3];

        curPoint++;
      }
    }

    vpMatrix L(nbPointTotal * 4, nbPose6 + 6);
    curPoint = 0; // current point indice
    double px = cam_est.get_px();
    double py = cam_est.get_py();
    double u0 = cam_est.get_u0();
    double v0 = cam_est.get_v0();

    double inv_px = 1 / px;
    double inv_py = 1 / py;

    double kud = cam_est.get_kud();
    double kdu = cam_est.get_kdu();

    double k2ud = 2 * kud;
    double k2du = 2 * kdu;

    for (unsigned int p = 0; p < nbPose; p++) {
      unsigned int q = 6 * p;
      for (unsigned int i = 0; i < nbPoint[p]; i++) {
        unsigned int curPoint4 = 4 * curPoint;
        double x = cX[curPoint];
        double y = cY[curPoint];
        double z = cZ[curPoint];

        double inv_z = 1 / z;
        double X = x * inv_z;
        double Y = y * inv_z;

        double X2 = X * X;
        double Y2 = Y * Y;
        double XY = X * Y;

        double up = u[curPoint];
        double vp = v[curPoint];

        Pd[curPoint4] = up;
        Pd[curPoint4 + 1] = vp;

        double up0 = up - u0;
        double vp0 = vp - v0;

        double xp0 = up0 * inv_px;
        double xp02 = xp0 * xp0;

        double yp0 = vp0 * inv_py;
        double yp02 = yp0 * yp0;

        double r2du = xp02 + yp02;
        double kr2du = kdu * r2du;

        P[curPoint4] = u0 + px * X - kr2du * (up0);
        P[curPoint4 + 1] = v0 + py * Y - kr2du * (vp0);

        double r2ud = X2 + Y2;
        double kr2ud = 1 + kud * r2ud;

        double Axx = px * (kr2ud + k2ud * X2);
        double Axy = px * k2ud * XY;
        double Ayy = py * (kr2ud + k2ud * Y2);
        double Ayx = py * k2ud * XY;

        Pd[curPoint4 + 2] = up;
        Pd[curPoint4 + 3] = vp;

        P[curPoint4 + 2] = u0 + px * X * kr2ud;
        P[curPoint4 + 3] = v0 + py * Y * kr2ud;

        r += (vpMath::sqr(P[curPoint4] - Pd[curPoint4]) + vpMath::sqr(P[curPoint4 + 1] - Pd[curPoint4 + 1]) +
              vpMath::sqr(P[curPoint4 + 2] - Pd[curPoint4 + 2]) + vpMath::sqr(P[curPoint4 + 3] - Pd[curPoint4 + 3])) *
             0.5;

        unsigned int curInd = curPoint4;
        //---------------
        {
          {
            L[curInd][q] = px * (-inv_z);
            L[curInd][q + 1] = 0;
            L[curInd][q + 2] = px * X * inv_z;
            L[curInd][q + 3] = px * X * Y;
            L[curInd][q + 4] = -px * (1 + X2);
            L[curInd][q + 5] = px * Y;
          }
          {
            L[curInd][nbPose6] = 1 + kr2du + k2du * xp02;
            L[curInd][nbPose6 + 1] = k2du * up0 * yp0 * inv_py;
            L[curInd][nbPose6 + 2] = X + k2du * xp02 * xp0;
            L[curInd][nbPose6 + 3] = k2du * up0 * yp02 * inv_py;
            L[curInd][nbPose6 + 4] = -(up0) * (r2du);
            L[curInd][nbPose6 + 5] = 0;
          }
          curInd++;
          {
            L[curInd][q] = 0;
            L[curInd][q + 1] = py * (-inv_z);
            L[curInd][q + 2] = py * Y * inv_z;
            L[curInd][q + 3] = py * (1 + Y2);
            L[curInd][q + 4] = -py * XY;
            L[curInd][q + 5] = -py * X;
          }
          {
            L[curInd][nbPose6] = k2du * xp0 * vp0 * inv_px;
            L[curInd][nbPose6 + 1] = 1 + kr2du + k2du * yp02;
            L[curInd][nbPose6 + 2] = k2du * vp0 * xp02 * inv_px;
            L[curInd][nbPose6 + 3] = Y + k2du * yp02 * yp0;
            L[curInd][nbPose6 + 4] = -vp0 * r2du;
            L[curInd][nbPose6 + 5] = 0;
          }
          curInd++;
          //---undistorted to distorted
          {
            L[curInd][q] = Axx * (-inv_z);
            L[curInd][q + 1] = Axy * (-inv_z);
            L[curInd][q + 2] = Axx * (X * inv_z) + Axy * (Y * inv_z);
            L[curInd][q + 3] = Axx * X * Y + Axy * (1 + Y2);
            L[curInd][q + 4] = -Axx * (1 + X2) - Axy * XY;
            L[curInd][q + 5] = Axx * Y - Axy * X;
          }
          {
            L[curInd][nbPose6] = 1;
            L[curInd][nbPose6 + 1] = 0;
            L[curInd][nbPose6 + 2] = X * kr2ud;
            L[curInd][nbPose6 + 3] = 0;
            L[curInd][nbPose6 + 4] = 0;
            L[curInd][nbPose6 + 5] = px * X * r2ud;
          }
          curInd++;
          {
            L[curInd][q] = Ayx * (-inv_z);
            L[curInd][q + 1] = Ayy * (-inv_z);
            L[curInd][q + 2] = Ayx * (X * inv_z) + Ayy * (Y * inv_z);
            L[curInd][q + 3] = Ayx * XY + Ayy * (1 + Y2);
            L[curInd][q + 4] = -Ayx * (1 + X2) - Ayy * XY;
            L[curInd][q + 5] = Ayx * Y - Ayy * X;
          }
          {
            L[curInd][nbPose6] = 0;
            L[curInd][nbPose6 + 1] = 1;
            L[curInd][nbPose6 + 2] = 0;
            L[curInd][nbPose6 + 3] = Y * kr2ud;
            L[curInd][nbPose6 + 4] = 0;
            L[curInd][nbPose6 + 5] = py * Y * r2ud;
          }
        } // end interaction
        curPoint++;
      } // end interaction
    }

    vpColVector error;
    error = P - Pd;
    // r = r/nbPointTotal ;

    vpMatrix Lp;
    /*double rank =*/
    L.pseudoInverse(Lp, 1e-10);
    vpColVector e;
    e = Lp * error;
    vpColVector Tc, Tc_v(6 * nbPose);
    Tc = -e * gain;
    for (unsigned int i = 0; i < 6 * nbPose; i++)
      Tc_v[i] = Tc[i];

    cam_est.initPersProjWithDistortion(px + Tc[nbPose6 + 2], py + Tc[nbPose6 + 3], u0 + Tc[nbPose6],
                                       v0 + Tc[nbPose6 + 1], kud + Tc[nbPose6 + 5], kdu + Tc[nbPose6 + 4]);

    vpColVector Tc_v_Tmp(6);
    for (unsigned int p = 0; p < nbPose; p++) {
      for (unsigned int i = 0; i < 6; i++)
        Tc_v_Tmp[i] = Tc_v[6 * p + i];

      table_cal[p].cMo_dist = vpExponentialMap::direct(Tc_v_Tmp).inverse() * table_cal[p].cMo_dist;
    }
    if (verbose)
      std::cout << " std dev: " << sqrt(r / nbPointTotal) << std::endl;
    // std::cout <<  "   residual: " << r << std::endl;
  }
  if (iter == nbIterMax) {
    vpERROR_TRACE("Iterations number exceed the maximum allowed (%d)", nbIterMax);
    throw(vpCalibrationException(vpCalibrationException::convergencyError, "Maximum number of iterations reached"));
  }

  for (unsigned int p = 0; p < nbPose; p++) {
    table_cal[p].cam_dist = cam_est;
    table_cal[p].computeStdDeviation_dist(table_cal[p].cMo_dist, cam_est);
  }
  if (verbose)
    std::cout << " Global std dev " << sqrt(r / (nbPointTotal)) << std::endl;

  // Restore ostream format
  std::cout.flags(original_flags);
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
