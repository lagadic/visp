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

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

#include <visp3/vision/vpHandEyeCalibration.h>

/*!
  \deprecated This function is deprecated. You should rather use vpHandEyeCalibration::calibrate().

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
  vpHandEyeCalibration::calibrate(cMo, rMe, eMc);
}

/*!
  \deprecated This function is deprecated. You should rather use vpHandEyeCalibration::calibrate().

  Compute extrinsic camera parameters : the constant transformation from
  the end-effector to the camera frame \f${^e}{\bf M}_c\f$ considering the
  camera model with or without distortion.

  \param[in] table_cal : Vector of vpCalibration that contains for each index
  a couple of \f${^r}{\bf M}_e\f$ (world to end-effector) and \f${^c}{\bf
  M}_o\f$ (camera to object) transformations.
  \param[out] eMc : Estimated pose of the camera in relation to the end-effector considering
  the camera model without distortion.
  \param[out] eMc_dist : Estimated pose of the camera in relation to the end-effector
  considering the model with distortion.
  \return 0 if the computation managed, -1 if less than three poses are provides as
  input.
*/
int vpCalibration::computeCalibrationTsai(const std::vector<vpCalibration> &table_cal, vpHomogeneousMatrix &eMc,
                                          vpHomogeneousMatrix &eMc_dist)
{
  unsigned int nbPose = (unsigned int)table_cal.size();
  if (nbPose > 2) {
    std::vector<vpHomogeneousMatrix> table_cMo(nbPose);
    std::vector<vpHomogeneousMatrix> table_cMo_dist(nbPose);
    std::vector<vpHomogeneousMatrix> table_rMe(nbPose);

    for (unsigned int i = 0; i < nbPose; i++) {
      table_cMo[i] = table_cal[i].cMo;
      table_cMo_dist[i] = table_cal[i].cMo_dist;
      table_rMe[i] = table_cal[i].rMe;
    }
    vpHandEyeCalibration::calibrate(table_cMo, table_rMe, eMc);
    vpHandEyeCalibration::calibrate(table_cMo_dist, table_rMe, eMc_dist);

    return 0;
  } else {
    throw (vpException(vpException::dimensionError, "At least 3 images are needed to compute hand-eye calibration !\n"));
  }
}

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)


#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
