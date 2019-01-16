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
 * Hand-eye calibration.
 *
 * Authors:
 * Francois Chaumette
 * Fabien Spindler
 *
 *****************************************************************************/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

#include <visp3/vision/vpHandEyeCalibration.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

/*!
  \brief Compute the distances of the data to the mean obtained.

  \param[in] cMo : Vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : Vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[out] eMc : Homogeneous matrix between the effector and the camera (input)
*/
void vpHandEyeCalibration::calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe, vpHomogeneousMatrix &eMc)
{
  unsigned int nbPose = (unsigned int) cMo.size();
  std::vector<vpTranslationVector> rTo(nbPose);
  std::vector<vpRotationMatrix> rRo(nbPose);

  for (unsigned int i = 0; i < nbPose; i++) {
    vpHomogeneousMatrix rMo = rMe[i] * eMc * cMo[i];
    rRo[i] = rMo.getRotationMatrix();
    rTo[i] = rMo.getTranslationVector();
  }
  vpRotationMatrix meanRot = vpRotationMatrix::mean(rRo);
  vpTranslationVector meanTrans = vpTranslationVector::mean(rTo);

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
  for (unsigned int i = 0; i < nbPose; i++) {
    vpRotationMatrix R = meanRot.t() * rRo[i]; // Rm^T  Ri
    vpThetaUVector P(R);
    // theta = Riemannian distance d(Rm,Ri)
    double theta = sqrt(P.sumSquare());
    std::cout << "Distance theta between rMo(" << i << ") and mean (deg) = " << vpMath::deg(theta) << std::endl;
    // Euclidean distance d(Rm,Ri) not used
    // theta = 2.0*sqrt(2.0)*sin(theta/2.0);
    resRot += theta*theta;
  }
  resRot = sqrt(resRot/nbPose);
  std::cout << "Mean residual rMo(" << nbPose << ") - rotation (deg) = " << vpMath::deg(resRot) << std::endl;
  // standard deviation, translational part
  double resTrans = 0.0;
  for (unsigned int i = 0; i < nbPose; i++) {
    vpColVector errTrans = ((vpColVector) rTo[i]) - meanTrans;
    resTrans += errTrans.sumSquare();
    std::cout << "Distance d between rMo(" << i << ") and mean (m) = " << sqrt(errTrans.sumSquare()) << std::endl;
  }
  resTrans = sqrt(resTrans/nbPose);
  std::cout << "Mean residual rMo(" << nbPose << ") - translation (m) = " << resTrans << std::endl;
  double resPos = (resRot*resRot + resTrans*resTrans)*nbPose;
  resPos = sqrt(resPos/(2*nbPose));
  std::cout << "Mean residual rMo(" << nbPose << ") - global = " << resPos << std::endl;
}

/*!
  \brief Compute the rotation part (eRc) of hand-eye pose by solving a
  Procrustes problem [... (theta u)_e ...] = eRc [ ... (theta u)_c ...]

  \param[in] cMo : Vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : Vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[out] eRc : Rotation matrix  between the effector and the camera (output)
*/
int vpHandEyeCalibration::calibrationRotationProcrustes(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
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
  eRc = vpRotationMatrix(A);

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
  \brief Compute the rotation part (eRc) of hand-eye pose by solving a
  linear system using R. Tsai and R. Lorenz method
  \cite Tsai89a.

  \param[in] cMo : Vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : Vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[out] eRc : Rotation matrix  between the effector and the camera (output)
*/
int vpHandEyeCalibration::calibrationRotationTsai(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
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

  \param[in] cMo : Vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : Vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[ou] eRc : Rotation matrix  between the effector and the camera (output)
*/
int vpHandEyeCalibration::calibrationRotationTsaiOld(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,vpRotationMatrix &eRc)
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
      if (j > i) { // we don't use two times same couples...
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

        for (unsigned int m = 0; m < 3; m++) {
          rPeij[m] = rPeij[m] * vpMath::sinc(theta / 2);
        }

        vpThetaUVector cijPo(cijRo);
        theta = sqrt(cijPo[0] * cijPo[0] + cijPo[1] * cijPo[1] + cijPo[2] * cijPo[2]);
        for (unsigned int m = 0; m < 3; m++) {
          cijPo[m] = cijPo[m] * vpMath::sinc(theta / 2);
        }

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
  \brief Compute the translation part (eTc) of hand-eye pose by solving a
  linear system (see for instance R. Tsai and R. Lorenz method)
  \cite Tsai89a.

  \param[in] cMo : Vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : Vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[out] eRc : Rotation matrix  between the effector and the camera (input)
  \param[out] eTc : Translation  between the effector and the camera (output)
*/
int vpHandEyeCalibration::calibrationTranslation(const std::vector<vpHomogeneousMatrix> &cMo,
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
      if (j > i) { // we don't use two times same couples...
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

  \param[in] cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene (input)
  \param[in] rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator) (input). Must be the same size as cMo.
  \param[out] eRc : rotation matrix  between the effector and the camera (input)
  \param[out] eTc : translation  between the effector and the camera (output)
*/
int vpHandEyeCalibration::calibrationTranslationOld(const std::vector<vpHomogeneousMatrix> &cMo,
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
  \brief Compute the set of errors minimised by VVS.

  \param[in] cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene.
  \param[in] rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator). Must be the same size as cMo.
  \param[in] eMc : homogeneous matrix between the effector and the camera (input)
  \param[out] err: set of errors minimised by VVS (3 for rotation, 3 for translation, etc.) (output)
*/

double vpHandEyeCalibration::calibrationErrVVS(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                                               const vpHomogeneousMatrix &eMc, vpColVector &errVVS)
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
  \brief Hand-Eye Calibration by VVS.

  \param[in] cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene.
  \param[in] rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator). Must be the same size as cMo.
  \param[in,out] eMc : homogeneous matrix between the effector and the camera.
*/
#define NB_ITER_MAX 30

int vpHandEyeCalibration::calibrationVVS(const std::vector<vpHomogeneousMatrix> &cMo,
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

  /* FC : on recalcule 2 fois tous les ejMei et cjMci a chaque iteration
    alors qu'ils sont constants. Ce serait sans doute mieux de les
    calculer une seule fois et de les stocker. Pourraient alors servir
    dans les autres fonctions HandEye. A voir si vraiment interessant vu la
    combinatoire. Idem pour les theta u */
  while ((res > 1e-7) && (it < NB_ITER_MAX))
  {
    /* compute s - s^* */
    vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, err);
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
  Compute extrinsic camera parameters : the constant transformation from
  the effector to the camera frames (eMc).

  \param[in] cMo : vector of homogeneous matrices representing the transformation
  between the camera and the scene.
  \param[in] rMe : vector of homogeneous matrices representing the transformation
  between the effector (where the camera is fixed) and the reference
  coordinates (base of the manipulator). Must be the same size as cMo.
  \param[out] eMc : homogeneous matrix representing the transformation
  between the effector and the camera (output)

  \return 0 if calibration succeed, -1 if the system is not full rank, 1 if the algorithm doesn't converge.
*/
int vpHandEyeCalibration::calibrate(const std::vector<vpHomogeneousMatrix> &cMo,
                                    const std::vector<vpHomogeneousMatrix> &rMe, vpHomogeneousMatrix &eMc)
{
  if (cMo.size() != rMe.size())
    throw vpException(vpException::dimensionError, "cMo and rMe have different sizes");

  vpRotationMatrix eRc;
  vpTranslationVector eTc;
  vpColVector errVVS;
  double resPos;

  /* initialisation of eMc to I in case all other methods fail */
  eMc.eye();
  resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
  double vmin = resPos;  // will serve to determine the best method
#if DEBUG_LEVEL1
  int He_method = HE_I;  // will serve to know which is the best method
#endif
  vpHomogeneousMatrix eMcMin = eMc;  // best initial estimation for VSS
  // Method using Old Tsai implementation
  int err = vpHandEyeCalibration::calibrationRotationTsaiOld(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Old Tsai method \n");
  else
  {
    eMc.insert(eRc);
    err = vpHandEyeCalibration::calibrationTranslationOld(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Old Tsai method for Rotation\n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by (old) Tsai, old implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
#if DEBUG_LEVEL1
        He_method = HE_TSAI_OROT;
#endif
      }
    }
    err = vpHandEyeCalibration::calibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Old Tsai method for Rotation\n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by (old) Tsai, new implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
#if DEBUG_LEVEL1
        He_method = HE_TSAI_ORNT;
#endif
      }
    }
  }
  // First method using Tsai formulation
  err = vpHandEyeCalibration::calibrationRotationTsaiOld(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Tsai method \n");
  else
  {
    eMc.insert(eRc);
    err = vpHandEyeCalibration::calibrationTranslationOld(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Tsai method for Rotation\n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by Tsai, old implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
#if DEBUG_LEVEL1
        He_method = HE_TSAI_NROT;
#endif
      }
    }
    err = vpHandEyeCalibration::calibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Tsai method for Rotation \n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by Tsai, new implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
#if DEBUG_LEVEL1
        He_method = HE_TSAI_NRNT;
#endif
      }
    }
  }
  // Second method by solving the orthogonal Procrustes problem
  err = vpHandEyeCalibration::calibrationRotationProcrustes(cMo, rMe, eRc);
  if (err != 0) printf("\n Problem in solving Hand-Eye Rotation by Procrustes method \n");
  else
  {
    eMc.insert(eRc);
    err = vpHandEyeCalibration::calibrationTranslationOld(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation by Old Tsai method after Procrustes method for Rotation\n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by Procrustes, old implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        vmin = resPos;
        eMcMin = eMc;
#if DEBUG_LEVEL1
        He_method = HE_PROCRUSTES_OT;
#endif
      }
    }
    err = vpHandEyeCalibration::calibrationTranslation(cMo, rMe, eRc, eTc);
    if (err != 0) printf("\n Problem in solving Hand-Eye Translation after Procrustes method for Rotation\n");
    else
    {
      eMc.insert(eTc);
#if DEBUG_LEVEL1
      {
        printf("\nRotation by Procrustes, new implementation for translation\n");
        vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
      }
#endif
      resPos = vpHandEyeCalibration::calibrationErrVVS(cMo, rMe, eMc, errVVS);
      if (resPos < vmin)
      {
        eMcMin = eMc;
#if DEBUG_LEVEL1
        vmin = resPos;
        He_method = HE_PROCRUSTES_NT;
#endif
      }
    }
  }

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
  err = vpHandEyeCalibration::calibrationVVS(cMo, rMe, eMc);
  // FC : err : 0 si tout OK, -1 si pb de rang, 1 si pas convergence
  if (err != 0) printf("\n Problem in solving Hand-Eye Calibration by VVS \n");
  else
  {
    printf("\nRotation and translation after VVS\n");
    vpHandEyeCalibration::calibrationVerifrMo(cMo, rMe, eMc);
  }
  return err;
}

#undef HE_I
#undef HE_TSAI_OROT
#undef HE_TSAI_ORNT
#undef HE_TSAI_NROT
#undef HE_TSAI_NRNT
#undef HE_PROCRUSTES_OT
#undef HE_PROCRUSTES_NT

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
