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

#include <visp3/vision/vpPose.h>

BEGIN_VISP_NAMESPACE

/**********************************************************************/
/*  FONCTION     :    CalculTranslation       */
/*  ROLE        : Calcul de la translation entre la   */
/*                                camera et l'outil connaissant la    */
/*                                rotation            */
/**********************************************************************/

static void calculTranslation(vpMatrix &a, vpMatrix &b, unsigned int nl, unsigned int nc1, unsigned int nc3,
                              vpColVector &x1, vpColVector &x2)
{
  unsigned int i, j;
  const unsigned int nbRows = 3;
  vpMatrix ct(nbRows, nl);
  for (i = 0; i < nbRows; ++i) {
    for (j = 0; j < nl; ++j) {
      ct[i][j] = b[j][i + nc3];
    }
  }

  vpMatrix c;
  c = ct.t();

  vpMatrix ctc;
  ctc = ct * c;

  vpMatrix ctc1; // (C^T C)^(-1)
  ctc1 = ctc.inverseByLU();

  vpMatrix cta;
  vpMatrix ctb;
  cta = ct * a; /* C^T A  */
  ctb = ct * b; /* C^T B  */

  vpColVector X2(nc3);
  vpMatrix CTB(nc1, nc3);
  for (i = 0; i < nc1; ++i) {
    for (j = 0; j < nc3; ++j) {
      CTB[i][j] = ctb[i][j];
    }
  }

  for (j = 0; j < nc3; ++j) {
    X2[j] = x2[j];
  }

  vpColVector sv;           // C^T A X1 + C^T B X2)
  sv = (cta * x1) + (CTB * X2); // C^T A X1 + C^T B X2)

  vpColVector X3; /* X3 = - (C^T C )^{-1} C^T (A X1 + B X2) */
  X3 = -ctc1 * sv;

  for (i = 0; i < nc1; ++i) {
    x2[i + nc3] = X3[i];
  }
}

//*********************************************************************
//   FONCTION LAGRANGE :
//   -------------------
// Resolution d'un systeme lineaire de la forme A x1 + B x2 = 0
//      sous la contrainte || x1 || = 1
//      ou A est de dimension nl x nc1 et B nl x nc2
//*********************************************************************
static void lagrange(vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
  unsigned int i, imin;

  vpMatrix ata; // A^T A
  ata = a.t() * a;
  vpMatrix btb; // B^T B
  btb = b.t() * b;

  vpMatrix bta; // B^T A
  bta = b.t() * a;

  vpMatrix btb1; // (B^T B)^(-1)

  /* Warning:
  when using btb.inverseByLU() that call cv::inv(cv::DECOMP_LU) with
  OpenCV 3.1.0 and 3.2.0 we notice that OpenCV is not able to compute the
  inverse of the following matrix:

  btb[9,9]=
  0.015925   0.0        0.0030866  0.00035    0.0        0.000041   0.105
  0.0        0.0346242 0.0        0.015925  -0.0050979  0.0        0.00035
  -0.000063   0.0        0.105     -0.0637464 0.0030866 -0.0050979  0.0032301
  0.000041  -0.000063   0.000016   0.0346242 -0.0637464  0.0311185 0.00035
  0.0        0.000041   0.0001     0.0        0.000012   0.01       0.0
  0.0011594 0.0        0.00035   -0.000063   0.0        0.0001    -0.000018
  0.0        0.01      -0.0018040 0.000041  -0.000063   0.000016   0.000012
  -0.000018   0.000005   0.0011594 -0.0018040  0.0004599 0.105      0.0
  0.0346242  0.01       0.0        0.0011594  5.0        0.0        0.13287
  0.0        0.105     -0.0637464  0.0        0.01      -0.0018040  0.0
  5.0       -0.731499 0.0346242 -0.0637464  0.0311185  0.0011594 -0.0018040
  0.0004599  0.13287   -0.731499   0.454006

  That's why instead of using inverseByLU() we are now using pseudoInverse()
  */
#if 0
  if (b.getRows() >= b.getCols()) {
    btb1 = btb.inverseByLU();
  }
  else {
    btb1 = btb.pseudoInverse();
  }
#else
  btb1 = btb.pseudoInverse();
#endif

  vpMatrix r; // (B^T B)^(-1) B^T A
  r = btb1 * bta;

  vpMatrix e; //   - A^T B (B^T B)^(-1) B^T A
  e = -(a.t() * b) * r;

  e += ata; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A

  e.svd(x1, ata); // destructif sur e
  // calcul du vecteur propre de E correspondant a la valeur propre min.
  imin = 0;

  unsigned int v_x1_rows = x1.getRows();
  for (i = 0; i < v_x1_rows; ++i) {
    if (x1[i] < x1[imin]) {
      imin = i;
    }
  }

  unsigned int x1_rows = x1.getRows();
  for (i = 0; i < x1_rows; ++i) {
    x1[i] = ata[i][imin];
  }

  x2 = -(r * x1); // X_2 = - (B^T B)^(-1) B^T A X_1
}

void vpPose::poseLagrangePlan(vpHomogeneousMatrix &cMo, bool *p_isPlan, double *p_a, double *p_b, double *p_c, double *p_d)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

  // determination of the plane equation a X + b Y + c Z + d = 0
  double a, b, c, d;

  // Checking if coplanar has already been called and if the plan coefficients have been given
  bool p_isplan_and_p_a_no_null = (p_isPlan != nullptr) && (p_a != nullptr);
  bool p_b_p_c_p_d_no_null = (p_b != nullptr) && (p_c != nullptr) && (p_d != nullptr);
  if (p_isplan_and_p_a_no_null && p_b_p_c_p_d_no_null) {
    if (*p_isPlan) {
      // All the pointers towards the plan coefficients are different from nullptr => using them in the rest of the method
      a = *p_a;
      b = *p_b;
      c = *p_c;
      d = *p_d;
    }
    else {
      // The call to coplanar that was performed outside vpPose::poseLagrangePlan indicated that the points are not coplanar.
      throw vpException(vpException::fatalError, "Called vpPose::poseLagrangePlan but the call to vpPose::coplanar done outside the method indicated that the points are not coplanar");
    }
  }
  else {
    // At least one of the coefficient is a nullptr pointer => calling coplanar by ourselves
    int coplanarType;
    bool areCoplanar = coplanar(coplanarType, &a, &b, &c, &d);
    if (!areCoplanar) {
      throw vpException(vpException::fatalError, "Called vpPose::poseLagrangePlan but call to vpPose::coplanar indicates that the points are not coplanar");
    }
  }

  if (c < 0.0) { // imposing c greater than or equal to 0
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }
  // to have (a,b,c) as a unit vector if it was not the case
  double n = 1.0 / sqrt((a * a) + (b * b) + (c * c)); // Not possible to have a NaN...
  a *= n;
  b *= n;
  c *= n;
  d *= n;
  // transformation to have object plane with equation Z = 0
  const unsigned int size = 3;
  vpColVector r1(size), r2(size), r3(size);

  r3[index_0] = a;
  r3[index_1] = b;
  r3[index_2] = c;
  // build r1 as a unit vector orthogonal to r3
  double n1 = sqrt(1.0 - (a * a));
  double n2 = sqrt(1.0 - (b * b));
  if (n1 >= n2) {
    r1[index_0] = n1;
    r1[index_1] = (-a * b) / n1;
    r1[index_2] = (-a * c) / n1;
  }
  else {
    r1[index_0] = (-a * b) / n2;
    r1[index_1] = n2;
    r1[index_2] = (-b * c) / n2;
  }

  r2 = vpColVector::crossProd(r3, r1);

  vpHomogeneousMatrix fMo;
  const unsigned int sizeRotation = 3;
  const unsigned int idX = 0, idY = 1, idZ = 2, idTranslation = 3;
  for (unsigned int i = 0; i < sizeRotation; ++i) {
    fMo[idX][i] = r1[i];
    fMo[idY][i] = r2[i];
    fMo[idZ][i] = r3[i];
  }
  fMo[idX][idTranslation] = 0.0;
  fMo[idY][idTranslation] = 0.0;
  fMo[idZ][idTranslation] = d;

  // Build and solve the system
  unsigned int k = 0;
  unsigned int nl = npt * 2;

  const unsigned int nbColsA = 3, nbColsB = 6;
  vpMatrix A(nl, nbColsA);
  vpMatrix B(nl, nbColsB);
  vpPoint P;

  std::list<vpPoint>::const_iterator listp_end = listP.end();
  const unsigned int idHomogeneous = 3, sizeHomogeneous = 4;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;

    // Transform each point in plane Z = 0
    vpColVector Xf, X(sizeHomogeneous);
    X[idX] = P.get_oX();
    X[idY] = P.get_oY();
    X[idZ] = P.get_oZ();
    X[idHomogeneous] = 1.0;
    Xf = fMo * X;
    // build the system
    A[k][index_0] = -Xf[0];
    A[k][index_1] = 0.0;
    A[k][index_2] = Xf[0] * P.get_x();

    A[k + 1][index_0] = 0.0;
    A[k + 1][index_1] = -Xf[0];
    A[k + 1][index_2] = Xf[0] * P.get_y();

    B[k][index_0] = -Xf[1];
    B[k][index_1] = 0.0;
    B[k][index_2] = Xf[1] * P.get_x();
    B[k][index_3] = -1.0;
    B[k][index_4] = 0.0;
    B[k][index_5] = P.get_x();

    B[k + 1][index_0] = 0.0;
    B[k + 1][index_1] = -Xf[1];
    B[k + 1][index_2] = Xf[1] * P.get_y();
    B[k + 1][index_3] = 0.0;
    B[k + 1][index_4] = -1.0;
    B[k + 1][index_5] = P.get_y();

    k += 2;
  }
  const unsigned int sizeX1 = nbColsA, sizeX2 = nbColsB, lastX2 = sizeX2 - 1; // X1 is of the size of A^T A and X2 of B^T B
  vpColVector X1(sizeX1);
  vpColVector X2(sizeX2);

  lagrange(A, B, X1, X2);

  if (X2[lastX2] < 0.0) { /* to obtain Zo > 0  */
    for (unsigned int i = 0; i < sizeX1; ++i) {
      X1[i] = -X1[i];
    }

    for (unsigned int i = 0; i < sizeX2; ++i) {
      X2[i] = -X2[i];
    }
  }
  double s = 0.0;
  for (unsigned int i = 0; i < sizeX1; ++i) {
    s += (X1[i] * X2[i]);
  }
  for (unsigned int i = 0; i < sizeX1; ++i) {
    X2[i] -= (s * X1[i]);
  } /* X1^T X2 = 0  */

  // --comment: s equals 0.0
  s = (X2[index_0] * X2[index_0]) + (X2[index_1] * X2[index_1]) + (X2[index_2] * X2[index_2]); // To avoid a Coverity copy/past error

  if (s < 1e-10) {
    throw(vpException(vpException::divideByZeroError, "Division by zero in Lagrange pose computation "
                      "(planar plane case)"));
  }

  s = 1.0 / sqrt(s);
  const unsigned int val_3 = 3, nc1 = 3, nc3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    X2[i] *= s;
  } /* X2^T X2 is equal to 1  */

  calculTranslation(A, B, nl, nc1, nc3, X1, X2);

  vpHomogeneousMatrix cMf;
  /* X1 x X2 */
  cMf[index_0][index_2] = (X1[index_1] * X2[index_2]) - (X1[index_2] * X2[index_1]);
  cMf[index_1][index_2] = (X1[index_2] * X2[index_0]) - (X1[index_0] * X2[index_2]);
  cMf[index_2][index_2] = (X1[index_0] * X2[index_1]) - (X1[index_1] * X2[index_0]);
  /* calcul de la matrice de passage  */
  for (unsigned int i = 0; i < val_3; ++i) {
    cMf[i][index_0] = X1[i];
    cMf[i][index_1] = X2[i];
    cMf[i][index_3] = X2[i + 3];
  }

  // Apply the transform to go back to object frame
  cMo = cMf * fMo;
}

void vpPose::poseLagrangeNonPlan(vpHomogeneousMatrix &cMo)
{
  try {
    double s;
    unsigned int i;

    unsigned int k = 0;
    const unsigned int twice = 2;
    unsigned int nl = npt * twice;
    const unsigned int npt_min = 6;

    if (npt < npt_min) {
      throw(vpException(vpException::dimensionError,
                        "Lagrange, non planar case, insufficient number of points %d < 6\n", npt));
    }

    const unsigned int nbColsA = 3, nbColsB = 9;
    vpMatrix a(nl, nbColsA);
    vpMatrix b(nl, nbColsB);
    b = 0;

    vpPoint P;
    i = 0;
    std::list<vpPoint>::const_iterator listp_end = listP.end();
    const unsigned int id0 = 0, id1 = 1, id2 = 2;
    const unsigned int id3 = 3, id4 = 4, id5 = 5;
    const unsigned int id6 = 6, id7 = 7, id8 = 8;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
      P = *it;
      a[k][id0] = -P.get_oX();
      a[k][id1] = 0.0;
      a[k][id2] = P.get_oX() * P.get_x();

      a[k + 1][id0] = 0.0;
      a[k + 1][id1] = -P.get_oX();
      a[k + 1][id2] = P.get_oX() * P.get_y();

      b[k][id0] = -P.get_oY();
      b[k][id1] = 0.0;
      b[k][id2] = P.get_oY() * P.get_x();

      b[k][id3] = -P.get_oZ();
      b[k][id4] = 0.0;
      b[k][id5] = P.get_oZ() * P.get_x();

      b[k][id6] = -1.0;
      b[k][id7] = 0.0;
      b[k][id8] = P.get_x();

      b[k + 1][id0] = 0.0;
      b[k + 1][id1] = -P.get_oY();
      b[k + 1][id2] = P.get_oY() * P.get_y();

      b[k + 1][id3] = 0.0;
      b[k + 1][id4] = -P.get_oZ();
      b[k + 1][id5] = P.get_oZ() * P.get_y();

      b[k + 1][id6] = 0.0;
      b[k + 1][id7] = -1.0;
      b[k + 1][id8] = P.get_y();

      k += 2;
    }
    vpColVector X1(nbColsA); // X1 is of size A^T A
    vpColVector X2(nbColsB); // X2 is of size B^T B

    lagrange(a, b, X1, X2);

    if (X2[id8] < 0.0) { /* because Zo greater than 0  */
      X1 *= -1;
      X2 *= -1;
    }
    s = 0.0;
    for (i = 0; i < nbColsA; ++i) {
      s += (X1[i] * X2[i]);
    }
    for (i = 0; i < nbColsA; ++i) {
      X2[i] -= (s * X1[i]);
    } /* X1^T X2 is null  */

    s = (X2[id0] * X2[id0]) + (X2[id1] * X2[id1]) + (X2[id2] * X2[id2]); // To avoid a Coverity copy/past error

    if (s < 1e-10) {
      throw(vpException(vpException::divideByZeroError, "Division by zero in Lagrange pose computation (non "
                        "planar plane case)"));
    }

    s = 1.0 / sqrt(s);
    for (i = 0; i < 3; ++i) {
      X2[i] *= s;
    } /* X2^T X2 = 1  */

    X2[id3] = (X1[id1] * X2[id2]) - (X1[id2] * X2[id1]);
    X2[id4] = (X1[id2] * X2[id0]) - (X1[id0] * X2[id2]);
    X2[id5] = (X1[id0] * X2[id1]) - (X1[id1] * X2[id0]);

    const unsigned int nc1 = 3, nc3 = 6;
    calculTranslation(a, b, nl, nc1, nc3, X1, X2);

    for (i = 0; i < 3; ++i) {
      cMo[i][id0] = X1[i];
      cMo[i][id1] = X2[i];
      cMo[i][id2] = X2[i + id3];
      cMo[i][id3] = X2[i + id6];
    }

  }
  catch (...) {
    throw; // throw the original exception
  }
}

END_VISP_NAMESPACE
