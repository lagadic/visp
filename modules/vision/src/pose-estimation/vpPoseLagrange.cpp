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

#include <visp3/vision/vpPose.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

/**********************************************************************/
/*	FONCTION 		:    CalculTranslation       */
/*	ROLE  			: Calcul de la translation entre la   */
/*                                camera et l'outil connaissant la    */
/*                                rotation			      */
/**********************************************************************/

static void calculTranslation(vpMatrix &a, vpMatrix &b, unsigned int nl, unsigned int nc1, unsigned int nc3,
                              vpColVector &x1, vpColVector &x2)
{

  try {
    unsigned int i, j;

    vpMatrix ct(3, nl);
    for (i = 0; i < 3; i++) {
      for (j = 0; j < nl; j++)
        ct[i][j] = b[j][i + nc3];
    }

    vpMatrix c;
    c = ct.t();

    vpMatrix ctc;
    ctc = ct * c;

    vpMatrix ctc1; // (C^T C)^(-1)
    ctc1 = ctc.inverseByLU();

    vpMatrix cta;
    vpMatrix ctb;
    cta = ct * a; /* C^T A	*/
    ctb = ct * b; /* C^T B	*/

#if (DEBUG_LEVEL2)
    {
      std::cout << "ctc " << std::endl << ctc;
      std::cout << "cta " << std::endl << cta;
      std::cout << "ctb " << std::endl << ctb;
    }
#endif

    vpColVector X2(nc3);
    vpMatrix CTB(nc1, nc3);
    for (i = 0; i < nc1; i++) {
      for (j = 0; j < nc3; j++)
        CTB[i][j] = ctb[i][j];
    }

    for (j = 0; j < nc3; j++)
      X2[j] = x2[j];

    vpColVector sv;           // C^T A X1 + C^T B X2)
    sv = cta * x1 + CTB * X2; // C^T A X1 + C^T B X2)

#if (DEBUG_LEVEL2)
    std::cout << "sv " << sv.t();
#endif

    vpColVector X3; /* X3 = - (C^T C )^{-1} C^T (A X1 + B X2) */
    X3 = -ctc1 * sv;

#if (DEBUG_LEVEL2)
    std::cout << "x3 " << X3.t();
#endif

    for (i = 0; i < nc1; i++)
      x2[i + nc3] = X3[i];
  } catch (...) {

    // en fait il y a des dizaines de raisons qui font que cette fonction
    // rende une erreur (matrice pas inversible, pb de memoire etc...)
    vpERROR_TRACE(" ");
    throw;
  }
}

//*********************************************************************
//   FONCTION LAGRANGE :
//   -------------------
// Resolution d'un systeme lineaire de la forme A x1 + B x2 = 0
//  		sous la contrainte || x1 || = 1
//  		ou A est de dimension nl x nc1 et B nl x nc2
//*********************************************************************

//#define EPS 1.e-5

static void lagrange(vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin (CLagrange.cc)Lagrange(...) " << std::endl;
#endif

  try {
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

#if (DEBUG_LEVEL1)
    {
      std::cout << " BTB1 * BTB : " << std::endl << btb1 * btb << std::endl;
      std::cout << " BTB * BTB1 : " << std::endl << btb * btb1 << std::endl;
    }
#endif

    vpMatrix r; // (B^T B)^(-1) B^T A
    r = btb1 * bta;

    vpMatrix e; //   - A^T B (B^T B)^(-1) B^T A
    e = -(a.t() * b) * r;

    e += ata; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A

#if (DEBUG_LEVEL1)
    {
      std::cout << " E :" << std::endl << e << std::endl;
    }
#endif

    //   vpColVector sv ;
    //    vpMatrix v ;
    e.svd(x1, ata); // destructif sur e
    // calcul du vecteur propre de E correspondant a la valeur propre min.
    /* calcul de SVmax	*/
    imin = 0;
    // FC : Pourquoi calculer SVmax ??????
    //     double  svm = 0.0;
    //    for (i=0;i<x1.getRows();i++)
    //    {
    //      if (x1[i] > svm) { svm = x1[i]; imin = i; }
    //    }
    //    svm *= EPS;	/* pour le rang	*/

    for (i = 0; i < x1.getRows(); i++)
      if (x1[i] < x1[imin])
        imin = i;

#if (DEBUG_LEVEL1)
    {
      printf("SV(E) : %.15lf %.15lf %.15lf\n", x1[0], x1[1], x1[2]);
      std::cout << " i_min " << imin << std::endl;
    }
#endif
    for (i = 0; i < x1.getRows(); i++)
      x1[i] = ata[i][imin];

    x2 = -(r * x1); // X_2 = - (B^T B)^(-1) B^T A X_1

#if (DEBUG_LEVEL1)
    {
      std::cout << " X1 : " << x1.t() << std::endl;
      std::cout << " V : " << std::endl << ata << std::endl;
    }
#endif
  } catch (...) {
    vpERROR_TRACE(" ");
    throw;
  }
#if (DEBUG_LEVEL1)
  std::cout << "end (CLagrange.cc)Lagrange(...) " << std::endl;
#endif
}

//#undef EPS

/*!
\brief  Compute the pose of a planar object using Lagrange approach.

\param cMo : Estimated pose. No initialisation is requested to estimate cMo.
*/

void vpPose::poseLagrangePlan(vpHomogeneousMatrix &cMo)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::PoseLagrangePlan(...) " << std::endl;
#endif
  // determination of the plane equation a X + b Y + c Z + d = 0
  // FC : long copy/paste from vpPose::coplanar. To be improved...
  vpPoint P1, P2, P3;
  double x1 = 0, x2 = 0, x3 = 0, y1 = 0, y2 = 0, y3 = 0, z1 = 0, z2 = 0, z3 = 0;

  // Get three 3D points that are not collinear and that are not at origin
  // FC : I think one point could be at origin (to be checked)

  bool degenerate = true;
  bool not_on_origin = true;
  std::list<vpPoint>::const_iterator it_tmp;

  std::list<vpPoint>::const_iterator it_i, it_j, it_k;
  for (it_i = listP.begin(); it_i != listP.end(); ++it_i) {
    if (degenerate == false) {
      // std::cout << "Found a non degenerate configuration" << std::endl;
      break;
    }
    P1 = *it_i;
    // Test if point is on origin
    if ((std::fabs(P1.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
        (std::fabs(P1.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
        (std::fabs(P1.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
      not_on_origin = false;
    } else {
      not_on_origin = true;
    }
    if (not_on_origin) {
      it_tmp = it_i;
      ++it_tmp; // j = i+1
      for (it_j = it_tmp; it_j != listP.end(); ++it_j) {
        if (degenerate == false) {
          // std::cout << "Found a non degenerate configuration" << std::endl;
          break;
        }
        P2 = *it_j;
        if ((std::fabs(P2.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(P2.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
            (std::fabs(P2.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
          not_on_origin = false;
        } else {
          not_on_origin = true;
        }
        if (not_on_origin) {
          it_tmp = it_j;
          ++it_tmp; // k = j+1
          for (it_k = it_tmp; it_k != listP.end(); ++it_k) {
            P3 = *it_k;
            if ((std::fabs(P3.get_oX()) <= std::numeric_limits<double>::epsilon()) &&
                (std::fabs(P3.get_oY()) <= std::numeric_limits<double>::epsilon()) &&
                (std::fabs(P3.get_oZ()) <= std::numeric_limits<double>::epsilon())) {
              not_on_origin = false;
            } else {
              not_on_origin = true;
            }
            if (not_on_origin) {
              x1 = P1.get_oX();
              x2 = P2.get_oX();
              x3 = P3.get_oX();

              y1 = P1.get_oY();
              y2 = P2.get_oY();
              y3 = P3.get_oY();

              z1 = P1.get_oZ();
              z2 = P2.get_oZ();
              z3 = P3.get_oZ();

              vpColVector a_b(3), b_c(3), cross_prod;
              a_b[0] = x1 - x2;
              a_b[1] = y1 - y2;
              a_b[2] = z1 - z2;
              b_c[0] = x2 - x3;
              b_c[1] = y2 - y3;
              b_c[2] = z2 - z3;

              cross_prod = vpColVector::crossProd(a_b, b_c);
              if (cross_prod.sumSquare() <= std::numeric_limits<double>::epsilon())
                degenerate = true; // points are collinear
              else
                degenerate = false;
            }
            if (degenerate == false)
              break;
          }
        }
      }
    }
  }
  double a = y1 * z2 - y1 * z3 - y2 * z1 + y2 * z3 + y3 * z1 - y3 * z2;
  double b = -x1 * z2 + x1 * z3 + x2 * z1 - x2 * z3 - x3 * z1 + x3 * z2;
  double c = x1 * y2 - x1 * y3 - x2 * y1 + x2 * y3 + x3 * y1 - x3 * y2;
  double d = -x1 * y2 * z3 + x1 * y3 * z2 + x2 * y1 * z3 - x2 * y3 * z1 - x3 * y1 * z2 + x3 * y2 * z1;

  if (c < 0.0) {  // imposing c >= 0
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }
  // to have (a,b,c) as a unit vector if it was not the case
  double n = 1.0/sqrt(a*a+ b*b + c*c);  // Not possible to have a NaN...
  a *= n;
  b *= n;
  c *= n;
  d *= n;
  // printf("a = %lf, b = %lf, c = %lf, d = %lf\n",a,b,c,d);
  // transformation to have object plane with equation Z = 0
  vpColVector r1(3), r2(3), r3(3);

  r3[0] = a;
  r3[1] = b;
  r3[2] = c;
  // build r1 as a unit vector orthogonal to r3
  double n1 = sqrt(1.0-a*a);
  double n2 = sqrt(1.0-b*b);
  if (n1 >= n2){
    r1[0] = n1;
    r1[1] = -a*b/n1;
    r1[2] = -a*c/n1;
  }
  else{
    r1[0] = -a*b/n2;
    r1[1] = n2;
    r1[2] = -b*c/n2;
  }
  // double norm = r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2];
  // double crossprod = r1[0]*r3[0] + r1[1]*r3[1] + r1[2]*r3[2];
  // printf("r1 norm = 1 ?  %lf, r1^T r3 = 0 ?  %lf\n",norm, crossprod);
  // r2 unit vector orthogonal to r3 and r1
  r2 = vpColVector::crossProd(r3, r1);

  vpHomogeneousMatrix fMo;
  for (unsigned int i=0;i<3;i++){
    fMo[0][i] = r1[i];
    fMo[1][i] = r2[i];
    fMo[2][i] = r3[i];
  }
  fMo[0][3] = fMo[1][3] = 0.0;
  fMo[2][3] = d;

  // std::cout << "fMo : "  << std::endl << fMo  << std::endl;
  // Build and solve the system
  unsigned int k = 0;
  unsigned int nl = npt * 2;

  vpMatrix A(nl, 3);
  vpMatrix B(nl, 6);
  vpPoint P;

  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;

    // Transform each point in plane Z = 0
    vpColVector Xf, X(4);
    X[0] = P.get_oX();
    X[1] = P.get_oY();
    X[2] = P.get_oZ();
    X[3] = 1.0;
    Xf = fMo * X;
    // printf("Z = 0 = %lf\n",Xf[2]);
    // build the system
    A[k][0] = -Xf[0];
    A[k][1] = 0.0;
    A[k][2] = Xf[0] * P.get_x();

    A[k + 1][0] = 0.0;
    A[k + 1][1] = -Xf[0];
    A[k + 1][2] = Xf[0] * P.get_y();

    B[k][0] = -Xf[1];
    B[k][1] = 0.0;
    B[k][2] = Xf[1] * P.get_x();
    B[k][3] = -1.0;
    B[k][4] = 0.0;
    B[k][5] = P.get_x();

    B[k + 1][0] = 0.0;
    B[k + 1][1] = -Xf[1];
    B[k + 1][2] = Xf[1] * P.get_y();
    B[k + 1][3] = 0.0;
    B[k + 1][4] = -1.0;
    B[k + 1][5] = P.get_y();

    k += 2;
  }
  vpColVector X1(3);
  vpColVector X2(6);

#if (DEBUG_LEVEL2)
  {
    std::cout << "A " << std::endl << A << std::endl;
    std::cout << "B " << std::endl << B << std::endl;
  }
#endif

  lagrange(A, B, X1, X2);

#if (DEBUG_LEVEL2)
  {
    std::cout << "A X1+B X2 (should be 0): " << (A * X1 + B * X2).t() << std::endl;
    std::cout << " X1 norm: " << X1.sumSquare() << std::endl;
  }
#endif

  if (X2[5] < 0.0) { /* to obtain Zo > 0	*/
    for (unsigned int i = 0; i < 3; i++)
      X1[i] = -X1[i];
    for (unsigned int i = 0; i < 6; i++)
      X2[i] = -X2[i];
  }
  double s = 0.0;
  for (unsigned int i = 0; i < 3; i++) {
    s += (X1[i] * X2[i]);
  }
  for (unsigned int i = 0; i < 3; i++) {
    X2[i] -= (s * X1[i]);
  } /* X1^T X2 = 0	*/

  // s = 0.0;
  // for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}
  s = X2[0] * X2[0] + X2[1] * X2[1] + X2[2] * X2[2]; // To avoid a Coverity copy/past error

  if (s < 1e-10) {
    //      std::cout << "Points that produce an error: " << std::endl;
    //      for (std::list<vpPoint>::const_iterator it = listP.begin(); it
    //      != listP.end(); ++it)
    //      {
    //        std::cout << "P: " << (*it).get_x() << " " << (*it).get_y() <<
    //        " "
    //                  << (*it).get_oX() << " " << (*it).get_oY() << " " <<
    //                  (*it).get_oZ() << std::endl;
    //      }
    throw(vpException(vpException::divideByZeroError, "Division by zero in Lagrange pose computation "
                                                      "(planar plane case)"));
  }

  s = 1.0 / sqrt(s);
  for (unsigned int i = 0; i < 3; i++) {
    X2[i] *= s;
  } /* X2^T X2 = 1	*/

  calculTranslation(A, B, nl, 3, 3, X1, X2);

  // if (err != OK)
  {
    // std::cout << "in (vpCalculPose_plan.cc)CalculTranslation returns " ;
    // PrintError(err) ;
    //    return err ;
  }
  vpHomogeneousMatrix cMf;
  /* X1 x X2 */
  cMf[0][2] = (X1[1] * X2[2]) - (X1[2] * X2[1]);
  cMf[1][2] = (X1[2] * X2[0]) - (X1[0] * X2[2]);
  cMf[2][2] = (X1[0] * X2[1]) - (X1[1] * X2[0]);
  /* calcul de la matrice de passage	*/
  for (unsigned int i = 0; i < 3; i++) {
    cMf[i][0] = X1[i];
    cMf[i][1] = X2[i];
    cMf[i][3] = X2[i + 3];
  }
  //std::cout << "cMf : "  << std::endl << cMf  << std::endl;

  // Apply the transform to go back to object frame
  cMo = cMf * fMo;

#if (DEBUG_LEVEL1)
  std::cout << "end vpCalculPose::PoseLagrangePlan(...) " << std::endl;
#endif
  //  return(OK);
}

void vpPose::poseLagrangeNonPlan(vpHomogeneousMatrix &cMo)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin CPose::PoseLagrangeNonPlan(...) " << std::endl;
#endif
  try {
    double s;
    unsigned int i;

    unsigned int k = 0;
    unsigned int nl = npt * 2;

    if (npt < 6) {
      throw(vpException(vpException::dimensionError,
                        "Lagrange, non planar case, insufficient number of points %d < 6\n", npt));
    }

    vpMatrix a(nl, 3);
    vpMatrix b(nl, 9);
    b = 0;

    vpPoint P;
    i = 0;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
      P = *it;
      a[k][0] = -P.get_oX();
      a[k][1] = 0.0;
      a[k][2] = P.get_oX() * P.get_x();

      a[k + 1][0] = 0.0;
      a[k + 1][1] = -P.get_oX();
      a[k + 1][2] = P.get_oX() * P.get_y();

      b[k][0] = -P.get_oY();
      b[k][1] = 0.0;
      b[k][2] = P.get_oY() * P.get_x();

      b[k][3] = -P.get_oZ();
      b[k][4] = 0.0;
      b[k][5] = P.get_oZ() * P.get_x();

      b[k][6] = -1.0;
      b[k][7] = 0.0;
      b[k][8] = P.get_x();

      b[k + 1][0] = 0.0;
      b[k + 1][1] = -P.get_oY();
      b[k + 1][2] = P.get_oY() * P.get_y();

      b[k + 1][3] = 0.0;
      b[k + 1][4] = -P.get_oZ();
      b[k + 1][5] = P.get_oZ() * P.get_y();

      b[k + 1][6] = 0.0;
      b[k + 1][7] = -1.0;
      b[k + 1][8] = P.get_y();

      k += 2;
    }
    vpColVector X1(3);
    vpColVector X2(9);

#if (DEBUG_LEVEL2)
    {
      std::cout << "a " << a << std::endl;
      std::cout << "b " << b << std::endl;
    }
#endif

    lagrange(a, b, X1, X2);
    //  if (err != OK)
    {
      //      std::cout << "in (CLagrange.cc)Lagrange returns " ;
      //    PrintError(err) ;
      //    return err ;
    }

#if (DEBUG_LEVEL2)
    {
      std::cout << "ax1+bx2 (devrait etre 0) " << (a * X1 + b * X2).t() << std::endl;
      std::cout << "norme X1 " << X1.sumSquare() << std::endl;
      ;
    }
#endif

    if (X2[8] < 0.0) { /* car Zo > 0	*/
      X1 *= -1;
      X2 *= -1;
    }
    s = 0.0;
    for (i = 0; i < 3; i++) {
      s += (X1[i] * X2[i]);
    }
    for (i = 0; i < 3; i++) {
      X2[i] -= (s * X1[i]);
    } /* X1^T X2 = 0	*/

    // s = 0.0;
    // for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}
    s = X2[0] * X2[0] + X2[1] * X2[1] + X2[2] * X2[2]; // To avoid a Coverity copy/past error

    if (s < 1e-10) {
      //      std::cout << "Points that produce an error: " << std::endl;
      //      for (std::list<vpPoint>::const_iterator it = listP.begin(); it
      //      != listP.end(); ++it)
      //      {
      //        std::cout << "P: " << (*it).get_x() << " " << (*it).get_y() <<
      //        " "
      //                  << (*it).get_oX() << " " << (*it).get_oY() << " " <<
      //                  (*it).get_oZ() << std::endl;
      //      }
      // vpERROR_TRACE(" division par zero " ) ;
      throw(vpException(vpException::divideByZeroError, "Division by zero in Lagrange pose computation (non "
                                                        "planar plane case)"));
    }

    s = 1.0 / sqrt(s);
    for (i = 0; i < 3; i++) {
      X2[i] *= s;
    } /* X2^T X2 = 1	*/

    X2[3] = (X1[1] * X2[2]) - (X1[2] * X2[1]);
    X2[4] = (X1[2] * X2[0]) - (X1[0] * X2[2]);
    X2[5] = (X1[0] * X2[1]) - (X1[1] * X2[0]);

    calculTranslation(a, b, nl, 3, 6, X1, X2);

    for (i = 0; i < 3; i++) {
      cMo[i][0] = X1[i];
      cMo[i][1] = X2[i];
      cMo[i][2] = X2[i + 3];
      cMo[i][3] = X2[i + 6];
    }

  } catch (...) {
    throw; // throw the original exception
  }

#if (DEBUG_LEVEL1)
  std::cout << "end vpCalculPose::PoseLagrangeNonPlan(...) " << std::endl;
#endif
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
