/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Homography estimation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpHomographyDLT.cpp

  This file implements the fonctions related with the homography
  estimation using the DLT algorithm
*/

#include <visp3/vision/vpHomography.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void
vpHomography::HartleyNormalization(const std::vector<double> &x, const std::vector<double> &y,
                                   std::vector<double> &xn, std::vector<double> &yn,
                                   double &xg, double &yg, double &coef)
{
  if (x.size() != y.size())
    throw(vpException(vpException::dimensionError,
                      "Hartley normalization require that x and y vector have the same dimension"));

  unsigned int n = (unsigned int) x.size();
  if (xn.size() != n)
    xn.resize(n);
  if (yn.size() != n)
    yn.resize(n);

  xg = 0 ;
  yg = 0 ;

  for (unsigned int i =0 ; i < n ; i++)
  {
    xg += x[i] ;
    yg += y[i] ;
  }
  xg /= n ;
  yg /= n ;

  // Changement d'origine : le centre de gravite doit correspondre
  // a l'origine des coordonnees
  double distance=0;
  for(unsigned int i=0; i<n;i++)
  {
    double xni=x[i]-xg;
    double yni=y[i]-yg;
    xn[i] = xni ;
    yn[i] = yni ;
    distance+=sqrt(vpMath::sqr(xni)+vpMath::sqr(yni));
  }//for translation sur tous les points

  //Changement d'echelle
  distance/=n;
  //calcul du coef de changement d'echelle
  //if(distance ==0)
  if(std::fabs(distance) <= std::numeric_limits<double>::epsilon())
    coef=1;
  else
    coef=sqrt(2.0)/distance;

  for(unsigned int i=0; i<n;i++)
  {
    xn[i] *= coef;
    yn[i] *= coef;
  }
}

void
vpHomography::HartleyNormalization(unsigned int n,
                                   const double *x, const double *y,
                                   double *xn, double *yn,
                                   double &xg, double &yg,
                                   double &coef)
{
  unsigned int i;
  xg = 0 ;
  yg = 0 ;

  for (i =0 ; i < n ; i++)
  {
    xg += x[i] ;
    yg += y[i] ;
  }
  xg /= n ;
  yg /= n ;

  //Changement d'origine : le centre de gravite doit correspondre
  // a l'origine des coordonnees
  double distance=0;
  for(i=0; i<n;i++)
  {
    double xni=x[i]-xg;
    double yni=y[i]-yg;
    xn[i] = xni ;
    yn[i] = yni ;
    distance+=sqrt(vpMath::sqr(xni)+vpMath::sqr(yni));
  }//for translation sur tous les points

  //Changement d'echelle
  distance/=n;
  //calcul du coef de changement d'echelle
  //if(distance ==0)
  if(std::fabs(distance) <= std::numeric_limits<double>::epsilon())
    coef=1;
  else
    coef=sqrt(2.0)/distance;

  for(i=0; i<n;i++)
  {
    xn[i] *= coef;
    yn[i] *= coef;
  }
}

//---------------------------------------------------------------------------------------

void
vpHomography::HartleyDenormalization(vpHomography &aHbn,
             vpHomography &aHb,
             double xg1, double yg1, double coef1,
             double xg2, double yg2, double coef2 )
{

  //calcul des transformations a appliquer sur M_norm pour obtenir M
  //en fonction des deux normalisations effectuees au debut sur
  //les points: aHb = T2^ aHbn T1
  vpMatrix T1(3,3);
  vpMatrix T2(3,3);
  vpMatrix T2T(3,3);

  T1.eye();
  T2.eye();
  T2T.eye();

  T1[0][0]=T1[1][1]=coef1;
  T1[0][2]=-coef1*xg1 ;
  T1[1][2]=-coef1*yg1 ;

  T2[0][0]=T2[1][1]=coef2;
  T2[0][2]=-coef2*xg2 ;
  T2[1][2]=-coef2*yg2 ;

  T2T=T2.pseudoInverse(1e-16) ;

  vpMatrix aHbn_(3,3);
  for(unsigned int i=0; i<3; i++)
    for(unsigned int j=0; j<3; j++)
      aHbn_[i][j] = aHbn[i][j];

  vpMatrix maHb=T2T*aHbn_*T1;

  for (unsigned int i=0 ; i < 3 ; i++)
    for (unsigned int j=0 ; j < 3 ; j++)
      aHb[i][j] = maHb[i][j] ;
}

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
  and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates, computes the
  homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}\f$
  using the DLT (Direct Linear Transform) algorithm.

  At least 4 couples of points are needed.

  To do so, we use the DLT algorithm on the data,
  ie we resolve the linear system  by SDV : \f$\bf{Ah} =0\f$ where
  \f$\bf{h}\f$ is the vector with the terms of \f$^a{\bf H}_b\f$ and
  \f$\mathbf{A}\f$ depends on the  points coordinates.

  For each point, in homogeneous coordinates we have:
  \f[
  ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
  \f]
  which is equivalent to:
  \f[
  ^a{\bf p} \times {^a{\bf H}_b \; ^b{\bf p}}  =0
  \f]
  If we note \f$\mathbf{h}_j^T\f$ the  \f$j^{\textrm{th}}\f$ line of  \f$^a{\bf H}_b\f$, we can write:
  \f[
  ^a{\bf H}_b \; ^b{\bf p}  = \left( \begin{array}{c}\mathbf{h}_1^T \;^b{\bf p} \\\mathbf{h}_2^T \; ^b{\bf p} \\\mathbf{h}_3^T \;^b{\bf p} \end{array}\right)
  \f]

  Setting \f$^a{\bf p}=(x_{a},y_{a},w_{a})\f$, the cross product  can be rewritten by:
  \f[
  ^a{\bf p} \times ^a{\bf H}_b \; ^b{\bf p}  =\left( \begin{array}{c}y_{a}\mathbf{h}_3^T \; ^b{\bf p}-w_{a}\mathbf{h}_2^T \; ^b{\bf p} \\w_{a}\mathbf{h}_1^T \; ^b{\bf p} -x_{a}\mathbf{h}_3^T \; ^b{\bf p} \\x_{a}\mathbf{h}_2^T \; ^b{\bf p}- y_{a}\mathbf{h}_1^T \; ^b{\bf p}\end{array}\right)
  \f]

  \f[
  \underbrace{\left( \begin{array}{ccc}\mathbf{0}^T & -w_{a} \; ^b{\bf p}^T
  & y_{a} \; ^b{\bf p}^T     \\     w_{a}
  \; ^b{\bf p}^T&\mathbf{0}^T & -x_{a} \; ^b{\bf p}^T      \\
  -y_{a} \; ^b{\bf p}^T & x_{a} \; ^b{\bf p}^T &
  \mathbf{0}^T\end{array}\right)}_{\mathbf{A}_i (3\times 9)}
  \underbrace{\left( \begin{array}{c}\mathbf{h}_{1}^{T}      \\
  \mathbf{h}_{2}^{T}\\\mathbf{h}_{3}^{T}\end{array}\right)}_{\mathbf{h} (9\times 1)}=0
  \f]

  leading to an homogeneous system to be solved:   \f$\mathbf{A}\mathbf{h}=0\f$ with
  \f$\mathbf{A}=\left(\mathbf{A}_1^T, ..., \mathbf{A}_i^T, ..., \mathbf{A}_n^T \right)^T\f$.

  It can be solved using an SVD decomposition:
  \f[\bf A = UDV^T \f]
  <b>h</b> is the column of <b>V</b> associated with the smalest singular value of <b>A
  </b>

  \param xb, yb : Coordinates vector of matched points in image b. These coordinates are expressed in meters.
  \param xa, ya : Coordinates vector of matched points in image a. These coordinates are expressed in meters.
  \param aHb : Estimated homography that relies the transformation from image a to image b.
  \param normalization : When set to true, the coordinates of the points are normalized. The normalization
  carried out is the one preconized by Hartley.

  \exception vpMatrixException::rankDeficient : When the rank of the matrix
  that should be 8 is deficient.
*/
void vpHomography::DLT(const std::vector<double> &xb, const std::vector<double> &yb,
                       const std::vector<double> &xa, const std::vector<double> &ya ,
                       vpHomography &aHb,
                       bool normalization)
{
  unsigned int n = (unsigned int) xb.size();
  if (yb.size() != n || xa.size() != n || ya.size() != n)
    throw(vpException(vpException::dimensionError,
                      "Bad dimension for DLT homography estimation"));

  // 4 point are required
  if(n<4)
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));

  try{
    std::vector<double> xan, yan, xbn, ybn;

    double xg1=0., yg1=0., coef1=0., xg2=0., yg2=0., coef2=0.;

    vpHomography aHbn;

    if (normalization) {
      vpHomography::HartleyNormalization(xb, yb, xbn, ybn, xg1, yg1, coef1);
      vpHomography::HartleyNormalization(xa, ya, xan, yan, xg2, yg2, coef2);
    }
    else {
      xbn = xb;
      ybn = yb;
      xan = xa;
      yan = ya;
    }

    vpMatrix A(2*n,9);
    vpColVector h(9);
    vpColVector D(9);
    vpMatrix V(9,9);

    // We need here to compute the SVD on a (n*2)*9 matrix (where n is
    // the number of points). if n == 4, the matrix has more columns
    // than rows. This kind of matrix is not supported by GSL for
    // SVD. The solution is to add an extra line with zeros
    if (n == 4)
      A.resize(2*n+1,9);

    // build matrix A
    for(unsigned int i=0; i<n;i++)
    {
      A[2*i][0]=0;
      A[2*i][1]=0;
      A[2*i][2]=0;
      A[2*i][3]=-xbn[i] ;
      A[2*i][4]=-ybn[i] ;
      A[2*i][5]=-1;
      A[2*i][6]=xbn[i]*yan[i] ;
      A[2*i][7]=ybn[i]*yan[i];
      A[2*i][8]=yan[i];


      A[2*i+1][0]=xbn[i] ;
      A[2*i+1][1]=ybn[i] ;
      A[2*i+1][2]=1;
      A[2*i+1][3]=0;
      A[2*i+1][4]=0;
      A[2*i+1][5]=0;
      A[2*i+1][6]=-xbn[i]*xan[i];
      A[2*i+1][7]=-ybn[i]*xan[i];
      A[2*i+1][8]=-xan[i] ;
    }

    // Add an extra line with zero.
    if (n == 4) {
      for (unsigned int  i=0; i < 9; i ++) {
        A[2*n][i] = 0;
      }
    }

    // solve Ah = 0
    // SVD  Decomposition A = UDV^T (destructive wrt A)
    A.svd(D,V);

    // on en profite pour effectuer un controle sur le rang de la matrice :
    // pas plus de 2 valeurs singulieres quasi=0
    int rank=0;
    for(unsigned int i = 0; i<9;i++) if(D[i]>1e-7) rank++;
    if(rank <7)
    {
      throw(vpMatrixException(vpMatrixException::rankDeficient,
                              "Matrix rank %d is deficient (should be 8)", rank));
    }
    //h = is the column of V associated with the smallest singular value of A

    // since  we are not sure that the svd implemented sort the
    // singular value... we seek for the smallest
    double smallestSv = 1e30 ;
    unsigned int indexSmallestSv  = 0 ;
    for (unsigned int i=0 ; i < 9 ; i++)
      if ((D[i] < smallestSv) ){ smallestSv = D[i] ;indexSmallestSv = i ; }

    h=V.getCol(indexSmallestSv);

    // build the homography
    for(unsigned int i =0;i<3;i++)
    {
      for(unsigned int j=0;j<3;j++)
        aHbn[i][j]=h[3*i+j];
    }

    if (normalization) {
      // H after denormalization
      vpHomography::HartleyDenormalization(aHbn, aHb, xg1, yg1, coef1, xg2, yg2, coef2);
    }
    else {
      aHb = aHbn;
    }

  }
  catch(vpMatrixException &me)
  {
    vpTRACE("Matrix Exception ") ;
    throw(me) ;
  }
  catch(vpException &me)
  {
    vpERROR_TRACE("caught another error") ;
    std::cout <<std::endl << me << std::endl ;
    throw(me) ;
  }
}
