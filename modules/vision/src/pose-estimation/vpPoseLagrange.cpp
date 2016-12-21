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

static
void
calculTranslation (vpMatrix &a, vpMatrix &b, unsigned int nl, unsigned int nc1,
                   unsigned int nc3, vpColVector &x1, vpColVector &x2)
{

  try
  {
    unsigned int i,j;

    vpMatrix ct(3,nl) ;
    for (i=0 ; i < 3 ; i++)
    {
      for (j=0 ; j < nl ; j++)
        ct[i][j] = b[j][i+nc3] ;
    }

    vpMatrix c ;
    c = ct.t() ;

    vpMatrix ctc ;
    ctc = ct*c ;

    vpMatrix ctc1 ; // (C^T C)^(-1)
    ctc1 = ctc.inverseByLU() ;

    vpMatrix cta ;
    vpMatrix ctb ;
    cta = ct*a ;  /* C^T A	*/
    ctb = ct*b ;  /* C^T B	*/

#if (DEBUG_LEVEL2)
    {
      std::cout <<"ctc " << std::endl << ctc ;
      std::cout <<"cta " << std::endl << cta ;
      std::cout <<"ctb " << std::endl << ctb ;
    }
#endif

    vpColVector X2(nc3)  ;
    vpMatrix CTB(nc1,nc3) ;
    for (i=0 ; i < nc1 ; i++)
    {
      for (j=0 ; j < nc3 ; j++)
        CTB[i][j] = ctb[i][j] ;
    }

    for (j=0 ; j < nc3 ; j++)
      X2[j] = x2[j] ;

    vpColVector sv ;       // C^T A X1 + C^T B X2)
    sv = cta*x1 + CTB*X2 ;// C^T A X1 + C^T B X2)

#if (DEBUG_LEVEL2)
    std::cout << "sv " << sv.t() ;
#endif

    vpColVector X3 ; /* X3 = - (C^T C )^{-1} C^T (A X1 + B X2) */
    X3 = -ctc1*sv ;

#if (DEBUG_LEVEL2)
    std::cout << "x3 " << X3.t()  ;
#endif

    for (i=0 ; i < nc1 ; i++)
      x2[i+nc3] = X3[i] ;
  }
  catch(...)
  {

    // en fait il y a des dizaines de raisons qui font que cette fonction
    // rende une erreur (matrice pas inversible, pb de memoire etc...)
    vpERROR_TRACE(" ") ;
    throw ;
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

static
void
lagrange (vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin (CLagrange.cc)Lagrange(...) " << std::endl;
#endif

  try{
    unsigned int i,imin;

    vpMatrix ata ; // A^T A
    ata = a.t()*a ;
    vpMatrix btb ; // B^T B
    btb = b.t()*b ;

    vpMatrix bta ;  // B^T A
    bta = b.t()*a ;

    vpMatrix btb1 ;  // (B^T B)^(-1)

    if (b.getRows() >= b.getCols()) btb1 = btb.inverseByLU() ;
    else btb1 = btb.pseudoInverse();

#if (DEBUG_LEVEL1)
    {
      std::cout << " BTB1 * BTB : " << std::endl << btb1*btb << std::endl;
      std::cout << " BTB * BTB1 : " << std::endl << btb*btb1 << std::endl;
    }
#endif

    vpMatrix r ;  // (B^T B)^(-1) B^T A
    r = btb1*bta ;

    vpMatrix e ;  //   - A^T B (B^T B)^(-1) B^T A
    e = - (a.t()*b) *r ;

    e += ata ; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A

#if (DEBUG_LEVEL1)
    {
      std::cout << " E :" << std::endl << e << std::endl;
    }
#endif

    //   vpColVector sv ;
    //    vpMatrix v ;
    e.svd(x1,ata) ;// destructif sur e
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

    for (i=0;i<x1.getRows();i++)
      if (x1[i] < x1[imin]) imin = i;

#if (DEBUG_LEVEL1)
    {
      printf("SV(E) : %.15lf %.15lf %.15lf\n",x1[0],x1[1],x1[2]);
      std::cout << " i_min " << imin << std::endl;
    }
#endif
    for (i=0;i<x1.getRows();i++)
      x1[i] = ata[i][imin];

    x2 = - (r*x1) ; // X_2 = - (B^T B)^(-1) B^T A X_1

#if (DEBUG_LEVEL1)
    {
      std::cout << " X1 : " <<  x1.t() << std::endl;
      std::cout << " V : " << std::endl << ata << std::endl;
    }
#endif
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
#if (DEBUG_LEVEL1)
  std::cout << "end (CLagrange.cc)Lagrange(...) " << std::endl;
#endif
}

//#undef EPS

/*!
\brief  Compute the pose of a planar object using Lagrange approach.

\param cMo : Estimated pose. No initialisation is requested to estimate cMo.
\param coplanar_plane_type : Type of coplanar plane:
   1: if plane x=cst
   2: if plane y=cst
   3: if plane z=cst
   0: any other plane
*/
void
vpPose::poseLagrangePlan(vpHomogeneousMatrix &cMo, const int coplanar_plane_type)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::PoseLagrange(...) " << std::endl ;
#endif
  try
  {
    double s;
    unsigned int i;

    unsigned int k=0;
    unsigned int nl=npt*2;


    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,6);
    vpPoint P ;
    i=0 ;

    if (coplanar_plane_type == 1) { // plane ax=d
      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
      {
        P = *it ;
        a[k][0]   = -P.get_oY();
        a[k][1]   = 0.0;
        a[k][2]   = P.get_oY()*P.get_x();

        a[k+1][0] = 0.0;
        a[k+1][1] = -P.get_oY();
        a[k+1][2] = P.get_oY()*P.get_y();

        b[k][0]   = -P.get_oZ();
        b[k][1]   = 0.0;
        b[k][2]   = P.get_oZ()*P.get_x();
        b[k][3]   =  -1.0;
        b[k][4]   =  0.0;
        b[k][5]   =  P.get_x();

        b[k+1][0] =  0.0;
        b[k+1][1] = -P.get_oZ();
        b[k+1][2] =  P.get_oZ()*P.get_y();
        b[k+1][3] =  0.0;
        b[k+1][4] = -1.0;
        b[k+1][5] =  P.get_y();

        k += 2;
      }

    }
    else if (coplanar_plane_type == 2) {  // plane by=d
      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
      {
        P = *it ;
        a[k][0]   = -P.get_oX();
        a[k][1]   = 0.0;
        a[k][2]   = P.get_oX()*P.get_x();

        a[k+1][0] = 0.0;
        a[k+1][1] = -P.get_oX();
        a[k+1][2] = P.get_oX()*P.get_y();

        b[k][0]   = -P.get_oZ();
        b[k][1]   = 0.0;
        b[k][2]   = P.get_oZ()*P.get_x();
        b[k][3]   =  -1.0;
        b[k][4]   =  0.0;
        b[k][5]   =  P.get_x();

        b[k+1][0] =  0.0;
        b[k+1][1] = -P.get_oZ();
        b[k+1][2] =  P.get_oZ()*P.get_y();
        b[k+1][3] =  0.0;
        b[k+1][4] = -1.0;
        b[k+1][5] =  P.get_y();

        k += 2;
      }

    }
    else { // plane cz=d or any other

      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
      {
        P = *it ;
        a[k][0]   = -P.get_oX();
        a[k][1]   = 0.0;
        a[k][2]   = P.get_oX()*P.get_x();

        a[k+1][0] = 0.0;
        a[k+1][1] = -P.get_oX();
        a[k+1][2] = P.get_oX()*P.get_y();

        b[k][0]   = -P.get_oY();
        b[k][1]   = 0.0;
        b[k][2]   = P.get_oY()*P.get_x();
        b[k][3]   =  -1.0;
        b[k][4]   =  0.0;
        b[k][5]   =  P.get_x();

        b[k+1][0] =  0.0;
        b[k+1][1] = -P.get_oY();
        b[k+1][2] =  P.get_oY()*P.get_y();
        b[k+1][3] =  0.0;
        b[k+1][4] = -1.0;
        b[k+1][5] =  P.get_y();

        k += 2;
      }
    }
    vpColVector X1(3) ;
    vpColVector X2(6) ;

#if (DEBUG_LEVEL2)
    {
      std::cout <<"a " << a << std::endl ;
      std::cout <<"b " << b << std::endl ;
    }
#endif

    lagrange(a,b,X1,X2);

#if (DEBUG_LEVEL2)
    {
      std::cout << "ax1+bx2 (devrait etre 0) " << (a*X1 + b*X2).t() << std::endl ;
      std::cout << "norme X1 " << X1.sumSquare() << std::endl ;;
    }
#endif

    if (X2[5] < 0.0)
    {		/* car Zo > 0	*/
      for (i=0;i<3;i++) X1[i] = -X1[i];
      for (i=0;i<6;i++) X2[i] = -X2[i];
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} /* X1^T X2 = 0	*/

    //s = 0.0;
    //for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}
    s = X2[0]*X2[0] + X2[1]*X2[1] + X2[2]*X2[2]; // To avoid a Coverity copy/past error

    if (s<1e-10)
    {
//      std::cout << "Points that produce an error: " << std::endl;
//      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
//      {
//        std::cout << "P: " << (*it).get_x() << " " << (*it).get_y() << " "
//                  << (*it).get_oX() << " " << (*it).get_oY() << " " << (*it).get_oZ() << std::endl;
//      }
      throw(vpException(vpException::divideByZeroError,
                        "Division by zero in Lagrange pose computation (planar plane case)")) ;
    }

    s = 1.0/sqrt(s);
    for (i=0;i<3;i++)  {X2[i] *= s;}		/* X2^T X2 = 1	*/


    calculTranslation (a, b, nl, 3, 3, X1, X2) ;

    // if (err != OK)
    {
      // std::cout << "in (vpCalculPose_plan.cc)CalculTranslation returns " ;
      // PrintError(err) ;
      //    return err ;
    }

    if (coplanar_plane_type == 1) { // plane ax=d
      cMo[0][0] = (X1[1]*X2[2])-(X1[2]*X2[1]);
      cMo[1][0] = (X1[2]*X2[0])-(X1[0]*X2[2]);
      cMo[2][0] = (X1[0]*X2[1])-(X1[1]*X2[0]);

      for (i=0;i<3;i++)
      { /* calcul de la matrice de passage	*/
        cMo[i][1] = X1[i];
        cMo[i][2] = X2[i];
        cMo[i][3] = X2[i+3];
      }

    }
    else if (coplanar_plane_type == 2) {  // plane by=d
      cMo[0][1] = (X1[1]*X2[2])-(X1[2]*X2[1]);
      cMo[1][1] = (X1[2]*X2[0])-(X1[0]*X2[2]);
      cMo[2][1] = (X1[0]*X2[1])-(X1[1]*X2[0]);

      for (i=0;i<3;i++)
      { /* calcul de la matrice de passage	*/
        cMo[i][0] = X1[i];
        cMo[i][2] = X2[i];
        cMo[i][3] = X2[i+3];
      }
    }
    else { // plane cz=d or any other

      cMo[0][2] = (X1[1]*X2[2])-(X1[2]*X2[1]);
      cMo[1][2] = (X1[2]*X2[0])-(X1[0]*X2[2]);
      cMo[2][2] = (X1[0]*X2[1])-(X1[1]*X2[0]);

      for (i=0;i<3;i++)
      { /* calcul de la matrice de passage	*/
        cMo[i][0] = X1[i];
        cMo[i][1] = X2[i];
        cMo[i][3] = X2[i+3];
      }
    }
  }
  catch(...)
  {
    throw; // throw the original exception
  }

#if (DEBUG_LEVEL1)
  std::cout << "end vpCalculPose::PoseLagrange(...) " << std::endl ;
#endif
  //  return(OK);
}


void
vpPose::poseLagrangeNonPlan(vpHomogeneousMatrix &cMo)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin CPose::PoseLagrange(...) " << std::endl ;
#endif
  try{
    double s;
    unsigned int i;

    unsigned int k=0;
    unsigned int nl=npt*2;

    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,9);
    b =0 ;

    vpPoint P ;
    i=0 ;
    for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
    {
      P = *it;
      a[k][0]   = -P.get_oX();
      a[k][1]   = 0.0;
      a[k][2]   = P.get_oX()*P.get_x();

      a[k+1][0] = 0.0;
      a[k+1][1] = -P.get_oX();
      a[k+1][2] = P.get_oX()*P.get_y();

      b[k][0]   = -P.get_oY();
      b[k][1]   = 0.0;
      b[k][2]   = P.get_oY()*P.get_x();

      b[k][3]   = -P.get_oZ();
      b[k][4]   =  0.0;
      b[k][5]   =  P.get_oZ()*P.get_x();

      b[k][6]   =  -1.0;
      b[k][7]   =  0.0;
      b[k][8]   =  P.get_x();

      b[k+1][0] =  0.0;
      b[k+1][1] = -P.get_oY();
      b[k+1][2] =  P.get_oY()*P.get_y();

      b[k+1][3] =  0.0;
      b[k+1][4] = -P.get_oZ();
      b[k+1][5] =  P.get_oZ()*P.get_y();

      b[k+1][6] =  0.0;
      b[k+1][7] = -1.0;
      b[k+1][8] =  P.get_y();

      k += 2;
    }
    vpColVector X1(3) ;
    vpColVector X2(9) ;

#if (DEBUG_LEVEL2)
    {
      std::cout <<"a " << a << std::endl ;
      std::cout <<"b " << b << std::endl ;
    }
#endif

    lagrange(a,b,X1,X2);
    //  if (err != OK)
    {
      //      std::cout << "in (CLagrange.cc)Lagrange returns " ;
      //    PrintError(err) ;
      //    return err ;
    }


#if (DEBUG_LEVEL2)
    {
      std::cout << "ax1+bx2 (devrait etre 0) " << (a*X1 + b*X2).t() << std::endl ;
      std::cout << "norme X1 " << X1.sumSquare() << std::endl ;;
    }
#endif

    if (X2[8] < 0.0)
    {		/* car Zo > 0	*/
      X1 *= -1 ;
      X2 *= -1 ;
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} /* X1^T X2 = 0	*/

    //s = 0.0;
    //for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}
    s = X2[0]*X2[0] + X2[1]*X2[1] + X2[2]*X2[2]; // To avoid a Coverity copy/past error

    if (s<1e-10)
    {
//      std::cout << "Points that produce an error: " << std::endl;
//      for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it)
//      {
//        std::cout << "P: " << (*it).get_x() << " " << (*it).get_y() << " "
//                  << (*it).get_oX() << " " << (*it).get_oY() << " " << (*it).get_oZ() << std::endl;
//      }
      //vpERROR_TRACE(" division par zero " ) ;
      throw(vpException(vpException::divideByZeroError,
                        "Division by zero in Lagrange pose computation (non planar plane case)")) ;
    }

    s = 1.0/sqrt(s);
    for (i=0;i<3;i++)  {X2[i] *= s;}		/* X2^T X2 = 1	*/

    X2[3] = (X1[1]*X2[2])-(X1[2]*X2[1]);
    X2[4] = (X1[2]*X2[0])-(X1[0]*X2[2]);
    X2[5] = (X1[0]*X2[1])-(X1[1]*X2[0]);

    calculTranslation (a, b, nl, 3, 6, X1, X2) ;

    for (i=0 ; i<3 ; i++)
    {
      cMo[i][0] = X1[i];
      cMo[i][1] = X2[i];
      cMo[i][2] = X2[i+3];
      cMo[i][3] = X2[i+6];
    }

  }
  catch(...)
  {
    throw; // throw the original exception
  }

#if (DEBUG_LEVEL1)
  std::cout << "end vpCalculPose::PoseLagrange(...) " << std::endl ;
#endif
}


#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
