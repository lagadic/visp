
/*
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/

#include <visp/vpPose.h>

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
calculTranslation (vpMatrix &a, vpMatrix &b, int nl, int nc1,
		   int nc3, vpColVector &x1, vpColVector &x2)
{

  try
  {
    int i,j;

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

    if (DEBUG_LEVEL2)
    {
      cout <<"ctc " << endl << ctc ;
      cout <<"cta " << endl << cta ;
      cout <<"ctb " << endl << ctb ;
    }



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

    if (DEBUG_LEVEL2)
      cout << "sv " << sv.t() ;
    vpColVector X3 ; /* X3 = - (C^T C )^{-1} C^T (A X1 + B X2) */
    X3 = -ctc1*sv ;

    if (DEBUG_LEVEL2)
      cout << "x3 " << X3.t()  ;
    for (i=0 ; i < nc1 ; i++)
      x2[i+nc3] = X3[i] ;
  }
  catch(...)
  {

    // en fait il y a des dizaines de raisons qui font que cette fonction
    // rende une erreur (matrice pas inversible, pb de memoire etc...)
    ERROR_TRACE(" ") ;
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
#define EPS 1.e-5

static
void
lagrange (vpMatrix &a, vpMatrix &b, vpColVector &x1, vpColVector &x2)
{
  if (DEBUG_LEVEL1)
    cout << "begin (CLagrange.cc)Lagrange(...) " << endl;

  try{
    int i,imin;

    vpMatrix ata ; // A^T A
    ata = a.t()*a ;
    vpMatrix btb ; // B^T B
    btb = b.t()*b ;

    vpMatrix bta ;  // B^T A
    bta = b.t()*a ;




    vpMatrix btb1 ;  // (B^T B)^(-1)
    btb1 = btb.inverseByLU() ;

    vpMatrix r ;  // (B^T B)^(-1) B^T A
    r = btb1*bta ;

    vpMatrix e ;  //   - A^T B (B^T B)^(-1) B^T A
    e = - (a.t()*b) *r ;

    e += ata ; // calcul E = A^T A - A^T B (B^T B)^(-1) B^T A

    vpColVector sv ;
    vpMatrix v ;
    e.svd(x1,ata) ;// destructuf sur e
    // calcul du vecteur propre de E correspondant a la valeur propre min.

    /* calcul de SVmax	*/
    double  svm = 0.0;
    imin = 0;
    for (i=0;i<x1.getRows();i++)
    {
      if (x1[i] > svm) { svm = x1[i]; imin = i; }
    }
    svm *= EPS;	/* pour le rang	*/

    for (i=0;i<x1.getRows();i++)
      if (x1[i] < x1[imin]) imin = i;

    for (i=0;i<x1.getRows();i++)
      x1[i] = ata[i][imin];

    x2 = - (r*x1) ; // X_2 = - (B^T B)^(-1) B^T A X_1
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  if (DEBUG_LEVEL1)
    cout << "end (CLagrange.cc)Lagrange(...) " << endl;

}


/*!
  \brief  Compute the pose using Lagrange approach
*/
void
vpPose::poseLagrangePlan(vpHomogeneousMatrix &cMo)
{

  if (DEBUG_LEVEL1)
    cout << "begin CPose::PoseLagrange(...) " << endl ;
  try
  {
    double s;
    int i;

    int k=0;
    int nl=npt*2;


    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,6);
    vpPoint P ;
    listP.front() ;
    i=0 ;
    while (!listP.outside())
    {
      P= listP.value() ;
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
      listP.next() ;
    }
    vpColVector X1(3) ;
    vpColVector X2(6) ;

    if (DEBUG_LEVEL2)
    {
      cout <<"a " << a << endl ;
      cout <<"b " << b << endl ;
    }


    lagrange(a,b,X1,X2);



    if (DEBUG_LEVEL2)
    {
      cout << "ax1+bx2 (devrait etre 0) " << (a*X1 + b*X2).t() << endl ;
      cout << "norme X1 " << X1.sumSquare() << endl ;;
    }
    if (X2[5] < 0.0)
    {		/* car Zo > 0	*/
      for (i=0;i<3;i++) X1[i] = -X1[i];
      for (i=0;i<6;i++) X2[i] = -X2[i];
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} /* X1^T X2 = 0	*/

    s = 0.0;
    for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}

    if (s<1e-10)
    {
      ERROR_TRACE( "in vpCalculPose::PosePlan(...) division par zero ") ;
      throw(vpException(vpException::divideByZeroERR,
			"division by zero  ")) ;
    }

    s = 1.0/sqrt(s);
    for (i=0;i<3;i++)  {X2[i] *= s;}		/* X2^T X2 = 1	*/


    calculTranslation (a, b, nl, 3, 3, X1, X2) ;

    // if (err != OK)
    {
      // cout << "in (vpCalculPose_plan.cc)CalculTranslation returns " ;
      // PrintError(err) ;
      //    return err ;
    }

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
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }


  if (DEBUG_LEVEL1)
    cout << "end vpCalculPose::PoseLagrange(...) " << endl ;
  //  return(OK);
}


void
vpPose::poseLagrangeNonPlan(vpHomogeneousMatrix &cMo)
{

  if (DEBUG_LEVEL1)
    cout << "begin CPose::PoseLagrange(...) " << endl ;

  try{
    double s;
    int i;

    int k=0;
    int nl=npt*2;


    vpMatrix a(nl,3)  ;
    vpMatrix b(nl,9);
    b =0 ;

    vpPoint P ;
    listP.front() ;
    i=0 ;
    while (!listP.outside())
    {
      P= listP.value() ;
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
      listP.next() ;
    }
    vpColVector X1(3) ;
    vpColVector X2(9) ;

    if (DEBUG_LEVEL2)
    {
      cout <<"a " << a << endl ;
      cout <<"b " << b << endl ;
    }

    lagrange(a,b,X1,X2);
    //  if (err != OK)
    {
      //      cout << "in (CLagrange.cc)Lagrange returns " ;
      //    PrintError(err) ;
      //    return err ;
    }


    if (DEBUG_LEVEL2)
    {
      cout << "ax1+bx2 (devrait etre 0) " << (a*X1 + b*X2).t() << endl ;
      cout << "norme X1 " << X1.sumSquare() << endl ;;
    }

    if (X2[8] < 0.0)
    {		/* car Zo > 0	*/
      X1 *= -1 ;
      X2 *= -1 ;
    }
    s = 0.0;
    for (i=0;i<3;i++) {s += (X1[i]*X2[i]);}
    for (i=0;i<3;i++)  {X2[i] -= (s*X1[i]);} /* X1^T X2 = 0	*/

    s = 0.0;
    for (i=0;i<3;i++)  {s += (X2[i]*X2[i]);}

    if (s<1e-10)
    {
      ERROR_TRACE(" division par zero " ) ;
      throw(vpException(vpException::divideByZeroERR,
			"division by zero  ")) ;

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
    ERROR_TRACE(" ") ;
    throw ;
  }

  if (DEBUG_LEVEL1)
    cout << "end vpCalculPose::PoseLagrange(...) " << endl ;
}


#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
