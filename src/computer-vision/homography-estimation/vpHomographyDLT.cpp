
/*
  #----------------------------------------------------------------------------
  #  Copyright (C) 2002  IRISA-INRIA Rennes Vista Project
  #  All Rights Reserved.
  #
  #    Contact:
  #       IRISA-INRIA Rennes
  #       Campus Universitaire de Beaulieu
  #       35042 Rennes Cedex
  #       France
  #
  #    email: marchand@irisa.fr
  #    www  : http://www.irisa.fr/lagadic
  #
  #----------------------------------------------------------------------------
*/


/*!
  \file vpHomographyDLT.cpp

  This file implements the fonctions related with the homography
  estimation using the DLT algorithm
*/
#include <visp/vpHomography.h>



void
vpHomography::HartleyNormalization(int n,
				   double *x, double *y,
				   double *xn, double *yn,
				   double &xg, double &yg,
				   double &coef)
{
  int i;
  xg = 0 ;
  yg = 0 ;

  for (i =0 ; i < n ; i++)
  {
    xg += x[i] ;
    yg += y[i] ;
  }
  xg /= n ;
  yg /= n ;

  //Changement d'origine : le centre de gravité doit correspondre
  // à l'origine des coordonnées
  double distance=0;
  for(i=0; i<n;i++)
  {
    double xni=x[i]-xg;
    double yni=y[i]-yg;
    xn[i] = xni ;
    yn[i] = yni ;
    distance+=sqrt(vpMath::sqr(xni)+vpMath::sqr(yni));
  }//for translation sur tous les points

  //Changement d'échelle
  distance/=n;
  //calcul du coef de changement d'échelle
  if(distance ==0)
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

  //calcul des transformations à appliquer sur M_norm pour obtenir M
  //en fonction des deux normalisations effectuées au début sur
  //les points: aHb = T2^ aHbn T1
  vpMatrix T1(3,3);
  vpMatrix T2(3,3);
  vpMatrix T2T(3,3);

  T1.setIdentity();
  T2.setIdentity();
  T2T.setIdentity();

  T1[0][0]=T1[1][1]=coef1;
  T1[0][2]=-coef1*xg1 ;
  T1[1][2]=-coef1*yg1 ;

  T2[0][0]=T2[1][1]=coef2;
  T2[0][2]=-coef2*xg2 ;
  T2[1][2]=-coef2*yg2 ;


  T2T=T2.pseudoInverse(1e-16) ;

  vpMatrix maHb=T2T*(vpMatrix)aHbn*T1;

  for (int i=0 ; i < 3 ; i++)
    for (int j=0 ; j < 3 ; j++) aHb[i][j] = maHb[i][j] ;

}


/*!
  \brief  Computes the homography matrix wrt. the
  data using  normalized  DLT (Direct Linear Transform)


  Normalizes data, computes H wrt. these normalized data and denormalizes
  the result. The normalization carried out is the one preconized by Hartley .
  At least 4 correspondant points couples are needed.
*/
void
vpHomography::HartleyDLT(int n,
			 double *xb, double *yb,
			 double *xa, double *ya ,
			 vpHomography &aHb)
{
  try{
  //initialise les données initiales
  // code_retour =InitialData(n, p1,p2);

  // normalize points
  double *xbn;
  double *ybn;
  xbn = new double [n];
  ybn = new double [n];

  double xg1, yg1, coef1 ;
  HartleyNormalization(n,
		       xb,yb,
		       xbn,ybn,
		       xg1, yg1,coef1);

  double *xan;
  double *yan;
  xan = new double [n];
  yan = new double [n];

  double xg2, yg2, coef2 ;
  vpHomography::HartleyNormalization(n,
		       xa,ya,
		       xan,yan,
		       xg2, yg2,coef2);

  vpHomography aHbn ;
  //compute the homography using the DLT from normalized data
  vpHomography::DLT(n,xbn,ybn,xan,yan,aHbn);

  //H dénormalisée
  vpHomography::HartleyDenormalization(aHbn,aHb,xg1,yg1,coef1,xg2,yg2, coef2);

  delete [] xbn;
  delete [] ybn;
  delete [] xan;
  delete [] yan;

  }
  catch(...)
  {
    TRACE(" ") ;
    throw ;
  }
}


/*!
  \brief Computes the homography matrix wrt. the
  data using DLT (Direct Linear Transform)  algorithm

  Computes H such as  \f[
  ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
  \f]

  To do so, we the DLT algorithm on the data,
  ie we resolve the linear system  by SDV : A.h =0 .
  h is the vector with the terms of H_norm,

  A depends on the  points coordinates.

  At least 4 correspondant points couples are needed.


 For each point, in homogeneous coordinates:
\f[
\mathbf{p}_{a}= \mathbf{H}\mathbf{p}_{b}
\f]
which is equivalent to:
\f[
\mathbf{p}_{a} \times \mathbf{H}\mathbf{p}_{b}  =0
\f]
if we note \f$\mathbf{h}_j^T\f$ the  \f$j^{\textrm{th}}\f$ line of  \f$\mathbf{H}\f$, we can write:
\f[
 \mathbf{H}\mathbf{p}_{b}  = \left( \begin{array}{c}\mathbf{h}_1^T\mathbf{p}_{b} \\\mathbf{h}_2^T\mathbf{p}_{b} \\\mathbf{h}_3^T \mathbf{p}_{b} \end{array}\right)
\f]

Setting \f$\mathbf{p}_{a}=(x_{a},y_{a},w_{a})\f$, the cross product  can be rewritten by:
 \f[
\mathbf{p}_{a} \times \mathbf{H}\mathbf{p}_{b}  =\left( \begin{array}{c}y_{a}\mathbf{h}_3^T\mathbf{p}_{b}-w_{a}\mathbf{h}_2^T\mathbf{p}_{b} \\w_{a}\mathbf{h}_1^T\mathbf{p}_{b} -x_{a}\mathbf{h}_3^T \mathbf{p}_{b} \\x_{a}\mathbf{h}_2^T \mathbf{p}_{b}- y_{a}\mathbf{h}_1^T\mathbf{p}_{b}\end{array}\right)
\f]


  \f[
\underbrace{\left( \begin{array}{ccc}\mathbf{0}^T & -w_{a} \mathbf{p}_{b}^T
      & y_{a} \mathbf{p}_{b}^T     \\     w_{a}
      \mathbf{p}_{b}^T&\mathbf{0}^T & -x_{a} \mathbf{p}_{b}^T      \\
      -y_{a} \mathbf{p}_{b}^T & x_{a} \mathbf{p}_{b}^T &
      \mathbf{0}^T\end{array}\right)}_{\mathbf{A}_i (3\times 9)}
      \underbrace{\left( \begin{array}{c}\mathbf{h}_1      \\
      \mathbf{h}_2\\\mathbf{h}_3\end{array}\right)}_{\mathbf{h} (9\times 1)}=0
\f]
  \f[
\underbrace{\left( \begin{array}{ccc}\mathbf{0}^T & -w_{a} \mathbf{p}_{b}^T
      & y_{a} \mathbf{p}_{b}^T     \\     w_{a}
      \mathbf{p}_{b}^T&\mathbf{0}^T & -x_{a} \mathbf{p}_{b}^T      \\
      -y_{a} \mathbf{p}_{b}^T & x_{a} \mathbf{p}_{b}^T &
      \mathbf{0}^T\end{array}\right)}_{\mathbf{A}_i (3\times 9)}
      \underbrace{\left( \begin{array}{c}\mathbf{h}_1      \\
      \mathbf{h}_2\\\mathbf{h}_3\end{array}\right)}_{\mathbf{h} (9\times 1)}=0
\f]
leading to an homogeneous system to be solve:   \f$\mathbf{A}\mathbf{h}=0\f$ with
\f$\mathbf{A}=\left(\mathbf{A}_1^T, ..., \mathbf{A}_i^T, ..., \mathbf{A}_n^T \right)^T\f$.

It can be solved using an SVD decomposition:
\f[\bf A = UDV^T \f]
<b>h</b> is the column of <b>V</b> associated with the smalest singular value of <b>A
</b>

*/
void vpHomography::DLT(int n,
		       double *xb, double *yb,
		       double *xa, double *ya ,
		       vpHomography &aHb)
{

  // 4 point are required
  if(n<4)
  {
    TRACE("there must be at least 4 points in the both images\n") ;
    throw ;
  }

  try{
    vpMatrix A(2*n,9);
    vpColVector h(9);
    vpColVector D(9);
    vpMatrix V(9,9);
	int i, j;

    // build matrix A
    for(i=0; i<n;i++)
    {

      A[2*i][0]=0;
      A[2*i][1]=0;
      A[2*i][2]=0;
      A[2*i][3]=-xb[i] ;
      A[2*i][4]=-yb[i] ;
      A[2*i][5]=-1;
      A[2*i][6]=xb[i]*ya[i] ;
      A[2*i][7]=yb[i]*ya[i];
      A[2*i][8]=ya[i];


      A[2*i+1][0]=xb[i] ;
      A[2*i+1][1]=yb[i] ;
      A[2*i+1][2]=1;
      A[2*i+1][3]=0;
      A[2*i+1][4]=0;
      A[2*i+1][5]=0;
      A[2*i+1][6]=-xb[i]*xa[i];
      A[2*i+1][7]=-yb[i]*xa[i];
      A[2*i+1][8]=-xa[i] ;
    }


    // solve Ah = 0
    // SVD  Decomposition A = UDV^T (destructive wrt A)
    A.svd(D,V);



    // on en profite pour effectuer un controle sur le rang de la matrice :
    // pas plus de 2 valeurs singulières quasi=0
    int rank=0;
    for(i = 0; i<9;i++) if(D[i]>1e-7) rank++;
    if(rank <7)
    {
     TRACE(" le rang est de : %d, shoud be 8", rank);
     throw ;
    }
    //h = is the column of V associated with the smallest singular value of A

    // since  we are not sure that the svd implemented sort the
    // singular value... we seek for the smallest
    double smallestSv = 1e30 ;
    int indexSmallestSv  = 0 ;
    for (i=0 ; i < 9 ; i++)
      if ((D[i] < smallestSv) ){ smallestSv = D[i] ;indexSmallestSv = i ; }


    h=V.column(indexSmallestSv+1);

    // build the homography
    for(i =0;i<3;i++)
    {
      for(j=0;j<3;j++)
	aHb[i][j]=h[3*i+j];
    }

  }
  catch(...)
  {
    TRACE(" ") ;
    throw ;
  }
}
