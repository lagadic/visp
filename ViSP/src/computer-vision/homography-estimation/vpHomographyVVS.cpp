//#include <computeHomography.h>
//#include <utilsHomography.h>
#include <visp/vpRobust.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpPoint.h>
#include <iostream>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

void
UpdatePoseRotation(vpColVector& dx,vpHomogeneousMatrix&  mati)
{
  int i,j;

  double sinu,cosi,mcosi,u[3],    s;
  vpRotationMatrix rd ;


  s = sqrt(dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2]);
  if (s > 1.e-25)
  {

    for (i=0;i<3;i++) u[i] = dx[i]/s;
    sinu = sin(s);
    cosi = cos(s);
    mcosi = 1-cosi;
    rd[0][0] = cosi + mcosi*u[0]*u[0];
    rd[0][1] = -sinu*u[2] + mcosi*u[0]*u[1];
    rd[0][2] = sinu*u[1] + mcosi*u[0]*u[2];
    rd[1][0] = sinu*u[2] + mcosi*u[1]*u[0];
    rd[1][1] = cosi + mcosi*u[1]*u[1];
    rd[1][2] = -sinu*u[0] + mcosi*u[1]*u[2];
    rd[2][0] = -sinu*u[1] + mcosi*u[2]*u[0];
    rd[2][1] = sinu*u[0] + mcosi*u[2]*u[1];
    rd[2][2] = cosi + mcosi*u[2]*u[2];
  }
  else
  {
    for (i=0;i<3;i++)
    {
      for(j=0;j<3;j++) rd[i][j] = 0.0;
      rd[i][i] = 1.0;
    }

  }

  vpHomogeneousMatrix Delta ;
  Delta.insert(rd) ;

  mati = Delta.inverse() * mati ;
}

double
computeRotation(int nbpoint,
		vpPoint *c1P,
		vpPoint *c2P,
		vpHomogeneousMatrix &c2Mc1,
		int display,
		vpImage<unsigned char> &I,
		vpCameraParameters &cam,
		int userobust, int save
		)
{
  vpColVector e(2) ;
  double r_1 = -1 ;

  vpColVector p2(3) ;
  vpColVector p1(3) ;
  vpColVector Hp2(3) ;
  vpColVector Hp1(3) ;

  vpMatrix H2(2,3) ;
  vpColVector e2(2) ;
  vpMatrix H1(2,3) ;
  vpColVector e1(2) ;

  int only_1 = 0 ;
  int only_2 = 0 ;
  int iter = 0 ;

  int n=0 ;
  for (int i=0 ; i < nbpoint ; i++)
    if ((c2P[i].get_x() !=-1) && (c1P[i].get_x() !=-1))
    {
      n++ ;
    }

  if ((only_1==1) || (only_2==1))  ; else n *=2 ;

  vpRobust robust(n);
  vpColVector res(n) ;
  vpColVector w(n) ;
  w =1 ;
  robust.setThreshold(0.0000) ;
  vpMatrix W(2*n,2*n)  ;
  W = 0 ;
  vpMatrix c2Rc1(3,3) ;

  double r =0 ;
  while  (((int)((r_1 - r)*1e7) !=0))
  {

    r_1 =r ;
    // compute current position

    //Change frame (current)
    for (int i=0 ; i < 3 ; i++)
      for (int j=0 ; j < 3 ; j++)
	c2Rc1[i][j] = c2Mc1[i][j] ;

    vpMatrix L(2,3), Lp ;
    int k =0 ;
    for (int i=0 ; i < nbpoint ; i++)
      if ((c2P[i].get_x() !=-1) && (c1P[i].get_x() !=-1))
      {
	p2[0] = c2P[i].get_x() ;
	p2[1] = c2P[i].get_y() ;
	p2[2] = 1.0 ;
	p1[0] = c1P[i].get_x() ;
	p1[1] = c1P[i].get_y() ;
	p1[2] = 1.0 ;

	Hp2 = c2Rc1.t()*p2 ; // p2 = Hp1
	Hp1 = c2Rc1*p1 ;     // p1 = Hp2

	Hp2 /= Hp2[2] ;  // normalisation
	Hp1 /= Hp1[2] ;


	// set up the interaction matrix
	double x = Hp2[0] ;
	double y = Hp2[1] ;

	H2[0][0] = x*y ;   H2[0][1] = -(1+x*x) ; H2[0][2] = y ;
	H2[1][0] = 1+y*y ; H2[1][1] = -x*y ;     H2[1][2] = -x ;
	H2 *=-1 ;
	H2 = H2*c2Rc1.t() ;

	// Set up the error vector
	e2[0] = Hp2[0] - c1P[i].get_x() ;
	e2[1] = Hp2[1] - c1P[i].get_y() ;

	// set up the interaction matrix
	x = Hp1[0] ;
	y = Hp1[1] ;

	H1[0][0] = x*y ;   H1[0][1] = -(1+x*x) ; H1[0][2] = y ;
	H1[1][0] = 1+y*y ; H1[1][1] = -x*y ;     H1[1][2] = -x ;

	// Set up the error vector
	e1[0] = Hp1[0] - c2P[i].get_x() ;
	e1[1] = Hp1[1] - c2P[i].get_y() ;

	if (only_2==1)
	{
	  if (k == 0) { L = H2 ; e = e2 ; }
	  else
	  {
	    L = vpMatrix::stackMatrices(L,H2) ;
	    e = vpMatrix::stackMatrices(e,e2) ;
	  }
	}
	else
	  if (only_1==1)
	  {
	    if (k == 0) { L = H1 ; e= e1 ; }
	    else
	    {
	      L =  vpMatrix::stackMatrices(L,H1) ;
	      e = vpMatrix::stackMatrices(e,e1) ;
	    }
	  }
	  else
	  {
	    if (k == 0) {L = H2 ; e = e2 ; }
	    else
	    {
	      L =  vpMatrix::stackMatrices(L,H2) ;
	      e =  vpMatrix::stackMatrices(e,e2) ;
	    }
	    L =  vpMatrix::stackMatrices(L,H1) ;
	    e =  vpMatrix::stackMatrices(e,e1) ;
	  }

	/*
	if (display==1)
	{
	if (only_2==1)
	  displayPoint(I, p1, Hp2,cam, vpColor::blue, 0) ;
	else
	  if (only_1==1)
	    displayPoint(I, p2, Hp1,cam, vpColor::green, 0) ;
	  else
	  {
	    displayPoint(I, p2, Hp1,cam, vpColor::green, 0) ;
	    displayPoint(I, p1, Hp2,cam, vpColor::blue, 0) ;
	  }
	  }*/
	k++ ;
      }

    if (userobust)
    {
      robust.setIteration(0);

      for(int k=0 ; k < n ; k++)
      {
	res[k] = vpMath::sqr(e[2*k]) + vpMath::sqr(e[2*k+1]) ;
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      //  cout << e.t() ;

      // cout << "----------------------------------" << endl ;


      // compute the pseudo inverse of the interaction matrix
      for (int k=0 ; k < n ; k++)
      {
	W[2*k][2*k] = w[k] ;
	W[2*k+1][2*k+1] = w[k] ;
      }
    }
    else
    {
      for (int k=0 ; k < 2*n ; k++) W[k][k] = 1 ;
    }
    // CreateDiagonalMatrix(w, W) ;
    (L).pseudoInverse(Lp, 1e-6) ;
    // Compute the camera velocity
    vpColVector c2Tc1 ;

    c2Tc1 = -2*Lp*W*e  ;

    // only for simulation

    UpdatePoseRotation(c2Tc1, c2Mc1) ;
    r =e.sumSquare() ;

    if ((W*e).sumSquare() < 1e-10) break ;
    if (iter>25) break ;
    iter++ ;    cout <<  iter <<"  e=" <<(e).sumSquare() <<"  e=" <<(W*e).sumSquare() <<endl ;

  }
  return (W*e).sumSquare() ;
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
