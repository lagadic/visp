#include <ComputeHomography.h>
#include <UtilHomographie.h>
#include <UtilEssential.h>
#include <iostream>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0

double
ComputeHomography(int nbpoint,
		  CPoint *c1P,
		  CPoint *c2P,
		  CMatrixHomogeneous &c2Mc1,
		  int display,
		  CImage<unsigned char> &I,
		  int userobust, int save
		  )
{
  CColVector e(2) ;
  double r_1 = -1 ;


  ofstream fe ;
  ofstream fv;

  if (save==1)
  {
    fe.open("error") ;
    fv.open("vel") ;
  }


  CColVector p2(3) ;
  CColVector p1(3) ;
  CColVector Hp2(3) ;
  CColVector Hp1(3) ;

	CMatrix H2(2,3) ;
	CColVector e2(2) ;
	CMatrix H1(2,3) ;
	CColVector e1(2) ;

  int only_1 = 0 ;
  int only_2 = 0 ;
  int iter = 0 ;
  int i ;
  int n=0 ;
  for (i=0 ; i < nbpoint ; i++)
    if ((c2P[i].x() !=-1) && (c1P[i].x() !=-1))
    {
      n++ ;
    }

  if ((only_1==1) || (only_2==1))  ; else n *=2 ;

  CRobustEstimator robust(n);
  CColVector res(n) ;
  CColVector w(n) ;
  w =1 ;
  robust.setThreshold(0.0000) ;
  CMatrix W(2*n,2*n)  ;
  W = 0 ;
  CMatrix c2Rc1 ;

  double r =0 ;
  while  (((int)((r_1 - r)*1e18) !=0))
  {

    r_1 =r ;
    // compute current position

    //Change frame (current)
    ExtractRotation(c2Mc1, c2Rc1) ;

    CMatrix L(2,3), Lp ;
    int k =0 ;
    for (i=0 ; i < nbpoint ; i++)
      if ((c2P[i].x() !=-1) && (c1P[i].x() !=-1))
      {
	p2[0] = c2P[i].x() ;
	p2[1] = c2P[i].y() ;
	p2[2] = 1.0 ;
	p1[0] = c1P[i].x() ;
	p1[1] = c1P[i].y() ;
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
	e2[0] = Hp2[0] - c1P[i].x() ;
	e2[1] = Hp2[1] - c1P[i].y() ;

	// set up the interaction matrix
	x = Hp1[0] ;
	y = Hp1[1] ;

	H1[0][0] = x*y ;   H1[0][1] = -(1+x*x) ; H1[0][2] = y ;
	H1[1][0] = 1+y*y ; H1[1][1] = -x*y ;     H1[1][2] = -x ;

	// Set up the error vector
	e1[0] = Hp1[0] - c2P[i].x() ;
	e1[1] = Hp1[1] - c2P[i].y() ;

	if (only_2==1)
	{
	  if (k == 0) { L = H2 ; e = e2 ; }
	  else
	  {
	    L = StackMatrices(L,H2) ;
	    e = StackMatrices(e,e2) ;
	  }
	}
	else
	  if (only_1==1)
	  {
	    if (k == 0) { L = H1 ; e= e1 ; }
	    else
	    {
	      L = StackMatrices(L,H1) ;
	      e = StackMatrices(e,e1) ;
	    }
	  }
	  else
	  {
	    if (k == 0) {L = H2 ; e = e2 ; }
	    else
	    {
	      L = StackMatrices(L,H2) ;
	      e = StackMatrices(e,e2) ;
	    }
	    L = StackMatrices(L,H1) ;
	    e = StackMatrices(e,e1) ;
	  }


	if (display==1)
	{
	if (only_2==1)
	  DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	else
	  if (only_1==1)
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	  else
	  {
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	    DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	  }
	}
	k++ ;
      }

    if (userobust)
    {
      robust.setIteration(0);

      for(int k=0 ; k < n ; k++)
      {
	res[k] = SQR(e[2*k]) + SQR(e[2*k+1]) ;
      }
      robust.Mestimator(TUKEY, res, w);

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
    (L).PseudoInverse(Lp, 1e-6) ;
    // Compute the camera velocity
    CColVector c2Tc1 ;

    c2Tc1 = -1*Lp*W*e  ;

    // only for simulation

    UpdatePoseRotation(c2Tc1, c2Mc1) ;
    r =e.SumSquare() ;

    if (save==1)
      fv << c2Tc1.t() ;
    if (save==1)
      fe << (W*e).SumSquare() <<endl ;

    //   cout <<  iter <<"  e=" <<(W*e).SumSquare() <<endl ;
    //    cout <<w.t();

    if ((W*e).SumSquare() < 1e-10) break ;
    if (iter>200) break ;
    iter++ ;
  }
  //  cout <<  iter <<"  e=" <<(e).SumSquare() <<"  e=" <<(W*e).SumSquare() <<endl ;
  return (W*e).SumSquare() ;
  //  return OK ;
}

double
ComputeHomographyPlane(int nbpoint,
		       CPoint *c1P,
		       CPoint *c2P,
		       CMatrixHomogeneous &c2Mc1,
		       CMatrixHomogeneous &c1Mo,
		       CPlane &oN,
		       int display,
		       CImage<unsigned char> &I,
		       int userobust, int save
		  )
{

  if (DEBUG_LEVEL1)
    cout << "begin ComputeHomographyPlane(..) " <<endl ;

  CColVector e(2) ;
  double r_1 = -1 ;


  ofstream fe ;
  ofstream fv;

  if (save==1)
  {
    fe.open("error") ;
    fv.open("vel") ;
  }


  CColVector p2(3) ;
  CColVector p1(3) ;
  CColVector Hp2(3) ;
  CColVector Hp1(3) ;

  CMatrix H2(2,6) ;
  CColVector e2(2) ;
  CMatrix H1(2,6) ;
  CColVector e1(2) ;


  int only_1 = 0 ;
  int only_2 = 0 ;
  int iter = 0 ;
  int i ;
  int n=0 ;
  for (i=0 ; i < nbpoint ; i++)
    if ((c2P[i].x() !=-1) && (c1P[i].x() !=-1))
    {
      n++ ;
    }


  if ((only_1==1) || (only_2==1))  ; else n *=2 ;

  CRobustEstimator robust(n);
  CColVector res(n) ;
  CColVector w(n) ;
  w =1 ;
  robust.setThreshold(0.0000) ;
  CMatrix W(2*n,2*n)  ;
  W = 0 ;

  CColVector N1(3), N2(3) ;
  double d1, d2 ;

  double r =1e10 ;
  iter =0 ;
  while  (((int)((r_1 - r)*1e18) !=0))
  {

    r_1 =r ;
    // compute current position


    //Change frame (current)
    CMatrixHomogeneous c1Mc2, c2Mo ;
    CMatrix c1Rc2, c2Rc1  ;
    CColVector c1Tc2, c2Tc1 ;
    c1Mc2 = c2Mc1.InvertHomogeneous() ;
    ExtractRotation(c2Mc1, c2Rc1) ;
    ExtractTranslation(c2Mc1, c2Tc1) ;
    ExtractRotation(c1Mc2, c1Rc2) ;
    ExtractTranslation(c1Mc2, c1Tc2) ;

    c2Mo = c2Mc1*c1Mo ;

    GetPlaneInfo(oN, c1Mo, N1, d1) ;
    GetPlaneInfo(oN, c2Mo, N2, d2) ;

    if (DEBUG_LEVEL2)    cout << " N1 " << N1.t() ;
    if (DEBUG_LEVEL2)   cout << " N2 " << N2.t() ;

    if (DEBUG_LEVEL2)   cout << " d1 " << d1 << endl ; ;
    if (DEBUG_LEVEL2)   cout << " d2 " << d2 << endl ;;


    CMatrix L(2,3), Lp ;
    int k =0 ;
    for (i=0 ; i < nbpoint ; i++)
      if ((c2P[i].x() !=-1) && (c1P[i].x() !=-1))
      {
	p2[0] = c2P[i].x() ;
	p2[1] = c2P[i].y() ;
	p2[2] = 1.0 ;
	p1[0] = c1P[i].x() ;
	p1[1] = c1P[i].y() ;
	p1[2] = 1.0 ;

	CMatrix H(3,3) ;

	Hp2 = (c1Rc2 + (c1Tc2*N2.t())/d2)*p2 ;     // p2 = Hp1
	Hp1 = (c2Rc1 + (c2Tc1*N1.t())/d1)*p1 ;     // p1 = Hp2

	Hp2 /= Hp2[2] ;  // normalisation
	Hp1 /= Hp1[2] ;


	// set up the interaction matrix
	double x = Hp2[0] ;
	double y = Hp2[1] ;
	double Z1  ;

	Z1 = (N1[0]*x+N1[1]*y+N1[2])/d1 ;        // 1/z


	H2[0][0] = -Z1 ;  H2[0][1] = 0  ;       H2[0][2] = x*Z1 ;
	H2[1][0] = 0 ;     H2[1][1] = -Z1 ;     H2[1][2] = y*Z1 ;
	H2[0][3] = x*y ;   H2[0][4] = -(1+x*x) ; H2[0][5] = y ;
	H2[1][3] = 1+y*y ; H2[1][4] = -x*y ;     H2[1][5] = -x ;
	H2 *=-1 ;

	CMatrix c1CFc2(6,6) ;
	{
	  CMatrix sTR = Skew(c1Tc2)*c1Rc2 ;
	  for (int k=0 ; k < 3 ; k++)
	    for (int l=0 ; l<3 ; l++)
	    {
	      c1CFc2[k][l] = c1Rc2[k][l] ;
	      c1CFc2[k+3][l+3] = c1Rc2[k][l] ;
	      c1CFc2[k][l+3] = sTR[k][l] ;
	    }
	}
	H2 = H2*c1CFc2 ;



	// Set up the error vector
	e2[0] = Hp2[0] - c1P[i].x() ;
	e2[1] = Hp2[1] - c1P[i].y() ;

	// set up the interaction matrix
	x = Hp1[0] ;
	y = Hp1[1] ;

	Z1 = (N2[0]*x+N2[1]*y+N2[2])/d2 ; // 1/z

	H1[0][0] = -Z1 ;  H1[0][1] = 0  ;       H1[0][2] = x*Z1 ;
	H1[1][0] = 0 ;     H1[1][1] = -Z1 ;     H1[1][2] = y*Z1;
	H1[0][3] = x*y ;   H1[0][4] = -(1+x*x) ; H1[0][5] = y ;
	H1[1][3] = 1+y*y ; H1[1][4] = -x*y ;     H1[1][5] = -x ;

	// Set up the error vector
	e1[0] = Hp1[0] - c2P[i].x() ;
	e1[1] = Hp1[1] - c2P[i].y() ;

	if (only_2==1)
	{
	  if (k == 0) { L = H2 ; e = e2 ; }
	  else
	  {
	    L = StackMatrices(L,H2) ;
	    e = StackMatrices(e,e2) ;
	  }
	}
	else
	  if (only_1==1)
	  {
	    if (k == 0) { L = H1 ; e= e1 ; }
	    else
	    {
	      L = StackMatrices(L,H1) ;
	      e = StackMatrices(e,e1) ;
	    }
	  }
	  else
	  {
	    if (k == 0) {L = H2 ; e = e2 ; }
	    else
	    {
	      L = StackMatrices(L,H2) ;
	      e = StackMatrices(e,e2) ;
	    }
	    L = StackMatrices(L,H1) ;
	    e = StackMatrices(e,e1) ;
	  }


	if (display==1)
	{
	if (only_2==1)
	  DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	else
	  if (only_1==1)
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	  else
	  {
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	    DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	  }
	}
	k++ ;
      }

    if (userobust)
    {
      robust.setIteration(0);
      for(int k=0 ; k < n ; k++)
      {
	res[k] = SQR(e[2*k]) + SQR(e[2*k+1]) ;
      }
     robust.Mestimator(TUKEY, res, w);


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
    (W*L).PseudoInverse(Lp, 1e-16) ;
    // Compute the camera velocity
    CColVector c2Tcc1 ;

    c2Tcc1 = -1*Lp*W*e  ;

    // only for simulation

    UpdatePose2(c2Tcc1, c2Mc1) ;
    r =(W*e).SumSquare() ;

    if (save==1)
      fv << c2Tc1.t() ;
    if (save==1)
      fe << (W*e).SumSquare() <<endl ;

    cout <<  iter <<"  e=" <<(W*e).SumSquare() <<endl ;
    //    cout <<w.t();
    if (DEBUG_LEVEL2)  c2Mc1.PrintVector() ;

    if ((W*e).SumSquare() < 1e-15)  {break ; }
    if (iter>1000){break ; }
    if (r>r_1) {  break ; }
    iter++ ;
  }
 if (DEBUG_LEVEL1)
    cout << "end ComputeHomographyPlane(..)" <<endl ;
 return (W*e).SumSquare() ;
  //  return OK ;
}

double
ComputePoseHomographyPlane(int nbpoint,
			   CPoint *c1Pm,
			   CPoint *c2Pm,
			   CMatrixHomogeneous &c2Mc1,
			   CMatrixHomogeneous &c1Mo,
			   CPlane &oN,
			   CPoint *oP,
			   int display,
			   CImage<unsigned char> &I,
			   int userobust, int save
			   )
{

  if (DEBUG_LEVEL1)
    cout << "begin ComputeHomographyPlane(..) " <<endl ;

  CColVector e(2) ;
  double r_1 = -1 ;


  ofstream fe, fer ;
  ofstream fv, fp;

  if (save==1)
  {
    fe.open("e") ;
    fer.open("error") ;
    fv.open("vel") ;
    fp.open("pose") ;
  }


  CColVector p2(3) ;
  CColVector p1(3) ;
  CColVector Hp2(3) ;
  CColVector Hp1(3) ;

  CMatrix H2(2,6) ;
  CColVector e2(2) ;
  CMatrix H1(2,6) ;
  CColVector e1(2) ;


  int only_1 = 0 ;
  int only_2 = 0 ;
  int iter = 0 ;
  int i ;
  int n=0 ;
  for (i=0 ; i < nbpoint ; i++)
    if ((c2Pm[i].x() !=-1) && (c1Pm[i].x() !=-1))
    {
      n++ ;
    }


  if ((only_1==1) || (only_2==1))  ; else n *=2 ;

  CRobustEstimator robust(n);
  CColVector res(n) ;
  CColVector w(n) ;
  w =1 ;
  robust.setThreshold(0.0000) ;
  CMatrix W(2*n,2*n)  ;
  W = 0 ;

  CColVector N1(3), N2(3) ;
  double d1, d2 ;

  double r =1e10 ;
  iter =0 ;
  while  (((int)((r_1 - r)*1e18) !=0))
  {

    r_1 =r ;
    // compute current position


    //Change frame (current)
    CMatrixHomogeneous c1Mc2, c2Mo ;
    CMatrix c1Rc2, c2Rc1  ;
    CColVector c1Tc2, c2Tc1 ;
    c1Mc2 = c2Mc1.InvertHomogeneous() ;
    ExtractRotation(c2Mc1, c2Rc1) ;
    ExtractTranslation(c2Mc1, c2Tc1) ;
    ExtractRotation(c1Mc2, c1Rc2) ;
    ExtractTranslation(c1Mc2, c1Tc2) ;

    c2Mo = c2Mc1*c1Mo ;

    GetPlaneInfo(oN, c1Mo, N1, d1) ;
    GetPlaneInfo(oN, c2Mo, N2, d2) ;

    if (DEBUG_LEVEL2)    cout << " N1 " << N1.t() ;
    if (DEBUG_LEVEL2)   cout << " N2 " << N2.t() ;

    if (DEBUG_LEVEL2)   cout << " d1 " << d1 << endl ; ;
    if (DEBUG_LEVEL2)   cout << " d2 " << d2 << endl ;;


    CMatrix L(2,3), Lp ;
    int k =0 ;
    for (i=0 ; i < nbpoint ; i++)
      if ((c2Pm[i].x() !=-1) && (c1Pm[i].x() !=-1))
      {
	p2[0] = c2Pm[i].x() ;
	p2[1] = c2Pm[i].y() ;
	p2[2] = 1.0 ;
	p1[0] = c1Pm[i].x() ;
	p1[1] = c1Pm[i].y() ;
	p1[2] = 1.0 ;

	CMatrix H(3,3) ;

	Hp2 = (c1Rc2 + (c1Tc2*N2.t())/d2)*p2 ;     // p2 = Hp1
	Hp1 = (c2Rc1 + (c2Tc1*N1.t())/d1)*p1 ;     // p1 = Hp2

	Hp2 /= Hp2[2] ;  // normalisation
	Hp1 /= Hp1[2] ;


	// set up the interaction matrix
	double x = Hp2[0] ;
	double y = Hp2[1] ;
	double Z1  ;

	Z1 = (N1[0]*x+N1[1]*y+N1[2])/d1 ;        // 1/z


	H2[0][0] = -Z1 ;  H2[0][1] = 0  ;       H2[0][2] = x*Z1 ;
	H2[1][0] = 0 ;     H2[1][1] = -Z1 ;     H2[1][2] = y*Z1 ;
	H2[0][3] = x*y ;   H2[0][4] = -(1+x*x) ; H2[0][5] = y ;
	H2[1][3] = 1+y*y ; H2[1][4] = -x*y ;     H2[1][5] = -x ;
	H2 *=-1 ;

	CMatrix c1CFc2(6,6) ;
	{
	  CMatrix sTR = Skew(c1Tc2)*c1Rc2 ;
	  for (int k=0 ; k < 3 ; k++)
	    for (int l=0 ; l<3 ; l++)
	    {
	      c1CFc2[k][l] = c1Rc2[k][l] ;
	      c1CFc2[k+3][l+3] = c1Rc2[k][l] ;
	      c1CFc2[k][l+3] = sTR[k][l] ;
	    }
	}
	H2 = H2*c1CFc2 ;



	// Set up the error vector
	e2[0] = Hp2[0] - c1Pm[i].x() ;
	e2[1] = Hp2[1] - c1Pm[i].y() ;

	// set up the interaction matrix
	x = Hp1[0] ;
	y = Hp1[1] ;

	Z1 = (N2[0]*x+N2[1]*y+N2[2])/d2 ; // 1/z

	H1[0][0] = -Z1 ;  H1[0][1] = 0  ;       H1[0][2] = x*Z1 ;
	H1[1][0] = 0 ;     H1[1][1] = -Z1 ;     H1[1][2] = y*Z1;
	H1[0][3] = x*y ;   H1[0][4] = -(1+x*x) ; H1[0][5] = y ;
	H1[1][3] = 1+y*y ; H1[1][4] = -x*y ;     H1[1][5] = -x ;

	// Set up the error vector
	e1[0] = Hp1[0] - c2Pm[i].x() ;
	e1[1] = Hp1[1] - c2Pm[i].y() ;

	if (only_2==1)
	{
	  if (k == 0) { L = H2 ; e = e2 ; }
	  else
	  {
	    L = StackMatrices(L,H2) ;
	    e = StackMatrices(e,e2) ;
	  }
	}
	else
	  if (only_1==1)
	  {
	    if (k == 0) { L = H1 ; e= e1 ; }
	    else
	    {
	      L = StackMatrices(L,H1) ;
	      e = StackMatrices(e,e1) ;
	    }
	  }
	  else
	  {
	    if (k == 0) {L = H2 ; e = e2 ; }
	    else
	    {
	      L = StackMatrices(L,H2) ;
	      e = StackMatrices(e,e2) ;
	    }
	    L = StackMatrices(L,H1) ;
	    e = StackMatrices(e,e1) ;
	  }


	if (display==1)
	{
	if (only_2==1)
	  DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	else
	  if (only_1==1)
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	  else
	  {
	    DisplayPoint(I, p2, Hp1,VERT, 0) ;
	    DisplayPoint(I, p1, Hp2,BLEU, 0) ;
	  }
	}
	k++ ;
      }
    //-------------------------------

     if (0)
    for (i=0 ; i < nbpoint ; i++)
    {
      CPoint c2P ;
      //Change frame (current)
      CMatrixHomogeneous c2Mo ;
      c2Mo =  c2Mc1*c1Mo ;
      c2P  =c2Mo*oP[i] ;
      //
      // perspective projetction
      c2P.Set_x( c2P.X()/c2P.Z() ) ;
      c2P.Set_y( c2P.Y()/c2P.Z() ) ;
	//      c2P[i].Set_Z(1) ; // La on simule le fait de ne pas connaitre Z
	// set up the interaction matrix
      CMatrix H(2,6) ;
      H = c2P.Interaction() ; // or  c2Pd[i].Interaction()
      L = StackMatrices(L,H) ;

      CColVector e1(2) ;
      // Set up the error vector
      e1[0] = c2P.x() - c2Pm[i].x() ;
      e1[1] = c2P.y() - c2Pm[i].y() ;

      e = StackMatrices(e,e1) ;
      k++ ;
    }


    //-------------------------------
    if (userobust)
    {
      robust.setIteration(0);
      for(int k=0 ; k < n ; k++)
      {
	res[k] = SQR(e[2*k]) + SQR(e[2*k+1]) ;
      }
      robust.Mestimator(TUKEY, res, w);


      // compute the pseudo inverse of the interaction matrix
      for (int k=0 ; k < n ; k++)
      {
	W[2*k][2*k] = w[k] ;
	W[2*k+1][2*k+1] = w[k] ;
      }
    }
    else
    {
      W.Resize(e.GetRows(), e.GetRows()) ;
      W =0 ;
      for (int k=0 ; k < e.GetRows() ; k++) W[k][k] = 1 ;

    }
    (W*L).PseudoInverse(Lp, 1e-6) ;
    // Compute the camera velocity
    CColVector c2Tcc1 ;

    c2Tcc1 = -0.1*Lp*W*e  ;

    // only for simulation

    UpdatePose2(c2Tcc1, c2Mc1) ;
    r =(W*e).SumSquare() ;

    if (save==1)
    {
      CColVector vc2Mo ;
      MatrixToVector( c2Mc1, vc2Mo) ;

      fv << vc2Mo.t() ;
      fe << (W*e).SumSquare() <<endl ;
      MatrixToVector( c2Mo, vc2Mo) ;
      fp << vc2Mo.t() ;
    }

    cout <<  iter <<"  e=" <<(W*e).SumSquare() <<endl ;
    //    cout <<w.t();
    if (DEBUG_LEVEL2)  c2Mc1.PrintVector() ;

    if ((W*e).SumSquare() < 1e-15)  {break ; }
    if (iter>1000){break ; }
    if (r>r_1) {  break ; }
    iter++ ;
  }
 if (DEBUG_LEVEL1)
    cout << "end ComputeHomographyPlane(..)" <<endl ;
 return (W*e).SumSquare() ;
  //  return OK ;
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
