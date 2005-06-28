


#include <visp/vpMeEllipse.h>

#include <visp/vpMe.h>
#include <visp/vpRobust.h>
#include <visp/vpTrackingException.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 3

#define OK 0
#define FATAL_ERROR 1


void
computeTheta(double &theta, vpColVector &K, double i, double j)
{

  double A = 2*i+2*K[1]*j + 2*K[2] ;
  double B = 2*K[0]*j + 2*K[1]*i + 2*K[3];

  theta =  vpMath::atan2(A,B) ; //tangente

  while (theta > M_PI) { theta -= M_PI ; }
  while (theta < 0) { theta += M_PI ; }

}


vpMeEllipse::vpMeEllipse():vpMeTracker()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpMeEllipse::vpMeEllipse() " <<  endl ;

  // redimensionnement du vecteur de parametre
  // i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4

  circle = false ;
  K.resize(5) ;

  alpha1 = 0 ;
  alpha2 = 2*M_PI ;
  if (DEBUG_LEVEL1)
    cout << "end vpMeEllipse::vpMeEllipse() " << endl ;
}

vpMeEllipse::~vpMeEllipse()
{
  if (DEBUG_LEVEL1) cout << "begin vpMeEllipse::~vpMeEllipse() " << endl ;

  list.kill();

  if (DEBUG_LEVEL1) cout << "end vpMeEllipse::~vpMeEllipse() " << endl ;
}


// ===================================================================
/*!
 * \brief Construct a list of vpMeSiteME at a particular
 * \n		 sampling step between the two extremities of
 * \n		 the line.
 * \pre  Requies me to hold the size of the sample_step
 * \post Calculates the a normal to the line and stores
 * 			 angle in 'alpha'. Creates a list of sites (list)
 *				 (requires calculation of Rho(),Theta())
 * \return status
 */
// ===================================================================
void
vpMeEllipse::sample(vpImage<unsigned char> & I)
{
  // if(DEBUG_LEVEL1)
  {
    cout <<"begin vpMeEllipse::sample() : "<<endl ;
  }

  int rows = I.getRows() ;
  int cols = I.getCols() ;

  double n_sample;

  if (me->sample_step==0)
  {
    cout << "In vpMeEllipse::sample: " ;
    cout << "function called with sample step = 0" ;
    //return FATAL_ERROR ;
  }

  double j, i, j11, i11;
  j = i = 0.0 ;

  double incr = vpMath::rad(me->sample_step) ; // angle increment en degree
  int col = vpColor::red ;
  getParameters() ;


  // Delete old list
  list.front();
  list.kill();

  // sample positions

  double k = alpha1 ;
  while (k<alpha2)
  {

    j = a *cos(k) ; // equation of an ellipse
    i = b *sin(k) ; // equation of an ellipse

    // (i,j) are the coordinates on the origin centered ellipse ;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    j11 = jc + ce *j - se *i ;
    i11 = ic -( se *j + ce *i) ;

    vpDisplay::displayCross(I, (int)i11,  (int)j11,  5, col) ;

    double theta ;
    computeTheta(theta, K,  i11,  j11)  ;

    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i11), vpMath::round(j11), 0, rows, cols))
    {
      vpMeSite pix ;
      pix.init((int)i11, (int)j11, theta) ;
      pix.setDisplay(selectDisplay) ;
      pix.suppress = 0 ;

      if(DEBUG_LEVEL3)
      {
	vpDisplay::displayCross(I,vpMath::round(i11), vpMath::round(j11), 5, vpColor::blue);
      }
      list.addRight(pix);
    }
    k += incr ;

  }
  vpMeTracker::initTracking(I) ;

  n_sample = list.nbElements() ;

  //  if(DEBUG_LEVEL1)
  {
    cout <<"end vpMeEllipse::sample() : " ;
    cout << n_sample << " point inserted in the list " << endl  ;
  }

}


void
vpMeEllipse::reSample(vpImage<unsigned char>  &I)
{
  sample(I) ;
}

/*!
  calcul les parametres de l'ellipse en terme de centre de gravite (ic,jc)
  petit axe (b) et grand axe (a) et exentricite (e)
*/
void
vpMeEllipse::getParameters()
{

  double k[6] ;
  for (int i=0 ; i < 5 ; i++)
    k[i+1] = K[i] ;
  k[0] = 1 ;

  double d = k[2]*k[2] - k[0]*k[1];


  ic = (k[1] * k[3] - k[2] * k[4]) / d;
  jc = (k[0] * k[4] - k[2] * k[3]) / d;

  double sq =  sqrt(vpMath::sqr(k[1]-k[0]) + 4.0*vpMath::sqr(k[2])) ;
  if (circle ==true)
  {
    e = 0 ;
  }
  else
  {
    e = (k[1] - k[0] + sq) / (2.0*k[2]);
    e = (-1/e) ;

    e = atan(e) ;
  }

  if(e < 0.0)  e += M_PI ;

  ce = cos(e) ;
  se = sin(e) ;

  double num = 2.0*(k[0]*ic*ic + 2.0*k[2]*jc*ic + k[1]*jc*jc - k[5]) ;
  double a2 = num / (k[0] + k[1] + sq ) ;
  double b2 = num / (k[0] + k[1] - sq ) ;

  a = sqrt( a2 ) ;
  b = sqrt( b2 ) ;

}

void
vpMeEllipse::printParameters()
{
  cout << "K" << endl ;
  cout << K.t() ;
  cout << ic << "  " << jc << endl ;
}

/*!
 * \brief computeAngle
 */
void
vpMeEllipse::computeAngle(int ip1, int jp1, double &_alpha1,
			  int ip2, int jp2, double &_alpha2)
{

  getParameters() ;
  double j1, i1, j11, i11;
  j1 =  i1 =  0.0 ;

  int number_of_points = 2000 ;
  double incr = 2 * M_PI / number_of_points ; // angle increment

  double dmin1 = 1e6  ;
  double dmin2 = 1e6  ;

  double k =  -M_PI ;
  while(k < M_PI) {

    j1 = a *cos(k) ; // equation of an ellipse
    i1 = b *sin(k) ; // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse ;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    j11 = jc + ce *j1 - se *i1 ;
    i11 = ic -( se *j1 + ce *i1) ;

    double  d = vpMath::sqr(ip1-i11) + vpMath::sqr(jp1-j11) ;
    if (d < dmin1)
    {
      dmin1 = d ;
      alpha1 = k ;
      _alpha1 = k ;
    }
    d = vpMath::sqr(ip2-i11) + vpMath::sqr(jp2-j11) ;
    if (d < dmin2)
    {
      dmin2 = d ;
      alpha2 = k ;
      _alpha2 = k ;
    }
    k += incr ;
  }

  if (alpha2 <alpha1) alpha2 += 2*M_PI ;


  if (DEBUG_LEVEL1)
    cout << "end vpMeEllipse::computeAngle(..)" << alpha1 << "  " << alpha2 << endl ;

}
/*!
 * \brief computeAngle
 */
void
vpMeEllipse::computeAngle(int ip1, int jp1, int ip2, int jp2)
{

  double a1, a2 ;
  computeAngle(ip1,jp1,a1, ip2, jp2,a2) ;
}

void
vpMeEllipse::updateTheta()
{
  vpMeSite p ;
  list.front() ;
  for (int i=0 ; i < list.nbElement() ; i++)
  {
    p = list.value() ;
    computeTheta(theta, K,  p.ifloat, p.jfloat) ;
    p.alpha = theta ;
    list.modify(p) ;
    list.next() ;
  }

}

void
vpMeEllipse::suppressPoints()
{
  // Loop through list of sites to track
  list.front();
  while(!list.outside())
  {
    vpMeSite s = list.value() ;//current reference pixel
    if (s.suppress != 0)
      list.suppress() ;
    else
      list.next() ;
  }

  list.front();
  vpMeSite s = list.value() ;//current reference pixel
  i1 = s.i ;
  j1 = s.j ;
  list.end();
  s = list.value() ;//current reference pixel
  i2 = s.i ;
  j2 = s.j ;
}


void
vpMeEllipse::seekExtremities(vpImage<unsigned char>  &I)
{
  double k;


  int rows = I.getRows() ;
  int cols = I.getCols() ;

  int  memory_range = me->range ;
  me->range = 2 ;

  double  memory_mu1 = me->mu1 ;
  me->mu1 = 0.1 ;

  double  memory_mu2 = me->mu2 ;
  me->mu2 = 0.1 ;

  double i,j,i11,j11 ;
  k = alpha2 ;
  double incr = vpMath::rad(2.0) ;



  vpMeSite Plast ;
  Plast = list.lastValue() ;
  while (k < alpha2 + vpMath::rad(seek))
  {
    k += incr ;

    j = a *cos(k) ; // equation of an ellipse
    i = b *sin(k) ; // equation of an ellipse
    j11 = jc + ce *j - se *i ;
    i11 = ic -( se *j + ce *i) ;

    double theta ;
    computeTheta(theta, K,  i11,  j11)  ;

    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i11), vpMath::round(j11), 2, rows, cols))
    {
      vpMeSite P ;
      P = Plast ;
      P.init((int)i11, (int)j11, theta) ;
      P.setDisplay(selectDisplay) ;
      P.suppress = 0 ;

      P.track(I,me,true) ;

      if (P.suppress ==0)
      {
	list.end() ;
	list.addRight(P);
	if (DEBUG_LEVEL3) 	vpDisplay::displayCross(I,P.i,P.j, 25, vpColor::green) ;
      }
      else
	if (DEBUG_LEVEL3) 	vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::red) ;

    }


  }
  vpMeSite Pfirst ;
  Pfirst = list.firstValue() ;
  k = alpha1 ;
  while (k > alpha1 - vpMath::rad(seek))
  {
    k -= incr ;

    j = a *cos(k) ; // equation of an ellipse
    i = b *sin(k) ; // equation of an ellipse
    j11 = jc + ce *j - se *i ;
    i11 = ic -( se *j + ce *i) ;
    double theta ;
    computeTheta(theta, K,  i11,  j11)  ;

    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i11), vpMath::round(j11), 2, rows, cols))
    {
      vpMeSite P ; P = Pfirst ;
      P.init((int)i11, (int)j11, theta) ;
      P.setDisplay(selectDisplay) ;
      P.suppress = 0 ;
      P.track(I,me,true) ;

      if (P.suppress ==0)
      {

	list.front() ;
	list.addLeft(P);
	if (DEBUG_LEVEL3) 	vpDisplay::displayCross(I,P.i,P.j, 25, vpColor::green) ;
      }
      else
	if (DEBUG_LEVEL3) 	vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::red) ;


    }
  }
  suppressPoints() ;

  me->range = memory_range ;
  me->mu1 = memory_mu1 ;
  me->mu2 = memory_mu2 ;


}

void
vpMeEllipse::leastSquare(vpImage<unsigned char>  &I)
{


  // Construction du systeme Ax=b
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )
  int i ;

  vpMeSite p ;

  int iter =0 ;


  if (list.nbElement() < 3)
  {
    ERROR_TRACE("Not enough point") ;
    throw(vpTrackingException(vpTrackingException::NOT_ENOUGH_POINT_ERR,
			      "not enough point")) ;
  }
  if (circle ==false)
  {
    vpMatrix A(numberOfSignal(),5) ;
    vpColVector b(numberOfSignal()) ;
    vpRobust r(numberOfSignal()) ;
    r.setIteration(0) ;
    vpMatrix D(numberOfSignal(),numberOfSignal()) ;
    D.setIdentity() ;
    vpMatrix DA, DAmemory ;
    vpColVector DAx ;
    vpColVector w(numberOfSignal()) ;
    w =1 ;

    while (iter < 2)
    {


      // -------------------
      list.front() ;
      int k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{

	  A[k][0] = vpMath::sqr(p.jfloat) ;
	  A[k][1] = 2 * p.ifloat * p.jfloat ;
	  A[k][2] = 2 * p.ifloat ;
	  A[k][3] = 2 * p.jfloat ;
	  A[k][4] = 1 ;

	  b[k] = - vpMath::sqr(p.ifloat) ;
	  k++ ;
	}
	list.next() ;
      }

      //
      DA = D*A ;
      vpMatrix DAp ;

      K = DA.pseudoInverse(1e-26) *D*b ;

      DAx = DA*K ;

      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;

      list.front() ;
      k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  D[k][k] =w[k]  ;
	  k++ ;
	}
	list.next() ;
      }
      iter++ ;
    }
  }
  else
  {
    vpMatrix A(numberOfSignal(),3) ;
    vpColVector b(numberOfSignal()) ;
    vpColVector Kc(3) ;
    vpRobust r(numberOfSignal()) ;
    r.setIteration(0) ;
    vpMatrix D(numberOfSignal(),numberOfSignal()) ;
    D.setIdentity() ;
    vpMatrix DA, DAmemory ;
    vpColVector DAx ;
    vpColVector w(numberOfSignal()) ;
    w =1 ;
    while (iter < 2)
    {


      // -------------------
      list.front() ;
      int k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{

	  A[k][0] =  2* p.ifloat ;
	  A[k][1] =  2* p.jfloat ;

	  A[k][2] = 1 ;
	  b[k] = - vpMath::sqr(p.ifloat) - vpMath::sqr(p.jfloat) ;

	  k++ ;
	}
	list.next() ;
      }

      //
      DA = D*A ;
      vpMatrix DAp ;

      Kc = DA.pseudoInverse(1e-26) *D*b ;
      K[0] = 1 ;
      K[1] = 0 ;
      K[2] = Kc[0] ;
      K[3] = Kc[1] ;
      K[4] = Kc[2] ;

      DAx = DA*Kc ;

      vpColVector res ;
      res = DAx-b ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,res,w) ;

      list.front() ;
      k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  D[k][k] =w[k]  ;
	  k++ ;
	}
	list.next() ;
      }
      iter++ ;
    }
  }
  getParameters() ;
}

/*!
 * \brief Display Ellipse
 */
void
vpMeEllipse::display(vpImage<unsigned char> &I, int col)
{

  double j1, i1, j11, i11 ;
  double j2, i2, j22, i22 ;
  j1 = j2 = i1 = i2 = 0.0 ;

  double incr = vpMath::rad(2) ; // angle increment

  vpDisplay::displayCross(I,(int)ic,(int)jc,20,vpColor::red) ;


  double k = alpha1 ;
  while (k+incr<alpha2)
  {
    j1 = a *cos(k) ; // equation of an ellipse
    i1 = b *sin(k) ; // equation of an ellipse

    j2 = a *cos(k+incr) ; // equation of an ellipse
    i2 = b *sin(k+incr) ; // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse ;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    j11 = jc + ce *j1 - se *i1 ;
    i11 = ic -( se *j1 + ce *i1) ;
    // to get the coordinates of the point on the shifted ellipse
    j22 = jc + ce *j2 - se *i2 ;
    i22 = ic -( se *j2 + ce *i2) ;


    vpDisplay::displayLine(I, (int)i11,  (int)j11,  (int)i22,  (int)j22, col, 3) ;

    k += incr ;
  }

    j1 = a *cos(alpha1) ; // equation of an ellipse
    i1 = b *sin(alpha1) ; // equation of an ellipse

    j2 = a *cos(alpha2) ; // equation of an ellipse
    i2 = b *sin(alpha2) ; // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse ;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    j11 = jc + ce *j1 - se *i1 ;
    i11 = ic -( se *j1 + ce *i1) ;
    // to get the coordinates of the point on the shifted ellipse
    j22 = jc + ce *j2 - se *i2 ;
    i22 = ic -( se *j2 + ce *i2) ;


    vpDisplay::displayLine(I,(int)ic,(int)jc, (int)i11,  (int)j11, vpColor::red, 3) ;
    vpDisplay::displayLine(I,(int)ic,(int)jc, (int)i22,  (int)j22, vpColor::blue, 3) ;



}



void
vpMeEllipse::initTracking(vpImage<unsigned char> &I)
{
  if (DEBUG_LEVEL1)
    cout<<" begin vpMeEllipse::initTracking()"<<endl ;

  int n=5 ;
  int *i, *j ;
  i = new int[n] ;
  j = new int[n] ;

   for (int k =0 ; k < n ; k++)
  {
    cout << "Click points "<< k+1 <<"/" << n ;
    cout << " on the ellipse in the trigonometric order" <<endl ;
    while (vpDisplay::getClick(I,i[k],j[k])!=true) ;
    cout << i[k] <<" " <<j[k] << endl;
    }


   i1 = i[0] ;
   j1 = j[0] ;
   i2 = i[n-1] ;
   j2 = j[n-1] ;

   initTracking(I, n, i, j) ;

   delete []i ;
   delete []j ;

}

void
vpMeEllipse::initTracking(vpImage<unsigned char> &I, int n, int *i,int *j)
{
  if (DEBUG_LEVEL1)
    cout<<" begin vpMeEllipse::initTracking()"<<endl ;

  if (circle==false)
  {
    vpMatrix A(n,5) ;
    vpColVector b(n) ;
    vpColVector x(5) ;

    // Construction du systeme Ax=b
    //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
    // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )

    for (int k =0 ; k < n ; k++)
    {
      A[k][0] = vpMath::sqr(j[k]) ;
      A[k][1] = 2* i[k] * j[k] ;
      A[k][2] = 2* i[k] ;
      A[k][3] = 2* j[k] ;
      A[k][4] = 1 ;

      b[k] = - vpMath::sqr(i[k]) ;
    }

    K = A.pseudoInverse(1e-26)*b ;
    cout << K << endl;
  }
  else
  {
    vpMatrix A(n,3) ;
    vpColVector b(n) ;
    vpColVector x(3) ;

    vpColVector Kc(3) ;
    for (int k =0 ; k < n ; k++)
    {
      A[k][0] =  2* i[k] ;
      A[k][1] =  2* j[k] ;

      A[k][2] = 1 ;
      b[k] = - vpMath::sqr(i[k]) - vpMath::sqr(j[k]) ;
    }

    Kc = A.pseudoInverse(1e-26)*b ;
    K[0] = 1 ;
    K[1] = 0 ;
    K[2] = Kc[0] ;
    K[3] = Kc[1] ;
    K[4] = Kc[2] ;

    cout << K << endl;
  }
  getParameters() ;
  computeAngle(i1,j1, i2, j2) ;
  display(I, vpColor::green) ;

  sample(I) ;

  //  2. On appelle ce qui n'est pas specifique
  {
    vpMeTracker::initTracking(I) ;
  }


  try{
    track(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  vpMeTracker::display(I) ;
  vpDisplay::flush(I) ;

}

void
vpMeEllipse::track(vpImage<unsigned char> &I)
{
  if (DEBUG_LEVEL1)
    cout<<"begin vpMeEllipse::track()"<<endl ;

  static int iter =0 ;
  //  1. On fait ce qui concerne les ellipse (peut etre vide)
  {
  }

  vpDisplay::display(I) ;
  //  2. On appelle ce qui n'est pas specifique
  {

  try{
       vpMeTracker::track(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
    //    cout << "number of signals " << numberOfSignal() << endl ;
  }

  // 3. On revient aux ellipses
  {
    // Estimation des parametres de la droite aux moindres carre
    suppressPoints() ;
    try{
      leastSquare(I) ;  }
    catch(...)
    {
      ERROR_TRACE(" ") ;
      throw ;
    }
    computeAngle(i1,j1, i2, j2) ;

    if (iter%5==0)
    {
      sample(I) ;
      try{
	leastSquare(I) ;  }
      catch(...)
      {
	ERROR_TRACE(" ") ;
	throw ;
      }
      computeAngle(i1,j1, i2, j2) ;
    }
    seekExtremities(I) ;

    vpMeTracker::display(I) ;
    vpDisplay::flush(I) ;

    // remet a jour l'angle theta pour chaque  point de la liste
    updateTheta() ;

  }

  iter++ ;


  if (DEBUG_LEVEL1)
    cout<<"end vpMeEllipse::track()"<<endl ;

}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3

