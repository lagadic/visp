/****************************************************************************
 *
 * $Id: vpMePath.cpp,v 1.1 2007-07-06 15:46:37 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Andrea Cherubini
 *
 *****************************************************************************/



#include <visp/vpMePath.h>

#include <visp/vpMe.h>
#include <visp/vpRobust.h>
#include <visp/vpTrackingException.h>
#include <visp/vpDebug.h>

#define   LSiter			2		//least square iterations 
#define   good_point_thresh		0.75		//threshold on least square line error

#define   sampleIter			5		//every sampleIter sample the curve
#define   pointPercentageWithinExtremes 1.0		//percent of samples within extremities

#define   seekLoops			5		//number of times extremities are seeked at each iteration
#define   numExtr			8		//extremities seeked

#define   goodPointGain			115.0   	//was 100
#define   maxLineScore			195.0   	//was 125.0
#define   par_det_threshold		0.00006		//was 0.000001
#define   aParThreshold			0.0009		//was 0.000001

void
vpMePath::computeNormAng(double &norm_ang, vpColVector &K, 
			 double i, double j, bool isLine)
{
  double A; // it is df/di
  double B;  // it is df/dj
  if (!isLine) {
    A = 2*i + K[1]*j + K[2] ;
    B = 2*K[0]*j + K[1]*i + K[3];
  } else {
    A = K[2] ;
    B = K[3] ;
  }
  norm_ang = atan2(A,B) ; //tangente
  while (norm_ang > M_PI) { norm_ang -= M_PI ; }
  while (norm_ang < 0) { norm_ang += M_PI ; }
}

vpMePath::vpMePath():vpMeTracker()
{
  setVerboseMode(false);

  // redimensionnement du vecteur de parametre
  // i^2 + K0 j^2 + K1 i j + K2 i + K3 j + K4
  K.resize(5) ;
  K_line.resize(5) ; 
  K_par.resize(5) ;
  j1 = j2 = i1 = i2 = 0 ;
  i_cir = new double[numPointCir] ;
  j_cir = new double[numPointCir] ;
  for (int i = 0 ; i < numPointCir ; i++) {
    i_cir [i] = -1000;
    j_cir [i] = -1000; 
  }
  sampling = false;

  // initialisation
  numPoints = 5; //initial points used to find parabola

  vpCDEBUG(1) << "end vpMePath::vpMePath() " << std::endl ;
}
vpMePath::~vpMePath()
{
  list.kill();
}
// ===================================================================
/*!
 * \brief Construct a list of vpMeSiteME at a particular
 * \n		 sampling step between the two extremities of
 * \n		 the line.
 * \pre  Requires me to hold the size of the sample_step
 * \post Calculates the a normal to the line and stores
 * 			 angle in 'alpha'. Creates a list of sites (list)
 *				 (requires calculation of Rho(),Theta())
 * \return status
 */
// ===================================================================
void
vpMePath::display(vpImage<unsigned char> &I, vpColor::vpColorType col)
{
  suppressPoints() ;
  double x_v = -bFin / (2*aFin),
    y_v = -bFin*bFin / (4*aFin) + cFin,
    ct = cos(thetaFin),
    st = sin(thetaFin);
  //display segment oriented along parabola symetric axis
  vpDisplay::displayLine(I, (int) i_ref [2], (int) j_ref [2],
			 (int)(i_ref [2] + 70*sin (thetaFin)), 
			 (int)(j_ref [2] + 70*cos (thetaFin)), 
			 vpColor::blue, 1) ;
  //display parabola vertex
  //    std::cout << "display vertex: i "<< x_v*ct + y_v*st << " j " << -x_v*st + y_v*ct << std::endl;
  if (!line) vpDisplay::displayCross(I,(int)(x_v*ct + y_v*st),
				     (int)(-x_v*st + y_v*ct),50,vpColor::cyan);
  //display initial points
  if (firstIter)
    for (int k = 0; k < numPoints; k++)
      vpDisplay::displayCross(I, (int) i_ref[k], (int) j_ref[k],10,
			      vpColor::green);
		
  double i_1, j_1, i_2, j_2, i_1o, j_1o; //image coords of top and bottom point
  if (firstIter) {
    i_1 = (double) i_ref[0];
    j_1 = (double) j_ref[0];
    i_2 = (double) i_ref[4];
    j_2 = (double) j_ref[4];
  } else {
    vpMeSite p = list.value() ;//current reference pixel
    list.front();
    p = list.value() ;
    i_1 = (double) p.i;
    j_1 = (double) p.j;
    list.end();
    p = list.value() ;//current reference pixel
    i_2 = (double) p.i;
    j_2 = (double) p.j;
  }
  vpDisplay::displayCross(I, (int) i1, (int) j1, 20, vpColor::red);	
  vpDisplay::displayCross(I, (int) i2, (int) j2, 20, vpColor::yellow);
  list.front();
  double i_dis, j_dis;
  while(!list.outside())
    {
      vpMeSite s = list.value() ;//current reference pixel
      list.next() ;
      i_dis = s.i ;
      j_dis = s.j ;
      vpDisplay::displayCross(I, (int) i_dis, (int) j_dis, 1, vpColor::red);
    }
  double x_2 = i_2*ct - j_2*st, 
    x_1 = i_1*ct - j_1*st,
    y_2 = i_2*st + j_2*ct, 
    y_1 = i_1*st + j_1*ct;
  int disp_samples = (int) (0.5 * sqrt (vpMath::sqr(i_2-i_1) + vpMath::sqr(j_2-j_1)));
  double incr = (double) ((x_2 - x_1) / disp_samples);
  if (line) {
    //project first and last points abscissa to estimated line 
    x_2 = (-x_2/bFin - y_2 + cFin) / (-bFin - 1/bFin);
    x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  } else {//TODO
    //project first and last points abscissa to estimated parabola 
    //x_2 = (-x_2/bFin - y_2 + cFin) / (-bFin - 1/bFin);
    //x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  }
  i_1 = x_1*ct + y_1*st;
  j_1 = -x_1*st + y_1*ct;
  for (int m = 0; m < disp_samples; m++) {
    i_1o = i_1;
    j_1o = j_1;
    x_1 = x_1 + incr; 
    y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
    i_1 = x_1*ct + y_1*st;
    j_1 = -x_1*st + y_1*ct;
    // vpDisplay::displayLine(I, (int) i_1o, (int) j_1o, (int) i_1, 
    // (int) j_1, col, 1) ;
  }
}
void 
vpMePath::seekExtremities(vpImage<unsigned char>  &I)
{
  unsigned height = I.getHeight() ;
  unsigned width = I.getWidth() ;
  vpMeSite p1, p2;//one extremity pixel
  suppressPoints() ;
  list.front();
  p1 = list.value() ;
  i1 = p1.ifloat;
  j1 = p1.jfloat;
  list.end();
  p2 = list.value();
  i2 = p2.ifloat;
  j2 = p2.jfloat;

  if (verbose)
    std::printf ("\e[32m 2-seekExtremities i1 %f j1 %f i2 %f j2 %f \e[30m \n",
		 i1, j1, i2, j2);

  // sample positions
  double ct = cos(thetaFin),
    st = sin(thetaFin),
    x_2 = i2*ct - j2*st, 
    x_1 = i1*ct - j1*st,
    y_2 = i2*st + j2*ct, 
    y_1 = i1*st + j1*ct;
  int samples = (int) (0.3 * sqrt (vpMath::sqr(i2-i1) + vpMath::sqr(j2-j1)));
  double incr = (double) ((x_2 - x_1) / samples);
  
  if (line) {
    x_2 = (-x_2/bFin - y_2 + cFin) / (-bFin - 1/bFin);//project first and last points abscissa to estimated line 
    x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  } else {
    //TODO project first and last points abscissa to estimated parabola
  }
  //if (verbose)
  //std::printf("start -------> x_1 %f x_2 %f incr %f \n", x_1, x_2, incr);
  double norm_ang ;
  vpMeSite pix ;
  double i_1, j_1, i_2, j_2;
  //  if (verbose)
  //std::printf("seek before adding points ---> list.nbElements() %d \n", list.nbElements());
  for (int m = 0; m < numExtr; m++) {
    x_1 = x_1 - incr; 
    y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
    i_1 = x_1*ct + y_1*st;
    j_1 = -x_1*st + y_1*ct;
    //if (verbose)
    //std::printf("-------> i_1 %f j_1 %f \n", i_1, j_1);
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i_1), vpMath::round(j_1), 0, height, width))
      {
	pix = p1;
	computeNormAng(norm_ang, K,  i_1,  j_1, line)  ;
	pix.init(i_1, j_1, norm_ang) ;
	pix.convlt = p1.convlt;
	//vpDisplay::displayCross(I, (unsigned)i_1,  (unsigned)j_1,  15, vpColor::blue) ;
	//if (verbose)
	//std::printf("1 extremity %f %f  \n", i_1, j_1);
	pix.setDisplay(NONE) ;//was selectdisplay
	pix.suppress = 0;
	pix.track(I,me,false) ;
	if (pix.suppress == 0)
	  {
	    vpDisplay::displayCross(I, (unsigned)pix.i,  (unsigned)pix.j,  2, vpColor::black) ;
	    list.front();
	    list.addLeft(pix);
	  }
      }
    x_2 = x_2 + incr; 
    y_2 = aFin*x_2*x_2 + bFin*x_2 + cFin;
    i_2 = x_2*ct + y_2*st;
    j_2 = -x_2*st + y_2*ct;
    //  if (verbose)
    //std::printf("-------> i_2 %f j_2 %f \n", i_2, j_2);
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i_2), vpMath::round(j_2), 0, height, width))
      {
	pix = p2;
	computeNormAng(norm_ang, K,  i_2,  j_2, line)  ;
	pix.init(i_2, j_2, norm_ang) ;
	pix.convlt = p2.convlt;
	//vpDisplay::displayCross(I, (unsigned)i_2,  (unsigned)j_2,  15, vpColor::blue) ;
	//  if (verbose)
	//std::printf("2 extremity %f %f  \n",i_2,j_2);
	pix.setDisplay(NONE) ;//was selectdisplay
	pix.suppress = 0;
	pix.track(I,me,false) ;
	if (pix.suppress == 0)
	  {
	    vpDisplay::displayCross(I, (unsigned)pix.i,  (unsigned)pix.j,  2, vpColor::black) ;
	    list.end();
	    list.addRight(pix);
	  }
      }
  }
  suppressPoints() ;
}

/*!


*/
void
vpMePath::sample(vpImage<unsigned char> & I)
{
  sampling = true;
  unsigned height = I.getHeight() ;
  unsigned width = I.getWidth() ;
  double incr ; 
  double i_1, j_1, i_2, j_2;	//image coords of first and last point
  int num_samples;
  double conv [1000];
  int j = 0;
  if (firstIter) {
    i_1 = (double) i_ref[0];
    j_1 = (double) j_ref[0];
    i_2 = (double) i_ref[4];
    j_2 = (double) j_ref[4];
    num_samples = (int) (2.0 * sqrt (vpMath::sqr(i_2-i_1) + vpMath::sqr(j_2-j_1)));
  } else {
    i_1 = (double) i1;
    j_1 = (double) j1;
    i_2 = (double) i2;
    j_2 = (double) j2;
    // Delete old list
    list.front();
    // Loop through list of sites to track
    while(!list.outside())
      {
	vpMeSite s = list.value() ;//current reference pixel
	list.next();
	conv[j] = s.convlt;
	j++;
      }
    num_samples = j;
    j = 0;
    list.kill();
  }
  //std::printf("sample() top %d %d bottom %d %d click to continue...\n",i_t, j_t, i_b, j_b);
  //vpDisplay::getClick(I) ;
  // sample positions
  double ct = cos(thetaFin),
    st = sin(thetaFin),
    x_2 = i_2*ct - j_2*st, 
    x_1 = i_1*ct - j_1*st,
    y_1 = i_1*st + j_1*ct;
  if (line) {
    x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);//project first point abscissa to estimated line 
  } else {//todo project first point abscissa to estimated parabola
    //x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  }
  incr = (double) ((x_2 - x_1) / (pointPercentageWithinExtremes *num_samples));
  x_1 = (double) (x_1 - ((1 - pointPercentageWithinExtremes) /2)  * num_samples*incr); 
  y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
  //std::printf("1 - sampling ---> num_samples %d incr %f \n",num_samples, incr);
  //std::printf("1 - sampling ---> aFin %f bFin %f cFin %f \n", aFin, bFin, cFin);
  for (int m = 0; m < num_samples; m++) {
    i_1 = x_1*ct + y_1*st;
    j_1 = -x_1*st + y_1*ct;
    double norm_ang ;
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(i_1), vpMath::round(j_1), 0, height, width)) {
      vpMeSite pix ;
      computeNormAng(norm_ang, K,  i_1, j_1, line);
      pix.init(i_1, j_1, norm_ang) ;
      if (!firstIter) {
	pix.convlt = conv[j];// aggiunto 29 giugno
	j++;
	pix.suppress = 0;
	//vpDisplay::displayCross(I, (unsigned)i_1,  (unsigned)j_1,  10, vpColor::yellow) ;
	//pix.setDisplay(NONE) ;//was selectdisplay //commento 27 giu
	//pix.suppress = 0 ; // commentato 29 giugno
	pix.track(I,me,false) ;// aggiunto 29 giugno
      }
      if(vpDEBUG_ENABLE(3))
	{
	  std::printf("\e[34m  vpDEBUG_ENABLE(3) sample \e[30m \n ");
	  //vpDisplay::displayCross(I,vpMath::round(i_1), vpMath::round(j_1), 5, vpColor::blue);
	}
      if ((firstIter) || (pix.suppress == 0))
	list.addRight(pix);
      //std::printf("sample IN IMAGE i %d j %d ---- norm_ang %f \n",pix.i, pix.j, norm_ang);
      //		vpDisplay::displayCross(I,vpMath::round(i_1), vpMath::round(j_1), 1, vpColor::yellow);
    }
    x_1 = x_1 + incr; 
    y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
  }
  //std::printf("2 - sampling list.nbElements() %d \n", list.nbElements());
  //std::cout << K << std::endl;
  //printf("line is %d \n",(int) line);
  /*
    if (firstIter) vpMeTracker::initTracking(I) ;
    else vpMeTracker::track(I) ; 
  */
  if (firstIter) vpMeTracker::initTracking(I) ;//should be above but exception in track (I, me true)
  //  std::cout << "----->   list.nbElements() "<<   list.nbElements() << std::endl;
}
void 
vpMePath::displayList(vpImage<unsigned char> &I)
{
  list.front();
  int k = 1;
  while(!list.outside())
    {
      vpMeSite s = list.value() ;//current reference pixel
      list.next() ;
      vpDisplay::displayCross(I, s.i, s.j, 1, vpColor::green);
      //std::printf("in displayList i %f j %f \n", s.ifloat, s.jfloat);
      k++;
    }
}
void
vpMePath::updateNormAng()
{
  vpMeSite p ;
  list.front() ;
  double norm_ang;
  for (int i=0 ; i < list.nbElement() ; i++)
    {
      p = list.value() ;
      computeNormAng(norm_ang, K,  p.ifloat, p.jfloat, line) ;
      p.alpha = norm_ang;
      list.modify(p) ;
      list.next() ;
    }
}
void
vpMePath::suppressPoints()
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
  i1 = s.ifloat;
  j1 = s.jfloat;
  list.end();
  s = list.value() ;//current reference pixel
  i2 = s.ifloat;
  j2 = s.jfloat;
}
void vpMePath::getCirclePoints() {
  
  double i_t, j_t, i_b, j_b;	//image coords of top and bottom point
  if (firstIter) {
    i_b = (double) i_ref[0];
    j_b = (double) j_ref[0];
    i_t = (double) i_ref[4];
    j_t = (double) j_ref[4];
  } else {
    i_b = i1;
    j_b = j1;
    i_t = i2;
    j_t = j2;
  }
  // sample positions
  double ct = cos(thetaFin);
  double st = sin(thetaFin);
  double x_t = i_t*ct - j_t*st; 
  double x_b = i_b*ct - j_b*st;
  double y_t = i_t*st + j_t*ct;
  //int num_samples = (int) (1.5 * sqrt (vpMath::sqr(i_t-i_b) + vpMath::sqr(j_t-j_b)));
  double incr = (double) ((x_b - x_t) / numPointCir);
  for (int m = 0; m < numPointCir; m++) {
    x_t = x_t + incr; 
    y_t = aFin*x_t*x_t + bFin*x_t + cFin;
    i_t = x_t*ct + y_t*st;
    j_t = -x_t*st + y_t*ct;
    i_cir [m] = i_t;
    j_cir [m] = j_t;
    //std::printf("vpMePath::getCirclePoints() i %d i_cir[i] %f j_cir[i] %f \n", m, i_cir [m], j_cir [m]); 
  }
}

/*!

  Compute .....

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void vpMePath::leastSquare()
{
  // Construction du systeme Ax=b
  //! i^2 + K0 j^2 + K1 i j + K2 i + K3 j + K4 = 0
  // A = (j^2 ij i j 1) x = b --> (K0 K1 K2 K3 K4)^T  b = (-i^2 )
  leastSquareLine();
  leastSquareParabola();
  vpMeSite p;
  std::printf("\e[36mline err %f list.nbElement() %d lineGoodPoints %d parab_error %f list.nbElement() %d parGoodPoints %d\e[30m\n", line_error, list.nbElement(), lineGoodPoints, parab_error, list.nbElement(), parGoodPoints);
  double lineScore, parScore;
  if (firstIter) {
    lineScore = line_error + (double) (goodPointGain * numPoints / lineGoodPoints),
      parScore = parab_error + (double) (goodPointGain * numPoints / parGoodPoints);
  } else {
    lineScore = line_error + (double) (goodPointGain * list.nbElement() / lineGoodPoints),
      parScore = parab_error + (double) (goodPointGain * list.nbElement() / parGoodPoints);
  }
  /*  
      if (sampling) {//during sampling iteration, keep last curve
      if (keepLine) {
      lineScore = 0; 
      parScore = 1000;
      } else {
      lineScore = 1000; 
      parScore = 0;
      }
      }
      sampling = false;
  */  
  if (verbose)
    std::printf("\e[36mline score %f par score %f \e[30m\n", lineScore, parScore);
  if ((lineScore < maxLineScore) || (lineScore < parScore)) {
    for (int i = 0 ; i < 5 ; i++) K[i] = K_line[i];
    line = true;
    keepLine = true;
    aFin = 0;
    //std::printf("\e[36mline paramd K_line[2] %f K_line[4] %f \e[30m\n", K_line[2], K_line[4]);
    if (fabs (K_line[4]) > 800) {//if the line is parallel to the j axis, i.e., horizontal, change reference //was 800
      thetaFin = M_PI / 2;
      bFin = 1 / K_line[2];
      cFin = - K_line[4] / K_line[2];
    } else {
      thetaFin = 0;
      bFin = -K_line[2];
      cFin = -K_line[4];
    }
    suppressPoints();
    std::printf("\e[36m leastSquare() line a %f b %f c %f theta %f \e[30m \n", aFin, bFin, cFin, thetaFin);
  } else {
    for (int i = 0 ; i < 5 ; i++) K[i] = K_par[i];
    line = false;
    keepLine = false;
    thetaFin = thetaPar;
    aFin = aPar;
    bFin = bPar;
    cFin = cPar;
    getCirclePoints();
    suppressPoints();
    std::printf("\e[36m leastSquare() parabola a %f b %f c %f theta %f \e[30m \n", aFin, bFin, cFin, thetaFin);
  }
  updateNormAng();
}

/*!

  Compute parabola parameters given point coordinates using a least
  square method.

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void 
vpMePath::leastSquareParabola()
{
  vpMeSite p ;
  int iter = 0 ;
  int pointsForLs;
  if (firstIter) pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  std::cout << "vpMePath::leastSquareParabola()  pointsForLs " << pointsForLs << std::endl;
  if (((firstIter) && (numPoints < 3)) || ((!firstIter) && (list.nbElement() < 3)))
    {
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point")) ;
    }
  vpMatrix A(pointsForLs,5) ;
  vpColVector b(pointsForLs) ;
  vpRobust r(pointsForLs) ;
  r.setIteration(0) ;
  vpMatrix D(pointsForLs,pointsForLs) ;
  D.setIdentity() ;
  vpMatrix DA, DAmemory ;
  vpColVector DAx ;
  vpColVector w(pointsForLs) ;
  w =1 ;
  while (iter < LSiter)
    {
      list.front() ;
      int k = 0;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++)
	  {
	    //  std::printf("least square parabola ref --> i %d j %d \n",i_ref[k], j_ref[k]);
	    //std::printf("least square parabola ref --> i %f j %f \n",i_ref[i], j_ref[i]);
	    A[i][0] = (double) vpMath::sqr(j_ref[i]) ;
	    A[i][1] = (double) (i_ref[i] * j_ref[i]) ;//2 removed andrea
	    A[i][2] = (double) i_ref[i] ;//2 removed andrea
	    A[i][3] = (double) j_ref[i] ;//2 removed andrea
	    A[i][4] = 1 ;
	    b[i] = - (double) vpMath::sqr(i_ref[i]) ;
	    //std::printf("i %d :: %f + %f = %f \n",i, A[i][0], A[i][1], b[i]);
	  }
      } else {
	for (int i = 0; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if ((p.suppress==0) && (k < pointsForLs))
	      {
		A[k][0] = vpMath::sqr(p.jfloat) ;
		A[k][1] = (p.ifloat * p.jfloat) ;//2 removed andrea
		A[k][2] = p.ifloat ;//2 removed andrea
		A[k][3] = p.jfloat ;//2 removed andrea
		A[k][4] = 1 ;
		b[k] = - vpMath::sqr(p.ifloat) ;
		k++ ;
	      }
	    list.next() ;
	  }
      }
      DA = D*A ;
      vpMatrix DAp ;
      K_par = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*K_par ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k =0 ;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++)
	  {
	    D[k][k] =w[k]  ;
	    k++ ;
	  }
      } else {      
	for (int i=0 ; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if (p.suppress==0) //&& (k < pointsForLs))
	      {
		D[k][k] =w[k]  ;
		k++ ;
	      }
	    list.next() ;
	  }
      }
      iter++ ;
    }
  // Loop through list of sites to track
  /*
    int j = 0;
    // Loop through list of sites to track
    for (int i=0 ; i < pointsForLs ; i++) {
    if (w[i] < good_point_thresh) {}
    else {
    j++;
    }
    }
    parGoodPointsTot = j;
    std::cout << "vpMePath::leastSquareParabola()  parGoodPointsTot " << parGoodPointsTot << std::endl;
    vpColVector err_vec(pointsForLs);
    err_vec = DAx-D*b;
    //std::cout << "errvec "<< err_vec << std::endl;
    //std::cout << " err_vec.sumSquare()  " << err_vec.sumSquare() << std::endl;
    //std::cout << " sqrt err_vec.sumSquare()  " << sqrt (err_vec.sumSquare()) << std::endl;
    //std::cout << "LSL K_par = " << K_par << "pointsForLs " << pointsForLs << std::endl;
    parab_errorTot = (double) (sqrt(err_vec.sumSquare()) / parGoodPointsTot);
    std::printf("\e[31m least square parabola --> errorTot %f \e[30m \n", parab_errorTot);
    //	std::cout << "vpMePath::leastSquareParabola()  list.nbElement() " << list.nbElement() << std::endl;
    */
  getParameters() ;
}

/*!

  Compute .....

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/

void
vpMePath::getParameters()
{
  //andrea prova parabola: a b c (j = a i^2 + b i + c) orientation thetaPar
  double aPar1, bPar1, cPar1, theta1, aPar2, bPar2, cPar2, theta2, parab_error1, parab_error2;
  int parGoodPoints1, parGoodPoints2;
  bool line1, line2;
  //teporarily save K_par values to be used in case theta1 is chosen
  double K_par_first[5];
  //	std::cout << "getparameters() starts K_par " << K_par <<  std::endl;
  //std::printf("\n\n det %f \n\n", fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]));
  if (fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]) < 0.000001) {	
    parab_error = 10000;//ensures that the line derived by LSL is chosen
    /*
      line = true;
      leastSquareLine();
      thetaPar = 0;
      aPar = 0;
      bPar = -K_line[2];
      cPar = -K_line[4];
    */
    std::printf ("\e[31m DETERMINANT CONDITION **** -> LINE \n");
  } else {
    if (fabs(K_par[0]-1) != 0) theta1 = atan (K_par[1]/(K_par[0]-1));
    else if (fabs(K_par[1]) > 0) theta1 = M_PI / 2;
    else theta1 = 0;
    //beware: if K_par[1] is it is a circle TODO 
    if (theta1 > 0) theta2 = theta1 - M_PI;	 else theta2 = M_PI + theta1;	
    theta1 = 0.5*theta1;
    theta2 = 0.5*theta2;
    //		std::cout << "The angles are " << theta1 << " " << theta2 <<  std::endl;
		
    //consider parabola with theta1
    K_par[0] = tan (theta1)*tan (theta1);
    K_par[1] = -2*tan (theta1);
    leastSquareParabolaGivenOrientation();
    parab_error1 = parab_error;
    parGoodPoints1 = parGoodPoints;
    bPar1 = (-(K_par[2]/K_par[3])*cos (theta1) + sin (theta1)) / ((K_par[2]/K_par[3])*sin 		(theta1) + cos (theta1)); 
    cPar1 = (K_par[4]*(-cos (theta1) -bPar1*sin (theta1))) / (K_par[3]);
    aPar1 = cPar1 / (K_par[4]*cos (theta1)*cos (theta1));
		
    std::printf("\n\n det1 %f \n\n", fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + 			K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]));
		
    std::printf("\n\n fabs(aPar1) %f \n\n", fabs(aPar1));
		
		
    if ((fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - 			K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]) < par_det_threshold) ||
	(fabs(aPar1) < aParThreshold)){	
      line1 = true;
      parab_error1 = 10000;
      /*
	leastSquareLine();
	theta1 = 0;
	aPar1 = 0;
	bPar1 = -K_line[2];
	cPar1 = -K_line[4];
	for (int i=0 ; i < 5 ; i++) K_par_first[i] = K_line[i] ;
      */
      std::printf ("\e[31m FIRST ****** -> LINE \e[30m\n");
    } else {
      line1 = false;
      std::printf ("\e[31m FIRST ****** -> PARABOLA a %f b %f c %f th %f ************* \e[30m \n", aPar1, bPar1, cPar1, theta1);
      for (int i=0 ; i < 5 ; i++) K_par_first[i] = K_par[i] ;
    }
		
    //consider parabola with theta2
    K_par[0] = tan (theta2)*tan (theta2);
    K_par[1] = -2*tan (theta2);
    leastSquareParabolaGivenOrientation();//todo cambia a list bla bla
    //std::cout << "second K_par   " << K_par << std::endl ;
    parab_error2 = parab_error;
    parGoodPoints2 = parGoodPoints;
    bPar2 = (-(K_par[2]/K_par[3])*cos (theta2) + sin (theta2)) / ((K_par[2]/K_par[3])*sin (theta2) + cos (theta2)); 
    cPar2 = (K_par[4]*(-cos (theta2) -bPar2*sin (theta2))) / (K_par[3]);
    aPar2 = cPar2 / (K_par[4]*cos (theta2)*cos (theta2));

    std::printf("\n\n det2 %f \n\n", fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + 			K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]));
    std::printf("\n\n fabs(aPar2) %f \n\n", fabs(aPar2));
		
    if ((fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - 			K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]) < par_det_threshold) ||
	(fabs(aPar2) < aParThreshold))  {	
      line2 = true;
      parab_error2 = 10000;
      /*
	leastSquareLine();
	theta2 = 0;
	aPar2 = 0;
	bPar2 = -K_line[2];
	cPar2 = -K_line[4];
	std::printf ("\e[31m SECOND ****** -> LINE a %f b %f c %f th %f ************* \e[30m \n", aPar2, bPar2, cPar2, theta2);
      */
      std::printf ("\e[31m SECOND ****** -> LINE  \n");
    } else {
      line2 = false;
      std::printf ("\e[31m SECOND ****** -> PARABOLA a %f b %f c %f th %f ************* \e[30m \n", aPar2, bPar2, cPar2, theta2);
    }
		
    if (line1 && line2) parab_error = 10000;//ensures that the line derived by LSL is chosen
    //compare the two parabolas
    else if ((line2 && !line1) || 
	     (!line2 && !line1 && (fabs(aPar1) < fabs(aPar2)))) {
      if (parab_error1 < parab_error2)
	parab_error = parab_error1;
      else parab_error = parab_error2;
      //was parab_error = parab_error1;
      parGoodPoints = parGoodPoints1;
      line = false;
      thetaPar = theta1;
      aPar = aPar1;
      bPar = bPar1;
      cPar = cPar1;
      for (int i=0 ; i < 5 ; i++) {
	K_par[i] = K_par_first[i];
      }	
      std::printf("\e[31m PICK FIRST ---> aPar %f , bPar %f , cPar %f , theta %f \e[30m  \n",aPar , bPar, cPar, thetaPar); 		
    } else {
      if (parab_error1 < parab_error2)
	parab_error = parab_error1;
      else parab_error = parab_error2;
      //was parab_error = parab_error2;
      parGoodPoints = parGoodPoints2;
      line = false;
      thetaPar = theta2;
      aPar = aPar2;
      bPar = bPar2;
      cPar = cPar2;
      std::printf("\e[31m PICK SECOND ---> aPar %f , bPar %f , cPar %f , theta %f \e[30m  \n",aPar , bPar, cPar, thetaPar); 	
    } 
  }
}

/*!

  Compute parabola parameters given point coordinates using a least
  square method.

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/

void vpMePath::leastSquareParabolaGivenOrientation()
{
  //std::cout << "ANDREA leastSquareParabolaGivenOrientation()...nOs " << numberOfSignal() << std::endl;
  // Construction du systeme Ax=b
  //! K2 i + K3 j + K4 = - i^2 - K0 j^2 - K1 i j 
  // (i j 1) * (K0 K1 K2)^T = b = (-i^2 -K0 j^2 -K1 i j )
  int i ;
  vpMeSite p ;
  int iter = 0 ;
  int pointsForLs;
  if (firstIter) pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  if (((firstIter) && (numPoints < 1)) || ((!firstIter) && (list.nbElement() < 1)))
    {
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point")) ;
    }
  std::cout << "vpMePath::leastSquareParabola()GivenOrient  pointsForLs " << pointsForLs << std::endl;
  vpMatrix A(pointsForLs, 3) ;
  vpColVector b(pointsForLs) ;
  vpColVector P(3) ;
  vpRobust r(pointsForLs) ;
  r.setIteration(0) ;
  vpMatrix D(pointsForLs, pointsForLs) ;
  D.setIdentity() ;
  vpMatrix DA, DAmemory ;
  vpColVector DAx ;
  vpColVector w(pointsForLs) ;
  w = 1;
  while (iter < LSiter)
    {
      list.front() ;
      int k = 0;
      if (firstIter) {
	for (i=0 ; i < pointsForLs ; i++)
	  {
	    A[i][0] = (double) i_ref[i];
	    A[i][1] = (double) j_ref[i];
	    A[i][2] = 1;
	    b[i] = - (double) (vpMath::sqr(i_ref[i]) + K_par[0]*vpMath::sqr(j_ref[i]) + K_par[1]* i_ref[i] * j_ref[i]);
	    //std::printf("i %d :: %f + %f = %f \n",i, A[i][0], A[i][1], b[i]);
	  }
      } else {
	for (i = 0; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if (p.suppress==0) //&& (k < pointsForLs))
	      {
		A[k][0] = p.ifloat;
		A[k][1] = p.jfloat;
		A[k][2] = 1 ;//2 removed andrea
		b[k] = - (vpMath::sqr(p.ifloat) + K_par[0]*vpMath::sqr(p.jfloat) + K_par[1]* p.ifloat * p.jfloat);
		k++ ;
	      }
	    list.next() ;
	  }
      }
      DA = D*A ;
      vpMatrix DAp ;
      P = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*P ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k =0 ;
      if (firstIter) {
	for (i=0 ; i < pointsForLs ; i++)
	  {
	    D[k][k] =w[k]  ;
	    k++ ;
	  }
      } else {      
	for (i=0 ; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if (p.suppress==0) //&& (k < pointsForLs))
	      {
		D[k][k] =w[k]  ;
		k++ ;
	      }
	    list.next() ;
	  }
      }
      iter++ ;
    }
  // Loop through list of sites to track
  int j = 0;
  // Loop through list of sites to track
  w_par.resize(pointsForLs) ; 
  for (int i=0 ; i < pointsForLs ; i++) {
    if (w[i] < good_point_thresh) w_par[i] = 0;
    else {
      w_par[i] = 1;
      j++;
    }
  }
  parGoodPoints = j;
  std::cout << "vpMePath::leastSquareParabola()  parGoodPoints " << parGoodPoints << std::endl;
  //    std::cout << "vpMePath::leastSquareParabolaGivenOrient()  list.nbElement() " << list.nbElement() << std::endl;
  vpColVector err_vec(pointsForLs);
  err_vec = DAx-D*b;
  //    std::cout << " err_vec.sumSquare()  " << err_vec.sumSquare() << std::endl;
  //    std::cout << " sqrt err_vec.sumSquare()  " << sqrt (err_vec.sumSquare()) << std::endl;
  parab_error = (double) (sqrt(err_vec.sumSquare()) / parGoodPoints);
  std::printf("\e[31m least square parabola --> error %f \e[30m \n", parab_error);
  for (i=0 ; i < 3 ; i++) {
    K_par[i+2] = P[i];
    //std::cout << "LSPGO K_par   " << P[i] << std::endl ;
  }
  //std::cout << "K_par   " << K_par << std::endl ;
		
}
/*!

  Compute parabola parameters given point coordinates using a least
  square method.

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void vpMePath::leastSquareLine()
{
  // Construction du systeme Ax=b
  //! K2/K3 i + K4/k3 = -j 
  vpMeSite p ;
  int iter = 0 ;
  //  std::cout << "andrea leastSquare() Parabola points: " << numPoints << std::endl ;
  int pointsForLs;
  if (firstIter) pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  //  std::cout << "----- andrea leastSquareLine() firstIter: " << firstIter << " pointsForLs " << pointsForLs << std::endl ;
  if (((firstIter) && (numPoints < 1)) || ((!firstIter) && (list.nbElement() < 1)))
    {
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point")) ;
    }
  std::cout << "vpMePath::leastSquareLine()  pointsForLs " << pointsForLs << std::endl;
  vpMatrix A(pointsForLs, 2) ;
  vpColVector b(pointsForLs) ;
  vpColVector P(2) ;
  vpRobust r(pointsForLs) ;
  r.setIteration(0) ;
  vpMatrix D(pointsForLs, pointsForLs) ;
  D.setIdentity() ;
  vpMatrix DA, DAmemory ;
  vpColVector DAx ;
  vpColVector w(pointsForLs) ;
  w =1 ;
  while (iter < LSiter)
    {
      list.front();
      int k = 0;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++)
	  {
	    //  std::printf("least square line ref --> i %d j %d \n",i_ref[k], j_ref[k]);
	    //std::printf("least square line ref --> i %f j %f \n",i_ref[k], j_ref[k]);
	    A[i][0] = (double) i_ref[i];
	    A[i][1] = 1;
	    b[i] = - (double) j_ref[i];
	    std::printf("i %d :: %f + %f = %f \n",i, A[i][0], A[i][1], b[i]);
	  }
      } else {
	for (int i = 0; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if (p.suppress==0) //&& (k < pointsForLs))
	      {
		A[k][0] = p.ifloat ;
		A[k][1] = 1 ;
		b[k] = - p.jfloat;
		k++ ;
		//				std::printf("least square line  --> i %f j %f \n", p.ifloat, p.jfloat);
	
				
	      }
	    list.next() ;
	  }
      }
      //	std::cout << "vpMePath::leastSquareLine()  points not suppressed " << k << std::endl;
      DA = D*A ;
      vpMatrix DAp ;
      P = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*P ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k = 0 ;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++)
	  {
	    D[k][k] =w[k]  ;
	    k++ ;
	  }
      } else {      
	for (int i=0 ; i < list.nbElement() ; i++)
	  {
	    p = list.value() ;
	    if (p.suppress==0) 
	      {
		D[k][k] =w[k]  ;
		k++ ;
	      }
	    list.next() ;
	    //std::printf("i %d --> %d %d w %f \n", i, p.i, p.j, w[i]);
	  }
      }
      //	std::cout << "2 vpMePath::leastSquareLine()  points not suppressed " << k << std::endl;
      iter++ ;
    }
  int j = 0;
  // Loop through list of sites to track
  w_line.resize(pointsForLs) ; 
  for (int i=0 ; i < pointsForLs ; i++) {
    if (w[i] < good_point_thresh) w_line[i] = 0;
    else {
      w_line[i] = 1;
      j++;
    }
  }
  lineGoodPoints = j;
  //    suppressPoints();
  std::cout << "vpMePath::leastSquareLine()  lineGoodPoints " << lineGoodPoints << std::endl;
  //  std::cout << "vpMePath::leastSquareLine()  lineGoodPoints for w " << i << std::endl;
  //  std::cout << "vpMePath::leastSquareline()  list.nbElement() " << list.nbElement() << std::endl;
  vpColVector err_vec(pointsForLs);
  err_vec = DAx-D*b;
  //    std::cout << "errvec "<< err_vec << std::endl;
  for (int i=0 ; i < pointsForLs ; i++) {
    err_vec [i] = err_vec [i]*err_vec [i];//prima non c'era
  }
  //    std::cout << " err_vec.sumSquare()  " << err_vec.sumSquare() << std::endl;
  //    std::cout << " sqrt err_vec.sumSquare()  " << sqrt (err_vec.sumSquare()) << std::endl;
  line_error = (double) (sqrt(err_vec.sumSquare()) / lineGoodPoints);
  std::printf("\e[31m least square line error %f \e[30m \n", line_error);
  K_line[0] = 0;
  K_line[1] = 0;
  K_line[2] = P[0];
  K_line[3] = 1;
  K_line[4] = P[1];
  //std::cout << "K_line " << K_line << std::endl;
  std::printf ("\e[31m LEAST SQUARE LINE a 0 b %f c %f th 0 ***************** \e[30m \n", -K_line[2], -K_line[4]);
}

/*!

  Initialise the tracking of...
*/
void
vpMePath::initTracking(vpImage<unsigned char> &I)
{
  unsigned *i, *j ;
  i = new unsigned[numPoints] ;
  j = new unsigned[numPoints] ;
  firstIter = true;
  //andrea prova parabola ha commentato
  for (int k =0 ; k < numPoints; k++)
    {
      std::cout << "Please do click on the points "<< k+1 <<"/" << numPoints;
      std::cout << " on the parabola in the trigonometric order" <<std::endl ;
      std::cout << "--------  "<< std::endl;
      while (vpDisplay::getClick(I,i[k],j[k])!=true) ;
      std::cout << i[k] <<" " <<j[k] << std::endl;
    }

  i_ref = new unsigned[numPoints] ;
  j_ref = new unsigned[numPoints] ;
  for (int k =0 ; k < numPoints ; k++) {
    i_ref [k] = i[k];
    j_ref [k] = j[k];
    std::printf("init tracking ref --> i %d j %d \n",i_ref[k], j_ref[k]);
  }
  i1 = i[0] ;
  j1 = j[0] ;
  i2 = i[numPoints-1] ;
  j2 = j[numPoints-1] ;
  //std::cout << "init tracking andrea 2 "<< std::endl;
  initTracking(I, numPoints, i, j) ;
  delete []i ;
  delete []j ;
}

/*!

  Initialise the tracking of  .....

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/

void
vpMePath::initTracking(vpImage<unsigned char> &I, int n,
		       unsigned *i, unsigned *j)
{
  vpCDEBUG(1) <<" begin vpMePath::initTracking()"<<std::endl ;
  std::cout << "init tracking (I,n,i,j) "<< std::endl;
  leastSquare();
  std::cout << "initTracking::parameters computed click to continue..." << std::endl;
  //vpDisplay::getClick(I) ;
  sample(I);
  display(I, vpColor::green);
  firstIter = false;
  //std::cout << "initTracking::sampling done click to continue..." << std::endl;
  vpDisplay::getClick(I) ;
  //  2. On appelle ce qui n'est pas specifique
  {
    vpMeTracker::initTracking(I) ;
  }
  try{
    //std::cout << "ANDREA init tracking before track "<< std::endl;
    track(I) ;
  }
  catch(...)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  vpMeTracker::display(I) ;
  vpDisplay::flush(I);
  display(I, vpColor::green);
  std::printf("init tracking finished \n");
}

/*!

  Track ....  

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void
vpMePath::track(vpImage<unsigned char> &I)
{
  static int iter = 0 ;
  {
  }
  vpDisplay::display(I) ;
  {
    try{
      vpMeTracker::track(I) ;
    }
    catch(...)
      {
  	vpERROR_TRACE("Error caught") ;
    	throw ;
      }
  }
  {
    suppressPoints() ;
    try{
      std::cout << " in track() before LEASTSQUARE " << std::endl;
      leastSquare() ;  }
    catch(...)
      {
	vpERROR_TRACE("Error caught") ;
	throw ;
      }
    display(I, vpColor::green) ; 
    if ((iter % sampleIter == 0) && (iter != 0)) {
      if ((!line) || (thetaFin != M_PI / 2)) { //avoids bug on horizontal lines
	if (line) {	
	  sample(I);
	  if (firstIter) {
	    try{
	      vpMeTracker::track(I) ;
	    }
	    catch(...)
	      {
		vpERROR_TRACE("Error caught") ;
		throw ;
	      }
	  }
	  //std::printf("before suppress list.nbElements() %d \n", list.nbElements());
	  suppressPoints();
	  //std::printf("after suppress list.nbElements() %d \n", list.nbElements());
	  displayList(I);
	  leastSquare(); //12 giugno
	  //if (!line) vpDisplay::getClick(I);
	}
      }
    }
    for (int k =0 ; k < seekLoops ; k++) 
      {
	if (verbose)
	  std::printf ("\e[34m  iteration %d k %d \e[30m  \n ", iter, k);
  	if (!firstIter) seekExtremities(I);
        try{
	  vpMeTracker::track(I) ;
        }
        catch(...)
	  {
	    vpERROR_TRACE("Error caught") ;
	    throw ;
	  }
      	suppressPoints() ;
      	try{
	  leastSquare() ;  }
      	catch(...)
	  {
	    vpERROR_TRACE("Error caught") ;
	    throw ;
	  }
      	display(I, vpColor::green) ; 
      	//vpDisplay::getClick(I) ;
      	vpDisplay::flush(I);
      }
    if (verbose)
      std::printf("\e[34m IMAGE %d --------> done \e[30m \n\n", iter);
  }
  iter++ ;
}
