/****************************************************************************
 *
 * $Id: vpMePath.cpp,v 1.9 2007-08-22 14:00:13 acherubi Exp $
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

/*
  Compute normal angle to the curve at a given point
*/
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
  norm_ang = atan2(A,B) ; 
  //normalize angle on ]-M_PI, M_PI]
  
  while (norm_ang > M_PI) { norm_ang -= M_PI ; }
  while (norm_ang < 0) { norm_ang += M_PI ; }

//  std::printf("K[2] %f  K[3] %f computeNormAng %f\n", A, B, norm_ang);
 
 // while (norm_ang > M_PI) { norm_ang -= 2*M_PI ; }
 // while (norm_ang < -M_PI) { norm_ang += 2*M_PI ; }
 
  //std::printf("after %f\n", norm_ang);

}
/*!
  
  Constructor

*/
vpMePath::vpMePath():vpMeTracker()
{
  std::printf ("\e[32m 9 july 2007 CO \e[30m \n");
  setVerboseMode(false);
  if (verbose)
  	std::printf ("\e[32m vpMePath::vpMePath() 10 july 2007 \e[30m \n");

  // conic parameters
  // i^2 + K0 j^2 + K1 i j + K2 i + K3 j + K4 = 0
  K.resize(5) ;
  K_line.resize(5) ; 
  K_par.resize(5) ;
  j1 = j2 = i1 = i2 = 0 ;
  
  // parameters
  firstIter = true;//first iteration
  horLine = false;//initialize to false
  numPoints = 5; //initial points used to find parabola
  n_points = numPoints; //points used for tracking at every iteration 
  numPointPar = 100; //parabola points used to find circle
  LSiter = 2; //least square iterations 
  good_point_thresh = 0.75; //threshold on least square line error
  sampleIter = 5; //every sampleIter iterations sample the curve
  pointPercentageWithinExtremes = 1.0; //percent of samples within extremities
  seekLoops = 5; //number of times extremities are seeked at each iteration
  numExtr = 8; //number of points seeked after each extremity
  goodPointGain = 115; //gain for considering good points when selecing curve
  maxLineScore = 195; //max error tolerated on line before parabola selection  
  par_det_threshold = 0.00006; //parabola det threshold for selecting a line
  aParThreshold	= 0.0009; //aPar threshold for selecting a line
  
  i_par = new double[numPointPar] ;
  j_par = new double[numPointPar] ;
  i_ref = new unsigned[numPoints] ;
  j_ref = new unsigned[numPoints] ;
  for (int i = 0 ; i < numPointPar ; i++) {
    i_par [i] = -1000;
    j_par [i] = -1000; 
  }
}
/*!
  
  Destructor

*/
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
/*!
  
  Display some curve features

*/
void vpMePath::display(vpImage<unsigned char> &I, vpColor::vpColorType col)
{
  suppressPoints() ;
  //parabola vertex coordinates
  double x_v = -bFin / (2*aFin);
  double y_v = -bFin*bFin / (4*aFin) + cFin;
  double ct = cos(thetaFin);
  double st = sin(thetaFin);
  //display parabola vertex
  if (!line) vpDisplay::displayCross(I,(int)(x_v*ct + y_v*st),
				     (int)(-x_v*st + y_v*ct),20,vpColor::cyan);
  //display segment oriented along parabola symetric axis
  vpDisplay::displayLine(I, (int) i_ref [2], (int) j_ref [2],
			 (int)(i_ref [2] + 70*sin (thetaFin)), 
			 (int)(j_ref [2] + 70*cos (thetaFin)), 
			 vpColor::blue, 1) ;
  //display initial points
  //if (firstIter)
  //  for (int k = 0; k < numPoints; k++)
  //    vpDisplay::displayCross(I, (int) i_ref[k], (int) j_ref[k],10,
  //			      vpColor::green);
  
  //image coords of top and bottom point
  double i_1;
  double j_1;
  double i_2;
  double j_2;
  double i_1o;
  double j_1o; 
  if (firstIter) {
    i_1 = (double) i_ref[0];
    j_1 = (double) j_ref[0];
    i_2 = (double) i_ref[4];
    j_2 = (double) j_ref[4];
  } else {
    vpMeSite p = list.value() ;
    list.front();//initial curve pixel
    p = list.value() ;
    i_1 = (double) p.i;
    j_1 = (double) p.j;
    list.end();//final curve pixel
    p = list.value() ;
    i_2 = (double) p.i;
    j_2 = (double) p.j;
  }
  //display initial and final curve pixels
  vpDisplay::displayCross(I, (int) i1, (int) j1, 20, vpColor::red);	
  vpDisplay::displayCross(I, (int) i2, (int) j2, 20, vpColor::yellow);
  list.front();
  //display all points in the list
  double i_dis;
  double j_dis;
  while(!list.outside())
    {
      vpMeSite s = list.value() ;//current reference pixel
      list.next() ;
      i_dis = s.i ;
      j_dis = s.j ;
      vpDisplay::displayCross(I, (int) i_dis, (int) j_dis, 1, vpColor::red);
    }
  //convert coordinates to rotated reference frame
  double x_2 = i_2*ct - j_2*st; 
  double x_1 = i_1*ct - j_1*st;
  double y_2 = i_2*st + j_2*ct; 
  double y_1 = i_1*st + j_1*ct;
  int disp_samples = (int) (0.5 * sqrt (vpMath::sqr(i_2-i_1) + 
  				vpMath::sqr(j_2-j_1)));
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
    //display the curve (line or parabola)
    //vpDisplay::displayLine(I, (int) i_1o, (int) j_1o, (int) i_1, 
    	//		(int) j_1, col, 1) ;
  }
}
/*
  Search curve extra points outside extremities 
*/
void vpMePath::seekExtremities(vpImage<unsigned char>  &I)
{
  unsigned height = I.getHeight() ;
  unsigned width = I.getWidth() ;
  vpMeSite p1;
  vpMeSite p2;
  suppressPoints() ;
  list.front();
  p1 = list.value() ;//initial curve pixel
  i1 = p1.ifloat;
  j1 = p1.jfloat;
  list.end();
  p2 = list.value();//final curve pixel
  i2 = p2.ifloat;
  j2 = p2.jfloat;
  if (verbose)
    std::printf ("\e[32m 2-seekExtremities i1 %f j1 %f i2 %f j2 %f \e[30m \n",
		 i1, j1, i2, j2);
  // sample positions
  double ct = cos(thetaFin);
  double st = sin(thetaFin);
  //convert coordinates to rotated reference frame
  double x_2 = i2*ct - j2*st;
  double x_1 = i1*ct - j1*st;
  double y_2 = i2*st + j2*ct; 
  double y_1 = i1*st + j1*ct;
  int samples = (int) (0.3 * sqrt (vpMath::sqr(i2-i1) + vpMath::sqr(j2-j1)));
  double incr = (double) ((x_2 - x_1) / samples);
  if (line) {
    //project first and last points abscissa to estimated line 
    x_2 = (-x_2/bFin - y_2 + cFin) / (-bFin - 1/bFin);
    x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  } else {
    //TODO project first and last points abscissa to estimated parabola
  }
  double norm_ang ;
  vpMeSite pix ;
  double i_1;
  double j_1;
  double i_2;
  double j_2;
  //move outside curve extremities to track and add extra points
  for (int m = 0; m < numExtr; m++) {
    x_1 = x_1 - incr; 
    y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
    i_1 = x_1*ct + y_1*st;
    j_1 = -x_1*st + y_1*ct;
    // If point is in the image, track it
    if(!outOfImage(vpMath::round(i_1), vpMath::round(j_1), 0, height, width))
      {
	pix = p1;
	computeNormAng(norm_ang, K,  i_1,  j_1, line)  ;
	pix.init(i_1, j_1, norm_ang) ;
	pix.convlt = p1.convlt;
	//vpDisplay::displayCross(I, (unsigned)i_1,  (unsigned)j_1,  
	//15, vpColor::blue) ;
	pix.setDisplay(NONE) ;//was selectdisplay
	pix.suppress = 0;
	pix.track(I,me,false) ;
	//if point is tracked, add it to the list
	if (pix.suppress == 0)
	  {
	    vpDisplay::displayCross(I, (unsigned)pix.i,  (unsigned)pix.j,  
	    				2, vpColor::black) ;
	    list.front();
	    list.addLeft(pix);
	  }
      }
    x_2 = x_2 + incr; 
    y_2 = aFin*x_2*x_2 + bFin*x_2 + cFin;
    i_2 = x_2*ct + y_2*st;
    j_2 = -x_2*st + y_2*ct;
    // If point is in the image, track it
    if(!outOfImage(vpMath::round(i_2), vpMath::round(j_2), 0, height, width))
      {
	pix = p2;
	computeNormAng(norm_ang, K,  i_2,  j_2, line)  ;
	pix.init(i_2, j_2, norm_ang) ;
	pix.convlt = p2.convlt;
	//vpDisplay::displayCross(I, (unsigned)i_2,  (unsigned)j_2,  
	//15, vpColor::blue) ;
	pix.setDisplay(NONE) ;//was selectdisplay
	pix.suppress = 0;
	pix.track(I,me,false) ;
	//if point is tracked, add it to the list
	if (pix.suppress == 0)
	  {
	    vpDisplay::displayCross(I, (unsigned)pix.i,  (unsigned)pix.j,  
	    				2, vpColor::black) ;
	    list.end();
	    list.addRight(pix);
	  }
      }
  }
  suppressPoints() ;
}
/*!

  Sample the curve     
  \exception vpMeTracker::initTracking : cannot track points

*/
void vpMePath::sample(vpImage<unsigned char> & I)
{
  unsigned height = I.getHeight() ;
  unsigned width = I.getWidth() ;
  double incr ; 
  double i_1; //image coordinates of first and last point
  double j_1;
  double i_2;
  double j_2;	
  int num_samples;//number of sample points
  double conv [1000];//previous point convolutions: used to intialize new points
  int j = 0;
  if (firstIter) {
    i_1 = (double) i_ref[0];
    j_1 = (double) j_ref[0];
    i_2 = (double) i_ref[4];
    j_2 = (double) j_ref[4];
    num_samples = (int) (0.7 * sqrt (vpMath::sqr(i_2-i_1) + 
    					vpMath::sqr(j_2-j_1)));//was 2.0 instead of 0.7
    //std::printf("firstIter samples %d\n",num_samples);
    //150;//					
  } else {
    i_1 = (double) i1;
    j_1 = (double) j1;
    i_2 = (double) i2;
    j_2 = (double) j2;
    list.front();
    // Loop through list of sites and store convolutions before deleting list 
    while(!list.outside())
    {
	vpMeSite s = list.value();
	list.next();
	conv[j] = s.convlt;
	j++;
    }
    num_samples = j;
    j = 0;
    list.kill();
  }
  //vpDisplay::getClick(I) ;
  double ct = cos(thetaFin);
  double st = sin(thetaFin);
  //convert coordinates to rotated frame
  double x_2 = i_2*ct - j_2*st; 
  double x_1 = i_1*ct - j_1*st;
  double y_1 = i_1*st + j_1*ct;
  if (line) {
    //project first point abscissa to estimated line 
    x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  } else {
    //TODO project first point abscissa to estimated parabola
    //x_1 = (-x_1/bFin - y_1 + cFin) / (-bFin - 1/bFin);
  }
  incr = (double) ((x_2 - x_1) / (pointPercentageWithinExtremes *num_samples));
  x_1 = (double) (x_1-((1-pointPercentageWithinExtremes)/2)*num_samples*incr); 
  y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
  //sample curve
  for (int m = 0; m < num_samples; m++) {
    i_1 = x_1*ct + y_1*st;
    j_1 = -x_1*st + y_1*ct;
    double norm_ang ;
    // If point is in the image, and it is not the first iteration, track it
    if(!outOfImage(vpMath::round(i_1), vpMath::round(j_1), 0, height, width)) {
      vpMeSite pix ;
      computeNormAng(norm_ang, K,  i_1, j_1, line);
      pix.init(i_1, j_1, norm_ang) ;
      if (!firstIter) {
	pix.convlt = conv[j];//intialise point convolution
	j++;
	pix.suppress = 0;
	//vpDisplay::displayCross(I, (unsigned)i_1,  (unsigned)j_1,  
	//10, vpColor::yellow) ;
	//pix.setDisplay(NONE) ;//was selectdisplay 
	//pix.suppress = 0 ; 
	pix.track(I,me,false);
      }
      //if the point is tracked, add it to the list
      if ((firstIter) || (pix.suppress == 0))
	list.addRight(pix);
      	//vpDisplay::displayCross(I,vpMath::round(i_1), vpMath::round(j_1), 
	//1, vpColor::yellow);
    }
    x_1 = x_1 + incr; 
    y_1 = aFin*x_1*x_1 + bFin*x_1 + cFin;
  }
  //if it is the first iteration track points 
  if (firstIter) vpMeTracker::initTracking(I) ;
}
/*
  Display list points
*/
void vpMePath::displayList(vpImage<unsigned char> &I)
{
  list.front();
  while(!list.outside())
    {
      vpMeSite s = list.value() ;//current reference pixel
      vpDisplay::displayCross(I, s.i, s.j, 1, vpColor::red);
      s.setDisplay(RESULT);
      list.next();
    }
}
void vpMePath::reduceList(vpList<vpMeSite> &list, int newSize) {
  std::printf("5 list.nbElement() %d newSize %d\n", list.nbElement(), newSize);
  int ratio = (int) (list.nbElement() / newSize);
  vpList<vpMeSite> tempList ;
  list.front();
  tempList.front();
  std::printf("6 ratio %d\n", ratio);
  while(!list.outside()) {
	tempList.value() = list.value();
	tempList.next() ;
	for (int i=0 ; i <= ratio ; i++) {
  		list.next() ;
  	}
  }
  std::printf("7\n");
  list.kill();
  list.front();
  tempList.front();
  while(!list.outside()) {
  	list.value() = tempList.value();
	tempList.next() ;
	list.next() ;
  }
  std::printf("8\n");
}

/*
  Compute normal angle for each point of the curve
*/
void vpMePath::updateNormAng()
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
/*
  Suppress list points which are not tracked and update curve extremities
*/
void vpMePath::suppressPoints()
{
  list.front();
  while(!list.outside())
    {
      vpMeSite s = list.value() ;
      if (s.suppress != 0)
	list.suppress() ;
      else
	list.next() ;
    }
  list.front();
  vpMeSite s = list.value() ;
  //first curve extremity pixel image coordinates
  i1 = s.ifloat;
  j1 = s.jfloat;
  list.end();
  s = list.value() ;
  //last curve extremity pixel image coordinates
  i2 = s.ifloat;
  j2 = s.jfloat;
}
/*
  Sample parabola between extremities and store numPointPar image point 
  coordinates in i_par, j_par
*/
void vpMePath::getParabolaPoints() {
  //image coords of top and bottom point
  double i_t;
  double j_t;
  double i_b;
  double j_b;	
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
  double ct = cos(thetaFin);
  double st = sin(thetaFin);
  //convert to rotated reference frame coordinates
  double x_t = i_t*ct - j_t*st; 
  double x_b = i_b*ct - j_b*st;
  double y_t = i_t*st + j_t*ct;
  double incr = (double) ((x_b - x_t) / numPointPar);
  for (int m = 0; m < numPointPar; m++) {
    x_t = x_t + incr; 
    y_t = aFin*x_t*x_t + bFin*x_t + cFin;
    i_t = x_t*ct + y_t*st;
    j_t = -x_t*st + y_t*ct;
    i_par [m] = i_t;
    j_par [m] = j_t;
  }
}
/*!

  Compute line and parabola parameters given the tracked point list by using:
  leastSquareLine() and leastSquareParabola()
  Select the most likely among line and parabola and stores its parameters
  
  \exception vpTrackingException::notEnoughPointError : Not enough point to 
  compute the parameters.

*/
void vpMePath::leastSquare (vpImage<unsigned char> &I)
{
/*  std::printf("1 list.nbElement() %d\n", list.nbElement()); 
  if (!firstIter) {
  	std::printf("2\n"); 
	if (list.nbElement() > 100) {
  		std::printf("3\n");
		reduceList(list, 100);//22agosto
		std::printf("9\n");
	}
  }
  */
  leastSquareLine(I);
  leastSquareParabola(I);
  vpMeSite p;
//  std::printf("leastSquare() numPts %d \n", list.nbElement());

//  std::printf("numPts %d lineGoodPts %d parGoodPts %d\n", 
//	list.nbElement(), lineGoodPoints, parGoodPoints);
	
  if (verbose)
    std::printf("numPts %d l_err %f lineGoodPts %d p_err %f parGoodPts %d\n", 
	list.nbElement(), line_error, lineGoodPoints, parab_error, 
	parGoodPoints);
  //scores used to pick the most likely curve
  double lineScore;
  double parScore;
  int nPoints;
  if (firstIter)
	nPoints = numPoints;
  else 
	nPoints = list.nbElement();
  if ((lineGoodPoints == 0) && (parGoodPoints == 0)) {
      vpDisplay::flush(I);
      static char s[30];
      sprintf(s,"/tmp/acherubi/NoPointImage.pgm");
      vpImageIo::writePGM(I,s);
      vpERROR_TRACE("Not enough line and par good points") ;
      std::printf("vpMePath::Exception Not enough line and par good points\n");
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, 
      				"Not enough line and par good points")) ;
  }
  if (lineGoodPoints != 0)
  	lineScore = line_error+(double)(goodPointGain * nPoints / lineGoodPoints);
  else 
	lineScore = 10000; 
  if (parGoodPoints != 0)
  	parScore = parab_error+(double)(goodPointGain * nPoints / parGoodPoints);
  else parScore = 10000;

  if (verbose)
    std::printf("\e[36mline score %f par score %f \e[30m\n", 
    	lineScore, parScore);
  //pick the line as most likely curve
  if ((lineScore < maxLineScore) || (lineScore < parScore)) {
    n_points = lineGoodPoints;
    for (int i = 0 ; i < 5 ; i++) {
    	K[i] = K_line[i];
    }
    line = true;
    aFin = 0;
    if (fabs (K_line[4]) > 800) {
      //if line is parallel to the j axis (i.e., horizontal) change reference
      //std::printf("\n\nHORIZONTAL\n\n");
      horLine = true;
      thetaFin = M_PI / 2;
      bFin = 1 / K_line[2];
      cFin = - K_line[4] / K_line[2];
    } else {
      horLine = false;
      thetaFin = 0;
      bFin = -K_line[2];
      cFin = -K_line[4];
    }
    suppressPoints();
    if (verbose)
      std::printf("\e[36mleastSquare()line a %f b %f c %f theta %f \e[30m \n",
       aFin, bFin, cFin, thetaFin);
  } else {
    n_points = parGoodPoints;
    //pick the parabola as most likely curve
    for (int i = 0 ; i < 5 ; i++) {
    	K[i] = K_par[i];
    }
    line = false;
    thetaFin = thetaPar;
    aFin = aPar;
    bFin = bPar;
    cFin = cPar;
    getParabolaPoints();
    suppressPoints();
    if (verbose)
      std::printf("\e[36mleastSquare()parabola a %f b %f c %f th %f \e[30m \n",
       aFin, bFin, cFin, thetaFin);
  }
  updateNormAng();
}
/*!

  Compute the five parabola parameters:
  K0 j^2 + K1 ij + K2 i + K3 j + K4 = -i^2  
  given tracked point coordinates using a least square method.

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void 
vpMePath::leastSquareParabola(vpImage<unsigned char> &I)
{
  vpMeSite p ;
  int iter = 0 ;
  int pointsForLs;
  if (firstIter) 
  	pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  if (((firstIter) && (numPoints < 3)) || 
  	((!firstIter) && (list.nbElement() < 3))) {
      vpDisplay::flush(I);
      static char s[30];
      sprintf(s,"/tmp/acherubi/NoPointImage.pgm");
      vpImageIo::writePGM(I,s);
				
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, 
      				"not enough point")) ;
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
  
  list.front() ;
  int k = 0;
  if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++) {
	    A[i][0] = (double) vpMath::sqr(j_ref[i]) ;
	    A[i][1] = (double) (i_ref[i] * j_ref[i]) ;
	    A[i][2] = (double) i_ref[i] ;
	    A[i][3] = (double) j_ref[i] ;
	    A[i][4] = 1 ;
	    b[i] = - (double) vpMath::sqr(i_ref[i]) ;
	}
  } else {
	for (int i = 0; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if ((p.suppress==0) && (k < pointsForLs)){
		A[k][0] = vpMath::sqr(p.jfloat) ;
		A[k][1] = (p.ifloat * p.jfloat) ;
		A[k][2] = p.ifloat ;
		A[k][3] = p.jfloat ;
		A[k][4] = 1 ;
		b[k] = - vpMath::sqr(p.ifloat) ;
		k++ ;
	    }
	    list.next() ;
	}
  }
  
  while (iter < LSiter)
  {
      DA = D*A ;
      vpMatrix DAp ;
      K_par = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*K_par ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k =0 ;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++) {
	    D[k][k] =w[k]  ;
	    k++ ;
	}
      } else {      
	for (int i=0 ; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if (p.suppress==0) 
	      {
		D[k][k] =w[k]  ;
		k++ ;
	      }
	    list.next() ;
	}
      }
      iter++ ;
  }
  //given the five parabola parameters, find parabola equation of the form
  //y = a x^2 + b x + c in a reference frame rotated by theta
  getParameters(I) ;
}

/*!

  Compute given the five parabola parameters, a parabola equation of the form
  y = a x^2 + b x + c 
  in a reference frame rotated by theta

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/

void
vpMePath::getParameters(vpImage<unsigned char> &I)
{
  double aPar1;
  double bPar1;
  double cPar1;
  double theta1;
  double aPar2;
  double bPar2;
  double cPar2;
  double theta2;
  double parab_error1;
  double parab_error2;
  int parGoodPoints1;
  int parGoodPoints2;
  bool line1;
  bool line2;
  double K_par_first[5];
  //check condition on determinant that makes the parabola degnerate into line
  if (fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3]
        - K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]) < 0.000001) {	
    parab_error = 10000;//ensures that the line is chosen
    if (verbose)
    	std::printf ("\e[31m DETERMINANT CONDITION **** -> LINE \n");
  } else {
    //compute given K0 and K1 two possible values for theta
    if (fabs(K_par[0]-1) != 0) theta1 = atan (K_par[1]/(K_par[0]-1));
    else if (fabs(K_par[1]) > 0) theta1 = M_PI / 2;
    else theta1 = 0;
    if (theta1 > 0) theta2 = theta1 - M_PI;	 else theta2 = M_PI + theta1;	
    theta1 = 0.5*theta1;
    theta2 = 0.5*theta2;
    //if (verbose)
    //std::printf("The angles are %f and %f \n", theta1, theta2);
    
    //consider parabola with theta1
    K_par[0] = tan (theta1)*tan (theta1);
    K_par[1] = -2*tan (theta1);
    //compute K2 K3 K4 again having fixed theta = theta1
    leastSquareParabolaGivenOrientation(I);
    parab_error1 = parab_error;
    parGoodPoints1 = parGoodPoints;
    bPar1 = (-(K_par[2]/K_par[3])*cos (theta1) + sin (theta1)) / 
    	((K_par[2]/K_par[3])*sin(theta1) + cos (theta1)); 
    cPar1 = (K_par[4]*(-cos (theta1) -bPar1*sin (theta1))) / (K_par[3]);
    aPar1 = cPar1 / (K_par[4]*cos (theta1)*cos (theta1));
    if (verbose) {		
      std::printf("\n\n det1 %f \n\n", fabs(4*K_par[0]*K_par[4] - 
        K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - 
	K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]));
      std::printf("\n\n fabs(aPar1) %f \n\n", fabs(aPar1));
    }	
    //check condition on determinant that makes the parabola degnerate into line
    //and value of a in a x^2 + bx + c
    if ((fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + 
    	K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - 
	K_par[0]*K_par[2]*K_par[2]) < par_det_threshold) ||
	(fabs(aPar1) < aParThreshold)){	
      line1 = true;
      parab_error1 = 10000;//ensures that the line is chosen
      if (verbose) 
      	std::printf ("\e[31m FIRST ****** -> LINE \e[30m\n");
    } else {
      line1 = false;
      if (verbose)
        std::printf ("\e[31m FIRST -> PAR a %f b %f c %f th %f \e[30m \n", 
			aPar1, bPar1, cPar1, theta1);
      //teporarily save K_par values to be used in case theta1 is chosen
      for (int i=0 ; i < 5 ; i++) {
      	K_par_first[i] = K_par[i] ;
      }
    }
		
    //consider parabola with theta2
    K_par[0] = tan (theta2)*tan (theta2);
    K_par[1] = -2*tan (theta2);
    //compute K2 K3 K4 again having fixed theta = theta1
    leastSquareParabolaGivenOrientation(I);//todo cambia a list bla bla
    parab_error2 = parab_error;
    parGoodPoints2 = parGoodPoints;
    bPar2 = (-(K_par[2]/K_par[3])*cos (theta2) + sin (theta2)) / 
    	((K_par[2]/K_par[3])*sin (theta2) + cos (theta2)); 
    cPar2 = (K_par[4]*(-cos (theta2) -bPar2*sin (theta2))) / (K_par[3]);
    aPar2 = cPar2 / (K_par[4]*cos (theta2)*cos (theta2));
    if (verbose) {
      std::printf("\n\n det2 %f \n\n", fabs(4*K_par[0]*K_par[4] - 
      	K_par[3]*K_par[3] + K_par[1]*K_par[2]*K_par[3] - 
	K_par[1]*K_par[1]*K_par[4] - K_par[0]*K_par[2]*K_par[2]));
      std::printf("\n\n fabs(aPar2) %f \n\n", fabs(aPar2));
    }
    //check condition on determinant that makes the parabola degnerate into line
    //and value of a in a x^2 + bx + c
    if ((fabs(4*K_par[0]*K_par[4] - K_par[3]*K_par[3] + 
    	K_par[1]*K_par[2]*K_par[3] - K_par[1]*K_par[1]*K_par[4] - 
	K_par[0]*K_par[2]*K_par[2]) < par_det_threshold) ||
	(fabs(aPar2) < aParThreshold))  {	
      line2 = true;
      parab_error2 = 10000;//ensures that the line is chosen
      if (verbose)
	std::printf ("\e[31m SECOND ****** -> LINE  \n");
    } else {
      line2 = false;
      if (verbose)
      	std::printf ("\e[31m SECOND -> PAR a %f b %f c %f th %f \e[30m \n", 
			aPar2, bPar2, cPar2, theta2);
    }
		
    if (line1 && line2) parab_error = 10000;//ensures that the line is chosen
    //compare the two curves and pick the likeliest
    else if ((line2 && !line1) || (!line2 && !line1 && 
    		(fabs(aPar1) < fabs(aPar2)))) {
      if (parab_error1 < parab_error2)
	parab_error = parab_error1;
      else parab_error = parab_error2;
      parGoodPoints = parGoodPoints1;
      line = false;
      thetaPar = theta1;
      aPar = aPar1;
      bPar = bPar1;
      cPar = cPar1;
      for (int i=0 ; i < 5 ; i++) {
	K_par[i] = K_par_first[i];
      }	
      if (verbose)	
      	std::printf("\e[31mPICK 1ST->aPar %f bPar %f cPar %f theta %f\e[30m\n",
      			aPar , bPar, cPar, thetaPar); 		
    } else {
      if (parab_error1 < parab_error2)
	parab_error = parab_error1;
      else parab_error = parab_error2;
      parGoodPoints = parGoodPoints2;
      line = false;
      thetaPar = theta2;
      aPar = aPar2;
      bPar = bPar2;
      cPar = cPar2;
      if (verbose)
         std::printf("\e[31mPICK 2ND->aPar %f bPar %f cPar %f theta %f\e[30m\n",
      			aPar , bPar, cPar, thetaPar); 	
    } 
  }
}

/*!

  Compute parabola parameters in the reference frame rotated by theta as:
  	y = a x^2 + bx + c
  given point coordinates and K0, K1 using a least square method on:
	K2 i + K3 j + K4 = - i^2 - K0 j^2 - K1 i j   
  
  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void vpMePath::leastSquareParabolaGivenOrientation(vpImage<unsigned char> &I)
{
  int i ;
  vpMeSite p ;
  int iter = 0 ;
  int pointsForLs;
  if (firstIter) pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  if (((firstIter) && (numPoints < 1)) || 
  	((!firstIter) && (list.nbElement() < 1))) {
      vpDisplay::flush(I);
      static char s[30];
      sprintf(s,"/tmp/acherubi/NoPointImage.pgm");
      vpImageIo::writePGM(I,s);
	
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, 
      		"not enough point")) ;
  }
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
  
  list.front() ;
  int k = 0;
  if (firstIter) {
	for (i=0 ; i < pointsForLs ; i++) {
	    A[i][0] = (double) i_ref[i];
	    A[i][1] = (double) j_ref[i];
	    A[i][2] = 1;
	    b[i] = - (double) (vpMath::sqr(i_ref[i]) + 
	    	K_par[0]*vpMath::sqr(j_ref[i]) + K_par[1]* i_ref[i] * j_ref[i]);
	}
  } else {
	for (i = 0; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if (p.suppress==0) {
		A[k][0] = p.ifloat;
		A[k][1] = p.jfloat;
		A[k][2] = 1 ;
		b[k] = -(vpMath::sqr(p.ifloat)+K_par[0]*vpMath::sqr(p.jfloat) + 
			K_par[1]* p.ifloat * p.jfloat);
		k++ ;
	    }
	    list.next() ;
	}
  }
  
  while (iter < LSiter) {
      DA = D*A ;
      vpMatrix DAp ;
      P = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*P ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k =0 ;
      if (firstIter) {
	for (i=0 ; i < pointsForLs ; i++) {
	    D[k][k] =w[k]  ;
	    k++ ;
	}
      } else {      
	for (i=0 ; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if (p.suppress==0) {
		D[k][k] =w[k]  ;
		k++ ;
	    }
	    list.next() ;
	}
      }
      iter++ ;
    }
  //Compute good parabola points
  int j = 0;
  for (int i=0 ; i < pointsForLs ; i++) {
    if (w[i] > good_point_thresh) {
      j++;
    }
  }
  parGoodPoints = j;
  if (verbose)
    std::printf("vpMePath::leastSquareParabola()  parGoodPoints %d\n",
    	parGoodPoints);
  //Compute parabola error
  vpColVector err_vec(pointsForLs);
  err_vec = DAx-D*b;
  parab_error = (double) (sqrt(err_vec.sumSquare()) / parGoodPoints);
  if (verbose)
    std::printf("\e[31mvpMePath::leastSquareParabola() error %f \e[30m \n", 
    		parab_error);
  for (i=0 ; i < 3 ; i++) {
    K_par[i+2] = P[i];
  }
}
/*!

  Compute line parameters given point coordinates using a least
  square method on:
  K2/K3 i + K4/K3 = -j 
  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void vpMePath::leastSquareLine(vpImage<unsigned char> &I)
{
  vpMeSite p ;
  int iter = 0 ;
  int pointsForLs;
  if (firstIter) pointsForLs = numPoints;
  else pointsForLs = numberOfSignal();
  if (((firstIter) && (numPoints < 1)) || 
  	((!firstIter) && (list.nbElement() < 1))) {
      vpDisplay::flush(I);
      static char s[30];
      sprintf(s,"/tmp/acherubi/NoPointImage.pgm");
      vpImageIo::writePGM(I,s);
      
      vpERROR_TRACE("Not enough point") ;
      throw(vpTrackingException(vpTrackingException::notEnoughPointError, 
      		"not enough point")) ;
  }
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
  list.front();
  int k = 0;
  if (firstIter) {
      for (int i=0 ; i < pointsForLs ; i++) {
	    A[i][0] = (double) i_ref[i];
	    A[i][1] = 1;
	    b[i] = - (double) j_ref[i];
	}
  } else {
	for (int i = 0; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if (p.suppress==0) {
		A[k][0] = p.ifloat ;
		A[k][1] = 1 ;
		b[k] = - p.jfloat;
		k++ ;
	    }
	    list.next() ;
	}
  }
  while (iter < LSiter) {
      DA = D*A ;
      vpMatrix DAp ;
      P = DA.pseudoInverse(1e-26) *D*b ;
      DAx = DA*P ;
      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;
      list.front() ;
      k = 0 ;
      if (firstIter) {
	for (int i=0 ; i < pointsForLs ; i++) {
	    D[k][k] =w[k]  ;
	    k++ ;
	}
      } else {      
	for (int i=0 ; i < list.nbElement() ; i++) {
	    p = list.value() ;
	    if (p.suppress==0) {
		D[k][k] =w[k]  ;
		k++ ;
	    }
	    list.next() ;
	}
      }
      iter++ ;
  }
  //Compute good line points
  int j = 0;
  for (int i=0 ; i < pointsForLs ; i++) {
    if (w[i] > good_point_thresh) {
      j++;
    }
  }
  lineGoodPoints = j;
  if (verbose)
    std::printf ("vpMePath::leastSquareLine() lineGoodPoints %d\n",
    		lineGoodPoints);
  //Compute line error
  vpColVector err_vec(pointsForLs);
  err_vec = DAx-D*b;
  for (int i=0 ; i < pointsForLs ; i++) {
    err_vec [i] = err_vec [i]*err_vec [i];
  }
  line_error = (double) (sqrt(err_vec.sumSquare()) / lineGoodPoints);
  if (verbose)
    std::printf("\e[31mvpMePath::leastSquareLine() line error %f \e[30m \n", 
    		line_error);
  K_line[0] = 0;
  K_line[1] = 0;
  K_line[2] = P[0];
  K_line[3] = 1;
  K_line[4] = P[1];
  if (verbose)
    std::printf ("\e[31mvpMePath::leastSquareLine() a 0 b %f c %f th 0\e[30m\n",
    		 -K_line[2], -K_line[4]);
}
/*!

  Initialise tracking by enabling the user to click 
  on five points on first image

*/
void vpMePath::initTracking(vpImage<unsigned char> &I)
{
  unsigned *i;
  unsigned *j ;
  i = new unsigned[numPoints] ;
  j = new unsigned[numPoints] ;
  for (int k =0 ; k < numPoints; k++) {
      std::cout << "Please do click on the points "<< k+1 <<"/" << numPoints;
      std::cout << " in the order desired for path following" <<std::endl ;
      std::cout << "points must be on the left edge of the path" <<std::endl ;
      std::cout << "--------  "<< std::endl;
      while (vpDisplay::getClick(I,i[k],j[k])!=true) ;
      std::cout << i[k] <<" " <<j[k] << std::endl;
  }
  for (int k =0 ; k < numPoints ; k++) {
    i_ref [k] = i[k];
    j_ref [k] = j[k];
    if (verbose)
    	std::printf("init tracking ref --> i %d j %d \n",i_ref[k], j_ref[k]);
  }
  i1 = i[0] ;
  j1 = j[0] ;
  i2 = i[numPoints-1] ;
  j2 = j[numPoints-1] ;
  //start tracking these points
  initTracking(I, numPoints, i, j) ;
  delete []i ;
  delete []j ;
}
/*!

  Initialise the tracking of the path points

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.
  \exception vpMeTracker::initTracking : cannot track points


*/ 
void vpMePath::initTracking(vpImage<unsigned char> &I, int n,
		       unsigned *i, unsigned *j)
{
  if (verbose) {
    std::printf("vpMePath::initTracking(I,n,i,j)\n");
    std::printf("n %d\n",n);
    for (int k =0 ; k < n; k++) {
    	std::printf("init tracking ref --> i %d j %d \n",i[k], j[k]);
    }
  }
  //pick line or parabola and compute its parameters
  leastSquare(I);
  if (verbose)
    std::printf("vpMePath::initTracking parameters computed\n");
  //vpDisplay::getClick(I) ;
  sample(I);
  //std::printf("1-initTracking numPts %d \n", list.nbElement());
  display(I, vpColor::green);
  firstIter = false;
  //vpDisplay::getClick(I) ;
  vpMeTracker::initTracking(I) ;
  try {
    //track points
    track(I) ;
  }
  catch(...) {
      vpERROR_TRACE("Error caught") ;
      throw ;
  }
  //std::printf("2-initTracking numPts %d \n", list.nbElement());
  vpMeTracker::display(I) ;
  vpDisplay::flush(I);
  display(I, vpColor::green);
  //std::printf("3-initTracking numPts %d \n", list.nbElement());
  if (verbose)
    std::printf("vpMePath::initTracking(I,n,i,j) finshed\n");
}
/*!

  Track the path points and compute curve characteristics at every iteration

  \exception vpTrackingException::notEnoughPointError : Not enough
  point to compute the parameters.

*/
void vpMePath::track(vpImage<unsigned char> &I)
{
  //counter for iterations
  static int iter = 0 ;
  {
  }
  vpDisplay::display(I) ;
  try {
      //track points
      vpMeTracker::track(I) ;
  }
  catch(...) {
  	vpERROR_TRACE("Error caught") ;
    	throw ;
  }
//  std::printf("1-track numPts %d \n", list.nbElement());
  {
    suppressPoints() ;
//    std::printf("2-track numPts %d \n", list.nbElement());
    try{
      //pick line or parabola and compute its parameters
      leastSquare (I);  
    } catch(...)
      {
	vpERROR_TRACE("Error caught") ;
	throw ;
      }
    display(I, vpColor::green) ; 
    //std::printf("3-track numPts %d \n", list.nbElement());
    //for non horizontal lines sample every sampleIter iterations
    //TODO do it also for parabolas and horizontal lines
    if ((iter % sampleIter == 0) && (iter != 0)) {
      if ((line) || (thetaFin != M_PI / 2)) { 
	if (line) {	
	  sample(I);
  	  //std::printf("4-track numPts %d \n", list.nbElement());
	  if (firstIter) {
	    try{
	      //track points
	      vpMeTracker::track(I) ;
	    }
	    catch(...)
	      {
		vpERROR_TRACE("Error caught") ;
		throw ;
	      }
	  }
	  suppressPoints();
	  //std::printf("5-track numPts %d \n", list.nbElement());
          displayList(I);
	  //pick line or parabola and compute its parameters
	  leastSquare (I);
	}
      }
    }
    //for seekLoops times seek curve extremities
    for (int k =0 ; k < seekLoops ; k++) {
	if (verbose)
	  std::printf ("\e[34m  iteration %d k %d \e[30m  \n ", iter, k);
  	if (!firstIter) seekExtremities(I);
  	//std::printf("6-track numPts %d \n", list.nbElement());
	try{
	  //track points
	  vpMeTracker::track(I) ;
        }
        catch(...) {
	    vpERROR_TRACE("Error caught") ;
	    throw ;
	}
	//std::printf("7-track numPts %d \n", list.nbElement());
	suppressPoints() ;
	displayList(I);
	//std::printf("8-track numPts %d \n", list.nbElement());
	try {
          //pick line or parabola and compute its parameters
	  leastSquare (I);
      	} catch(...) {
	    vpERROR_TRACE("Error caught") ;
	    throw ;
	}
      	display(I, vpColor::green) ; 
	//std::printf("9-track numPts %d \n", list.nbElement());
      	vpDisplay::flush(I);
    }
    if (verbose)
      std::printf("\e[34m IMAGE %d --------> done \e[30m \n\n", iter);
  }
  iter++ ;
}
