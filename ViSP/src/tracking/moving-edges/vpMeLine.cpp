/****************************************************************************
 *
 * $Id: vpMeLine.cpp,v 1.18 2008-10-31 17:20:13 fspindle Exp $
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
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpMeLine.cpp
  \brief Moving edges
*/

#include <stdlib.h>
#include <visp/vpMeTracker.h>
#include <visp/vpMe.h>
#include <visp/vpMeSite.h>
#include <visp/vpMeLine.h>
#include <visp/vpRobust.h>
#include <visp/vpTrackingException.h>


static void
normalizeAngle(double &delta)
{
  //
  while (delta > M_PI) { delta -= M_PI ; }
  while (delta < 0) { delta += M_PI ; }
}

void
computeDelta(double &delta, int i1, int j1, int i2, int j2)
{

  double B = double(i1-i2) ;
  double A = double(j1-j2) ;

  delta =  atan2(B,A) ;
  delta -= M_PI/2.0 ;
  normalizeAngle(delta) ;

}

vpMeLine::vpMeLine():vpMeTracker()
{
}

vpMeLine::~vpMeLine()
{
  list.kill();
}



// ===================================================================
/*!
 * \brief Construct a list of vpMeSiteME at a particular
 * \n		 sampling step between the two extremities of
 * \n		 the line.
 * \pre  Requies me to hold the size of the sample_step
 * \post Calculates the a normal to the line and stores
 * 			 angle in 'alpha'. Creates a list of sites (list)
 *				 (requires calculation of Rho(),Delta())
 * \return status
 */
// ===================================================================
void
vpMeLine::sample(vpImage<unsigned char>& I)
{
  int rows = I.getHeight() ;
  int cols = I.getWidth() ;
  double n_sample;

  if (me->sample_step==0)
  {
    vpERROR_TRACE("function called with sample step = 0") ;
    throw(vpTrackingException(vpTrackingException::fatalError,
			      "sample step = 0")) ;
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat-PExt[1].ifloat;
  double diffsj = PExt[0].jfloat-PExt[1].jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi)+vpMath::sqr(diffsj)));

  // number of samples along line_p
  n_sample = length_p/(double)me->sample_step;

  double stepi = diffsi/(double)n_sample;
  double stepj = diffsj/(double)n_sample;

  // Choose starting point
  double is = PExt[1].ifloat;
  double js = PExt[1].jfloat;

  // Delete old list
  list.front();
  list.kill();

  // sample positions at i*me->sample_step interval along the
  // line_p, starting at PSiteExt[0]
  for(int i=0; i<=vpMath::round(n_sample); i++)
  {
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(is), vpMath::round(js), 0, rows, cols))
    {
      vpMeSite pix ; //= list.value();
      pix.init((int)is, (int)js, delta) ;
      pix.setDisplay(selectDisplay) ;

      if(vpDEBUG_ENABLE(3))
      {
	vpDisplay::displayCross(I,vpMath::round(is), vpMath::round(js), 2, vpColor::blue);
      }

      list.addRight(pix);
    }
    is += stepi;
    js += stepj;

  }

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}


/*!
  Display line.

  \warning To effectively display the line a call to
  vpDisplay::flush() is needed.

 */
void
vpMeLine::display(vpImage<unsigned char>&I, vpColor::vpColorType col)
{
  list.front();

  while (!list.outside())
  {
    vpMeSite pix = list.value() ;

    if (pix.suppress==3)
      vpDisplay::displayCross(I,vpMath::round(pix.ifloat), vpMath::round(pix.jfloat), 5, vpColor::green);
    else
      vpDisplay::displayCross(I,vpMath::round(pix.ifloat), vpMath::round(pix.jfloat), 5, col);

    //vpDisplay::flush(I);
    list.next() ;
  }


  if (fabs(a) < fabs(b)) {
    double i1, j1, i2, j2;
    i1 = 0;
    j1 = (-a*i1 -c) / b;
    i2 = I.getHeight() - 1.0;
    j2 = (-a*i2 -c) / b;
    vpDisplay::displayLine(I, vpMath::round(i1), vpMath::round(j1), vpMath::round(i2), vpMath::round(j2), col);
    //vpDisplay::flush(I);

  }
  else {
    double i1, j1, i2, j2;
    j1 = 0;
    i1 = -(b * j1 + c) / a;
    j2 = I.getWidth() - 1.0;
    i2 = -(b * j2 + c) / a;
    vpDisplay::displayLine(I, vpMath::round(i1), vpMath::round(j1), vpMath::round(i2), vpMath::round(j2), col);
    //vpDisplay::flush(I);
  }
  vpDisplay::displayCross(I,vpMath::round(PExt[0].ifloat), vpMath::round(PExt[0].jfloat), 10, vpColor::green);
  vpDisplay::displayCross(I,vpMath::round(PExt[1].ifloat), vpMath::round(PExt[1].jfloat), 10, vpColor::green);
  //vpDisplay::flush(I) ;
}


void
vpMeLine::initTracking(vpImage<unsigned char> &I)
{
  unsigned i1, j1, i2, j2 ;

  std::cout << "Click on the line first point..." <<std::endl ;
  while (vpDisplay::getClick(I,i1,j1)!=true) ;
  std::cout << "Click on the line second point..." <<std::endl ;
  while (vpDisplay::getClick(I,i2,j2)!=true) ;

  try
  {
    initTracking(I, i1, j1,  i2,  j2) ;
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

}

/*
void
vpMeLine::leastSquare(vpImage<unsigned char> &I)
{
  vpMatrix A(numberOfSignal(),3) ;
  vpColVector x(3) ;

  // Construction du systeme Ax=b
  // a i + b j + c = 0
  // A = (i j 1)   x = (a b c)^T
  int i ;

  vpRobust r(numberOfSignal()) ;
  r.setIteration(0) ;
  vpMatrix D(numberOfSignal(),numberOfSignal()) ;
  D.setIdentity() ;
  vpMatrix DA, DAmemory ;
  vpColVector DAx ;
  vpColVector w(numberOfSignal()) ;
  w =1 ;
  vpMeSite p ;
  int iter =0 ;
  int nos_1 = -1 ;

  if (list.nbElement() < 2)
  {
    vpERROR_TRACE("Not enough point") ;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError,
			      "not enough point")) ;
  }

  while (iter < 1)
  {
    // -------------------
    nos_1 = numberOfSignal() ;
    list.front() ;
    int k =0 ;
    for (i=0 ; i < list.nbElement() ; i++)
    {
      p = list.value() ;
      if (p.suppress==0)
      {
	A[k][0] = p.ifloat ;
	A[k][1] = p.jfloat ;
	A[k][2] = 1 ;
	k++ ;
      }
      list.next() ;
    }

    DA = D*A ;
    DAmemory = DA ; // utile car SVDcmp est destructive wrt DA...

    vpColVector W(3) ;
    vpMatrix V(3,3) ;

    DA.svd(W,V) ; // a = UWV^T

    //  La solution du systeme Ax = 0 est donnee par x = derniere colonne de V
    x = V.column(3) ;

    DAx = DAmemory*x ;


    vpColVector res = DAx - b ;
    r.MEstimator(vpRobust::TUKEY,res,w) ;


    list.front() ;
    k =0 ;
    for (i=0 ; i < list.nbElement() ; i++)
    {
      p = list.value() ;
      if (p.suppress==0)
      {
	if (w[k] < 0.3)
	{
	  D[k][k] =w[k]  ;
	  p.suppress  = 3 ;
	  list.modify(p) ;
	}
	k++ ;
      }
      list.next() ;
    }

    iter++ ;
  // -------------------
  }
  // mise a jour de l'equation de la droite
  a = x[0] ;
  b = x[1] ;
  c = x[2] ;

  std::cout << "x "<< x.t() ;

  // mise a jour du delta
  delta = atan2(a,b) ;

  normalizeAngle(delta) ;

}
*/
void
vpMeLine::leastSquare()
{
  vpMatrix A(numberOfSignal(),2) ;
  vpColVector x(3) ;

  // Construction du systeme Ax=b
  // a i + b j + c = 0
  // A = (i j 1)   x = (a b c)^T
  int i ;

  vpRobust r(numberOfSignal()) ;
  r.setIteration(0) ;
  vpMatrix D(numberOfSignal(),numberOfSignal()) ;
  D.setIdentity() ;
  vpMatrix DA, DAmemory ;
  vpColVector DAx ;
  vpColVector w(numberOfSignal()) ;
  vpColVector B(numberOfSignal()) ;
  w =1 ;
  vpMeSite p ;
  int iter =0 ;
  int nos_1 = -1 ;

  if (list.nbElement() < 2)
  {
    vpERROR_TRACE("Not enough point") ;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError,
			      "not enough point")) ;
  }

  // vpTRACE("%f %f %f ",a,b,c) ;
  if ((fabs(b) >=0.9) )// ax+y +c =0
  {
    while (iter < 1)
    {
      // -------------------
      nos_1 = numberOfSignal() ;
      list.front() ;
      int k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  A[k][0] = p.ifloat ;
	  A[k][1] = 1 ;
	  B[k] = -p.jfloat ;
	  k++ ;
	}
	list.next() ;
      }


      DA = D*A ;
      vpMatrix DAp ;

      x = DA.pseudoInverse(1e-26) *D*B ;

      DAx = DA*x ;

      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;

      list.front() ;
      k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  if (w[k] < 0.3)
	  {
	    D[k][k] =w[k]  ;
	    p.suppress  = 3 ;
	    list.modify(p) ;
	  }
	  k++ ;
	}
	list.next() ;
      }

      iter++ ;
      // mise a jour de l'equation de la droite
      a = x[0] ;
      b = 1 ;
      c = x[1] ;

      double s =sqrt( vpMath::sqr(a)+vpMath::sqr(b)) ;
      a /= s ;
      b /= s ;
      c /= s ;
      //   vpTRACE("ax+y+c %f %f %f",a,b,c) ;
      // -------------------
    }

  }
  else
  {
    while (iter < 1) // x + by+c =0
    {
      // -------------------
      nos_1 = numberOfSignal() ;
      list.front() ;
      int k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  A[k][0] = p.jfloat ;
	  A[k][1] = 1 ;
	  B[k] = -p.ifloat ;
	  k++ ;
	}
	list.next() ;
      }


      DA = D*A ;
      vpMatrix DAp ;

      x = DA.pseudoInverse(1e-26) *D*B ;

      DAx = DA*x ;

      r.setIteration(0) ;
      r.MEstimator(vpRobust::TUKEY,DAx,w) ;

      list.front() ;
      k =0 ;
      for (i=0 ; i < list.nbElement() ; i++)
      {
	p = list.value() ;
	if (p.suppress==0)
	{
	  if (w[k] < 0.3)
	  {
	    D[k][k] =w[k]  ;
	    p.suppress  = 3 ;
	    list.modify(p) ;
	  }
	  k++ ;
	}
	list.next() ;
      }
      iter++ ;
      a = 1 ;
      b = x[0] ;
      c = x[1] ;

      double s = sqrt(vpMath::sqr(a)+vpMath::sqr(b)) ;
      a /= s ;
      b /= s ;
      c /= s ;
      //   vpTRACE("x+by+c %f %f %f",a,b,c) ;
      // -------------------
    }
  }
  // mise a jour de l'equation de la droite

  //  vpTRACE("ai+bj+c %f %f %f",a,b,c) ;
  // mise a jour du delta
  delta = atan2(a,b) ;

  normalizeAngle(delta) ;
}

void
vpMeLine::initTracking(vpImage<unsigned char> &I,
		       unsigned i1,unsigned j1,
		       unsigned i2, unsigned j2)
{
  vpCDEBUG(1) <<" begin vpMeLine::initTracking()"<<std::endl ;

  int i1s, j1s, i2s, j2s;

  i1s = (int)i1;
  i2s = (int)i2;
  j1s = (int)j1;
  j2s = (int)j2;

  try{

    //  1. On fait ce qui concerne les droites (peut etre vide)
    {
      // Points extremites
      PExt[0].ifloat = i1 ;
      PExt[0].jfloat = j1 ;
      PExt[1].ifloat = i2 ;
      PExt[1].jfloat = j2 ;

      double angle = atan2((double)(i1s-i2s),(double)(j1s-j2s)) ;
      a = cos(angle) ;
      b = sin(angle) ;

      // Real values of a, b can have an other sign. So to get the good values
      // of a and b in order to initialise then c, we call track(I) just below

      computeDelta(delta,i1s,j1s,i2s,j2s) ;

      //      vpTRACE("a: %f b: %f c: %f -b/a: %f delta: %f", a, b, c, -(b/a), delta);

      sample(I) ;

    }
    //  2. On appelle ce qui n'est pas specifique
    {
      vpMeTracker::initTracking(I) ;
    }
    // Call track(I) to give the good sign to a and b and to initialise c which can be used for the display
    track(I);
  }
  catch(...)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}

void
vpMeLine::suppressPoints()
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
}

void
vpMeLine::setExtremities()
{
  double imin = +1e6 ;
  double jmin = +1e6;
  double imax = -1 ;
  double jmax = -1 ;


  // Loop through list of sites to track
  list.front();
  while(!list.outside())
  {
    vpMeSite s = list.value() ;//current reference pixel
    if (s.ifloat < imin)
    {
      imin = s.ifloat ;
      jmin = s.jfloat ;
    }

    if (s.ifloat > imax)
    {
      imax = s.ifloat ;
      jmax = s.jfloat ;
    }
    list.next() ;
  }

  PExt[0].ifloat = imin ;
  PExt[0].jfloat = jmin ;
  PExt[1].ifloat = imax ;
  PExt[1].jfloat = jmax ;

  if (fabs(imin-imax) < 5)
  {
    list.front();
    while(!list.outside())
    {
      vpMeSite s = list.value() ;//current reference pixel
      if (s.jfloat < jmin)
      {
	imin = s.ifloat ;
	jmin = s.jfloat ;
      }

      if (s.jfloat > jmax)
      {
	imax = s.ifloat ;
	jmax = s.jfloat ;
      }
      list.next() ;
    }
    PExt[0].ifloat = imin ;
    PExt[0].jfloat = jmin ;
    PExt[1].ifloat = imax ;
    PExt[1].jfloat = jmax ;
  }


}

void
vpMeLine::seekExtremities(vpImage<unsigned char> &I)
{
  vpCDEBUG(1) <<"begin vpMeLine::sample() : "<<std::endl ;

  int rows = I.getHeight() ;
  int cols = I.getWidth() ;
  double n_sample;

  if (me->sample_step==0)
  {

    vpERROR_TRACE("function called with sample step = 0") ;
    throw(vpTrackingException(vpTrackingException::fatalError,
			      "sample step = 0")) ;
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat-PExt[1].ifloat;
  double diffsj = PExt[0].jfloat-PExt[1].jfloat;

  double s = vpMath::sqr(diffsi)+vpMath::sqr(diffsj) ;

  double di = diffsi/sqrt(s) ; // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj/sqrt(s) ;

  double length_p = sqrt((vpMath::sqr(diffsi)+vpMath::sqr(diffsj)));

  // number of samples along line_p
  n_sample = length_p/(double)me->sample_step;
  double sample = (double)me->sample_step;

  vpMeSite P ;
  P.init((int) PExt[0].ifloat, (int)PExt[0].jfloat, delta) ;
  P.setDisplay(selectDisplay) ;

  int  memory_range = me->range ;
  me->range = 1 ;

  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat + di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat + dj*sample ; P.j = (int)P.jfloat ;

    if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;

      if (P.suppress ==0)
      {
	list += P ;
	if (vpDEBUG_ENABLE(3)) 	vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
	if (vpDEBUG_ENABLE(3)) 	vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }

  P.init((int) PExt[1].ifloat, (int)PExt[1].jfloat, delta) ;
  P.setDisplay(selectDisplay) ;
  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat - di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat - dj*sample ; P.j = (int)P.jfloat ;

    if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;

      if (P.suppress ==0)
      {
	list += P ;
	if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
	if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }

  me->range = memory_range ;

  vpCDEBUG(1) <<"end vpMeLine::sample() : " ;
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}

static void
project(double a, double b, double c, double i, double j, double &ip,double  &jp)
{
  if (fabs(a)>fabs(b))
  {
    jp = (vpMath::sqr(a)*j - a*b*i - c*b)/(vpMath::sqr(a)+vpMath::sqr(b)) ;
    ip = (-c-b*jp)/a;
  }
  else
  {
   ip = (vpMath::sqr(b)*i-a*b*j-c*a)/(vpMath::sqr(a)+vpMath::sqr(b)) ;
   jp = (-c-a*ip)/b;
  }
}

void
vpMeLine::reSample(vpImage<unsigned char> &I)
{

  double i1,j1,i2,j2 ;

  project(a,b,c,PExt[0].ifloat,PExt[0].jfloat,i1,j1) ;
  project(a,b,c,PExt[1].ifloat,PExt[1].jfloat,i2,j2) ;

  // Points extremites
  PExt[0].ifloat = i1 ;
  PExt[0].jfloat = j1 ;
  PExt[1].ifloat = i2 ;
  PExt[1].jfloat = j2 ;

  double d = sqrt(vpMath::sqr(i1-i2)+vpMath::sqr(j1-j2)) ;

  int n = numberOfSignal() ;
  double expecteddensity = d / (double)me->sample_step;

  if (n<0.8*expecteddensity)
  {
    sample(I) ;
    //  2. On appelle ce qui n'est pas specifique
    {
      vpMeTracker::initTracking(I) ;
    }
  }
}

void
vpMeLine::updateDelta()
{
  vpMeSite p ;
  list.front() ;
  for (int i=0 ; i < list.nbElement() ; i++)
  {
    p = list.value() ;
    p.alpha = delta ;
    list.modify(p) ;
    list.next() ;
  }
}

void
vpMeLine::track(vpImage<unsigned char> &I)
{
  vpCDEBUG(1) <<"begin vpMeLine::track()"<<std::endl ;

  //  1. On fait ce qui concerne les droites (peut etre vide)
  {
  }
  //  2. On appelle ce qui n'est pas specifique
  {
    vpMeTracker::track(I) ;
  }

  // 3. On revient aux droites
  {
    // supression des points rejetes par les ME
    suppressPoints() ;
    setExtremities() ;


    // Estimation des parametres de la droite aux moindres carre
    try
    {
      leastSquare() ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error caught") ;
	  throw ;
    }


    // recherche de point aux extremite de la droites
    // dans le cas d'un glissement
    seekExtremities(I) ;

    setExtremities() ;
    try
    {
      leastSquare() ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error caught") ;
	  throw ;
    }

    // suppression des points rejetes par la regression robuste
    suppressPoints() ;
    setExtremities() ;

    //reechantillonage si necessaire
    reSample(I) ;

    // remet a jour l'angle delta pour chaque  point de la liste

    updateDelta() ;

    // Remise a jour de delta dans la liste de site me
    if (vpDEBUG_ENABLE(2))
      {
	display(I,vpColor::red) ;
	vpMeTracker::display(I) ;
	vpDisplay::flush(I) ;
      }


  }

  computeRhoTheta(I) ;

  vpCDEBUG(1) <<"end vpMeLine::track()"<<std::endl ;

}


void
vpMeLine::computeRhoTheta(vpImage<unsigned char>& I)
{
  rho = -c ;
  theta = atan2(a,b) ;

  /*  while(theta < -M_PI)	theta += 2*M_PI ;
  while(theta >= M_PI)	theta -= 2*M_PI ;

  // If theta is between -90 and -180 get the equivalent
  // between 0 and 90
  if(theta <-M_PI/2)
    {
      theta += M_PI ;
      rho *= -1 ;
    }
  // If theta is between 90 and 180 get the equivalent
  // between 0 and -90
  if(theta >M_PI/2)
    {
      theta -= M_PI ;
      rho *= -1 ;
    }
  */
  // convention pour choisir le signe de rho
  int i,j ;
  i = vpMath::round((PExt[0].ifloat + PExt[1].ifloat )/2) ;
  j = vpMath::round((PExt[0].jfloat + PExt[1].jfloat )/2) ;

  int  end = false ;
  double incr = 10 ;

  int i1=0,i2=0,j1=0,j2=0 ;
  unsigned char v1=0,v2=0 ;
  while (!end)
    {
      end = true;
      i1 = (int)(i + a *incr) ;
      j1 = (int)(j + b *incr) ;
      v1 = I[i1][j1] ;


      i2 = (int)(i - a *incr) ;
      j2 = (int)(j - b *incr) ;
      v2 = I[i2][j2] ;


      if (abs(v1-v2) < 1)
      {

	incr-- ;
	end = false ;
	if (incr==1)
	{
	  std::cout << "In CStraightLine::GetParameters() " ;
	  std::cout << " Error Tracking " << abs(v1-v2) << std::endl ;
	}
      }
    }

  int sgn ;
  int i3, j3 ;
  if (v1<v2)
  { i3 = i1 ; j3 = j1 ; sgn = -1 ; }
  else
  { i3 = i2 ; j3 = j2 ; sgn =1 ; }


  while (theta > M_PI)    theta -=2*M_PI ;
  while (theta < -M_PI)   theta +=2*M_PI ;


    vpTRACE("%f %f %d", rho, theta, sgn) ;
  if ( (theta > M_PI/4) &&  (theta < 3*M_PI/4))
  {
    if (sgn <0)
    {
            vpTRACE(" ") ;
      rho *=-1 ;
      theta += M_PI ;
    }
  }
  else
    if ( (theta < -M_PI/4) &&  (theta > -3*M_PI/4))
    {
      if (sgn>0)
      {
	      vpTRACE(" ") ;
	rho *=-1 ;
	theta += M_PI ;
      }
    }
  else
    if ((theta <M_PI/4) || (theta>3*M_PI/4))
    {
      if (sgn>0)
      {
	      vpTRACE(" ") ;
	rho *=-1 ;
	theta += M_PI ;
      }
    }
    else
    {
      if (sgn<0)
      {
	vpTRACE(" ") ;
	rho *=-1 ;
	theta += M_PI ;
      }
    }
     vpTRACE("%f %f", rho, theta) ;


  while (theta > M_PI)     theta -=2*M_PI ;
  while (theta < -M_PI)    theta +=2*M_PI ;
     vpTRACE("%f %f", rho, theta) ;

  if (vpDEBUG_ENABLE(2))
    vpDisplay::displayArrow(I,i,j,i3,j3, vpColor::green) ;

}
double
vpMeLine::getRho() const
{


  /*  double s = vpMath::sqr(a)+vpMath::sqr(b) ;
  //  vpTRACE("%f %f %f %f", a,b,c,s) ;
  if (fabs(s) < 1e-10)
  {
    vpERROR_TRACE("Division par zero") ;
    throw(vpTrackingException(vpException::divideByZeroERR,
			      "division by zero in getRho")) ;
  }
  */
  return  rho ; //-c ;
}
double
vpMeLine::getTheta() const
{
  //  double theta =  atan2(a,b) ;
  return theta ;
}

void
vpMeLine::getExtremities(double& i1, double& j1, double& i2, double& j2)
{
  /*Return the coordinates of the extremities of the line*/
  i1 = PExt[0].ifloat;
  j1 = PExt[0].jfloat;
  i2 = PExt[1].ifloat;
  j2 = PExt[1].jfloat;
}

