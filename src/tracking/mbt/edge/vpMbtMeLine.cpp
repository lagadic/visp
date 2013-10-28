/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/
#include <visp/vpConfig.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file vpMbtMeLine.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#include <visp/vpMbtMeLine.h>
#include <visp/vpTrackingException.h>
#include <visp/vpRobust.h>

//! Normalize an angle between -Pi and Pi
static void
normalizeAngle(double &delta)
{
  while (delta > M_PI) { delta -= M_PI ; }
  while (delta < -M_PI) { delta += M_PI ; }
}


/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMbtMeLine::vpMbtMeLine():vpMeTracker()
{
  sign = 1;
  theta_1 = M_PI/2;
  rho = theta = delta = delta_1 = a = b = c = imin = imax = jmin = jmax = expecteddensity = 0.;
}

/*!
  Basic destructor.
*/
vpMbtMeLine::~vpMbtMeLine()
{
  list.clear();
}

/*!
  Initialization of the tracking. The line is defined thanks to the
  coordinates of two points corresponding to the extremities and its (\f$\rho \: \theta\f$) parameters.
  
  Remeber the equation of a line : \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

  \param I : Image in which the line appears.
  \param ip1 : Coordinates of the first point.
  \param ip2 : Coordinates of the second point.
  \param rho : The \f$\rho\f$ parameter
  \param theta : The \f$\theta\f$ parameter
*/
void
vpMbtMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2, double rho, double theta)
{
  vpCDEBUG(1) <<" begin vpMeLine::initTracking()"<<std::endl ;

  try
  {
    //  1. On fait ce qui concerne les droites (peut etre vide)
    // Points extremites
    PExt[0].ifloat = (float)ip1.get_i() ;
    PExt[0].jfloat = (float)ip1.get_j() ;
    PExt[1].ifloat = (float)ip2.get_i() ;
    PExt[1].jfloat = (float)ip2.get_j() ;
     
    this->rho = rho;
    this->theta = theta;
      
    a = cos(theta);
    b = sin(theta);
    c = -rho;
    
    double d = sqrt(vpMath::sqr(ip1.get_i()-ip2.get_i())+vpMath::sqr(ip1.get_j()-ip2.get_j())) ;

    expecteddensity = d / (double)me->getSampleStep();
      
    delta = - theta + M_PI/2.0;
    normalizeAngle(delta);
    delta_1 = delta;

    sample(I) ;

    vpMeTracker::track(I);
  }
  catch(...)
  {
//    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  vpCDEBUG(1) <<" end vpMeLine::initTracking()"<<std::endl ;
}


/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities of the line.

  \param I : Image in which the line appears.
*/
void
vpMbtMeLine::sample(const vpImage<unsigned char>& I)
{
  int rows = (int)I.getHeight() ;
  int cols = (int)I.getWidth() ;
  double n_sample;

  //if (me->getSampleStep==0)
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon())
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
  n_sample = length_p/(double)me->getSampleStep();

  double stepi = diffsi/(double)n_sample;
  double stepj = diffsj/(double)n_sample;

  // Choose starting point
  double is = PExt[1].ifloat;
  double js = PExt[1].jfloat;

  // Delete old list
  list.clear();

  // sample positions at i*me->getSampleStep() interval along the
  // line_p, starting at PSiteExt[0]

  vpImagePoint ip;
  for(int i=0; i<=vpMath::round(n_sample); i++)
  {
    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(is), vpMath::round(js), 0, rows, cols))
    {
      vpMeSite pix ; //= list.value();
      pix.init((int)is, (int)js, delta, 0, sign) ;
  
      pix.track(I,me,0);
      
      pix.setDisplay(selectDisplay) ;

      if(vpDEBUG_ENABLE(3))
      {
	      ip.set_i( is );
	      ip.set_j( js );
	      vpDisplay::displayCross(I, ip, 2, vpColor::blue);
      }

      list.push_back(pix);
    }
    is += stepi;
    js += stepj;

  }

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}


/*!
  Suppress the moving which belong no more to the line.
  
  \param I : The image.
*/
void
vpMbtMeLine::suppressPoints(const vpImage<unsigned char> & /*I*/)
{
  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ){
    vpMeSite s = *it;//current reference pixel

  if (fabs(sin(theta)) > 0.9) // Vertical line management
  {
    if ((s.i < imin) ||(s.i > imax)) 
    {
      s.setState(vpMeSite::CONSTRAST);
    }
  }

  else if (fabs(cos(theta)) > 0.9) // Horizontal line management
  {
    if ((s.j < jmin) || (s.j > jmax))
    {
      s.setState(vpMeSite::CONSTRAST);
    }
  }

  else
  {
    if ((s.i < imin) ||(s.i > imax) || (s.j < jmin) || (s.j > jmax) ) 
    {
      s.setState(vpMeSite::CONSTRAST);
    }
  }

  if (s.getState() != vpMeSite::NO_SUPPRESSION)
    it = list.erase(it);
  else
    ++it;
  }
}


/*!
 Seek along the line defined by its equation, the two extremities of the line. This function is useful in case of translation of the line.
 
 \param I : Image in which the line appears.
*/
void
vpMbtMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) <<"begin vpMeLine::sample() : "<<std::endl ;

  int rows = (int)I.getHeight() ;
  int cols = (int)I.getWidth() ;
  double n_sample;

  //if (me->getSampleStep()==0)
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon())
  {

    vpERROR_TRACE("function called with sample step = 0") ;
    throw(vpTrackingException(vpTrackingException::fatalError,"sample step = 0")) ;
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat-PExt[1].ifloat;
  double diffsj = PExt[0].jfloat-PExt[1].jfloat;

  double s = vpMath::sqr(diffsi)+vpMath::sqr(diffsj) ;

  double di = diffsi/sqrt(s) ; // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj/sqrt(s) ;

  double length_p = sqrt(s); /*(vpMath::sqr(diffsi)+vpMath::sqr(diffsj))*/

  // number of samples along line_p
  n_sample = length_p/(double)me->getSampleStep();
  double sample = (double)me->getSampleStep();

  vpMeSite P ;
  P.init((int) PExt[0].ifloat, (int)PExt[0].jfloat, delta_1, 0, sign) ;
  P.setDisplay(selectDisplay) ;

  unsigned int  memory_range = me->getRange() ;
  me->setRange(1);

  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat + di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat + dj*sample ; P.j = (int)P.jfloat ;


    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) ) 
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }
    else
    if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;

      if (P.getState() == vpMeSite::NO_SUPPRESSION)
      {
        list.push_back(P);
        if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
        if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }
	
  P.init((int) PExt[1].ifloat, (int)PExt[1].jfloat, delta_1, 0, sign) ;
  P.setDisplay(selectDisplay) ;
  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat - di*sample ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat - dj*sample ; P.j = (int)P.jfloat ;


    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) ) 
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }

  else
  if(!outOfImage(P.i, P.j, 5, rows, cols))
    {
      P.track(I,me,false) ;
			
      if (P.getState() == vpMeSite::NO_SUPPRESSION)
      {
        list.push_back(P);
				if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 5, vpColor::green) ;
      }
      else
				if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j, 10, vpColor::blue) ;
    }
  }
	
  me->setRange(memory_range);
	
  vpCDEBUG(1) <<"end vpMeLine::sample() : " ;
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl  ;
}

/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
*/
void
vpMbtMeLine::reSample(const vpImage<unsigned char> &I)
{
  double d = sqrt(vpMath::sqr(PExt[0].ifloat-PExt[1].ifloat)+vpMath::sqr(PExt[0].jfloat-PExt[1].jfloat)) ;

  unsigned int n = numberOfSignal() ;
  double expecteddensity = d / (double)me->getSampleStep();

  if ((double)n<0.5*expecteddensity && n > 0)
  {
    double delta_new = delta;
    delta = delta_1;
    sample(I) ;
    delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    {
      vpMeTracker::initTracking(I) ;
    }
  }
}


/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
*/
void
vpMbtMeLine::reSample(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2)
{
  double d = sqrt(vpMath::sqr(ip1.get_i()-ip2.get_i())+vpMath::sqr(ip1.get_j()-ip2.get_j())) ;

  size_t n = list.size();
  expecteddensity = d / (double)me->getSampleStep();

  if ((double)n<0.5*expecteddensity && n > 0)
  {
    double delta_new = delta;
    delta = delta_1;
    PExt[0].ifloat = (float)ip1.get_i() ;
    PExt[0].jfloat = (float)ip1.get_j() ;
    PExt[1].ifloat = (float)ip2.get_i() ;
    PExt[1].jfloat = (float)ip2.get_j() ;
    sample(I) ;
    delta = delta_new;
    vpMeTracker::track(I) ;
  }
}

/*!
  Set the alpha value of the different vpMeSite to the value of delta.
*/
void
vpMbtMeLine::updateDelta()
{
  vpMeSite p ;

  double diff = 0;

  //if(fabs(theta) == M_PI )
  if(std::fabs(std::fabs(theta) - M_PI) <= vpMath::maximum(std::fabs(theta), (double)M_PI)*std::numeric_limits<double>::epsilon() )
  {
    theta = 0 ;
  }

  diff = fabs(theta - theta_1);
  if (diff > M_PI/2.0)
  sign *= -1;

  theta_1 = theta;
  
  delta = - theta + M_PI/2.0;
  normalizeAngle(delta);

  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ++it){
    p = *it;
    p.alpha = delta ;
    p.mask_sign = sign;
    *it = p;
  }
  delta_1 = delta;
}

/*!
 Track the line in the image I.
 
 \param I : Image in which the line appears.
 */
void
vpMbtMeLine::track(const vpImage<unsigned char> &I)
{
  //  2. On appelle ce qui n'est pas specifique
  try
  {
    vpMeTracker::track(I);
  }
  catch(...)
  {
    throw ;
  }

  // supression des points rejetes par les ME
  //  suppressPoints(I);
  //  setExtremities();
}


/*!
  Update the moving edges parameters after the virtual visual servoing.
  
  \param  I : The image.
  \param  rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param  theta : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, double rho, double theta)
{
  this->rho = rho;
  this->theta = theta;
  a = cos(theta);
  b = sin(theta);
  c = -rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  //reechantillonage si necessaire
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}


/*!
  Update the moving edges parameters after the virtual visual servoing.
  
  \param I : The image.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
  \param rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param theta : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2, double rho, double theta)
{
  this->rho = rho;
  this->theta = theta;
  a = cos(theta);
  b = sin(theta);
  c = -rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  //reechantillonage si necessaire
  reSample(I,ip1,ip2);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}


/*!
  Seek in the list of available points the two extremities of the line.
*/
void
vpMbtMeLine::setExtremities()
{
  double imin = +1e6 ;
  double jmin = +1e6;
  double imax = -1 ;
  double jmax = -1 ;

  // Loop through list of sites to track
  for(std::list<vpMeSite>::const_iterator it=list.begin(); it!=list.end(); ++it){
    vpMeSite s = *it;//current reference pixel
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
  }

  if ( ! list.empty() )
  {
    PExt[0].ifloat = imin ;
    PExt[0].jfloat = jmin ;
    PExt[1].ifloat = imax ;
    PExt[1].jfloat = jmax ;
  }

  if (fabs(imin-imax) < 25)
  {
    for(std::list<vpMeSite>::const_iterator it=list.begin(); it!=list.end(); ++it){
      vpMeSite s = *it;//current reference pixel
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
    }

    if (! list.empty())
    {
      PExt[0].ifloat = imin ;
      PExt[0].jfloat = jmin ;
      PExt[1].ifloat = imax ;
      PExt[1].jfloat = jmax ;
    }
    bubbleSortJ();
  }

  else
    bubbleSortI();
}


static bool sortByI(const vpMeSite& s1, const vpMeSite& s2){
  return (s1.ifloat > s2.ifloat);
}

void
vpMbtMeLine::bubbleSortI()
{
#if 0
  unsigned int nbElmt = list.size();
  for (unsigned int pass = 1; pass < nbElmt; pass++)
  {
    list.front();
    for (unsigned int i=0; i < nbElmt-pass; i++)
    {
      vpMeSite s1 = list.value() ;
      vpMeSite s2 = list.nextValue() ;
      if (s1.ifloat > s2.ifloat)
        list.swapRight();
      else
        list.next();
    }
  }
#endif
  list.sort(sortByI);
}


static bool sortByJ(const vpMeSite& s1, const vpMeSite& s2){
  return (s1.jfloat > s2.jfloat);
}

void
vpMbtMeLine::bubbleSortJ()
{
#if 0
  unsigned int nbElmt = list.size();
  for(unsigned int pass=1; pass < nbElmt; pass++)
  {
    list.front();
    for (unsigned int i=0; i < nbElmt-pass; i++)
    {
      vpMeSite s1 = list.value() ;
      vpMeSite s2 = list.nextValue() ;
      if (s1.jfloat > s2.jfloat)
        list.swapRight();
      else
        list.next();
    }
  }
#endif
  list.sort(sortByJ);
}


void
vpMbtMeLine::findSignal(const vpImage<unsigned char>& I, const vpMe *me, double *conv)
{
  vpImagePoint itest(PExt[0].ifloat+(PExt[1].ifloat-PExt[0].ifloat)/2, PExt[0].jfloat+(PExt[1].jfloat-PExt[0].jfloat)/2);
  
  vpMeSite pix ; //= list.value();
  pix.init(itest.get_i(), itest.get_j(), delta, 0, sign);
  
  vpMeSite  *list_query_pixels;
//  double  convolution = 0;
  unsigned int range  = me->getRange();
  
  list_query_pixels = pix.getQueryList(I, (int)range);
  
  vpDisplay::displayCross(I,itest,5,vpColor::cyan,3);
  vpDisplay::displayLine(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),vpImagePoint(list_query_pixels[2*range].ifloat,list_query_pixels[2*range].jfloat),vpColor::cyan);
  vpDisplay::displayCross(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),5,vpColor::orange,3);

  for(unsigned int n = 0 ; n < 2 * range + 1 ; n++)
  {
    conv[n] = list_query_pixels[n].convolution(I, me);
  }
  delete [] list_query_pixels;
}

#endif

