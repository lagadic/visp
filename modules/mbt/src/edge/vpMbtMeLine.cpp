/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file vpMbtMeLine.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <algorithm>    // std::min

#include <visp3/mbt/vpMbtMeLine.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpRobust.h>

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
vpMbtMeLine::vpMbtMeLine()
  : rho(0.), theta(0.), theta_1(M_PI/2), delta(0.), delta_1(0), sign(1),
    a(0.), b(0.), c(0.), imin(0), imax(0), jmin(0), jmax(0),
    expecteddensity(0.)
{
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
  
  Remember the equation of a line : \f$ i \; cos(\theta) + j \; sin(\theta) - \rho = 0 \f$

  \param I : Image in which the line appears.
  \param ip1 : Coordinates of the first point.
  \param ip2 : Coordinates of the second point.
  \param rho_ : The \f$\rho\f$ parameter
  \param theta_ : The \f$\theta\f$ parameter
*/
void
vpMbtMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                          double rho_, double theta_)
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
     
    this->rho = rho_;
    this->theta = theta_;
    theta_1 = theta_;
      
    a = cos(theta);
    b = sin(theta);
    c = -rho;
          
    delta = - theta + M_PI/2.0;
    normalizeAngle(delta);
    delta_1 = delta;

    sample(I) ;
    expecteddensity = (double)list.size();

    vpMeTracker::track(I);
  }
  catch(vpException &e)
  {
    throw e;
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
    throw(vpTrackingException(vpTrackingException::fatalError,
                              "Function vpMbtMeLine::sample() called with moving-edges sample step = 0")) ;
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
    if(!outOfImage(vpMath::round(is), vpMath::round(js), (int)(me->getRange()+me->getMaskSize()+1), (int)rows, (int)cols))
    {
      vpMeSite pix ; //= list.value();
      pix.init((int)is, (int)js, delta, 0, sign) ;
  
      pix.track(I, me, false);
      
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
  vpCDEBUG(1) << list.size() << " point inserted in the list " << std::endl  ;
}


/*!
  Suppress the moving which belong no more to the line.
  
  \param I : The image.
*/
void
vpMbtMeLine::suppressPoints(const vpImage<unsigned char> & I)
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

    if (outOfImage(s.i, s.j, (int)(me->getRange()+me->getMaskSize()+1), (int)I.getHeight(), (int)I.getWidth()))
    {
      s.setState(vpMeSite::TOO_NEAR);
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
    throw(vpTrackingException(vpTrackingException::fatalError,
                              "Function called with sample step = 0")) ;
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
  double sample_step = (double)me->getSampleStep();

  vpMeSite P ;
  P.init((int) PExt[0].ifloat, (int)PExt[0].jfloat, delta_1, 0, sign) ;
  P.setDisplay(selectDisplay) ;

  unsigned int  memory_range = me->getRange() ;
  me->setRange(1);

  for (int i=0 ; i < 3 ; i++)
  {
    P.ifloat = P.ifloat + di*sample_step ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat + dj*sample_step ; P.j = (int)P.jfloat ;

    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) )
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }
    else
    if(!outOfImage(P.i, P.j, (int)(me->getRange()+me->getMaskSize()+1), (int)rows, (int)cols))
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
    P.ifloat = P.ifloat - di*sample_step ; P.i = (int)P.ifloat ;
    P.jfloat = P.jfloat - dj*sample_step ; P.j = (int)P.jfloat ;

    if ((P.i < imin) ||(P.i > imax) || (P.j < jmin) || (P.j > jmax) )
    {
      if (vpDEBUG_ENABLE(3)) vpDisplay::displayCross(I,P.i,P.j,5,vpColor::cyan) ;
    }

    else
    if(!outOfImage(P.i, P.j, (int)(me->getRange()+me->getMaskSize()+1), (int)rows, (int)cols))
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
  Compute the projection error of the line. Compare the gradient direction around samples of the line to its direction.
  Error is expressed in radians between 0 and M_PI/2.0;

  \param _I : Image in which the line appears.
  \param _sumErrorRad : sum of the error per feature.
  \param _nbFeatures : Number of features used to compute _sumErrorRad.
*/
void
vpMbtMeLine::computeProjectionError(const vpImage<unsigned char>& _I, double &_sumErrorRad, unsigned int &_nbFeatures)
{
  _sumErrorRad = 0;
  _nbFeatures = 0;
  double deltaNormalized = theta;
  unsigned int iter = 0;

  while (deltaNormalized<0) deltaNormalized += M_PI;
  while (deltaNormalized>M_PI) deltaNormalized -= M_PI;

  vpColVector vecLine(2);
  vecLine[0] = cos(deltaNormalized);
  vecLine[1] = sin(deltaNormalized);
  vecLine.normalize();

//  vpMatrix filterX(3,3);
//  filterX[0][0] = -1;
//  filterX[1][0] = -2;
//  filterX[2][0] = -1;

//  filterX[0][1] = 0;
//  filterX[1][1] = 0;
//  filterX[2][1] = 0;

//  filterX[0][2] = 1;
//  filterX[1][2] = 2;
//  filterX[2][2] = 1;

//  vpMatrix filterY(3,3);
//  filterY[0][0] = -1;
//  filterY[0][1] = -2;
//  filterY[0][2] = -1;

//  filterY[1][0] = 0;
//  filterY[1][1] = 0;
//  filterY[1][2] = 0;

//  filterY[2][0] = 1;
//  filterY[2][1] = 2;
//  filterY[2][2] = 1;

  vpMatrix filterX(5,5);
  filterX[0][0] = -1;
  filterX[1][0] = -4;
  filterX[2][0] = -6;
  filterX[3][0] = -4;
  filterX[4][0] = -1;

  filterX[0][1] = -2;
  filterX[1][1] = -8;
  filterX[2][1] = -12;
  filterX[3][1] = -8;
  filterX[4][1] = -2;

  filterX[0][2] = 0;
  filterX[1][2] = 0;
  filterX[2][2] = 0;
  filterX[3][2] = 0;
  filterX[4][2] = 0;

  filterX[0][3] = 2;
  filterX[1][3] = 8;
  filterX[2][3] = 12;
  filterX[3][3] = 8;
  filterX[4][3] = 2;

  filterX[0][4] = 1;
  filterX[1][4] = 4;
  filterX[2][4] = 6;
  filterX[3][4] = 4;
  filterX[4][4] = 1;

  vpMatrix filterY(5,5);
  filterY[0][0] = -1;
  filterY[0][1] = -4;
  filterY[0][2] = -6;
  filterY[0][3] = -4;
  filterY[0][4] = -1;

  filterY[1][0] = -2;
  filterY[1][1] = -8;
  filterY[1][2] = -12;
  filterY[1][3] = -8;
  filterY[1][4] = -2;

  filterY[2][0] = 0;
  filterY[2][1] = 0;
  filterY[2][2] = 0;
  filterY[2][3] = 0;
  filterY[2][4] = 0;

  filterY[3][0] = 2;
  filterY[3][1] = 8;
  filterY[3][2] = 12;
  filterY[3][3] = 8;
  filterY[3][4] = 2;

  filterY[4][0] = 1;
  filterY[4][1] = 4;
  filterY[4][2] = 6;
  filterY[4][3] = 4;
  filterY[4][4] = 1;

  double offset = std::floor(filterX.getRows() / 2.0f);

  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ++it){
    if(iter != 0 && iter+1 != list.size()){
      double gradientX = 0;
      double gradientY = 0;

      double iSite = it->ifloat;
      double jSite = it->jfloat;

      for(unsigned int i = 0; i<filterX.getRows() ; i++){
		double iImg = iSite + (i - offset);
		for (unsigned int j = 0; j< filterX.getCols(); j++){
          double jImg = jSite + (j-offset);

          if(iImg < 0) iImg = 0.0;
          if(jImg < 0) jImg = 0.0;

          if(iImg > _I.getHeight()-1) iImg = _I.getHeight()-1;
          if(jImg > _I.getWidth()-1) jImg = _I.getWidth()-1;

		  gradientX += filterX[i][j] * _I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      for(unsigned int i = 0; i<filterY.getRows() ; i++){
		double iImg = iSite + (i - offset);
		for (unsigned int j = 0; j< filterY.getCols(); j++){
          double jImg = jSite + (j-offset);

          if(iImg < 0) iImg = 0.0;
          if(jImg < 0) jImg = 0.0;

          if(iImg > _I.getHeight()-1) iImg = _I.getHeight()-1;
          if(jImg > _I.getWidth()-1) jImg = _I.getWidth()-1;

		  gradientY += filterY[i][j] * _I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      double angle = atan2(gradientX,gradientY);
      while (angle<0) angle += M_PI;
      while (angle>M_PI) angle -= M_PI;

      vpColVector vecGrad(2);
      vecGrad[0] = cos(angle);
      vecGrad[1] = sin(angle);
      vecGrad.normalize();

      double angle1 = acos(vecLine * vecGrad);
      double angle2 = acos(vecLine * (-vecGrad));

//      double angle1 = sqrt(vpMath::sqr(deltaNormalized-angle));
//      double angle2 = sqrt(vpMath::sqr(deltaNormalized- (angle-M_PI)));

      _sumErrorRad += std::min(angle1,angle2);

//      if(std::fabs(deltaNormalized-angle) > M_PI / 2)
//      {
//        sumErrorRad += sqrt(vpMath::sqr(deltaNormalized-angle)) - M_PI / 2;
//      } else {
//        sumErrorRad += sqrt(vpMath::sqr(deltaNormalized-angle));
//      }

      _nbFeatures++;
    }
    iter++;
  }
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
  unsigned int n = numberOfSignal() ;

  if ((double)n<0.5*expecteddensity && n > 0)
  {
    double delta_new = delta;
    delta = delta_1;
    sample(I) ;
    expecteddensity = (double)list.size();
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
  size_t n = list.size();

  if ((double)n<0.5*expecteddensity /*&& n > 0*/) // n is always > 0
  {
    double delta_new = delta;
    delta = delta_1;
    PExt[0].ifloat = (float)ip1.get_i() ;
    PExt[0].jfloat = (float)ip1.get_j() ;
    PExt[1].ifloat = (float)ip2.get_i() ;
    PExt[1].jfloat = (float)ip2.get_j() ;
    sample(I) ;
    expecteddensity = (double)list.size();
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
  vpMeSite p_me ;

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
    p_me = *it;
    p_me.alpha = delta ;
    p_me.mask_sign = sign;
    *it = p_me;
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
  catch(vpException &e)
  {
    throw e;
  }

  // supression des points rejetes par les ME
  //  suppressPoints(I);
  //  setExtremities();
}


/*!
  Update the moving edges parameters after the virtual visual servoing.
  
  \param  I : The image.
  \param  rho_ : The \f$\rho\f$ parameter used in the line's polar equation.
  \param  theta_ : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, double rho_, double theta_)
{
  this->rho = rho_;
  this->theta = theta_;
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
  \param rho_ : The \f$\rho\f$ parameter used in the line's polar equation.
  \param theta_ : The \f$\theta\f$ parameter used in the line's polar equation.
*/
void
vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, vpImagePoint ip1, vpImagePoint ip2,
                              double rho_, double theta_)
{
  this->rho = rho_;
  this->theta = theta_;
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
  double i_min = +1e6 ;
  double j_min = +1e6;
  double i_max = -1 ;
  double j_max = -1 ;

  // Loop through list of sites to track
  for(std::list<vpMeSite>::const_iterator it=list.begin(); it!=list.end(); ++it){
    vpMeSite s = *it;//current reference pixel
    if (s.ifloat < i_min)
    {
      i_min = s.ifloat ;
      j_min = s.jfloat ;
    }

    if (s.ifloat > i_max)
    {
      i_max = s.ifloat ;
      j_max = s.jfloat ;
    }
  }

  if ( ! list.empty() )
  {
    PExt[0].ifloat = i_min ;
    PExt[0].jfloat = j_min ;
    PExt[1].ifloat = i_max ;
    PExt[1].jfloat = j_max ;
  }

  if (fabs(i_min-i_max) < 25)
  {
    for(std::list<vpMeSite>::const_iterator it=list.begin(); it!=list.end(); ++it){
      vpMeSite s = *it;//current reference pixel
      if (s.jfloat < j_min)
      {
        i_min = s.ifloat ;
        j_min = s.jfloat ;
      }

      if (s.jfloat > j_max)
      {
        i_max = s.ifloat ;
        j_max = s.jfloat ;
      }
    }

    if (! list.empty())
    {
      PExt[0].ifloat = i_min ;
      PExt[0].jfloat = j_min ;
      PExt[1].ifloat = i_max ;
      PExt[1].jfloat = j_max ;
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
vpMbtMeLine::findSignal(const vpImage<unsigned char>& I, const vpMe *p_me, double *conv)
{
  vpImagePoint itest(PExt[0].ifloat+(PExt[1].ifloat-PExt[0].ifloat)/2, PExt[0].jfloat+(PExt[1].jfloat-PExt[0].jfloat)/2);
  
  vpMeSite pix ; //= list.value();
  pix.init(itest.get_i(), itest.get_j(), delta, 0, sign);
  
  vpMeSite  *list_query_pixels;
//  double  convolution = 0;
  unsigned int range  = p_me->getRange();
  
  list_query_pixels = pix.getQueryList(I, (int)range);
  
  vpDisplay::displayCross(I,itest,5,vpColor::cyan,3);
  vpDisplay::displayLine(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),vpImagePoint(list_query_pixels[2*range].ifloat,list_query_pixels[2*range].jfloat),vpColor::cyan);
  vpDisplay::displayCross(I,vpImagePoint(list_query_pixels[0].ifloat,list_query_pixels[0].jfloat),5,vpColor::orange,3);

  for(unsigned int n = 0 ; n < 2 * range + 1 ; n++)
  {
    conv[n] = list_query_pixels[n].convolution(I, p_me);
  }
  delete [] list_query_pixels;
}

#endif

