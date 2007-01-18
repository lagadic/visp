/****************************************************************************
 *
 * $Id: vpDot.cpp,v 1.15 2007-01-18 16:02:07 fspindle Exp $
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
 * Track a white dot.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*
  \file vpDot.cpp
  \brief Track a white dot
*/

#include <visp/vpDot.h>

#include <visp/vpDisplay.h>
#include <visp/vpColor.h>

// exception handling
#include <visp/vpTrackingException.h>

/*
  \class vpDot
  \brief Track a white dot
*/

const int vpDot::SPIRAL_SEARCH_SIZE = 50; /* spiral size for the dot search */

/*!

  Initialize the tracker with default parameters.
  - connexity is set to 4 (see setConnexity())
  - dot maximal number of pixels is set to 10000 (see setNbMaxPoint())
*/
void vpDot::init()
{
  cog_u = 0 ;
  cog_v = 0 ;

  cog_ufloat = 0 ;
  cog_vfloat = 0 ;

  threshold_min = 200 ;
  compute_moment = false ;
  graphics = false ;
  nbMaxPoint = 10000 ;

  m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  connexity = CONNEXITY_4;
}

vpDot::vpDot() : vpTracker()
{
  init() ;
}

/*!
  \brief constructor with initialization of the dot location

  \param u : dot location (column)
  \param v : dot location (row)
 */
vpDot::vpDot(const int u, const int v) : vpTracker()
{
  init() ;

  cog_u = u ;
  cog_v = v ;

  cog_ufloat = u ;
  cog_vfloat = v ;

}

/*!
  \brief constructor with initialization of the dot location

  \param u : dot location (column)
  \param v : dot location (row)
 */
vpDot::vpDot(const double u,const  double v) : vpTracker()
{

  init() ;

  cog_u = (int)u ;
  cog_v = (int)v ;

  cog_ufloat = u ;
  cog_vfloat = v ;

}

/*!
  \brief copy constructor
 */
vpDot::vpDot(const vpDot& c)  : vpTracker()
{

  *this = c ;

}


/*!
  \brief destructor
 */
vpDot::~vpDot()
{

  Lu.kill() ;
  Lv.kill() ;

}

/*!
  \brief copy operator
 */
vpDot&
vpDot::operator=(const vpDot& pt)
{

  cog_u = pt.cog_u ;
  cog_v = pt.cog_v ;

  cog_ufloat = pt.cog_ufloat ;
  cog_vfloat = pt.cog_vfloat ;

  graphics = pt.graphics ;
  threshold = pt.threshold ;
  compute_moment = pt.compute_moment ;

  m00 = pt.m00;
  m01 = pt.m01;
  m10 = pt.m10;
  m02 = pt.m02;
  m20 = pt.m20;

  return *this ;
}

int
vpDot::operator!=(const vpDot& m)
{
  return ((cog_u!=m.cog_v) || (cog_v!=m.cog_v)) ;
}

int
vpDot::operator==(const vpDot& m)
{
  return ((cog_u==m.cog_u) && (cog_v==m.cog_v)) ;
}

/*!
  Perform the tracking of a dot by connex components.

  \param mean_value : Threshold to use for the next call to track()
  and corresponding to the mean value of the dot intensity.

  \warning The content of the image is modified.

  \return vpDot::out if an error occurs, vpDot::in otherwise.
*/
int
vpDot::connexe(vpImage<unsigned char>& I, int u, int v, int threshold,
	       double &mean_value, double &u_cog, double &v_cog, double &n)
{

  // Test if we are in the image
  if ( (u < 0) || (v < 0) || (u >= I.getCols()) || (v >= I.getRows()) ) {
    return  vpDot::out ;
  }
  if (I[v][u] >= threshold)
  {
    if (graphics==true)
    {
      vpDisplay::displayPoint(I,v,u,vpColor::green) ;
    }
    Lu += u ;
    Lv += v ;
    u_cog += u ;
    v_cog += v ;
    n+=1 ;
    // Mean value of the dot intensities
    mean_value = (mean_value *(n-1) + I[v][u]) / n;
    if (compute_moment==true)
    {
      m00++ ;
      m10 += u ;
      m01 += v ;
      m11 += (u*v) ;
      m20 += u*u ;
      m02 += v*v ;
    }
    I[v][u] = 0 ;
  }
  else
  {
    return vpDot::out ;
  }
  if ( u-1 >= 0)
  {
    if (I[v][u-1] >= threshold)
      connexe(I,u-1,v, threshold, mean_value, u_cog,v_cog, n) ;
  }

  if (u+1 <  I.getCols())
  {
    if (I[v][u+1] >= threshold)
      connexe(I,u+1,v,threshold, mean_value, u_cog, v_cog, n) ;
  }
  if  (v-1 >= 0)
  {
    if (I[v-1][u] >=threshold)
      connexe(I,u, v-1,threshold, mean_value, u_cog, v_cog, n) ;
  }
  if  (v+1 < I.getRows())
  {
    if (I[v+1][u] >=threshold)
      connexe(I,u,v+1,threshold, mean_value, u_cog, v_cog, n) ;
  }

  if (connexity == CONNEXITY_8) {
    if ( (u-1 >= 0) && (v-1 >= 0) )
    {

      if (I[v-1][u-1] >=threshold)
	connexe(I,u-1,v-1,threshold, mean_value, u_cog, v_cog, n) ;
    }

    if ( (u+1 <  I.getCols()) && (v-1 >= 0 ) )
    {

      if (I[v-1][u+1] >=threshold)
	connexe(I,u+1,v-1,threshold, mean_value, u_cog, v_cog, n) ;
    }
    if  ( (v+1 < I.getRows()) && (u-1 >= 0) )
    {

      if (I[v+1][u-1] >=threshold)
	connexe(I,u-1,v+1,threshold, mean_value, u_cog, v_cog, n) ;
    }
    if  ( (v+1 < I.getRows()) && (u+1 < I.getCols()) )
    {

      if (I[v+1][u+1] >=threshold)
	connexe(I,u+1,v+1,threshold, mean_value, u_cog, v_cog, n) ;
    }
  }

  return vpDot::in ;
}

/*!

  Compute the center of gravity (COG) of the dot using connex
  components.  We assume the origin pixel (u, v) is in the dot. If
  not, the dot is seach arround this origin using a spiral search.

  \param I : Image to process.
  \param u : Starting pixel coordinate along the columns from where the
  dot is searched .

  \param v : Starting pixel coordinate along the rows from where the
  dot is searched .

  \warning The content of the image is modified.

  \sa connexe()
*/
void
vpDot::COG(vpImage<unsigned char> &I, double& u, double& v)
{
  // segmentation de l'image apres seuillage
  // (etiquetage des composante connexe)
  if (compute_moment)
    m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  double u_cog = 0 ;
  double v_cog = 0 ;
  double npoint = 0 ;
  double mean_value = 0 ;
  Lu.kill() ;
  Lv.kill() ;

#if 0
  // Original version
  if (  connexe(I, (int)u, (int)v, threshold, mean_value,
		u_cog, v_cog, npoint) == vpDot::out)
  {
    bool sol = false ;
    int pas  ;
    for (pas = 2 ; pas <= 25 ; pas ++ )if (sol==false)
    {
      for (int k=-1 ; k <=1 ; k++) if (sol==false)
	for (int l=-1 ; l <=1 ; l++) if (sol==false)
	{
	  u_cog = 0 ;
	  v_cog = 0 ;
	  Lu.kill() ;
	  Lv.kill() ;
	  mean_value = 0;
	  if (connexe(I, (int)(u+k*pas),(int)(v+l*pas), threshold,
		      mean_value,u_cog, v_cog, npoint) != vpDot::out)
	  {
	    sol = true ; u += k*pas ; v += l*pas ;
	  }
	}
    }
    if (sol == false)
    {
      vpERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"Dot has been lost")) ;
    }
  }
#else
  // If the dot is not found, search around using a spiral
  if (  connexe(I,(int)u,(int)v, threshold, mean_value,
		u_cog, v_cog, npoint) == vpDot::out)
  {

    bool sol = false ;

    int right = 1;
    int botom = 1;
    int left = 2;
    int up = 2;
    double u_ = u, v_ = v;
    int k;

    // balayage en spiral à partir du centre pour trouver le point le plus proche

    while( (right < SPIRAL_SEARCH_SIZE) && (sol == false) ) {
      for (k=1; k <= right; k++) if(sol==false) {
	u_cog = 0 ;
	v_cog = 0 ;
	Lu.kill() ;
	Lv.kill() ;
	mean_value = 0;
	if (connexe(I, (int)u_+k, (int)(v_), threshold, mean_value,
		    u_cog, v_cog, npoint) != vpDot::out) {
	  sol = true; u = u_+k; v = v_;
	}
      }
      u_ += k;
      right += 2;

      for (k=1; k <= botom; k++) if (sol==false) {
	u_cog = 0 ;
	v_cog = 0 ;
	Lu.kill() ;
	Lv.kill() ;
	mean_value = 0;

	if (connexe(I, (int)(u_), (int)(v_+k), threshold, mean_value,
		    u_cog, v_cog, npoint)
	    != vpDot::out) {
	  sol = true; u = u_; v = v_+k;
	}
      }
      v_ += k;
      botom += 2;

      for (k=1; k <= left; k++) if (sol==false) {
	u_cog = 0 ;
	v_cog = 0 ;
	Lu.kill() ;
	Lv.kill() ;
	mean_value = 0;

	if (connexe(I, (int)(u_-k), (int)(v_), threshold, mean_value,
		    u_cog, v_cog, npoint)
	    != vpDot::out) {
	  sol = true ; u = u_-k; v = v_;
	}
      }
      u_ -= k;
      left += 2;

      for (k=1; k <= up; k++) if(sol==false) {
	u_cog = 0 ;
	v_cog = 0 ;
	Lu.kill() ;
	Lv.kill() ;
	mean_value = 0;

	if (connexe(I, (int)(u_), (int)(v_-k), threshold, mean_value,
		    u_cog, v_cog, npoint)
	    != vpDot::out) {
	  sol = true ; u = u_; v = v_-k;
	}
      }
      v_ -= k;
      up += 2;
    }

    if (sol == false) {
      vpERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"Dot has been lost")) ;
    }
  }

#endif
  Lu.front() ; Lv.front() ;
  while (!Lu.outside())
  {
    int u,v ;
    u = Lu.value() ; v = Lv.value() ;
    I[v][u] = 255 ;
    Lu.next() ;
    Lv.next() ;
  }

  u_cog = u_cog/npoint ;
  v_cog = v_cog/npoint ;


  u = u_cog ;
  v = v_cog ;

  // Initialize the threshold for the next call to track()
  threshold = (int) (mean_value * 0.8);
  threshold_min = (int) ( mean_value * 0.6);

  //vpCTRACE << "threshold: " << threshold << endl;

  if (npoint < 5)
  {
    vpERROR_TRACE("Dot has been lost") ;
    throw(vpTrackingException(vpTrackingException::featureLostError,
			      "Dot has been lost")) ;
  }
  if (npoint > nbMaxPoint)
  {
    vpERROR_TRACE("Too many point %f (max allowed is %f)", npoint, nbMaxPoint) ;
    vpERROR_TRACE("This threshold can be modified using the setNbMaxPoint(int) method") ;

    throw(vpTrackingException(vpTrackingException::featureLostError,
			      "Dot has been lost")) ;
  }
}


void
vpDot::setNbMaxPoint(double nb)
{
  nbMaxPoint = nb ;
}
/*!

  Initialize the tracking with a mouse click and update the dot
  characteristics (center of gravity, moments).

  Wait a user click in a white area in the image I. The clicked pixel
  will be the starting point from which the dot will be tracked.

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use get_u() and get_v(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \warning The content of the image modified since we call track() to
  compute the dot characteristics.

  \param I : Image to process.

  \sa track(), get_u(), get_v()
*/
void
vpDot::initTracking(vpImage<unsigned char>& I)
{
  int i1,j1;

  while (vpDisplay::getClick(I,i1,j1)!=true) ;

  threshold = (int) (I[i1][j1] * 0.8);
  threshold_min = (int) (I[i1][j1] * 0.6);

  double u,v ;
  u = j1 ;
  v = i1 ;

  cog_ufloat = u ;
  cog_vfloat = v ;

  if ((u-(int)u) < 0.5)   cog_u = (int)u ; else  cog_u = (int)u+1 ;
  if ((v-(int)v) < 0.5)   cog_v = (int)v ; else  cog_v = (int)v+1 ;

  try {
    track( I );
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and
  updates the dot characteristics (center of gravity, moments).

  The sub pixel coordinates of the dot are updated. To get the center
  of gravity coordinates of the dot, use get_u() and get_v(). To
  compute the moments use setComputeMoments(true) before a call to
  initTracking().

  \warning The content of the image modified since we call track() to
  compute the dot characteristics.

  \param I : Image to process.

  \param u : Dot location or starting point (column pixel coordinate)
  from which the dot will be tracked in the image.

  \param v : Dot location or starting point (row pixel coordinate)
  from which the dot will be tracked in the image.

  \sa track(), get_u(), get_v()
*/
void
vpDot::initTracking(vpImage<unsigned char>& I, int u, int v)
{

  cog_ufloat = u ;
  cog_vfloat = v ;

  cog_u = u ;
  cog_v = v ;

  threshold = (int) (I[cog_v][cog_u] * 0.8);
  threshold_min = (int) (I[cog_v][cog_u] * 0.6);

  try {
    track( I );
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  Track and compute the dot characteristics.

  To get the center of gravity coordinates of the dot, use get_u() and
  get_v(). To compute the moments use setComputeMoments(true) before a
  call to initTracking().

  \warning The image is modified (all the pixels that belong to the point
  are set to white (ie to 255).

  \param I : Image to process.

  \sa get_u(), get_v()
*/
void
vpDot::track(vpImage<unsigned char> &I)
{
  double u = cog_ufloat ;
  double v = cog_vfloat ;

  try{

    COG(I,u,v) ;
  }
  catch(...)
  {
    vpERROR_TRACE(" ") ;
    throw ;
  }

  cog_ufloat = u ;
  cog_vfloat = v ;

  if ((u-(int)u) < 0.5)   cog_u = (int)u ; else  cog_u = (int)u+1 ;
  if ((v-(int)v) < 0.5)   cog_v = (int)v ; else  cog_v = (int)v+1 ;
}

/*!
  Track and updates the new dot coordinates

  To compute the moments use setComputeMoments(true) before a call to
  initTracking() or track().

  \warning The image is modified (all the pixels that belong to the point
  are set to white (ie to 255).

  \param I : Image to process.

  \param u : Sub pixel coordinate (along the columns) of the tracked dot.
  \param v : Sub pixel coordinate (along the rows) of the tracked dot.
*/
void
vpDot::track(vpImage<unsigned char> &I, double &u, double &v)
{
  track(I) ;
  u = vpDot::get_u() ;
  v = vpDot::get_v() ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


