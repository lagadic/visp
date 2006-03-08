
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDot.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpDot.cpp,v 1.11 2006-03-08 15:51:24 fspindle Exp $
 *
 * Description
 * ============
 *   Track a white dot
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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

  seuil_min = 200 ;
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
  seuil = pt.seuil ;
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

  \return vpDot::out if an error occurs, vpDot::in otherwise.
*/
int
vpDot::connexe(vpImage<unsigned char>& I, int u, int v, int seuil,
	       double &u_cog, double &v_cog,  double &n)
{

  // Test if we are in the image
  if ( (u < 0) || (v < 0) || (u >= I.getRows()) || (v >= I.getCols()) ) {
    return  vpDot::out ;
  }
  if (I[v][u] >=seuil)
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
    if (I[v][u-1] >=seuil)
      connexe(I,u-1,v,seuil,u_cog,v_cog,n) ;
  }

  if (u+1 <  I.getCols())
  {
    if (I[v][u+1] >=seuil)
      connexe(I,u+1,v,seuil,u_cog,v_cog,n) ;
  }
  if  (v-1 >= 0)
  {
    if (I[v-1][u] >=seuil)
      connexe(I,u, v-1,seuil,u_cog,v_cog,n) ;
  }
  if  (v+1 < I.getRows())
  {
    if (I[v+1][u] >=seuil)
      connexe(I,u,v+1,seuil,u_cog,v_cog,n) ;
  }

  if (connexity == CONNEXITY_8) {
    if ( (u-1 >= 0) && (v-1 >= 0) )
    {

      if (I[v-1][u-1] >=seuil)
	connexe(I,u-1,v-1,seuil,u_cog,v_cog,n) ;
    }

    if ( (u+1 <  I.getCols()) && (v-1 >= 0 ) )
    {

      if (I[v-1][u+1] >=seuil)
	connexe(I,u+1,v-1,seuil,u_cog,v_cog,n) ;
    }
    if  ( (v+1 < I.getRows()) && (u-1 >= 0) )
    {

      if (I[v+1][u-1] >=seuil)
	connexe(I,u-1,v+1,seuil,u_cog,v_cog,n) ;
    }
    if  ( (v+1 < I.getRows()) && (u+1 < I.getCols()) )
    {

      if (I[v+1][u+1] >=seuil)
	connexe(I,u+1,v+1,seuil,u_cog,v_cog,n) ;
    }
  }



  return vpDot::in ;
}

void
vpDot::COG(vpImage<unsigned char> &I, double& u, double& v)
{
  // segmentation de l'image apres seuillage
  // (etiquetage des composante connexe)
  if (compute_moment)
    m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  double u_cog = 0 ;
  double v_cog = 0 ;
  double npoint =0 ;
  Lu.kill() ;
  Lv.kill() ;

#if 0
  // Original version
  if (  connexe(I,(int)u,(int)v,seuil,u_cog, v_cog, npoint) == vpDot::out)
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
	  if (connexe(I,
		      (int)(u+k*pas),(int)(v+l*pas),
		      seuil,u_cog, v_cog, npoint)     != vpDot::out)
	  {
	    sol = true ; u += k*pas ; v += l*pas ;
	  }
	}
    }
    if (sol == false)
    {
      ERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostError,
				"Dot has been lost")) ;
    }
  }
#else
  // If the dot is not found, search around using a spiral
  if (  connexe(I,(int)u,(int)v,seuil,u_cog, v_cog, npoint) == vpDot::out)
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

	if (connexe(I, (int)u_+k, (int)(v_), seuil, u_cog, v_cog, npoint)
	    != vpDot::out) {
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

	if (connexe(I, (int)(u_), (int)(v_+k), seuil, u_cog, v_cog, npoint)
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

	if (connexe(I, (int)(u_-k), (int)(v_), seuil, u_cog, v_cog, npoint)
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

	if (connexe(I, (int)(u_), (int)(v_-k), seuil, u_cog, v_cog, npoint)
	    != vpDot::out) {
	  sol = true ; u = u_; v = v_-k;
	}
      }
      v_ -= k;
      up += 2;
    }

    if (sol == false) {
      ERROR_TRACE("Dot has been lost") ;
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

  if (npoint < 5)
  {
    ERROR_TRACE("Dot has been lost") ;
    throw(vpTrackingException(vpTrackingException::featureLostError,
			      "Dot has been lost")) ;
  }
  if (npoint > nbMaxPoint)
  {
    ERROR_TRACE("Too many point %f (max allowed is %f)", npoint, nbMaxPoint) ;
    ERROR_TRACE("This threshold can be modified using the setNbMaxPoint(int) method") ;

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

  Wait a user click in a white area in the image. The clicked pixel
  will be the starting point from which the dot will be tracked.

  To get center of gravity of the dot, see get_u() and get_v(). To compute the
  moments see setComputeMoments().

  \sa track()
*/
void
vpDot::initTracking(vpImage<unsigned char>& I)
{
  int i1,j1;

  while (vpDisplay::getClick(I,i1,j1)!=true) ;

  seuil = (int) (I[i1][j1] * 0.8);
  seuil_min = (int) (I[i1][j1] * 0.6);
  if (seuil <seuil_min) seuil = seuil_min ;


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
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!

  Initialize the tracking for a dot supposed to be located at (u,v) and
  update the dot characteristics (center of gravity, moments).

  \param I : image
  \param u : dot location (column)
  \param v : dot location (row)

  To get center of gravity of the dot, see get_u() and get_v(). To compute the
  moments see setComputeMoments().

*/
void
vpDot::initTracking(vpImage<unsigned char>& I, int u, int v)
{

  cog_ufloat = u ;
  cog_vfloat = v ;

  cog_u = u ;
  cog_v = v ;

  try {
    track( I );
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  track and get the new dot coordinates

  \warning the image is modified (all the pixel that belong to the point
  are set to black.

*/
void
vpDot::track(vpImage<unsigned char> &I)
{
  seuil = (int) (I[cog_v][cog_u] * 0.8);
  seuil_min = (int) (I[cog_v][cog_u] * 0.6);
  if (seuil < seuil_min) seuil = seuil_min ;

  double u = cog_ufloat ;
  double v = cog_vfloat ;

  try{

    COG(I,u,v) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  cog_ufloat = u ;
  cog_vfloat = v ;

  if ((u-(int)u) < 0.5)   cog_u = (int)u ; else  cog_u = (int)u+1 ;
  if ((v-(int)v) < 0.5)   cog_v = (int)v ; else  cog_v = (int)v+1 ;
}

/*!
  track and get the new dot coordinates

  \param I : image
  \param u : dot location (column)
  \param v : dot location (row)
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


