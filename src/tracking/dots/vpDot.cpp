
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
 *  $Id: vpDot.cpp,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
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


void vpDot::init()
{
  cog_i = 0 ;
  cog_j = 0 ;

  cog_ifloat = 0 ;
  cog_jfloat = 0 ;

  seuil_min = 200 ;
  compute_moment = false ;
  graphics = false ;
}

vpDot::vpDot() : vpTracker()
{
  init() ;
}

vpDot::vpDot(const int i, const int j) : vpTracker()
{
  init() ;

  cog_i = i ;
  cog_j = j ;

  cog_ifloat = i ;
  cog_jfloat = j ;

}


vpDot::vpDot(const double i,const  double j) : vpTracker()
{

  init() ;

  cog_i = (int)i ;
  cog_j = (int)j ;

  cog_ifloat = i ;
  cog_jfloat = j ;

}

vpDot::vpDot(const vpDot& c)  : vpTracker()
{

  *this = c ;

}



vpDot::~vpDot()
{

  Li.kill() ;
  Lj.kill() ;

}


void vpDot::setComputeMoments(const bool c)
{
  compute_moment = c ;
}


vpDot&
vpDot::operator=(const vpDot& pt)
{

  cog_i = pt.cog_i ;
  cog_j = pt.cog_j ;

  cog_ifloat = pt.cog_ifloat ;
  cog_jfloat = pt.cog_jfloat ;

  graphics = pt.graphics ;
  seuil = pt.seuil ;
  compute_moment =  pt. compute_moment ;

  return *this ;
}

int
vpDot::operator!=(const vpDot& m)
{
  return ((cog_i!=m.cog_j) || (cog_j!=m.cog_j)) ;
}

int
vpDot::operator==(const vpDot& m)
{
  return ((cog_i==m.cog_i) && (cog_j==m.cog_j)) ;
}

int
vpDot::connexe(vpImage<unsigned char>& I, int i, int j, int seuil,
	       double &i_cog, double &j_cog,  int &n)
{

  if (I[i][j] >=seuil)
  {
    if (graphics==true)
    {
      vpDisplay::displayPoint(I,i,j,vpColor::green) ;
    }
    Li += i ;
    Lj += j ;
    i_cog += i ;
    j_cog += j ;
    n++ ;
    if (compute_moment==true)
    {
      m00++ ;
      m10 += j ;
      m01 += i ;
      m11 += (i*j) ;
      m20 += j*j ;
      m02 += i*i ;
    }
    I[i][j] = 0 ;
  }
  else
  {
    return vpDot::out ;
  }
  if ( j-1 >= 0)
  {
    if (I[i][j-1] >=seuil)
      connexe(I,i,j-1,seuil,i_cog,j_cog,n) ;
  }

  if (j+1 <  I.getCols())
  {
    if (I[i][j+1] >=seuil)
      connexe(I,i,j+1,seuil,i_cog,j_cog,n) ;
  }
  if  (i-1 >= 0)
  {
    if (I[i-1][j] >=seuil)
      connexe(I,i-1,j,seuil,i_cog,j_cog,n) ;
  }
  if  (i+1 < I.getRows())
  {
    if (I[i+1][j] >=seuil)
      connexe(I,i+1,j,seuil,i_cog,j_cog,n) ;
  }
  return vpDot::in ;
}

void
vpDot::COG(vpImage<unsigned char> &I, double& i, double& j)
{
  // segmentation de l'image apres seuillage
  // (etiquetage des composante connexe)
  if (compute_moment)
    m00 = m11 = m02 = m20 = m10 = m01 = 0 ;

  double i_cog = 0 ;
  double j_cog = 0 ;
  int npoint =0 ;
  Li.kill() ;
  Lj.kill() ;


  if (  connexe(I,(int)i,(int)j,seuil,i_cog, j_cog, npoint) == vpDot::out)
  {
    bool sol = false ;
    int pas  ;
    for (pas = 2 ; pas <= 25 ; pas ++ )if (sol==false)
    {
      for (int k=-1 ; k <=1 ; k++) if (sol==false)
	for (int l=-1 ; l <=1 ; l++) if (sol==false)
	{
	  i_cog = 0 ;
	  j_cog = 0 ;
	  Li.kill() ;
	  Lj.kill() ;
	  if (connexe(I,
		      (int)(i+k*pas),(int)(j+l*pas),
		      seuil,i_cog, j_cog, npoint)     != vpDot::out)
	  {
	    sol = true ; i += k*pas ; j += l*pas ;
	  }
	}
    }
    if (sol == false)
    {
      ERROR_TRACE("Dot has been lost") ;
      throw(vpTrackingException(vpTrackingException::featureLostERR,
				"Dot has been lost")) ;
    }
  }

  Li.front() ; Lj.front() ;
  while (!Li.outside())
  {
    int i,j ;
    i = Li.value() ; j = Lj.value() ;
    I[i][j] = 255 ;
    Li.next() ;
    Lj.next() ;
  }

  i_cog /= npoint ;
  j_cog /= npoint ;


  i = i_cog ;
  j = j_cog ;

  if (npoint < 5)
  {
    ERROR_TRACE("Dot has been lost") ;
    throw(vpTrackingException(vpTrackingException::featureLostERR,
			      "Dot has been lost")) ;
  }
}




//! init the traking with a mouse click
void
vpDot::initTracking(vpImage<unsigned char>& I)
{
  int i1,j1;

  while (vpDisplay::getClick(I,i1,j1)!=true) ;

  seuil = (int) (I[i1][j1] * 0.8);
  seuil_min = (int) (I[i1][j1] * 0.6);
  if (seuil <seuil_min) seuil = seuil_min ;


  double i,j ;
  i = i1 ;
  j = j1 ;

  cog_ifloat = i ;
  cog_jfloat = j ;

  if ((i-(int)i) < 0.5)   cog_i = (int)i ; else  cog_i = (int)i+1 ;
  if ((j-(int)j) < 0.5)   cog_j = (int)j ; else  cog_j = (int)j+1 ;

}

  //! init the tracking for a dot supposed to be located at (i,j)
void
vpDot::initTracking(vpImage<unsigned char>& I, int i, int j)
{

  cog_ifloat = i ;
  cog_jfloat = j ;

  cog_i = i ;
  cog_j = j ;

}


/*!
  track and get the new dot coordinates
*/
void
vpDot::track(vpImage<unsigned char> &I)
{
  seuil = (int) (I[cog_i][cog_j] * 0.8);
  seuil_min = (int) (I[cog_i][cog_j] * 0.6);
  if (seuil < seuil_min) seuil = seuil_min ;

  double i = cog_ifloat ;
  double j = cog_jfloat ;

  try{

    COG(I,i, j) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  cog_ifloat = i ;
  cog_jfloat = j ;

  if ((i-(int)i) < 0.5)   cog_i = (int)i ; else  cog_i = (int)i+1 ;
  if ((j-(int)j) < 0.5)   cog_j = (int)j ; else  cog_j = (int)j+1 ;
}

/*!
  track and get the new dot coordinates
*/
void
vpDot::track(vpImage<unsigned char> &I, double &i, double &j)
{
  track(I) ;
  i = vpDot::I() ;
  j = vpDot::J() ;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


