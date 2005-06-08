
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDot.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpDot.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *   Track a white dot
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*
  \file vpDot.h
  \brief Track a white dot
*/

#ifndef vpDot_hh
#define vpDot_hh

#include <math.h>
#include <fstream>

#include <visp/vpList.h>

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

#include <visp/vpTracker.h>

/*
  \class vpDot
  \brief Track a white dot
*/
class vpDot : public vpTracker
{
private:
  //! internal use only
  vpList<int> Li, Lj ;
private:

  //! coordinates of the point center of gravity
  int cog_i, cog_j ;
  //! coordinates (float) of the point center of gravity
  double cog_ifloat, cog_jfloat ;


public :
  //!  moments d'ordre 0,1 et 2 sum_i sum_j ( i^k,j^l ) k+l =0,1,2
  double m00,m01,m10,m20,m11,m02 ;

public:
  void init() ;
  vpDot() ;
  vpDot(const int a, const int b) ;
  vpDot(const double a, const double b) ;
  vpDot(const vpDot& c) ;
  ~vpDot() ;

public:

public:
  vpDot& operator =(const vpDot& f) ;
  int  operator ==(const vpDot& m);
  int  operator !=(const vpDot& m);


public:
  double I() const { return cog_ifloat ; }
  double J() const { return cog_jfloat ; }

  void setI(double i) { cog_ifloat = i ; cog_i = (int)i ; }
  void setJ(double j) { cog_jfloat = j ; cog_j = (int)j ; }

public:
  friend ostream& operator<< (ostream& os, vpDot& p) { return (os << p ) ; } ;
  void print(ostream& os) { os << *this << endl ; }

private:
  enum pixelInDot
    {
      in,
      out
    } ;

  int connexe(vpImage<unsigned char>& I, int i, int j, int seuil,
	       double &i_cog, double &j_cog,  int &n) ;
  void COG(vpImage<unsigned char> &I,double& i, double& j) ;

  int seuil ;
  int seuil_min ;
public:
  void setSeuil(int n) { seuil_min = n ; } ;
private:
  //! flag : true moment are computed
  bool compute_moment ;
public:
  //! to be used if you want to compute the moments
  void setComputeMoments(const bool c) ;
public:
  //! init the traking with a mouse click
  void initTracking(vpImage<unsigned char> &I) ;
  //! init the tracking for a dot supposed to be located at (i,j)
  void initTracking(vpImage<unsigned char> &I, int i, int j) ;
  //! track and get the new dot coordinates
  void track(vpImage<unsigned char> & I) ;
  //! track and get the new dot coordinates
  void track(vpImage<unsigned char> & I, double &i, double &j) ;

private:
  bool graphics ;
public:
  void setGraphics(const bool k) { graphics = k ; }

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


