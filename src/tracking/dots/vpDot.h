
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
 *  $Id: vpDot.h,v 1.4 2005-12-05 10:31:54 marchand Exp $
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

  \sa vpDotExample.cpp
*/
class vpDot : public vpTracker
{
private:
  //! internal use only
  vpList<int> Lu, Lv ;
private:

  //! coordinates of the point center of gravity
  int cog_u, cog_v ;
  //! coordinates (float) of the point center of gravity
  double cog_ufloat, cog_vfloat ;


public :
  double m00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$. */
  double m01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$. */
  double m10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$. */
  double m11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$. */
  double m20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$. */
  double m02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		x_h^i y_h^i \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$. */
public:
  void init() ;
  vpDot() ;
  vpDot(const int u, const int v) ;
  vpDot(const double u, const double v) ;
  vpDot(const vpDot& c) ;
  ~vpDot() ;

public:

public:
  vpDot& operator =(const vpDot& f) ;
  int  operator ==(const vpDot& m);
  int  operator !=(const vpDot& m);


public:
  double get_u() const { return cog_ufloat ; }
  double get_v() const { return cog_vfloat ; }

  void set_u(double u) { cog_ufloat = u ; cog_u = (int)u ; }
  void set_v(double v) { cog_vfloat = v ; cog_v = (int)v ; }

public:
  friend ostream& operator<< (ostream& os, vpDot& p) { return (os << p ) ; } ;
  void print(ostream& os) { os << *this << endl ; }

private:
  enum pixelInDot
    {
      in,
      out
    } ;
  double nbMaxPoint ;
  int connexe(vpImage<unsigned char>& I, int u, int v, int seuil,
	       double &u_cog, double &v_cog,  double &n) ;
  void COG(vpImage<unsigned char> &I,double& u, double& v) ;

  int seuil ;
  int seuil_min ;
public:
  void setSeuil(int n) { seuil_min = n ; } ;
private:
  //! flag : true moment are computed
  bool compute_moment ;
public:
  /*!

    Activates the dot's moments computation.

    \param activate true, if you want to compute the moments. If false, moments
    are not computed.

  */
  void setComputeMoments(const bool activate) { compute_moment = activate; }
public:
  //! init the traking with a mouse click
  void initTracking(vpImage<unsigned char> &I) ;
  //! init the tracking for a dot supposed to be located at (u,v)
  void initTracking(vpImage<unsigned char> &I, int u, int v) ;
  //! track and get the new dot coordinates
  void track(vpImage<unsigned char> & I) ;
  //! track and get the new dot coordinates
  void track(vpImage<unsigned char> & I, double &u, double &v) ;
  //! the maximum in pixel of a dot (default 5000 that is a radius of 40pixels)
  void setNbMaxPoint(double nb) ;

private:
  bool graphics ;
public:
  /*!
    Activates the display of all the pixels of the dot during the tracking.

    \param activate true to activate the display of dot pixels, false to turn
    off the display
  */
  void setGraphics(const bool activate) { graphics = activate ; }

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */


