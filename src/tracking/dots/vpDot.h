
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
 *  $Id: vpDot.h,v 1.7 2006-02-20 15:29:18 fspindle Exp $
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
public:
  /*! \enum ConnexityEnum
  Type of connexity 4, or 8.
  */
  typedef enum {
    CONNEXITY_4, /*!< For a given pixel 4 neighbors are considered (left,
		   right, up, down) */
    CONNEXITY_8 /*!< For a given pixel 8 neighbors are considered (left,
		  right, up, down, and the 4 pixels located on the diagonal) */
  } ConnexityEnum;

  static const int SPIRAL_SEARCH_SIZE; /*!< spiral size for the dot search */
private:
  //! internal use only
  vpList<int> Lu, Lv ;

  //! Type of connexity
  ConnexityEnum connexity;

  //! coordinates of the point center of gravity
  int cog_u, cog_v ;
  //! coordinates (float) of the point center of gravity
  double cog_ufloat, cog_vfloat ;


public :
  double m00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$.

		\sa setComputeMoments()
	      */
  double m01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$.

		\sa setComputeMoments()
	      */
  double m10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$.

		\sa setComputeMoments()
	      */
  double m11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
  double m20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
  double m02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^i \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$.

		\warning This moment is not computed for the moment.

		\sa setComputeMoments()
	      */
public:
  void init() ;
  vpDot() ;
  vpDot(const int u, const int v) ;
  vpDot(const double u, const double v) ;
  vpDot(const vpDot& c) ;
  ~vpDot() ;

public:
  vpDot& operator =(const vpDot& f) ;
  int  operator ==(const vpDot& m);
  int  operator !=(const vpDot& m);


public:
  /*!

  Return the "u" (column) coordinate of the center of the dot within the image
  it comes from.
  */
  double get_u() const { return cog_ufloat ; }
  /*!

  Return the "v" (row) coordinate of the center of the dot within the image it
  comes from.
  */

  double get_v() const { return cog_vfloat ; }
  /*!

  Return the list of the "u" coordinates (row) of all the pixels on the dot
  border.

  \param u_list The "u" coordinate of the pixels on the dot border. This list
  is update after a call to track().


  */
  void   get_u(vpList<int> & u_list) { u_list = Lu; };
  /*!

  Return the list of the "v" coordinates (column) of all the pixels on the dot
  border.

  \param v_list The "v" coordinate of the pixels on the dot border. This list
  is update after a call to track().

  */
  void   get_v(vpList<int> & v_list) { v_list = Lv; };

  void set_u(double u) { cog_ufloat = u ; cog_u = (int)u ; }
  void set_v(double v) { cog_vfloat = v ; cog_v = (int)v ; }

  /*!
    Set the type of connexity: 4 or 8.
  */
  void setConnexity(ConnexityEnum connexity) {this->connexity = connexity; };

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

    Computed moment are vpDot::m00, vpDot::m10, vpDot::m01, vpDot::m11,
    vpDot::m20, vpDot::m02.

    The coordinates of the region's centroid (u, v) can be computed from the
    moments by \f$u=\frac{m10}{m00}\f$ and  \f$v=\frac{m01}{m00}\f$.

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


