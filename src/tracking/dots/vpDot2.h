
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpDot.h
 * Project:   ViSP2
 * Author:    Fabien Spindler
 *
 * Version control
 * ===============
 *
 *  $Id: vpDot2.h,v 1.8 2006-03-08 15:49:13 fspindle Exp $
 *
 * Description
 * ============
 *   Track a white dot
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*
  \file vpDot2.h
  \brief This tracker is meant to track some white zones on a vpImage.
*/

#ifndef vpDot2_hh
#define vpDot2_hh

#include <visp/vpList.h>

#include <visp/vpImage.h>

#include <visp/vpTracker.h>

class vpDot2 : public vpTracker
{
public:
  vpDot2();
  vpDot2( int u, int v );
  vpDot2( const vpDot2& twinDot );
  ~vpDot2();
  void operator=( const vpDot2& twinDot );

  void initTracking(vpImage<unsigned char>& I);
  void initTracking(vpImage<unsigned char>& I, int u, int v);

  void track(vpImage<unsigned char> &I);
  void track(vpImage<unsigned char> &I, double &u, double &v);

  double get_u() const;
  double get_v() const;
  double getWidth() const;
  double getHeight() const;
  double getSurface() const;
  virtual int getInLevel() const;
  virtual int getOutLevel() const;
  double getAccuracy() const;

   /*!
    Activates the display of the border of the dot during the tracking.

    \param activate If true, the border of the dot will be painted. false to
    turn off border painting.

  */
  void setGraphics(const bool activate) { graphics = activate ; }
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
  void set_u( const double & u );
  void set_v( const double & v );
  void setWidth( const double & width );
  void setHeight( const double & height );
  void setSurface( const double & surface );
  virtual void setInLevel( const int & inLevel );
  virtual void setOutLevel( const int & outLevel );
  void setAccuracy( const double & accuracy );

  double getDistance( const vpDot2& distantDot ) const;


  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I,
				    int area_u, int area_v,
				    int area_w, int area_h );

  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I );


  virtual bool isValid(vpImage<unsigned char>& I, const vpDot2& wantedDot);

  virtual bool hasGoodLevel(const vpImage<unsigned char>& I,
			    const int &u, const int &v) const;
  virtual bool hasReverseLevel(vpImage<unsigned char>& I,
			       const int &u, const int &v) const;

  virtual vpDot2* getInstance();

  //! minumum level for the dot, pixel with lower level
  //! don't belong to this dot.
  static const int MIN_IN_LEVEL;// = 70;

  //! Default level for the pixels inside the dot
  static const int DEFAULT_IN_LEVEL;// = 220;

  //! Default level for the pixels surrounding the dot
  static const int DEFAULT_OUT_LEVEL;// = 140;

public:
  friend ostream& operator<< (ostream& os, vpDot2& p) { return (os << p ) ; } ;
  void print(ostream& os) { os << *this << endl ; }

public :
  double m00; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{00} \f$ is a zero order moment obtained
		with \f$i = j = 0 \f$. This moment corresponds to the dot
		surface.

		\sa setComputeMoments()
	      */
  double m10; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{10} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 0 \f$. \f$ m_{10} \f$
		corresponds to the inertia first order moment along the v axis.

		\sa setComputeMoments()
	      */
  double m01; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{01} \f$ is a first order moment
		obtained with \f$i = 0 \f$ and \f$ j = 1 \f$. \f$ m_{01} \f$
		corresponds to the inertia first order moment along the u axis.

		\sa setComputeMoments()
	      */
  double m11; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{11} \f$ is a first order moment
		obtained with \f$i = 1 \f$ and \f$ j = 1 \f$.

		\sa setComputeMoments()
	      */
  double m20; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{20} \f$ is a second order moment
		obtained with \f$i = 2 \f$ and \f$ j = 0 \f$. \f$ m_{20} \f$
		corresponds to the inertia second order moment along the v
		axis.

		\sa setComputeMoments()
	      */
  double m02; /*!< Considering the general distribution moments for \f$ N \f$
		points defined by the relation \f$ m_{ij} = \sum_{h=0}^{N}
		u_h^i v_h^j \f$, \f$ m_{02} \f$ is a second order moment
		obtained with \f$i = 0 \f$ and \f$ j = 2 \f$. \f$ m_{02} \f$
		corresponds to the inertia second order moment along the u
		axis.

		\sa setComputeMoments()
	      */

protected:
  vpList<int> getListFreemanElement() ;
  vpList<int> getList_u();
  vpList<int> getList_v() ;

private:
  typedef struct {
    int u_min;
    int u_max;
    int v_min;
    int v_max;
    int w; // u_max - u_min
    int h; // v_max - v_min
    double cog_u;
    double cog_v;
  } vpAreaType;


  bool computeParameters( vpImage<unsigned char> &I,
			  const double &u = -1.0,
			  const double &v = -1.0);

  bool computeFreemanChainElement(const vpImage<unsigned char> &I,
				  const int &u, const int &v, int &element);
  void computeFreemanParameters(const vpImage<unsigned char> &I,
				const int &u_p, const int &v_p, int &element,
				int &du, int &dv, float &dS,
				float &dMu, float &dMv,
				float &dMuv,
				float &dMu2, float &dMv2);
  void updateFreemanPosition       ( int& u, int& v, const int &dir );


  bool isInImage( vpImage<unsigned char> &I ) const;
  bool isInImage( vpImage<unsigned char> &I, const int &u, const int &v) const;

  bool isInArea(const vpImage<unsigned char> &I) const;
  bool isInArea(const vpImage<unsigned char> &I, const int &u, const int &v) const;

  void getGridSize( int &gridWidth, int &gridHeight );
  void setArea(vpImage<unsigned char> &I, int u, int v, int w, int h);
  void setArea(vpImage<unsigned char> &I);
  void setArea(const vpAreaType & a);

  //! coordinates (float) of the point center of gravity
  double cog_ufloat, cog_vfloat ;

  double width;
  double height;
  double surface;
  int inLevel;
  int outLevel;
  double accuracy;

  // Area where the dot is to search
  vpAreaType area;

  // other
  vpList<int> direction_list;
  vpList<int> u_list;
  vpList<int> v_list;

  // flag
  bool compute_moment ; // true moment are computed
  bool graphics ; // true for graphic overlay display
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

