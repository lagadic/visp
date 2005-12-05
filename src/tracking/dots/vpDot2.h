
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
 *  $Id: vpDot2.h,v 1.3 2005-12-05 16:23:45 fspindle Exp $
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
				    int corner_u, int corner_v,
				    int width, int height );

  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I );


  virtual bool isValid(vpImage<unsigned char>& I,
			const vpDot2& wantedDotParam );

  virtual bool hasGoodLevel(vpImage<unsigned char>& I,
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

protected:
  vpList<int> getList_directions() ;
  vpList<int> getList_u();
  vpList<int> getList_v() ;

private:

  bool computeParameters( vpImage<unsigned char> &I,
			  const double &u = -1.0,
			  const double &v = -1.0);

  void move( vpImage<unsigned char> &I, int& u, int& v, int& dir);

  void getLeftPixelPosition( int& u, int& v, const int &dir );
  void turnLeft            ( int& u, int& v, const int &dir );
  void turnRight           ( int& u, int& v, const int &dir );
  void getTop              ( int& u, int& v, const int &dir );

  bool isInImage( vpImage<unsigned char> &I, const int &border=10 ) const;
  bool isInImage( vpImage<unsigned char> &I,
		  const int &u, const int &v, const int &border=10 ) const;

  void getGridSize( int &gridWidth, int &gridHeight );

  //! coordinates (float) of the point center of gravity
  double cog_ufloat, cog_vfloat ;

  double width;
  double height;
  double surface;
  int inLevel;
  int outLevel;
  double accuracy;

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

