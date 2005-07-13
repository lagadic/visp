
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
 *  $Id: vpDot2.h,v 1.1 2005-07-13 10:38:49 fspindle Exp $
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
  vpDot2( int i, int j );
  vpDot2( const vpDot2& twinDot );
  ~vpDot2();
  void operator=( const vpDot2& twinDot );

  void track(vpImage<unsigned char> &I);

  double I() const;
  double J() const;
  double getWidth() const;
  double getHeight() const;
  double getSurface() const;
  virtual int getInLevel() const;
  virtual int getOutLevel() const;
  double getAccuracy() const;
  int getIterNumber() const;

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
  void setI( double iCoord );
  void setJ( double jCoord );
  void setWidth( double width );
  void setHeight( double height );
  void setSurface( double surface );
  virtual void setInLevel( int inLevel );
  virtual void setOutLevel( int outLevel );

  void setAccuracy( double accuracy );
  void setIterNumber(int iter);

  void initTracking(vpImage<unsigned char>& I);
  int getNumberOfInformation();
  vpColVector getState();

  double getDistance( const vpDot2& distantDot ) const;


  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I,
				    int areaX, int areaY,
				    int areaHeight, int areaWidth );

  vpList<vpDot2>* searchDotsInArea( vpImage<unsigned char>& I );


  virtual bool isValid( vpImage<unsigned char>& I,
			const vpDot2& wantedDotParam );

  virtual bool hasGoodLevel( vpImage<unsigned char>& I,
			     int iCoord,
			     int jCoord ) const;

  virtual bool hasReverseLevel( vpImage<unsigned char>& I,
				int iCoord,
				int jCoord ) const;

  virtual vpDot2* getInstance();



  //! max dot I speed, in pixels per milliseconds
  static const double MAX_DOT_I_SPEED;//= 1.;
  //! max dot J speed, in pixels per milliseconds
  static const double MAX_DOT_J_SPEED;// = 1.;
  //! max change of volume rate for the dot per milliseconds
  static const double MAX_VOLUME_CHANGE;// = 0.005;

  //! minumum level for the dot, pixel with lower level
  //! don't belong to this dot.
  static const int MIN_IN_LEVEL;// = 70;

  //! Default level for the pixels inside the dot
  static const int DEFAULT_IN_LEVEL;// = 220;

  //! Default level for the pixels surrounding the dot
  static const int DEFAULT_OUT_LEVEL;// = 140;

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
  vpList<int> getDirections() ;
  vpList<int> getICoords();
  vpList<int> getJCoords() ;

private:

  bool computeParameters( vpImage<unsigned char> &I,
			  double iEstimated = -1.,
			  double jEstimated = -1.);

  bool estimateCurrentPosition(double& estimatedI, double& estimatedJ);

  void move( vpImage<unsigned char> &I, int& posI, int& posJ, int& dir);

  void getLeftPixelPosition( int& posI, int& posJ, int dir );
  void turnLeft( int& posI, int& posJ, int dir );
  void turnRight( int& posI, int& posJ, int dir );
  void getTop( int& posI, int& posJ, int dir );

  bool isInImage( vpImage<unsigned char> &I, int border=10 ) const;
  bool isInImage( vpImage<unsigned char> &I, int i, int j, int border=10 ) const;

  void getGridSize( int &gridWidth, int &gridHeight );

  // dot parameters
  double i;
  double j;
  double width;
  double height;
  double surface;
  int inLevel;
  int outLevel;
  double accuracy;

  // attributes for dot parameters computation
  int iterNumber;

  double p_t1_i;
  double p_t1_j;
  double t0;
  double dt01;

  double v_t1_i;
  double v_t1_j;


  // other
  vpList<int> directions;
  vpList<int> iCoords;
  vpList<int> jCoords;

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

