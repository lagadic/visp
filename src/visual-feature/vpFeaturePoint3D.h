
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpFeaturePoint3D.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeaturePoint3D.h,v 1.3 2006-04-19 09:01:23 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines 3D point visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeaturePoint3d_H
#define vpFeaturePoint3d_H

/*!
  \file vpFeaturePoint3D.h
  \brief class that defines 3D point visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpPoint.h>

#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpFeaturePoint3D
  \brief class that defines 3D point visual feature
*/
class vpFeaturePoint3D : public vpBasicFeature

{

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeaturePoint3D() ;
  //! destructor
  ~vpFeaturePoint3D() { ; }

public:
  /*
    /section Set coordinates
  */

  //@{

  //! set the point depth (camera frame)
  void set_Z(const double Z) ;
  //! get the point depth (camera frame)
  double get_Z() const  ;

  //! set the point X-coordinates
  void set_X(const double X) ;
  //! get the point X-coordinates
  double get_X()  const ;
  //! set the point Y-coordinates
  void set_Y(const double Y) ;
  //! get the point Y-coordinates
  double get_Y()   const ;
  //! set the point XY and Z-coordinates
  void set_XYZ(const double X, const double Y, const double Z) ;

  //! build feature from a point (vpPoint)
  void buildFrom(const vpPoint &p) ;
  //! set the point XY and Z-coordinates
  void buildFrom(const double X, const double Y, const double Z) ;
  //@}

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  inline static int selectX()  { return FEATURE_LINE[0] ; }
  inline static int selectY()  { return FEATURE_LINE[1] ; }
  inline static int selectZ()  { return FEATURE_LINE[2] ; }
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  //! compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;
  //! print the name of the feature
  void print(const int select ) const ;

  //! feature duplication
  vpFeaturePoint3D *duplicate() const ;

public:
  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       int color=vpColor::green) const ;
} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
