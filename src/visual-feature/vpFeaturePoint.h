
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoint.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeaturePoint.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeaturePoint_H
#define vpFeaturePoint_H

/*!
  \file vpFeaturePoint.h
  \brief Class that defines 2D point visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpPoint.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeaturePoint
  \brief Class that defines 2D point visual feature
*/
class vpFeaturePoint : public vpBasicFeature
{
public:
  enum pointFeatureEnum
    {
      X,   // x coordinates
      Y    // y coordinates
    };
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar usefull but not mandatory
  */
private:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z ;

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeaturePoint() ;
  //! destructor
  ~vpFeaturePoint() { ; }

public:
  /*
    /section Set coordinates
  */

  //@{

  //! set the point depth (camera frame)
  void set_Z(const double _Z) ;
  //! get the point depth (camera frame)
  double get_Z() const  ;

  //! set the point x-coordinates
  void set_x(const double _x) ;
  //! get the point x-coordinates
  double get_x()  const ;
  //! set the point y-coordinates
  void set_y(const double _y) ;
  //! get the point y-coordinates
  double get_y()   const ;
  //! set the point xy and Z-coordinates
  void set_xyZ(const double _x, const double _y, const double _Z) ;

  // void buildFrom(const vpPoint &p) ;
  void buildFrom(const double _x, const double _y, const double _Z) ;

  //@}

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  static int selectX()  { return FEATURE_LINE1 ; }
  static int selectY()  { return FEATURE_LINE2 ; }
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  //! compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;
  //! print the name of the feature
  void print(const int select = FEATURE_ALL ) const ;

  //! feature duplication
  vpFeaturePoint *duplicate() const ;


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
