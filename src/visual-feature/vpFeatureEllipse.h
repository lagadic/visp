
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpEllipse.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureEllipse.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeatureEllipse_H
#define vpFeatureEllipse_H

/*!
  \file vpFeatureEllipse.h
  \brief Class that defines 2D ellipse visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureEllipse
  \brief Class that defines 2D ellipse visual feature
*/
class vpFeatureEllipse : public vpBasicFeature
{
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar usefull but not mandatory
  */
private:
  //! FeatureEllipse depth (required to compute the interaction matrix)
  //! default Z = 1m
  double A,B,C ;


public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeatureEllipse() ;
  //! destructor
  ~vpFeatureEllipse() { ; }

public:
  /*
    /section Set coordinates
  */
 //! basic constructor
  vpFeatureEllipse(const double _x, const double _y,
		   const double _mu20, const double _mu11, const double _mu02);

  // void buildFrom(const vpEllipse &p) ;
  void buildFrom(const double _x, const double _y,
		 const double _mu20, const double _mu11, const double _mu02) ;
  void buildFrom(const double _x, const double _y,
		 const double _mu20, const double _mu11, const double _mu02,
		 const double _A, const double _B, const double _C) ;
  void setABC(const double _A, const double _B, const double _C) ;
  void setMu(const double _mu20, const double _mu11, const double _mu02) ;
  //@}

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  static int selectX()  { return FEATURE_LINE1 ; }
  static int selectY()  { return FEATURE_LINE2 ; }
  static int selectMu20()  { return FEATURE_LINE3 ; }
  static int selectMu11()  { return FEATURE_LINE4 ; }
  static int selectMu02()  { return FEATURE_LINE5 ; }

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
  vpFeatureEllipse *duplicate() const ;


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
