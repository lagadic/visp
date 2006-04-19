
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
 *  $Id: vpFeatureEllipse.h,v 1.3 2006-04-19 09:01:23 fspindle Exp $
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
  vpFeatureEllipse(const double x, const double y,
		   const double mu20, const double mu11, const double mu02);

  // void buildFrom(const vpEllipse &p) ;
  void buildFrom(const double x, const double y,
		 const double mu20, const double mu11, const double mu02) ;
  void buildFrom(const double x, const double y,
		 const double mu20, const double mu11, const double mu02,
		 const double A, const double B, const double C) ;
  void setABC(const double A, const double B, const double C) ;
  void setMu(const double mu20, const double mu11, const double mu02) ;
  //@}

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  inline static int selectX()  { return FEATURE_LINE[0] ; }
  inline static int selectY()  { return FEATURE_LINE[1] ; }
  inline static int selectMu20()  { return FEATURE_LINE[2] ; }
  inline static int selectMu11()  { return FEATURE_LINE[3] ; }
  inline static int selectMu02()  { return FEATURE_LINE[4] ; }

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
