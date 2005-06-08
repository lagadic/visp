
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpLine.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureLine.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *
 *Class that defines 2D line visual feature
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeatureLine_H
#define vpFeatureLine_H

/*!
  \file vpFeatureLine.h
  \brief Class that defines 2D line visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>


#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureLine
  \brief Class that defines 2D line visual feature
*/
class vpFeatureLine : public vpBasicFeature
{
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar usefull but not mandatory
  */
private:
  //! FeatureLine depth (required to compute the interaction matrix)
  //!  equation of a plane
  double A,B,C,D ;

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeatureLine() ;
  //! destructor
  ~vpFeatureLine() { ; }

public:
  /*
    /section Set coordinates
  */

  //! set the point xy and Z-coordinates
  void setRhoTheta(const double _rho, const double _theta) ;
  void setABCD(const double _A, const double _B,
	       const double _C, const double _D) ;

  //  void buildFrom(const vpLine &l) ;
  //  void buildFrom(const vpCylinder &c, const int l) ;
  void buildFrom(const double _rho, const double _theta) ;
  void buildFrom(const double _rho, const double _theta,
		 const double _A, const double _B,
		 const double _C, const double _D) ;

  //@}

  double getRho() const  { return s[0] ; }
  double getTheta() const { return s[1] ; }
public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  static int selectRho()  { return FEATURE_LINE1 ; }
  static int selectTheta()  { return FEATURE_LINE2 ; }
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
  vpFeatureLine *duplicate() const ;


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
