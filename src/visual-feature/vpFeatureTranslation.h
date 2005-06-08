
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTranslation.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureTranslation.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *       class that defines the thetaU visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeatureTranslation_H
#define vpFeatureTranslation_H

/*!
  \file vpFeatureTranslation.h
  \brief class that defines the thetaU visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureTranslation
  \brief class that defines the thetaU visual feature
*/
class vpFeatureTranslation : public vpBasicFeature
{
public:
  enum thetauFeatureEnum
    {
      TUx,
      TUy,
      TUz
    };
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar usefull but not mandatory
  */

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeatureTranslation() ;
  //! constructor : build from a rotation matrix
  //! cdMc is the displacement that the camera has to  realize
  vpFeatureTranslation(vpHomogeneousMatrix &cdMc) ;
  //! destructor
  ~vpFeatureTranslation() { ; }

private:
  //! displacement that the camera has to realize
  vpHomogeneousMatrix cdMc ;

public:


  double get_Tx()  const ;
  double get_Ty()   const ;
  double get_Tz() const  ;

  //! build from an homogeneous  matrix
  //! cdMc is the displacement that the camera has to  realize
  void buildFrom(const vpHomogeneousMatrix &cdMc) ;

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  static int selectTx()  { return FEATURE_LINE1 ; }
  static int selectTy()  { return FEATURE_LINE2 ; }
  static int selectTz()  { return FEATURE_LINE3 ; }
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
 //! compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;
  //! print the name of the feature
  void print(const int select= FEATURE_ALL) const ;

public:
  //! feature duplication
  vpFeatureTranslation *duplicate() const ;

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
