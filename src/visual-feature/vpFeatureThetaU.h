
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpThetaU.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpFeatureThetaU.h,v 1.1.1.1 2005-06-08 07:08:10 fspindle Exp $
 *
 * Description
 * ============
 *       class that defines the thetaU visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpFeatureThetaU_H
#define vpFeatureThetaU_H

/*!
  \file vpFeatureThetaU.h
  \brief class that defines the thetaU visual feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureThetaU
  \brief class that defines the thetaU visual feature
*/
class vpFeatureThetaU : public vpBasicFeature
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
  vpFeatureThetaU() ;
  //! constructor : build from a rotation matrix
  //! cdRc is the rotation that the camera has to  realize
  vpFeatureThetaU(vpRotationMatrix &cdRc) ;
  vpFeatureThetaU(vpHomogeneousMatrix &cdMc) ;
  void buildFrom(vpThetaUVector &tu) ;
  //! destructor
  ~vpFeatureThetaU() { ; }

public:


  void set_TUx(const double _X) ;
  double get_TUx()  const ;
  void set_TUy(const double _Y) ;
  double get_TUy()   const ;
  void set_TUz(const double _Z) ;
  double get_TUz() const  ;

  //! build from a rotation matrix
  //! cdRc is the rotation that the camera has to  realize
  void buildFrom(const vpRotationMatrix &cdRc) ;
  //! build from an homogeneous  matrix
  //! cdMc is the displacement that the camera has to  realize
  void buildFrom(const vpHomogeneousMatrix &cdMc) ;

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  static int selectTUx()  { return FEATURE_LINE1 ; }
  static int selectTUy()  { return FEATURE_LINE2 ; }
  static int selectTUz()  { return FEATURE_LINE3 ; }
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  //! print the name of the feature
  void print(const int select= FEATURE_ALL) const ;

  //! feature duplication
  vpFeatureThetaU *duplicate() const ;

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
