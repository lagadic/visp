
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpGenericFeature.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpGenericFeature.h,v 1.2 2005-12-05 10:21:04 marchand Exp $
 *
 * Description
 * ============
 *     class that defines what is a generic feature (used to create new
 *     feature not implemented in ViSP2
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpGenericFeature_hh
#define vpGenericFeature_hh

#include <math.h>
/*!
  \file vpGenericFeature.h
  \brief class that defines what is a generic feature (used to create new
     feature not implemented in ViSP2
 */

#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>

/*!
  \class vpGenericFeature
  \brief class that defines what is a generic feature (used to create new
     feature not implemented in ViSP2
 */
class vpGenericFeature : public vpBasicFeature
{
private:
  vpGenericFeature() ;
public:
  void init() ;
  vpGenericFeature(int dim) ;
  ~vpGenericFeature() ;
public:
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
  vpGenericFeature *duplicate() const ;

private:
  vpMatrix L ;
  vpColVector err ;
  int errorStatus ;

  enum errorEnum
    {
      errorNotInitalized,
      errorInitialized,
      errorHasToBeUpdated
    } ;
public:
  void setInteractionMatrix(const vpMatrix &_L) ;
  vpMatrix getInteractionMatrix() const { return L ; }
  void setError(vpColVector &_error)  ;
  void set_s(const vpColVector &_s) ;
  void set_s(const double s0) ;
  void set_s(const double s0, const double s1) ;
  void set_s(const double s0, const double s1, const double s2) ;

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
