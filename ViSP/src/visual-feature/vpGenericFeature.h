/****************************************************************************
 *
 * $Id: vpGenericFeature.h,v 1.6 2007-05-02 13:29:41 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Generic feature (used to create new feature not implemented in ViSP).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpGenericFeature_hh
#define vpGenericFeature_hh

#include <math.h>
/*!
  \file vpGenericFeature.h
  \brief class that defines what is a generic feature (used to create new
     feature not implemented in ViSP2
 */

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>

/*!
  \class vpGenericFeature
  \brief class that defines what is a generic feature (used to create new
     feature not implemented in ViSP2
 */
class VISP_EXPORT vpGenericFeature : public vpBasicFeature
{
private:
  vpGenericFeature() ;
public:
  void init() ;
  vpGenericFeature(int dim) ;
  virtual ~vpGenericFeature() ;
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
  void setInteractionMatrix(const vpMatrix &L) ;
  vpMatrix getInteractionMatrix() const { return L ; }
  void setError(vpColVector &_error)  ;
  void set_s(const vpColVector &s) ;
  void set_s(const double s0) ;
  void set_s(const double s0, const double s1) ;
  void set_s(const double s0, const double s1, const double s2) ;

public:
  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       vpColor::vpColorType color=vpColor::green) const ;


} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
