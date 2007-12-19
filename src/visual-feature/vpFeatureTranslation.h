/****************************************************************************
 *
 * $Id: vpFeatureTranslation.h,v 1.7 2007-12-19 08:25:25 fspindle Exp $
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
 * 3D translation visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpFeatureTranslation_H
#define vpFeatureTranslation_H

/*!
  \file vpFeatureTranslation.h
  \brief class that defines the thetaU visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureTranslation
  \brief class that defines the thetaU visual feature
*/
class VISP_EXPORT vpFeatureTranslation : public vpBasicFeature
{
public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeatureTranslation() ;
  //! constructor : build from a rotation matrix
  //! cdMc is the displacement that the camera has to  realize
  vpFeatureTranslation(vpHomogeneousMatrix &cdMc) ;
  //! destructor
  virtual ~vpFeatureTranslation() { ; }

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
  inline static int selectTx()  { return FEATURE_LINE[0] ; }
  inline static int selectTy()  { return FEATURE_LINE[1] ; }
  inline static int selectTz()  { return FEATURE_LINE[2] ; }
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
	       bool useDistortion=false,
	       vpColor::vpColorType color=vpColor::green) const ;

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
