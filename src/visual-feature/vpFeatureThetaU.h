/****************************************************************************
 *
 * $Id: vpFeatureThetaU.h,v 1.5 2007-02-26 17:13:55 fspindle Exp $
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
 * ThetaU visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpFeatureThetaU_H
#define vpFeatureThetaU_H

/*!
  \file vpFeatureThetaU.h
  \brief class that defines the thetaU visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureThetaU
  \brief class that defines the thetaU visual feature
*/
class VISP_EXPORT vpFeatureThetaU : public vpBasicFeature
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


  void set_TUx(const double X) ;
  double get_TUx()  const ;
  void set_TUy(const double Y) ;
  double get_TUy()   const ;
  void set_TUz(const double Z) ;
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
  inline static int selectTUx()  { return FEATURE_LINE[0] ; }
  inline static int selectTUy()  { return FEATURE_LINE[1] ; }
  inline static int selectTUz()  { return FEATURE_LINE[2] ; }
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
	       vpColor::vpColorType color=vpColor::green) const ;

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
