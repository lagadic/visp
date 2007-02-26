/****************************************************************************
 *
 * $Id: vpFeatureEllipse.h,v 1.6 2007-02-26 17:13:55 fspindle Exp $
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
 * 2D ellipse visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpFeatureEllipse_H
#define vpFeatureEllipse_H

/*!
  \file vpFeatureEllipse.h
  \brief Class that defines 2D ellipse visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpFeatureEllipse
  \brief Class that defines 2D ellipse visual feature
*/
class VISP_EXPORT vpFeatureEllipse : public vpBasicFeature
{
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities are usefull but not mandatory
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
  /*!
    \section Set coordinates
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
  

public:
  /*!
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
	       vpColor::vpColorType color=vpColor::green) const ;

} ;



#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
