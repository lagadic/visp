/****************************************************************************
 *
 * $Id: vpFeaturePoint3D.h,v 1.4 2006-05-30 08:40:48 fspindle Exp $
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
 * 3D point visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpFeaturePoint3d_H
#define vpFeaturePoint3d_H

/*!
  \file vpFeaturePoint3D.h
  \brief class that defines 3D point visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpPoint.h>

#include <visp/vpHomogeneousMatrix.h>

/*!
  \class vpFeaturePoint3D
  \brief class that defines 3D point visual feature
*/
class VISP_EXPORT vpFeaturePoint3D : public vpBasicFeature

{

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeaturePoint3D() ;
  //! destructor
  ~vpFeaturePoint3D() { ; }

public:
  /*
    /section Set coordinates
  */

  //@{

  //! set the point depth (camera frame)
  void set_Z(const double Z) ;
  //! get the point depth (camera frame)
  double get_Z() const  ;

  //! set the point X-coordinates
  void set_X(const double X) ;
  //! get the point X-coordinates
  double get_X()  const ;
  //! set the point Y-coordinates
  void set_Y(const double Y) ;
  //! get the point Y-coordinates
  double get_Y()   const ;
  //! set the point XY and Z-coordinates
  void set_XYZ(const double X, const double Y, const double Z) ;

  //! build feature from a point (vpPoint)
  void buildFrom(const vpPoint &p) ;
  //! set the point XY and Z-coordinates
  void buildFrom(const double X, const double Y, const double Z) ;
  //@}

public:
  /*
    vpBasicFeature method instantiation
  */

  // feature selection
  inline static int selectX()  { return FEATURE_LINE[0] ; }
  inline static int selectY()  { return FEATURE_LINE[1] ; }
  inline static int selectZ()  { return FEATURE_LINE[2] ; }
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix  interaction(const int select = FEATURE_ALL) const;
  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star,
		    const int select = FEATURE_ALL)  ;
  //! compute the error between a visual features and zero
  vpColVector error(const int select = FEATURE_ALL)  ;
  //! print the name of the feature
  void print(const int select ) const ;

  //! feature duplication
  vpFeaturePoint3D *duplicate() const ;

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
