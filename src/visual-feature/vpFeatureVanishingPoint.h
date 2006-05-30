/****************************************************************************
 *
 * $Id: vpFeatureVanishingPoint.h,v 1.2 2006-05-30 08:40:48 fspindle Exp $
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
 * 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/


#ifndef vpFeatureVanishingPoint_H
#define vpFeatureVanishingPoint_H



/*!  \file vpFeatureVanishingPoint.h \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpPoint.h>

#include <visp/vpHomogeneousMatrix.h>



/*!  \class vpFeatureVanishingPoint \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/
class VISP_EXPORT vpFeatureVanishingPoint : public vpBasicFeature
{
public:
  enum pointFeatureEnum
    {
      X,   // x coordinates
      Y    // y coordinates
    };

  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar usefull but not mandatory
  */
  //no Z required

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpFeatureVanishingPoint() ;
  //! destructor
  ~vpFeatureVanishingPoint() { ; }


public:
  /*
    /section Set coordinates
  */


   //! set the point x-coordinates
  void set_x(const double _x) ;
  //! get the point x-coordinates
  double get_x()  const ;
  //! set the point y-coordinates
  void set_y(const double _y) ;
  //! get the point y-coordinates
  double get_y()   const ;
  //! set the point xy coordinates
  void set_xy(const double _x, const double _y) ;
   // void buildFrom(const vpPoint &p) ;
  void buildFrom(const double _x, const double _y) ;



  public:
  /*
    vpBasicFeature method instantiation
  */
  // feature selection
  inline static int selectX()  { return FEATURE_LINE[0] ; }
  inline static int selectY()  { return FEATURE_LINE[1] ; }


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
  vpFeatureVanishingPoint *duplicate() const ;

public:
  void display(const vpCameraParameters &cam,
	       vpImage<unsigned char> &I,
	       int color=vpColor::green) const ;

} ;



#endif
