/****************************************************************************
 *
 * $Id: vpBasicFeature.h,v 1.7 2007-12-18 15:03:17 fspindle Exp $
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
 * Visual feature.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard
 *
 *****************************************************************************/



#ifndef vpBasicFeature_H
#define vpBasicFeature_H

/*!
  \file vpBasicFeature.h
  \brief class that defines what is a visual feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

// Display Issue

// Meter/pixel conversion
#include <visp/vpCameraParameters.h>

//Color / image / display
#include <visp/vpColor.h>
#include <visp/vpImage.h>

// #define FEATURE_ALL 0xff

// #define FEATURE_LINE1 0x1
// #define FEATURE_LINE2 0x2
// #define FEATURE_LINE3 0x4
// #define FEATURE_LINE4 0x8
// #define FEATURE_LINE5 0x10
// #define FEATURE_LINE6 0x20
// #define FEATURE_LINE7 0x40
// #define FEATURE_LINE8 0x80


/*!
  \class vpBasicFeature
  \brief class that defines what is a visual feature
*/
class VISP_EXPORT vpBasicFeature
{
 public: // Public constantes
  static const int FEATURE_LINE [32];
  static const int FEATURE_ALL;

 protected:
  //! state of the visual feature
  vpColVector s ;
  //! dimension of the visual feature
  int dim_s ;
  //int featureLine[8] ;

 public:
  int dimension_s() { return dim_s ; }

 public:

  virtual void init() = 0 ;

  vpBasicFeature() ;
  virtual ~vpBasicFeature() { /*vpTRACE("0x%x", this)*/; }
  //! write element in the state vector (usage : A[i] = x )
  virtual inline double operator[](const int n) {  return s[n]; }
  //! read element in the state vector  (usage : x = A[i] )
  virtual inline double operator[](const int n) const {  return s[n]; }

  //! select all the feature
  static  int selectAll()  { return FEATURE_ALL ; }

  //! get the feature vecror
  vpColVector get_s() const;

  //! get the feature dimension
  int getDimension(int select=FEATURE_ALL) const;
  //! compute the interaction matrix from a subset a the possible features
  virtual vpMatrix interaction(const int select = FEATURE_ALL) const = 0;
  //! compute the error between two visual features from a subset
  //! a the possible features
  virtual vpColVector error(const vpBasicFeature &s_star,
			    const int select= FEATURE_ALL)  = 0 ;
  //! print the name of the feature
  virtual void print(const int select= FEATURE_ALL) const = 0 ;

  virtual vpBasicFeature *duplicate() const = 0 ;


 public:
  virtual void display(const vpCameraParameters &cam,
		       vpImage<unsigned char> &I,
		       bool useDistortion = false,
		       vpColor::vpColorType color=vpColor::green) const = 0;

  // memory issue (used by the vpServo class)
 public:
  enum whoShouldDeallocateEnum
    {
      user,
      vpServo
    } ;

 private:
  int deallocate ;
 public:
  void setDeallocate(int d) { deallocate = d ; }
  int getDeallocate() { return deallocate ; }
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
