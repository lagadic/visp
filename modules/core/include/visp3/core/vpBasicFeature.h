/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>

// Display Issue

// Meter/pixel conversion
#include <visp3/core/vpCameraParameters.h>

//Color / image / display
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>

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
  \ingroup group_core_features
  \brief class that defines what is a visual feature
*/
class VISP_EXPORT vpBasicFeature
{
public: // Public constantes
  static const unsigned int FEATURE_LINE [32];

  enum {
    FEATURE_ALL = 0xffff
  };

protected:
  //! State of the visual feature.
  vpColVector s ;
  //! Dimension of the visual feature.
  unsigned int dim_s ;
  //int featureLine[8] ;
  //! Ensure that all the parameters needed to compute the iteraction matrix are set.
  bool *flags;
  //! Number of parameters needed to compute the interaction matrix.
  unsigned int nbParameters;

public:
  /*! Return the dimension of the feature vector \f$\bf s\f$. */
  unsigned int dimension_s() { return dim_s ; }

public:

  virtual void init() = 0 ;

  vpBasicFeature() ;
  vpBasicFeature(const vpBasicFeature &f) ;
  vpBasicFeature &operator=(const vpBasicFeature &f) ;
  virtual ~vpBasicFeature();
  //! Return element \e i in the state vector  (usage : x = s[i] )
  virtual inline double operator[](const unsigned int i) const {  return s[i]; }

  //! Select all the features.
  static  unsigned int selectAll()  { return FEATURE_ALL ; }

  // Get the feature vector.
  vpColVector get_s(unsigned int select=FEATURE_ALL) const;

  // Get the feature vector dimension.
  unsigned int getDimension(const unsigned int select=FEATURE_ALL) const;
  //! Compute the interaction matrix from a subset of the possible features.
  virtual vpMatrix interaction(const unsigned int select = FEATURE_ALL) = 0;
  virtual vpColVector error(const vpBasicFeature &s_star,
                            const unsigned int select= FEATURE_ALL);
  //! Print the name of the feature.
  virtual void print(const unsigned int select= FEATURE_ALL) const = 0 ;

  virtual vpBasicFeature *duplicate() const = 0 ;

public:
  virtual void display(const vpCameraParameters &cam,
                       const vpImage<unsigned char> &I,
                       const vpColor &color=vpColor::green,
                       unsigned int thickness=1) const = 0;
  virtual void display(const vpCameraParameters &cam,
                       const vpImage<vpRGBa> &I,
                       const vpColor &color=vpColor::green,
                       unsigned int thickness=1) const = 0;

  void setFlags();

protected:
  void resetFlags();
  // memory issue (used by the vpServo class)
public:

  /*!
    \enum vpBasicFeatureDeallocatorType
    Indicates who should deallocate the feature.

  */
  typedef enum
  {
    user,
    vpServo
  } vpBasicFeatureDeallocatorType;

protected:
  vpBasicFeatureDeallocatorType deallocate ;
public:
  void setDeallocate(vpBasicFeatureDeallocatorType d) { deallocate = d ; }
  vpBasicFeatureDeallocatorType getDeallocate() { return deallocate ; }
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
