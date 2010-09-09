/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/



#ifndef vpTRANSLATIONVECTOR_H
#define vpTRANSLATIONVECTOR_H

/*!
  \file vpTranslationVector.h
  \brief Class that consider the case of a translation vector.
*/

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>


/*!
  \class vpTranslationVector

  \ingroup TransTransformation
  
  \brief Class that consider the case of a translation vector.

  Let be \f$^{a}{\bf t}_{b} = [t_x,t_y,t_z]^\top\f$ be a translation
  from frame \f$ a \f$ to frame \f$ b \f$.  The representation of a
  translation is a column vector of dimension 3.

  Translations are expressed in meters.

  The code below shows how to use a translation vector to build an
  homogeneous matrix.

  \code
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpTranslationVector.h>

int main()
{
  vpTranslationVector t; // Translation vector 

  // Initialization of the translation vector
  t[0] =  0.2; // tx = 0.2 meters
  t[1] = -0.1; // ty = -0.1 meters
  t[2] =  1.0; // tz = 1 meters

  // Construction of a rotation matrix
  vpRotationMatrix R; // Set to identity by default

  // Construction of an homogeneous matrix
  vpHomogeneousMatrix M(t, R);
}
  \endcode

*/
class VISP_EXPORT vpTranslationVector : public vpColVector
{
private:
    //! initialize a size 3 vector
    void init() ;

public:

    /*!
      Default constructor.
      The translation vector is initialized to zero.
    */
    vpTranslationVector() { init() ; }
    // constructor from double in meter
    vpTranslationVector(const double tx, const double ty, const double tz) ;
    // copy constructor
    vpTranslationVector(const vpTranslationVector &t);
    void set(const double tx, const double ty, const double tz) ;

    // operators

    // translation vectors additions  c = a + b (a, b  unchanged)
    vpTranslationVector operator+(const vpTranslationVector &t) const ;
    // translation vectors substraction  c = a - b (a, b  unchanged)
    vpTranslationVector operator-(const vpTranslationVector &t) const ;
    // negate t = -a  (t is unchanged)
    vpTranslationVector operator-() const ;
    // b = x * a (x=scalar)
    vpTranslationVector operator*(const double x) const;
    // Copy operator.   Allow operation such as A = v
    vpTranslationVector &operator=(const vpTranslationVector &t);

    vpTranslationVector &operator=(double x) ;


    // Skew Symmetric matrix
    vpMatrix skew() const ;
    static vpMatrix skew(const vpTranslationVector &t) ;
    static void skew(const  vpTranslationVector &t, vpMatrix &M) ;
    static vpTranslationVector cross(const vpTranslationVector &a,
				     const vpTranslationVector &b) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
