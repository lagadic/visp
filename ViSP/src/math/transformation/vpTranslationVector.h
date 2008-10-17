/****************************************************************************
 *
 * $Id: vpTranslationVector.h,v 1.8 2008-10-17 17:25:07 fspindle Exp $
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
