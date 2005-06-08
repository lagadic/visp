
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTranslationVector.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpTranslationVector.h, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpTranslationVector.h,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *     class that consider the case of a translation vector
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpTRANSLATIONVECTOR_H
#define vpTRANSLATIONVECTOR_H

/*!
  \file vpTranslationVector.h
  \brief class that consider the case of a translation vector
*/

#include <visp/vpColVector.h>


/*!
  \class vpTranslationVector
  \brief class that consider the case of a translation vector
*/
class vpTranslationVector : public vpColVector
{
private:
    //! initialize a size 3 vector
    void init() ;

public:
    vpTranslationVector() { init() ; }
    //! constructor from double in meter
    vpTranslationVector(const double tx, const double ty, const double tz) ;

    // operators

    //! translation vectors additions  c = a + b (a, b  unchanged)
    vpTranslationVector operator+(const vpTranslationVector &b) const ;
    //! negate t = -a  (t is unchanged)
    vpTranslationVector vpTranslationVector::operator-() const ;


    //! Skew Symmetric matrix
    vpMatrix skew() const ;
    static vpMatrix skew(const vpTranslationVector &v) ;
    static void skew(const  vpTranslationVector &v, vpMatrix &m) ;
    static vpTranslationVector cross(const vpTranslationVector &a,
				     const vpTranslationVector &b) ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
