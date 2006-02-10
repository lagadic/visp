
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTwistMatrix.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTwistMatrix.h,v 1.2 2006-02-10 14:17:45 fspindle Exp $
 *
 * Description
 * ============
 *     Class that consider the particular case of twist transformation matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifndef vpTWISTMATRIX_HH
#define vpTWISTMATRIX_HH

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>


class vpTwistMatrix : public vpMatrix
{
    friend class vpMatrix;

public:
    //! Basic initialisation (identity)
    void init() ;

    //! Basic initialisation (identity)
    void setIdentity() ;
    //! basic constructor
    vpTwistMatrix()   ;
    //! basic constructor
    vpTwistMatrix(const vpTwistMatrix &M) ;
    //! copy constructor
    vpTwistMatrix(const vpHomogeneousMatrix &M) ;

    vpTwistMatrix operator*(const vpTwistMatrix &mat) const ;

    vpColVector operator*(const vpColVector &v) const ;

    //! Construction from Translation and rotation (ThetaU parameterization)
    vpTwistMatrix(const vpTranslationVector &T, const vpThetaUVector &R) ;
    //! Construction from Translation and rotation (euler parameterization)
    vpTwistMatrix(const vpTranslationVector &T, const vpEulerVector &R) ;
    //! Construction from Translation and rotation (matrix parameterization)
    vpTwistMatrix(const vpTranslationVector &T, const vpRotationMatrix &R) ;
    vpTwistMatrix(const double tux,  const double tuy,  const double tuz,
		  const double Tx,   const double Ty,   const double Tz) ;

   //! Assigment
    vpTwistMatrix &operator<<(const vpTwistMatrix &m);

    //! copy operator from vpMatrix (handle with care)
    //  vpTwistMatrix &operator=(const vpMatrix &m);
    //! copy operator from vpMatrix (handle with care)
    vpTwistMatrix &operator=(const vpTwistMatrix &m);



    vpTwistMatrix buildFrom(const vpTranslationVector &t,
			    const vpRotationMatrix &R);
    vpTwistMatrix buildFrom(const vpTranslationVector &t,
			    const vpEulerVector &e);
    vpTwistMatrix buildFrom(const vpTranslationVector &t,
			    const vpThetaUVector &e);
    vpTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;


} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
