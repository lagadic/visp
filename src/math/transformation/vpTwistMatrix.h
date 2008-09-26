/****************************************************************************
 *
 * $Id: vpTwistMatrix.h,v 1.5 2008-09-26 15:20:55 fspindle Exp $
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
 * Twist transformation matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpTWISTMATRIX_HH
#define vpTWISTMATRIX_HH

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>


/*!
  \class vpTwistMatrix

  \ingroup TwistTransformation

  \brief Class that consider the particular case of twist
  transformation matrix. 

  The vpTwistMatrix is derived from vpMatrix.

  \author  Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  An twist transformation matrix is 6x6 matrix defines as
  \f[
  ^a{\bf V}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf T}_b]_\times ^a{\bf R}_b\\
  {\bf 0}_{1\times 3} & ^a{\bf R}_b
  \end{array}
  \right)
  \f]
  that expressed a velocity in frame <em>a</em> knowing velocity in <em>b</em>

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf T}_b \f$ is a translation vector.
*/
class VISP_EXPORT vpTwistMatrix : public vpMatrix
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
    vpTwistMatrix(const double Tx,   const double Ty,   const double Tz,
                  const double tux,  const double tuy,  const double tuz) ;

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
