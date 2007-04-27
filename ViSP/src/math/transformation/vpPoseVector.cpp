/****************************************************************************
 *
 * $Id: vpPoseVector.cpp,v 1.7 2007-04-27 16:40:15 fspindle Exp $
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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpPoseVector.cpp
  \brief  pose object. a pose is a size 6 vector [t,tu]^T where tu is
    a rotation vector (theta u representation) and t is a translation
    vector
*/

#include <visp/vpPoseVector.h>
#include <visp/vpMath.h>
#include <visp/vpDebug.h>
#include <visp/vpMatrixException.h>
#include <visp/vpException.h>

//! initialize a size 3 vector
void vpPoseVector::init()
{
    resize(6) ;
}

//! constructor
vpPoseVector::vpPoseVector()
{
    init() ;
}

//! constructor from 3 translations and 3 angles (in radian)
vpPoseVector::vpPoseVector(const double tx,
			   const double ty,
			   const double tz,
			   const double tux,
			   const double tuy,
			   const double tuz)
{
    init() ;


    (*this)[0] = tx ;
    (*this)[1] = ty ;
    (*this)[2] = tz ;

    (*this)[3] = tux ;
    (*this)[4] = tuy ;
    (*this)[5] = tuz ;


}

//! constructor convert a "euler" vector and a translation into a pose
vpPoseVector::vpPoseVector(const vpEulerVector &e,
			   const vpTranslationVector& t)
{
    init() ;
    buildFrom(e,t) ;
}

//! constructor  convert a "thetau" vector and a translation into a pose
vpPoseVector::vpPoseVector(const vpThetaUVector& tu,
			   const vpTranslationVector& t)
{
    init() ;
    buildFrom(tu,t) ;
}

//! constructor  convert arotation matrix  and a translation into a pose
vpPoseVector::vpPoseVector(const vpRotationMatrix& R,
			   const vpTranslationVector& t)
{
    init() ;
    buildFrom(R,t) ;
}

//! constructor convert an homogeneous matrix in a pose
vpPoseVector::vpPoseVector(const vpHomogeneousMatrix& M)
{
    init() ;
    buildFrom(M) ;
}

//! convert an homogeneous matrix in a pose
vpPoseVector
vpPoseVector::buildFrom(const vpHomogeneousMatrix& M)
{
    vpRotationMatrix R ;    M.extract(R) ;
    vpTranslationVector t ; M.extract(t) ;
    buildFrom(R,t) ;
    return *this ;
}
//! convert a "euler" vector and a translation into a pose
vpPoseVector
vpPoseVector::buildFrom(const vpEulerVector &e,
			const vpTranslationVector& t)
{
    vpThetaUVector tu ;
    tu.buildFrom(e) ;
    buildFrom(tu,t) ;
    return *this ;
}

//!  convert a "thetau" vector and a translation into a pose
vpPoseVector
vpPoseVector::buildFrom(const vpThetaUVector& tu,
			const vpTranslationVector& t)
{
    for(int i =0  ; i < 3 ; i++)
    {
	(*this)[i] = t[i] ;
	(*this)[i+3] = tu[i] ;
    }
    return *this ;
}

//!  convert arotation matrix  and a translation into a pose
vpPoseVector
vpPoseVector::buildFrom(const vpRotationMatrix& R,
			const vpTranslationVector& t)
{
    vpThetaUVector tu ;
    tu.buildFrom(R) ;

    buildFrom(tu,t) ;
    return *this ;
}



void
vpPoseVector::print()
{
    for(int i =0  ; i < 6 ; i++)
	if (i<3) std::cout << (*this)[i] <<" " ;
	else  std::cout << vpMath::deg((*this)[i]) <<" " ;
    std::cout <<std::endl ;

}
void
vpPoseVector::save(std::ofstream &f) const
{
  if (f != NULL)
  {
    f << *this ;
  }
  else
  {
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}


/*!
  Read an homogeneous matrix in a file, verify if it is really an homogeneous
  matrix

  \param f : The file stream.
*/
void
vpPoseVector::load(std::ifstream &f)
{
  if (f != NULL)
  {
    for (int i=0 ; i < 6 ; i++)
    {
	f>>   (*this)[i] ;
    }
  }
  else
  {
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
