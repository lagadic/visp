
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpPoseVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseVector.cpp,v 1.1.1.1 2005-06-08 07:08:07 fspindle Exp $
 *
 * Description
 * ============
 *   class that consider the case of the Theta U parameterization for the
 *   rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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
	if (i<3) cout << (*this)[i] <<" " ;
	else  cout << vpMath::deg((*this)[i]) <<" " ;
    cout <<endl ;

}
void
vpPoseVector::save(ofstream &f) const
{
  if (f != NULL)
  {
    f << *this ;
  }
  else
  {
    ERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioERR, "\t\t file not open")) ;
  }
}


/*!
  Read an homogeneous matrix in a file, verify if it is really an homogeneous
  matrix

  \param ifstream &f : the file
*/
void
vpPoseVector::load(ifstream &f)
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
    ERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioERR, "\t\t file not open")) ;
  }
}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
