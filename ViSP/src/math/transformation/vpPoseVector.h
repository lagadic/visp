
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoseVector.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoseVector.h,v 1.3 2005-11-30 10:28:57 marchand Exp $
 *
 * Description
 * ============
 *   pose object. a pose is a size 6 vector [t, tu]^T where tu is
 *   a rotation vector (theta u representation) and t is a translation
 *   vector
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpPOSEVECTOR_H
#define vpPOSEVECTOR_H

/*!
  \file vpPoseVector.h
  \brief  pose object. a pose is a size 6 vector [t,tu]^T where tu is
    a rotation vector (theta u representation) and t is a translation
    vector
*/
class vpRotationMatrix;
class vpHomogeneousMatrix;
class vpTranslationVector;
class vpEulerVector;
class vpThetaUVector;

#include <visp/vpMatrix.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpHomogeneousMatrix.h>

class vpRotationMatrix;
class vpHomogeneousMatrix;
class vpTranslationVector;
class vpEulerVector;
class vpThetaUVector;


/*!
  \class vpPoseVector
  \brief  pose object. a pose is a size 6 vector [t,tu]^T where tu is
    a rotation vector (theta u representation) and t is a translation
    vector

    [Tx, Ty, Tz, Tux, Tuy, tuz]
*/
class vpPoseVector : public vpColVector
{
 
private:
    //! initialize a size 6 vector
    void init() ;

public:
    //! constructor
    vpPoseVector() ;
    //! constructor from 3 angles (in radian)
    vpPoseVector(const double tx, const double ty, const double tz,
		 const double tux, const double tuy, const double tuz) ;
    //! constructor convert an homogeneous matrix in a pose
    vpPoseVector(const vpHomogeneousMatrix& R) ;
    //! constructor convert a "euler" vector and a translation into a pose
    vpPoseVector(const vpEulerVector &e,
		 const vpTranslationVector& t) ;
    //! constructor  convert a "thetau" vector and a translation into a pose
    vpPoseVector(const vpThetaUVector& tu,
		 const vpTranslationVector& t) ;
    //! constructor  convert arotation matrix  and a translation into a pose
    vpPoseVector(const vpRotationMatrix& R,
		 const vpTranslationVector& t) ;

    //! convert an homogeneous matrix in a pose
    vpPoseVector buildFrom(const vpHomogeneousMatrix& R) ;
     //! convert a "euler" vector and a translation into a pose
    vpPoseVector buildFrom(const vpEulerVector &e,
			   const vpTranslationVector& t) ;
    //!  convert a "thetau" vector and a translation into a pose
    vpPoseVector buildFrom(const vpThetaUVector& tu,
			   const vpTranslationVector& t) ;
    //!  convert arotation matrix  and a translation into a pose
    vpPoseVector buildFrom(const vpRotationMatrix& R,
			   const vpTranslationVector& t) ;

    //! Access  V[i] = x
    inline double &operator [](int n) {  return *(data + n);  }
    //! Access x = V[i]
    inline const double &operator [](int n) const { return *(data+n);  }

  //! Load an homogeneous matrix from a file
  void load(ifstream &f) ;
  //! Save an homogeneous matrix in a file
  void save(ofstream &f) const ;

  //! Print  a vector [T thetaU] thetaU in degree
  void print() ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

