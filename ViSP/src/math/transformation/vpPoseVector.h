/****************************************************************************
 *
 * $Id: vpPoseVector.h,v 1.5 2007-04-20 14:22:16 asaunier Exp $
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

#include <visp/vpConfig.h>
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
class VISP_EXPORT vpPoseVector : public vpColVector
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
  void load(std::ifstream &f) ;
  //! Save an homogeneous matrix in a file
  void save(std::ofstream &f) const ;

  //! Print  a vector [T thetaU] thetaU in degree
  void print() ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

