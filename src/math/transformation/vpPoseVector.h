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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpPOSEVECTOR_H
#define vpPOSEVECTOR_H

/*!
  \file vpPoseVector.h

  \brief Pose representation. A pose is a 6 dimension vector [t,tu]^T
    where tu is a rotation vector (theta u representation) and t is a
    translation vector.
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

  \ingroup PoseTransformation

  \brief The pose is a complete representation of every rigid motion in the
  euclidian space.  

  It is composed of a translation and a rotation
  minimaly represented by a 6 dimension pose vector as: \f[ ^{a}{\bf
  r}_b = [^{a}{\bf t}_{b},\Theta {\bf u}]^\top \in R^6\f]

  where \f$ ^{a}{\bf r}_b \f$ is the pose from frame \f$ a \f$ to
  frame \f$ b \f$, with \f$ ^{a}{\bf t}_{b} \f$ being the translation
  vector between these frames along the x,y,z
  axis and \f$\Theta \bf u \f$, the \f$\Theta \bf u \f$ representation of the
  rotation \f$^{a}\bf{R}_{b}\f$ between these frames.

  To know more about the \f$\Theta \bf u\f$ rotation representation,
  see vpThetaUVector documentation.

*/
class VISP_EXPORT vpPoseVector : public vpColVector
{

 private:
  // initialize a size 6 vector
  void init() ;

 public:
  // constructor
  vpPoseVector() ;
  // constructor from 3 angles (in radian)
  vpPoseVector(const double tx, const double ty, const double tz,
	       const double tux, const double tuy, const double tuz) ;
  // constructor convert an homogeneous matrix in a pose
  vpPoseVector(const vpHomogeneousMatrix& M) ;
  // constructor  convert a translation and a "thetau" vector into a pose
  vpPoseVector(const vpTranslationVector& t,
	       const vpThetaUVector& tu) ;
  // constructor  convert a translation and a rotation matrix into a pose
  vpPoseVector(const vpTranslationVector& t,
	       const vpRotationMatrix& R) ;
  

  // convert an homogeneous matrix in a pose
  vpPoseVector buildFrom(const vpHomogeneousMatrix& M) ;
  //  convert a translation and a "thetau" vector into a pose
  vpPoseVector buildFrom(const vpTranslationVector& t,
			 const vpThetaUVector& tu) ;
  //  convert a translation and a rotation matrix into a pose
  vpPoseVector buildFrom(const vpTranslationVector& t,
			 const vpRotationMatrix& R) ;
    

  /*! 
    Set the value of an element of the pose vector: r[i] = x.

    \param i : Pose vector element index

    \code
    // Create a pose vector with translation and rotation set to zero
    vpPoseVector r; 

    // Initialize the pose vector
    r[0] = 1;
    r[1] = 2;
    r[2] = 3;
    r[3] = M_PI;
    r[4] = -M_PI;
    r[5] = 0;
    \endcode

    This code produces the same effect:
    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
    \endcode

  */
  inline double &operator [](int i) {  return *(data + i);  }
  /*!
    Get the value of an element of the pose vector: x = r[i].

    \param i : Pose vector element index

    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
    
    double tx,ty,tz; // Translation
    double tux, tuy,tuz; // Theta u rotation
    tx  = r[0];
    ty  = r[1];
    tz  = r[2];
    tux = r[3];
    tuy = r[4];
    tuz = r[5];
    \endcode
  */
  inline const double &operator [](int i) const { return *(data+i);  }

  // Load an homogeneous matrix from a file
  void load(std::ifstream &f) ;
  // Save an homogeneous matrix in a file
  void save(std::ofstream &f) const ;

  // Print  a vector [T thetaU] thetaU in degree
  void print() ;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /*!
    @name Deprecated functions
  */
  // constructor convert a "euler" vector and a translation into a
  // pose (deprecated)
  vp_deprecated vpPoseVector(const vpEulerVector &e,
	       const vpTranslationVector& t) ;
  // constructor convert a "thetau" vector and a translation into a
  // pose (deprecated)
  vp_deprecated vpPoseVector(const vpThetaUVector& tu,
	       const vpTranslationVector& t) ;
  // constructor convert a rotation matrix and a translation into a
  // pose (deprecated)
  vp_deprecated vpPoseVector(const vpRotationMatrix& R,
	       const vpTranslationVector& t) ;
  // constructor convert a translation and a "euler" vector into a pose
  vp_deprecated vpPoseVector(const vpTranslationVector& t,
	       const vpEulerVector &e) ;

  // convert a "euler" vector and a translation into a pose (deprecated)
  vp_deprecated vpPoseVector buildFrom(const vpEulerVector &e,
			 const vpTranslationVector& t) ;
  // convert a "thetau" vector and a translation into a pose (deprecated)
  vp_deprecated vpPoseVector buildFrom(const vpThetaUVector& tu,
			 const vpTranslationVector& t) ;
  // convert a rotation matrix  and a translation into a pose (deprecated)
  vp_deprecated vpPoseVector buildFrom(const vpRotationMatrix& R,
			 const vpTranslationVector& t) ;
  // convert a translation and a "euler" vector into a pose
  vp_deprecated vpPoseVector buildFrom(const vpTranslationVector& t,
			 const vpEulerVector &e) ;
#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

