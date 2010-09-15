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
 * See the file LICENSE.txt at the root directory of this source
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
 * Homogeneous matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



/*!
  \file vpHomogeneousMatrix.h
  \brief Definition and computation on the homogeneous matrices
*/


#ifndef VPHOMOGENEOUSMATRIX_HH
#define VPHOMOGENEOUSMATRIX_HH

class vpPoseVector;
class vpMatrix;
class vpRotationMatrix;
class vpPoseVector;
class vpThetaUVector;

#include <fstream>

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>

#include <visp/vpRotationMatrix.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpPoseVector.h>

class vpMatrix;
class vpRotationMatrix;
class vpPoseVector;
class vpThetaUVector;

/*!
  \class vpHomogeneousMatrix

  \ingroup PoseTransformation

  \brief  The class provides a data structure for the homogeneous matrices
  as well as a set of operations on these matrices.

  The vpHomogeneousMatrix is derived from vpMatrix.

  \author  Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes


  An homogeneous matrix is 4x4 matrix defines as
  \f[
  ^a{\bf M}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & ^a{\bf t}_b \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right)
  \f]
  that defines the position of frame <em>b</em> in frame <em>a</em>

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.


*/
class VISP_EXPORT vpHomogeneousMatrix : public vpMatrix
{
 public:
  //! Basic initialisation (identity)
  void init() ;

  //! Basic initialisation (identity)
  void setIdentity() ;
  //! basic constructor
  vpHomogeneousMatrix()   ;
  //! copy constructor
  vpHomogeneousMatrix(const vpHomogeneousMatrix &M) ;
  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const vpTranslationVector &t,
                      const vpRotationMatrix &R) ;
  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const vpTranslationVector &t,
                      const vpThetaUVector &tu) ;
  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const vpPoseVector &p) ;  

  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const double tx,const  double ty, const double tz,
		      const double tux,const  double tuy, const double tuz  ) ;

  //! Construction from Translation and rotation
  void buildFrom(const vpTranslationVector &t,
                 const vpRotationMatrix &R) ;
  //! Construction from Translation and rotation
  void buildFrom(const vpTranslationVector &t,
                 const vpThetaUVector &tu) ;
  //! Construction from Translation and rotation
  void buildFrom(const vpPoseVector &p) ;
  //! Construction from Translation and rotation
  void buildFrom(const double tx,const  double ty, const double tz,
		 const double tux,const  double tuy, const double tuz  ) ;
    
  //! copy operator from vpHomogeneousMatrix
  vpHomogeneousMatrix &operator=(const vpHomogeneousMatrix &M);

  //! multiply two homogeneous matrices  aMb = aMc*cMb
  vpHomogeneousMatrix operator*(const vpHomogeneousMatrix &M) const;

  //! multiply by a vector ! size 4 !!!
  vpColVector operator*(vpColVector &v) const;

  //! invert the homogeneous matrix
  vpHomogeneousMatrix inverse() const ;
  //! invert the homogeneous matrix
  void inverse(vpHomogeneousMatrix &Mi) const;

  //! test if the rotational part of the matrix is a rotation matrix
  bool isAnHomogeneousMatrix() const  ;

  //! insert a rotation matrix
  void insert(const vpRotationMatrix &R) ;
  //! insert a theta u vector (transformation into a rotation matrix)
  void insert(const vpThetaUVector &tu) ;
  //! insert a translation vector
  void insert(const vpTranslationVector &t) ;

  //! extract the rotational matrix from the homogeneous  matrix
  void extract( vpRotationMatrix &R) const;
  //! extract the translation vector from the homogeneous  matrix
  void extract(vpTranslationVector &t) const;
  // extract the rotation as a Theta U vector.
  void extract(vpThetaUVector &tu) const;

  // Load an homogeneous matrix from a file
  void load(std::ifstream &f) ;
  // Save an homogeneous matrix in a file
  void save(std::ofstream &f) const ;

  // Set to identity
  void eye();

  //! Print the matrix as a vector [T thetaU]
  void print() ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
