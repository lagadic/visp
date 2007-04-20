/****************************************************************************
 *
 * $Id: vpHomogeneousMatrix.h,v 1.8 2007-04-20 14:22:16 asaunier Exp $
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
class vpEulerVector;
class vpPoseVector;
class vpThetaUVector;

#include <fstream>

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>

#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpPoseVector.h>

class vpMatrix;
class vpRotationMatrix;
class vpEulerVector;
class vpPoseVector;
class vpThetaUVector;

/*!
  \class vpHomogeneousMatrix

  \brief  The class provides a data structure for the homogeneous matrices
  as well    as a set of operations on these matrices.

  the vpHomogeneousMatrix is derived from vpMatrix.

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
  vpHomogeneousMatrix(const vpRotationMatrix &R,
		      const vpTranslationVector &T) ;
   //! Construction from Translation and rotation
  vpHomogeneousMatrix(const vpThetaUVector &tu,
		      const vpTranslationVector &T) ;
  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const vpPoseVector &p) ;
  //! Construction from Translation and rotation
  vpHomogeneousMatrix(const double Tx,const  double Ty, const double Tz,
		      const double tux,const  double tuy, const double tuz  ) ;

  //! Construction from Translation and rotation
  void buildFrom(const vpRotationMatrix &R,
		      const vpTranslationVector &T) ;
   //! Construction from Translation and rotation
  void buildFrom(const vpThetaUVector &tu,
		      const vpTranslationVector &T) ;
  //! Construction from Translation and rotation
  void buildFrom(const vpPoseVector &p) ;
  //! Construction from Translation and rotation
  void buildFrom(const double Tx,const  double Ty, const double Tz,
		      const double tux,const  double tuy, const double tuz  ) ;
  //! copy operator from vpHomogeneousMatrix
  vpHomogeneousMatrix &operator=(const vpHomogeneousMatrix &m);

  //! multiply two homogeneous matrices  aMb = aMc*cMb
  vpHomogeneousMatrix operator*(const vpHomogeneousMatrix &mat) const;

  //! multiply by a vector ! size 4 !!!
  vpColVector operator*(vpColVector &mat) const;

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
  void insert(const vpTranslationVector &T) ;

  //! extract the rotational matrix from the homogeneous  matrix
  void extract( vpRotationMatrix &R) const;
  //! extract the translation vector from the homogeneous  matrix
  void extract(vpTranslationVector &T) const;

  //! Load an homogeneous matrix from a file
  void load(std::ifstream &f) ;
  //! Load an homogeneous matrix from a file
  //! proposed for compatibilty issue
  void loadMatrix34(std::ifstream &f) ;
  //! Save an homogeneous matrix in a file
  void save(std::ofstream &f) const ;

  //! Print the matrix as a vector [T thetaU]
  void print() ;

 } ;


//void ComputeCameraPosition(const vpColVector& dx,vpHomogeneousMatrix&  mati,int typ_mvt=0) ;
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
