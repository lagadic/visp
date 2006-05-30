/****************************************************************************
 *
 * $Id: vpHomography.h,v 1.4 2006-05-30 08:40:42 fspindle Exp $
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
 * Homography transformation.
 *
 * Authors:
 * Muriel Pressigout
 *
 *****************************************************************************/


/*!
\file vpHomography.h

This file defines an homography transformation. This class aims to provide some
tools for homography computation.
*/

#ifndef vpHomography_hh
#define vpHomography_hh

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPlane.h>
#include <visp/vpList.h>


/*!  \class vpHomography

\brief This class aims to compute the homography wrt. 2
  images, which are both described by a set of points. The 2 sets (one per
  image) are sets of corresponding points : for a point in a image, there is
  the corresponding point (image of the same 3D point) in the other image
  points set.  These 2 sets are the only data needed to compute the
  homography.  One method used is the one introduced by Ezio Malis during his
  PhD. A normalization is carried out on this points in order to improve the
  conditioning of the problem, what leads to improve the stability of the
  result.


  store and compute the homography such that
  \f[
  ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
  \f]

  with
  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]
*/
class VISP_EXPORT vpHomography : public vpMatrix
{

private:
  vpHomogeneousMatrix aMb ;
  //! reference plane coordinates  expressed in Rb
  vpPlane bP ;
  void init() ;


public:
  vpHomography() ;
  ~vpHomography() { ; }

private:
  virtual void resize(int , int ) { ; }

public:

  //! copy constructor
  vpHomography(const vpHomography &aMb) ;
   //! Construction from Translation and rotation and a plane
  vpHomography(const vpHomogeneousMatrix &aMb,
	       const vpPlane &bP) ;
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpRotationMatrix &aRb,
	       const vpTranslationVector &aTb,
	       const vpPlane &bP) ;
   //! Construction from Translation and rotation and a plane
  vpHomography(const vpThetaUVector &tu,
	       const vpTranslationVector &aTb,
	       const vpPlane &bP) ;
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpPoseVector &arb,
	       const vpPlane &bP) ;

  //! Construction from Translation and rotation and a plane
  void buildFrom(const vpRotationMatrix &aRb,
		 const vpTranslationVector &aTb,
		 const vpPlane &bP) ;
   //! Construction from Translation and rotation and a plane
  void buildFrom(const vpThetaUVector &tu,
		 const vpTranslationVector &aTb,
		 const vpPlane &bP) ;
  //! Construction from Translation and rotation  and a plane
  void buildFrom(const vpPoseVector &arb,
		 const vpPlane &bP) ;
  //! Construction from homogeneous matrix and a plane
  void buildFrom(const vpHomogeneousMatrix &aMb,
		 const vpPlane &bP) ;

  //! invert the homography
  vpHomography inverse() const ;
  //! invert the homography
  void inverse(vpHomography &Hi) const;

  //! insert a rotation matrix
  void insert(const vpHomogeneousMatrix &aRb) ;
  //! insert a rotation matrix
  void insert(const vpRotationMatrix &aRb) ;
  //! insert a theta u vector (transformation into a rotation matrix)
  void insert(const vpThetaUVector &tu) ;
  //! insert a translation vector
  void insert(const vpTranslationVector &aTb) ;
  //! insert a translation vector
  void insert(const vpPlane &bP) ;

  //! extract the homogeneous matrix from the homography
  void extract( vpRotationMatrix &aRb, vpTranslationVector &aTb) const;

  //! extract the rotational matrix and translational vector
  //! from the homography
  void extract( vpHomogeneousMatrix &aRb) const;

  //! Load an homography from a file
  void load(ifstream &f) ;
  //! Save an homography in a file
  void save(ofstream &f) const ;

  //! Print the matrix
  void print() ;

  //! build the homography from aMb and Rb
  void build() ;

 //! build the homography from aMb and Rb
  static void build(vpHomography &aHb,
		    const vpHomogeneousMatrix &aMb,
		    const vpPlane &bP) ;


  //! compute camera displacement from an homography
  void computeDisplacement(vpRotationMatrix &aRb,
			   vpTranslationVector &aTb,
			   vpColVector &n) ;
  //! compute camera displacement from an homography H
  static void computeDisplacement (const vpHomography &aHb,
				   vpRotationMatrix &aRb,
				   vpTranslationVector &aTb,
				   vpColVector &n) ;

  static void computeDisplacement(const vpMatrix H,
				  const double x,
				  const double y,
				  vpList<vpRotationMatrix> & vR,
				  vpList<vpTranslationVector> & vT,
				  vpList<vpColVector> & vN) ;
private:
  static void HartleyNormalization(int n,
				   double *x, double *y,
				   double *xn, double *yn,
				   double &xg, double &yg,
				   double &coef);
  static void HartleyDenormalization(vpHomography &aHbn,
				     vpHomography &aHb,
				     double xg1, double yg1, double coef1,
				     double xg2, double yg2, double coef2 ) ;

public:
  //! compute the homography using the DLT from normalized data
  static void HartleyDLT(int n,
			 double *xb, double *yb ,
			 double *xa, double *ya,
			 vpHomography &aHb) ;
  //! compute the homography using the DLT
  static void DLT(int n,
		  double *xb, double *yb ,
		  double *xa, double *ya,
		  vpHomography &aHb) ;

  //!  compute the homography using the linear method (HLM) proposed by Ezio Malis
  static void HLM(int n,
		    double *xb, double *yb,
		    double *xa, double *ya ,
		    bool isplan,
		    vpHomography &aHb) ;

private:
  static void  initRansac(int n,
		   double *xb, double *yb,
		   double *xa, double *ya,
		   vpColVector &x  ) ;
public:
  static bool degenerateConfiguration(vpColVector &x,int *ind) ;
  static void computeTransformation(vpColVector &x,int *ind, vpColVector &M) ;
  static double computeResidual(vpColVector &x,  vpColVector &M, vpColVector &d) ;
  static void ransac(int n,
		     double *xb, double *yb,
		     double *xa, double *ya ,
		     vpHomography &aHb) ;
} ;



#endif
