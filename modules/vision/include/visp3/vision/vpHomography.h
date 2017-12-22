/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Homography transformation.
 *
 * Authors:
 * Muriel Pressigout
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
\file vpHomography.h

This file defines an homography transformation. This class aims to provide
some tools for homography computation.
*/

#ifndef vpHomography_hh
#define vpHomography_hh

#include <list>
#include <vector>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPoint.h>

/*!

  \class vpHomography
  \ingroup group_vision_homography

  \brief Implementation of an homography and operations on homographies.

  This class aims to compute the homography wrt. two images \cite Marchand16a.

  The vpHomography class is derived from vpArray2D<double>.

  These two images are both described by a set of points. The 2 sets (one per
  image) are sets of corresponding points : for a point in a image, there is
  the corresponding point (image of the same 3D point) in the other image
  points set.  These 2 sets are the only data needed to compute the
  homography.  One method used is the one introduced by Ezio Malis during his
  PhD \cite TheseMalis. A normalization is carried out on this points in order
to improve the conditioning of the problem, what leads to improve the
stability of the result.

  Store and compute the homography such that
  \f[
  ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
  \f]

  with
  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]

  The \ref tutorial-homography explains how to use this class.

  The example below shows also how to manipulate this class to first
  compute a ground truth homography from camera poses, project pixel
  coordinates points using an homography and lastly estimate an
  homography from a subset of 4 matched points in frame a and frame b
  respectively.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/vision/vpHomography.h>

int main()
{
  // Initialize in the object frame the coordinates in meters of 4 points that
  // belong to a planar object
  vpPoint Po[4];
  Po[0].setWorldCoordinates(-0.1, -0.1, 0);
  Po[1].setWorldCoordinates( 0.2, -0.1, 0);
  Po[2].setWorldCoordinates( 0.1,  0.1, 0);
  Po[3].setWorldCoordinates(-0.1,  0.3, 0);

  // Initialize the pose between camera frame a and object frame o
  vpHomogeneousMatrix aMo(0, 0, 1, 0, 0, 0); // Camera is 1 meter far

  // Initialize the pose between camera frame a and camera frame
  // b. These two frames correspond for example to two successive
  // camera positions
  vpHomogeneousMatrix aMb(0.2, 0.1, 0, 0, 0, vpMath::rad(2));

  // Compute the pose between camera frame b and object frame
  vpHomogeneousMatrix bMo = aMb.inverse() * aMo;

  // Initialize camera intrinsic parameters
  vpCameraParameters cam;

  // Compute the coordinates in pixels of the 4 object points in the
  // camera frame a
  vpPoint Pa[4];
  std::vector<double> xa(4), ya(4); // Coordinates in pixels of the points in frame a
  for(int i=0 ; i < 4 ; i++) {
    Pa[i] = Po[i]; Pa[i].project(aMo); // Project the points from object frame to camera frame a
    vpMeterPixelConversion::convertPoint(cam,
                                         Pa[i].get_x(), Pa[i].get_y(),
                                         xa[i], ya[i]);
  }

  // Compute the coordinates in pixels of the 4 object points in the
  // camera frame b
  vpPoint Pb[4];
  std::vector<double> xb(4), yb(4); // Coordinates in pixels of the points in frame b
  for(int i=0 ; i < 4 ; i++) {
    Pb[i] = Po[i]; Pb[i].project(bMo); // Project the points from object frame to camera frame a
  }

  // Compute equation of the 3D plane containing the points in camera frame b
  vpPlane bP(Pb[0], Pb[1], Pb[2]);

  // Compute the corresponding ground truth homography
  vpHomography aHb(aMb, bP);

  std::cout << "Ground truth homography aHb: \n" << aHb<< std::endl;

  // Compute the coordinates of the points in frame b using the ground
  // truth homography and the coordinates of the points in frame a
  vpHomography bHa = aHb.inverse();
  for(int i = 0; i < 4 ; i++){
    double inv_z = 1. / (bHa[2][0] * xa[i] + bHa[2][1] * ya[i] + bHa[2][2]);

    xb[i] = (bHa[0][0] * xa[i] + bHa[0][1] * ya[i] + bHa[0][2]) * inv_z;
    yb[i] = (bHa[1][0] * xa[i] + bHa[1][1] * ya[i] + bHa[1][2]) * inv_z;
  }

  // Estimate the homography from 4 points coordinates expressed in pixels
  vpHomography::DLT(xb, yb, xa, ya, aHb, true);
  aHb /= aHb[2][2]; // Apply a scale factor to have aHb[2][2] = 1

  std::cout << "Estimated homography aHb: \n" << aHb<< std::endl;
}
  \endcode

*/
class VISP_EXPORT vpHomography : public vpArray2D<double>
{
private:
  static const double sing_threshold; // = 0.0001;
  static const double threshold_rotation;
  static const double threshold_displacement;
  vpHomogeneousMatrix aMb;
  //  bool isplanar;
  //! reference plane coordinates  expressed in Rb
  vpPlane bP;

private:
  //! build the homography from aMb and Rb
  void build();

  //! insert a rotation matrix
  void insert(const vpHomogeneousMatrix &aRb);
  //! insert a rotation matrix
  void insert(const vpRotationMatrix &aRb);
  //! insert a theta u vector (transformation into a rotation matrix)
  void insert(const vpThetaUVector &tu);
  //! insert a translation vector
  void insert(const vpTranslationVector &atb);
  //! insert a translation vector
  void insert(const vpPlane &bP);

  static void initRansac(unsigned int n, double *xb, double *yb, double *xa, double *ya, vpColVector &x);

public:
  vpHomography();
  vpHomography(const vpHomography &H);
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpHomogeneousMatrix &aMb, const vpPlane &bP);
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from Translation and rotation and a plane
  vpHomography(const vpPoseVector &arb, const vpPlane &bP);
  virtual ~vpHomography(){};

  //! Construction from Translation and rotation and a plane
  void buildFrom(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from Translation and rotation and a plane
  void buildFrom(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from Translation and rotation  and a plane
  void buildFrom(const vpPoseVector &arb, const vpPlane &bP);
  //! Construction from homogeneous matrix and a plane
  void buildFrom(const vpHomogeneousMatrix &aMb, const vpPlane &bP);
  vpMatrix convert() const;

  void computeDisplacement(vpRotationMatrix &aRb, vpTranslationVector &atb, vpColVector &n);

  void computeDisplacement(const vpColVector &nd, vpRotationMatrix &aRb, vpTranslationVector &atb, vpColVector &n);

  void eye();

  //! invert the homography
  vpHomography inverse() const;
  //! invert the homography
  void inverse(vpHomography &Hi) const;

  //! Load an homography from a file
  void load(std::ifstream &f);

  // Multiplication by an homography
  vpHomography operator*(const vpHomography &H) const;
  // Multiplication by a scalar
  vpHomography operator*(const double &v) const;
  vpColVector operator*(const vpColVector &b) const;
  // Multiplication by a point
  vpPoint operator*(const vpPoint &H) const;

  // Division by a scalar
  vpHomography operator/(const double &v) const;
  vpHomography &operator/=(double v);
  vpHomography &operator=(const vpHomography &H);
  vpHomography &operator=(const vpMatrix &H);

  vpImagePoint projection(const vpImagePoint &p);

  /*!
    This function is not applicable to an homography that is always a
    3-by-3 matrix.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize an homography matrix"));
  };

  void save(std::ofstream &f) const;

  static void DLT(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                  const std::vector<double> &ya, vpHomography &aHb, bool normalization = true);

  static void HLM(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                  const std::vector<double> &ya, bool isplanar, vpHomography &aHb);

  static bool ransac(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                     const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers, double &residual,
                     unsigned int nbInliersConsensus, double threshold, bool normalization = true);

  static vpImagePoint project(const vpCameraParameters &cam, const vpHomography &bHa, const vpImagePoint &iPa);
  static vpPoint project(const vpHomography &bHa, const vpPoint &Pa);

  static void robust(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                     const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inlier, double &residual,
                     double weights_threshold = 0.4, unsigned int niter = 4, bool normalization = true);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //! build the homography from aMb and Rb
  static void build(vpHomography &aHb, const vpHomogeneousMatrix &aMb, const vpPlane &bP);

  static void computeDisplacement(const vpHomography &aHb, const vpColVector &nd, vpRotationMatrix &aRb,
                                  vpTranslationVector &atb, vpColVector &n);

  static void computeDisplacement(const vpHomography &aHb, vpRotationMatrix &aRb, vpTranslationVector &atb,
                                  vpColVector &n);

  static void computeDisplacement(const vpHomography &H, const double x, const double y,
                                  std::list<vpRotationMatrix> &vR, std::list<vpTranslationVector> &vT,
                                  std::list<vpColVector> &vN);
  static double computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane &oN,
                                    vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust);
  static double computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane *oN,
                                    vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust);
  static double computeResidual(vpColVector &x, vpColVector &M, vpColVector &d);
  // VVS
  static double computeRotation(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpHomogeneousMatrix &c2Mc1,
                                int userobust);
  static void computeTransformation(vpColVector &x, unsigned int *ind, vpColVector &M);
  static bool degenerateConfiguration(vpColVector &x, unsigned int *ind);
  static bool degenerateConfiguration(vpColVector &x, unsigned int *ind, double threshold_area);
  static bool degenerateConfiguration(const std::vector<double> &xb, const std::vector<double> &yb,
                                      const std::vector<double> &xa, const std::vector<double> &ya);
  static void HartleyNormalization(unsigned int n, const double *x, const double *y, double *xn, double *yn, double &xg,
                                   double &yg, double &coef);
  static void HartleyNormalization(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &xn,
                                   std::vector<double> &yn, double &xg, double &yg, double &coef);
  static void HartleyDenormalization(vpHomography &aHbn, vpHomography &aHb, double xg1, double yg1, double coef1,
                                     double xg2, double yg2, double coef2);

#endif // DOXYGEN_SHOULD_SKIP_THIS

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    \deprecated You should rather use eye().
  */
  //@{
  void setIdentity();
//@}
#endif
};

#endif
