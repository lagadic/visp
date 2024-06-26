/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpHomography.h
 *
 * This file defines an homography transformation. This class aims to provide
 * some tools for homography computation.
 */

#ifndef VP_HOMOGRAPHY_H
#define VP_HOMOGRAPHY_H

#include <list>
#include <vector>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpPoint.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpHomography
 * \ingroup group_vision_homography
 *
 * \brief Implementation of an homography and operations on homographies.
 *
 * This class aims to compute the homography wrt. two images \cite Marchand16a.
 *
 * The vpHomography class is derived from vpArray2D<double>.
 *
 * These two images are both described by a set of points. The 2 sets (one per
 * image) are sets of corresponding points : for a point in a image, there is
 * the corresponding point (image of the same 3D point) in the other image
 * points set. These 2 sets are the only data needed to compute the
 * homography. One method used is the one introduced by Ezio Malis during his
 * PhD \cite TheseMalis. A normalization is carried out on this points in order
 * to improve the conditioning of the problem, what leads to improve the
 * stability of the result.
 *
 * Store and compute the homography such that
 * \f[
 * ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
 * \f]
 *
 * with
 * \f[ * ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
 * { ^b{\bf n}^T}
 * \f]
 *
 * The \ref tutorial-homography explains how to use this class.
 *
 * The example below shows also how to manipulate this class to first
 * compute a ground truth homography from camera poses, project pixel
 * coordinates points using an homography and lastly estimate an
 * homography from a subset of 4 matched points in frame a and frame b
 * respectively.
 *
 * \code
 * #include <visp3/core/vpHomogeneousMatrix.h>
 * #include <visp3/core/vpMath.h>
 * #include <visp3/core/vpMeterPixelConversion.h>
 * #include <visp3/vision/vpHomography.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   // Initialize in the object frame the coordinates in meters of 4 points that
 *   // belong to a planar object
 *   vpPoint Po[4];
 *   Po[0].setWorldCoordinates(-0.1, -0.1, 0);
 *   Po[1].setWorldCoordinates( 0.2, -0.1, 0);
 *   Po[2].setWorldCoordinates( 0.1,  0.1, 0);
 *   Po[3].setWorldCoordinates(-0.1,  0.3, 0);
 *
 *   // Initialize the pose between camera frame a and object frame o
 *   vpHomogeneousMatrix aMo(0, 0, 1, 0, 0, 0); // Camera is 1 meter far
 *
 *   // Initialize the pose between camera frame a and camera frame
 *   // b. These two frames correspond for example to two successive
 *   // camera positions
 *   vpHomogeneousMatrix aMb(0.2, 0.1, 0, 0, 0, vpMath::rad(2));
 *
 *   // Compute the pose between camera frame b and object frame
 *   vpHomogeneousMatrix bMo = aMb.inverse() * aMo;
 *
 *   // Initialize camera intrinsic parameters
 *   vpCameraParameters cam;
 *
 *   // Compute the coordinates in pixels of the 4 object points in the
 *   // camera frame a
 *   vpPoint Pa[4];
 *   std::vector<double> xa(4), ya(4); // Coordinates in pixels of the points in frame a
 *   for(int i=0 ; i < 4 ; ++i) {
 *     Pa[i] = Po[i]; Pa[i].project(aMo); // Project the points from object frame to camera frame a
 *     vpMeterPixelConversion::convertPoint(cam,
 *                                         Pa[i].get_x(), Pa[i].get_y(),
 *                                         xa[i], ya[i]);
 *   }
 *
 *   // Compute the coordinates in pixels of the 4 object points in the
 *   // camera frame b
 *   vpPoint Pb[4];
 *   std::vector<double> xb(4), yb(4); // Coordinates in pixels of the points in frame b
 *   for(int i=0 ; i < 4 ; ++i) {
 *     Pb[i] = Po[i]; Pb[i].project(bMo); // Project the points from object frame to camera frame a
 *   }
 *
 *   // Compute equation of the 3D plane containing the points in camera frame b
 *   vpPlane bP(Pb[0], Pb[1], Pb[2]);
 *
 *   // Compute the corresponding ground truth homography
 *   vpHomography aHb(aMb, bP);
 *
 *   std::cout << "Ground truth homography aHb: \n" << aHb<< std::endl;
 *
 *   // Compute the coordinates of the points in frame b using the ground
 *   // truth homography and the coordinates of the points in frame a
 *   vpHomography bHa = aHb.inverse();
 *   for(int i = 0; i < 4 ; ++i){
 *     double inv_z = 1. / (bHa[2][0] * xa[i] + bHa[2][1] * ya[i] + bHa[2][2]);
 *
 *     xb[i] = (bHa[0][0] * xa[i] + bHa[0][1] * ya[i] + bHa[0][2]) * inv_z;
 *     yb[i] = (bHa[1][0] * xa[i] + bHa[1][1] * ya[i] + bHa[1][2]) * inv_z;
 *   }
 *
 *   // Estimate the homography from 4 points coordinates expressed in pixels
 *   vpHomography::DLT(xb, yb, xa, ya, aHb, true);
 *   aHb /= aHb[2][2]; // Apply a scale factor to have aHb[2][2] = 1
 *
 *   std::cout << "Estimated homography aHb: \n" << aHb<< std::endl;
  }
  \endcode
 *
*/
class VISP_EXPORT vpHomography : public vpArray2D<double>
{
public:
  /*!
   * Initialize an homography as identity.
   */
  vpHomography();
  /*!
   * Initialize an homography from another homography.
   */
  vpHomography(const vpHomography &H);
  //! Construction from translation and rotation and a plane.
  vpHomography(const vpHomogeneousMatrix &aMb, const vpPlane &bP);
  //! Construction from translation and rotation and a plane.
  vpHomography(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation and a plane.
  vpHomography(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation and a plane.
  vpHomography(const vpPoseVector &arb, const vpPlane &bP);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
//! Construction from translation and rotation and a plane
  void buildFrom(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation and a plane
  void buildFrom(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation  and a plane
  void buildFrom(const vpPoseVector &arb, const vpPlane &bP);
  //! Construction from homogeneous matrix and a plane
  void buildFrom(const vpHomogeneousMatrix &aMb, const vpPlane &bP);
#endif
//! Construction from translation and rotation and a plane
  vpHomography &build(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation and a plane
  vpHomography &build(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &bP);
  //! Construction from translation and rotation  and a plane
  vpHomography &build(const vpPoseVector &arb, const vpPlane &bP);
  //! Construction from homogeneous matrix and a plane
  vpHomography &build(const vpHomogeneousMatrix &aMb, const vpPlane &bP);

  /*!
   * Transform an homography from pixel space to calibrated domain.
   *
   * Given homography \f$\bf G\f$ corresponding to the collineation matrix in the pixel space,
   * compute the homography matrix \f$\bf H\f$ in the Euclidean space or calibrated domain using:
   * \f[ {\bf H} = {\bf K}^{-1} {\bf G} {\bf K} \f]
   * \param[in] cam : Camera parameters used to fill \f${\bf K}\f$ matrix such as
   * \f[{\bf K} =
   * \left[ \begin{array}{ccc}
   * p_x & 0   & u_0  \\
   * 0   & p_y & v_0 \\
   * 0   & 0   & 1
   * \end{array}\right]
   * \f]
   * \return The corresponding homography matrix \f$\bf H\f$ in the Euclidean space or calibrated domain.
   *
   * \sa homography2collineation()
   */
  vpHomography collineation2homography(const vpCameraParameters &cam) const;

  /*!
   * Converts an homography to a matrix.
   * \return The 3x3 matrix corresponding to the homography.
   */
  vpMatrix convert() const;

  /*!
   * Compute the camera displacement between two images from the homography \f$
   * {^a}{\bf H}_b \f$ which is here an implicit parameter (*this).
   *
   * \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.
   * \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.
   * \param n : Normal vector to the plane as an output.
   */
  void computeDisplacement(vpRotationMatrix &aRb, vpTranslationVector &atb, vpColVector &n);

  /*!
   * Compute the camera displacement between two images from the homography \f$
   * {^a}{\bf H}_b \f$ which is here an implicit parameter (*this).
   *
   * Camera displacement between \f$ {^a}{\bf p} \f$ and \f$ {^a}{\bf p} \f$ is
   * represented as a rotation matrix \f$ {^a}{\bf R}_b \f$ and a translation
   * vector \f$ ^a{\bf t}_b \f$ from which an homogeneous matrix can be build
   * (vpHomogeneousMatrix).
   *
   * \param nd : Input normal vector to the plane used to compar with the normal
   * vector \e n extracted from the homography.
   * \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.
   * \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.
   * \param n : Normal vector to the plane as an output.
   */
  void computeDisplacement(const vpColVector &nd, vpRotationMatrix &aRb, vpTranslationVector &atb, vpColVector &n);

  /*!
   * Return homography determinant.
   */
  double det() const;

  /*!
   * Set the homography as identity transformation by setting the diagonal to 1
   * and all other values to 0.
   */
  void eye();

  /*!
   * Transform an homography from calibrated domain to pixel space.
   *
   * Given homography \f$\bf H\f$ in the Euclidean space or in the calibrated domain,
   * compute the homography \f$\bf G\f$ corresponding to the collineation matrix in the pixel space using:
   * \f[ {\bf G} = {\bf K} {\bf H} {\bf K}^{-1} \f]
   * \param[in] cam : Camera parameters used to fill \f${\bf K}\f$ matrix such as
   * \f[{\bf K} =
   * \left[ \begin{array}{ccc}
   * p_x & 0   & u_0  \\
   * 0   & p_y & v_0 \\
   * 0   & 0   & 1
   * \end{array}\right]
   * \f]
   * \return The corresponding collineation matrix \f$\bf G\f$ in the pixel space.
   *
   * \sa collineation2homography()
   */
  vpHomography homography2collineation(const vpCameraParameters &cam) const;

  /*!
   * Return inverted homography.
   *
   * \param[in] sv_threshold : Threshold used to test the singular values. If
   * a singular value is lower than this threshold we consider that the
   * homography is not full rank.
   *
   * \param[out] rank : Rank of the homography that should be 3.
   *
   * \return  \f$\bf H^{-1}\f$
   */
  vpHomography inverse(double sv_threshold = 1e-16, unsigned int *rank = nullptr) const;

  /*!
   * Invert the homography.
   *
   * \param bHa : \f$\bf H^{-1}\f$ with H = *this.
   */
  void inverse(vpHomography &bHa) const;

  /*!
   * Read an homography in a file, verify if it is really an homogeneous
   * matrix.
   *
   * \param f : the file. This file has to be written using save().
   *
   * \sa save()
   */
  void load(std::ifstream &f);

  /*!
   * Multiplication by an homography.
   *
   * \param H : Homography to multiply with.
   *
   * \code
   * vpHomography aHb, bHc;
   * // Initialize aHb and bHc homographies
   * vpHomography aHc = aHb * bHc;
   * \endcode
   */
  vpHomography operator*(const vpHomography &H) const;

  /*!
   * Multiply an homography by a scalar.
   *
   * \param v : Value of the scalar.
   *
   * \code
   * double v = 1.1;
   * vpHomography aHb;
   * // Initialize aHb
   * vpHomography H = aHb * v;
   * \endcode
   */
  vpHomography operator*(const double &v) const;

  /*!
   * Operation a = aHb * b.
   *
   * \param b : 3 dimension vector.
   */
  vpColVector operator*(const vpColVector &b) const;

  /*!
   * From the coordinates of the point in image plane b and the homography
   * between image a and b computes the coordinates of the point in image plane
   * a.
   *
   * \param b_P : 2D coordinates of the point in the image plane b.
   *
   * \return A point with 2D coordinates in the image plane a.
   */
  vpPoint operator*(const vpPoint &b_P) const;

  /*!
   * Divide an homography by a scalar.
   *
   * \param v : Value of the scalar.
   *
   * \code
   * vpHomography aHb;
   * // Initialize aHb
   * vpHomography H = aHb / aHb[2][2];
   * \endcode
   */
  vpHomography operator/(const double &v) const;

  /*!
   * Divide all the element of the homography matrix by v : Hij = Hij / v
   */
  vpHomography &operator/=(double v);

  /*!
   * Copy operator.
   * Allow operation such as aHb = H
   *
   * \param H : Homography matrix to be copied.
   */
  vpHomography &operator=(const vpHomography &H);

  /*!
   * Copy operator.
   * Allow operation such as aHb = H
   *
   * \param H : Matrix to be copied.
   */
  vpHomography &operator=(const vpMatrix &H);

  /*!
   * Project the current image point (in frame b) into the frame a using the
   * homography aHb.
   *
   * \param ipb : Homography defining the relation between frame a and frame b.
   * \return The projected image point in the frame a.
   */
  vpImagePoint projection(const vpImagePoint &ipb);

  /*!
   * This function is not applicable to an homography that is always a
   * 3-by-3 matrix.
   * \exception vpException::fatalError When this function is called.
   */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize an homography matrix"));
  };

  /*!
   * Save an homography in a file.
   * The load() function allows then to read and set the homography from this
   * file.
   *
   * \sa load()
   */
  void save(std::ofstream &f) const;

  /*!
   * From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
   * and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
   * computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
   * ^b{\bf p}\f$ using the DLT (Direct Linear Transform) algorithm.
   *
   * At least 4 couples of points are needed.
   *
   * To do so, we use the DLT algorithm on the data,
   * ie we resolve the linear system  by SDV : \f$\bf{Ah} =0\f$ where
   * \f$\bf{h}\f$ is the vector with the terms of \f$^a{\bf H}_b\f$ and
   * \f$\mathbf{A}\f$ depends on the  points coordinates.
   *
   * For each point, in homogeneous coordinates we have:
   * \f[
   * ^a{\bf p} = ^a{\bf H}_b\; ^b{\bf p}
   * \f]
   * which is equivalent to:
   * \f[
   * ^a{\bf p} \times {^a{\bf H}_b \; ^b{\bf p}}  =0
   * \f]
   * If we note \f$\mathbf{h}_j^T\f$ the  \f$j^{\textrm{th}}\f$ line of
   * \f$^a{\bf H}_b\f$, we can write: \f[ ^a{\bf H}_b \; ^b{\bf p}  = \left(
   * \begin{array}{c}\mathbf{h}_1^T \;^b{\bf p} \\\mathbf{h}_2^T \; ^b{\bf p}
   * \\\mathbf{h}_3^T \;^b{\bf p} \end{array}\right) \f]
   *
   * Setting \f$^a{\bf p}=(x_{a},y_{a},w_{a})\f$, the cross product  can be
   * rewritten by: \f[ ^a{\bf p} \times ^a{\bf H}_b \; ^b{\bf p}  =\left(
   * \begin{array}{c}y_{a}\mathbf{h}_3^T \; ^b{\bf p}-w_{a}\mathbf{h}_2^T \;
   * ^b{\bf p} \\w_{a}\mathbf{h}_1^T \; ^b{\bf p} -x_{a}\mathbf{h}_3^T \; ^b{\bf
   * p} \\x_{a}\mathbf{h}_2^T \; ^b{\bf p}- y_{a}\mathbf{h}_1^T \; ^b{\bf
   * p}\end{array}\right) \f]
   *
   * \f[
   * \underbrace{\left( \begin{array}{ccc}\mathbf{0}^T & -w_{a} \; ^b{\bf p}^T
   * & y_{a} \; ^b{\bf p}^T     \\     w_{a}
   * \; ^b{\bf p}^T&\mathbf{0}^T & -x_{a} \; ^b{\bf p}^T      \\
   * -y_{a} \; ^b{\bf p}^T & x_{a} \; ^b{\bf p}^T &
   * \mathbf{0}^T\end{array}\right)}_{\mathbf{A}_i (3\times 9)}
   * \underbrace{\left( \begin{array}{c}\mathbf{h}_{1}^{T}      \\
   * \mathbf{h}_{2}^{T}\\\mathbf{h}_{3}^{T}\end{array}\right)}_{\mathbf{h}
   * (9\times 1)}=0 \f]
   *
   * leading to an homogeneous system to be solved:
   * \f$\mathbf{A}\mathbf{h}=0\f$ with \f$\mathbf{A}=\left(\mathbf{A}_1^T, ...,
   * \mathbf{A}_i^T, ..., \mathbf{A}_n^T \right)^T\f$.
   *
   * It can be solved using an SVD decomposition:
   * \f[\bf A = UDV^T \f]
   * <b>h</b> is the column of <b>V</b> associated with the smallest singular
   * value of <b>A
   * </b>
   *
   * \param xb, yb : Coordinates vector of matched points in image b. These
   * coordinates are expressed in meters.
   * \param xa, ya : Coordinates vector of
   * matched points in image a. These coordinates are expressed in meters.
   * \param
   * aHb : Estimated homography that relies the transformation from image a to
   * image b.
   * \param normalization : When set to true, the coordinates of the
   * points are normalized. The normalization carried out is the one preconized
   * by Hartley.
   *
   * \exception vpMatrixException::rankDeficient : When the rank of the matrix
   * that should be 8 is deficient.
   */
  static void DLT(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                  const std::vector<double> &ya, vpHomography &aHb, bool normalization = true);

  /*!
   * From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
   * and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
   * computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
   * ^b{\bf p}\f$ using Ezio Malis linear method (HLM) \cite Malis00b.
   *
   * This method can consider points that are planar or non planar. The algorithm
   * for planar scene implemented in this file is described in Ezio Malis PhD
   * thesis \cite TheseMalis.
   *
   * \param xb, yb : Coordinates vector of matched points in image b. These
   * coordinates are expressed in meters.
   * \param xa, ya : Coordinates vector of
   * matched points in image a. These coordinates are expressed in meters.
   * \param isplanar : If true the points are assumed to be in a plane, otherwise there
   * are assumed to be non planar.
   * \param aHb : Estimated homography that relies
   * the transformation from image a to image b.
   *
   * If the boolean isplanar is true the points are assumed to be in a plane
   * otherwise there are assumed to be non planar.
   *
   * \sa DLT() when the scene is planar.
   */
  static void HLM(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                  const std::vector<double> &ya, bool isplanar, vpHomography &aHb);

  /*!
   * From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
   * and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
   * computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
   * ^b{\bf p}\f$ using Ransac algorithm.
   *
   * \param xb, yb : Coordinates vector of matched points in image b. These
   * coordinates are expressed in meters.
   * \param xa, ya : Coordinates vector of
   * matched points in image a. These coordinates are expressed in meters.
   * \param aHb : Estimated homography that relies the transformation from image a to
   * image b.
   * \param inliers : Vector that indicates if a matched point is an
   * inlier (true) or an outlier (false).
   * \param residual : Global residual
   * computed as \f$r = \sqrt{1/n \sum_{inliers} {\| {^a{\bf p} - {\hat{^a{\bf
   * H}_b}} {^b{\bf p}}} \|}^{2}}\f$ with \f$n\f$ the number of inliers.
   * \param nbInliersConsensus : Minimal number of points requested to fit the
   * estimated homography.
   * \param threshold : Threshold for outlier removing. A point is considered as
   * an outlier if the reprojection error \f$\| {^a{\bf p} - {\hat{^a{\bf H}_b}}
   * {^b{\bf p}}} \|\f$ is greater than this threshold.
   * \param normalization : When set to true, the coordinates of the points are
   * normalized. The normalization carried out is the one preconized by Hartley.
   *
   * \return true if the homography could be computed, false otherwise.
   */
  static bool ransac(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                     const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers, double &residual,
                     unsigned int nbInliersConsensus, double threshold, bool normalization = true);

  /*!
   * Given `iPa` a pixel with coordinates \f$(u_a,v_a)\f$ in
   * image a, and the homography `bHa` in the Euclidean space or calibrated domain that links image a and b, computes the
   * coordinates of the pixel \f$(u_b,v_b)\f$ in the image b using the camera
   * parameters matrix \f$\bf K\f$.
   *
   * Compute \f$^b{\bf p} = {\bf K} \; {^b}{\bf H}_a \; {\bf K}^{-1} {^a}{\bf
   * p}\f$ with \f$^a{\bf p}=(u_a,v_a,1)\f$ and \f$^b{\bf p}=(u_b,v_b,1)\f$
   *
   * \return The coordinates in pixel of the point with coordinates
   * \f$(u_b,v_b)\f$.
   */
  static vpImagePoint project(const vpCameraParameters &cam, const vpHomography &bHa, const vpImagePoint &iPa);

  /*!
   * Given `Pa` a point with normalized coordinates \f$(x_a,y_a,1)\f$ in the
   * image plane a, and the homography `bHa` in the Euclidean space that links image a and b, computes
   * the normalized coordinates of the point \f$(x_b,y_b,1)\f$ in the image plane
   * b.
   *
   * Compute \f$^b{\bf p} = {^b}{\bf H}_a \; {^a}{\bf p}\f$ with \f$^a{\bf
   * p}=(x_a,y_a,1)\f$ and \f$^b{\bf p}=(x_b,y_b,1)\f$
   *
   * \return The coordinates in meter of the point with coordinates
   * \f$(x_b,y_b)\f$.
   */
  static vpPoint project(const vpHomography &bHa, const vpPoint &Pa);

  /*!
   * From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
   * and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
   * computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
   * ^b{\bf p}\f$ using a robust estimation scheme.
   *
   * This method is to compare to DLT() except that here a robust estimator is
   * used to reject couples of points that are considered as outliers.
   *
   * At least 4 couples of points are needed.
   *
   * \param xb, yb : Coordinates vector of matched points in image b. These
   * coordinates are expressed in meters.
   * \param xa, ya : Coordinates vector of
   * matched points in image a. These coordinates are expressed in meters.
   * \param aHb : Estimated homography that relies the transformation from image a to
   * image b.
   * \param inliers : Vector that indicates if a matched point is an
   * inlier (true) or an outlier (false).
   * \param residual : Global residual computed as
   * \f$r = \sqrt{1/n \sum_{inliers} {\| {^a{\bf p} - {\hat{^a{\bf H}_b}} {^b{\bf p}}} \|}^{2}}\f$
   * with \f$n\f$ the number of inliers.
   * \param weights_threshold : Threshold applied on the weights updated during the
   * robust estimation and used to consider if a point is an outlier or an
   * inlier. Values should be in [0:1]. A couple of matched points that have a
   * weight lower than this threshold is considered as an outlier. A value equal
   * to zero indicates that all the points are inliers.
   * \param niter : Number of iterations of the estimation process.
   * \param normalization : When set to true, the coordinates of the points are normalized.
   * The normalization carried out is the one preconized by Hartley.
   *
   * \sa DLT(), ransac()
   */
  static void robust(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                     const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers, double &residual,
                     double weights_threshold = 0.4, unsigned int niter = 4, bool normalization = true);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  static void build(vpHomography &aHb, const vpHomogeneousMatrix &aMb, const vpPlane &bP);

  static void computeDisplacement(const vpHomography &aHb, const vpColVector &nd, vpRotationMatrix &aRb,
                                  vpTranslationVector &atb, vpColVector &n);

  static void computeDisplacement(const vpHomography &aHb, vpRotationMatrix &aRb, vpTranslationVector &atb,
                                  vpColVector &n);

  static void computeDisplacement(const vpHomography &H, double x, double y, std::list<vpRotationMatrix> &vR,
                                  std::list<vpTranslationVector> &vT, std::list<vpColVector> &vN);
  static double computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane &oN,
                                    vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust);
  static double computeDisplacement(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpPlane *oN,
                                    vpHomogeneousMatrix &c2Mc1, vpHomogeneousMatrix &c1Mo, int userobust);
  static double computeResidual(vpColVector &x, vpColVector &M, vpColVector &d);
  // VVS
  static double computeRotation(unsigned int nbpoint, vpPoint *c1P, vpPoint *c2P, vpHomogeneousMatrix &c2Mc1,
                                int userobust);
  static void computeTransformation(vpColVector &x, unsigned int *ind, vpColVector &M);
  static bool degenerateConfiguration(const vpColVector &x, unsigned int *ind);
  static bool degenerateConfiguration(const vpColVector &x, unsigned int *ind, double threshold_area);
  static bool degenerateConfiguration(const std::vector<double> &xb, const std::vector<double> &yb,
                                      const std::vector<double> &xa, const std::vector<double> &ya);
  static void hartleyNormalization(unsigned int n, const double *x, const double *y, double *xn, double *yn, double &xg,
                                   double &yg, double &coef);
  static void hartleyNormalization(const std::vector<double> &x, const std::vector<double> &y, std::vector<double> &xn,
                                   std::vector<double> &yn, double &xg, double &yg, double &coef);
  static void hartleyDenormalization(vpHomography &aHbn, vpHomography &aHb, double xg1, double yg1, double coef1,
                                     double xg2, double yg2, double coef2);

#endif // DOXYGEN_SHOULD_SKIP_THIS

private:
  static const double m_sing_threshold; /* equals 0.0001 */
  static const double m_threshold_rotation;
  static const double m_threshold_displacement;
  vpHomogeneousMatrix m_aMb;

  //! Reference plane coordinates  expressed in frame b
  vpPlane m_bP;

  //! Build the homography from aMb and Rb
  void build();

  /*!
   * Insert the rotational part of an homogeneous transformation.
   * To recompute the homography call build().
   */
  void insert(const vpHomogeneousMatrix &aRb);

  /*!
   * Insert the rotational matrix.
   * To recompute the homography call build().
   */
  void insert(const vpRotationMatrix &aRb);

  /*!
   * Insert a theta u vector transformed internally into a rotation matrix.
   * To recompute the homography call build().
   */
  void insert(const vpThetaUVector &tu);

  /*!
   * Insert a translation vector.
   * To recompute the homography call build().
   */
  void insert(const vpTranslationVector &atb);

  /*!
   * Insert the reference plane.
   * To recompute the homography call build().
   */
  void insert(const vpPlane &bP);

  static void initRansac(unsigned int n, double *xb, double *yb, double *xa, double *ya, vpColVector &x);
};

END_VISP_NAMESPACE

#endif
