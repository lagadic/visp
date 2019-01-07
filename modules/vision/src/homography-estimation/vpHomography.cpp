/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
  \file vpHomography.cpp
  \brief Definition de la classe vpHomography. Class that consider
  the particular case of homography
*/

#include <stdio.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRobust.h>
#include <visp3/vision/vpHomography.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

/*!
  \brief initialize an homography as Identity
*/
vpHomography::vpHomography() : vpArray2D<double>(3, 3), aMb(), bP() { eye(); }

/*!
  \brief initialize an homography from another homography
*/

vpHomography::vpHomography(const vpHomography &H) : vpArray2D<double>(3, 3), aMb(), bP() { *this = H; }

/*!
  \brief initialize an homography from another homography
*/
vpHomography::vpHomography(const vpHomogeneousMatrix &M, const vpPlane &p) : vpArray2D<double>(3, 3), aMb(), bP()
{
  buildFrom(M, p);
}

vpHomography::vpHomography(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &p)
  : vpArray2D<double>(3, 3), aMb(), bP()
{
  buildFrom(tu, atb, p);
}

vpHomography::vpHomography(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &p)
  : vpArray2D<double>(3, 3), aMb(), bP()
{
  buildFrom(aRb, atb, p);
}

vpHomography::vpHomography(const vpPoseVector &arb, const vpPlane &p) : vpArray2D<double>(3, 3), aMb(), bP()
{
  buildFrom(arb, p);
}

void vpHomography::buildFrom(const vpHomogeneousMatrix &M, const vpPlane &p)
{
  insert(M);
  insert(p);
  build();
}

void vpHomography::buildFrom(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &p)
{
  insert(tu);
  insert(atb);
  insert(p);
  build();
}

void vpHomography::buildFrom(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &p)
{
  insert(aRb);
  insert(atb);
  insert(p);
  build();
}

void vpHomography::buildFrom(const vpPoseVector &arb, const vpPlane &p)
{
  aMb.buildFrom(arb[0], arb[1], arb[2], arb[3], arb[4], arb[5]);
  insert(p);
  build();
}

/*********************************************************************/

/*!
  \brief insert the rotational component.
  To recompute the homography call build().
*/
void vpHomography::insert(const vpRotationMatrix &aRb) { aMb.insert(aRb); }

/*!
  \brief insert the rotational component.
  To recompute the homography call build().
*/
void vpHomography::insert(const vpHomogeneousMatrix &M) { this->aMb = M; }

/*!  \brief insert the rotational component, insert a
  theta u vector (transformation into a rotation matrix).
  To recompute the homography call build().

*/
void vpHomography::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix aRb(tu);
  aMb.insert(aRb);
}

/*!
  \brief  insert the translational component in a homography.
  To recompute the homography call build().
*/
void vpHomography::insert(const vpTranslationVector &atb) { aMb.insert(atb); }

/*!
  \brief  insert the reference plane.
  To recompute the homography call build().
*/
void vpHomography::insert(const vpPlane &p) { this->bP = p; }

/*!
  \brief Invert the homography

  \return  \f$\bf H^{-1}\f$
*/
vpHomography vpHomography::inverse() const
{
  vpMatrix M = (*this).convert();
  vpMatrix Minv;
  M.pseudoInverse(Minv, 1e-16);

  vpHomography H;

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      H[i][j] = Minv[i][j];

  return H;
}

/*!
  \brief Invert the homography.

  \param bHa : \f$\bf H^{-1}\f$ with H = *this.
*/
void vpHomography::inverse(vpHomography &bHa) const { bHa = inverse(); }

/*!
   Save an homography in a file.
   The laod() function allows then to read and set the homography from this
   file.

   \sa load()
 */
void vpHomography::save(std::ofstream &f) const
{
  if (!f.fail()) {
    f << *this;
  } else {
    throw(vpException(vpException::ioError, "Cannot write the homography to the output stream"));
  }
}

/*!

  Multiplication by an homography.

  \param H : Homography to multiply with.

  \code
  vpHomography aHb, bHc;
  // Initialize aHb and bHc homographies
  vpHomography aHc = aHb * bHc;
  \endcode

*/
vpHomography vpHomography::operator*(const vpHomography &H) const
{
  vpHomography Hp;
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      double s = 0.;
      for (unsigned int k = 0; k < 3; k++) {
        s += (*this)[i][k] * H[k][j];
      }
      Hp[i][j] = s;
    }
  }
  return Hp;
}

/*!
  Operation a = aHb * b.

  \param b : 3 dimension vector.
*/
vpColVector vpHomography::operator*(const vpColVector &b) const
{
  if (b.size() != 3)
    throw(vpException(vpException::dimensionError, "Cannot multiply an homography by a vector of dimension %d",
                      b.size()));

  vpColVector a(3);
  for (unsigned int i = 0; i < 3; i++) {
    a[i] = 0.;
    for (unsigned int j = 0; j < 3; j++)
      a[i] += (*this)[i][j] * b[j];
  }

  return a;
}

/*!

  Multiply an homography by a scalar.

  \param v : Value of the scalar.

  \code
  double v = 1.1;
  vpHomography aHb;
  // Initialize aHb
  vpHomography H = aHb * v;
  \endcode

*/
vpHomography vpHomography::operator*(const double &v) const
{
  vpHomography H;

  for (unsigned int i = 0; i < 9; i++) {
    H.data[i] = this->data[i] * v;
  }

  return H;
}

/*!
  From the coordinates of the point in image plane b and the homography
  between image a and b computes the coordinates of the point in image plane
  a.

  \param b_P : 2D coordinates of the point in the image plane b.

  \return A point with 2D coordinates in the image plane a.
*/
vpPoint vpHomography::operator*(const vpPoint &b_P) const
{
  vpPoint a_P;
  vpColVector v(3), v1(3);

  v[0] = b_P.get_x();
  v[1] = b_P.get_y();
  v[2] = b_P.get_w();

  v1[0] = (*this)[0][0] * v[0] + (*this)[0][1] * v[1] + (*this)[0][2] * v[2];
  v1[1] = (*this)[1][0] * v[0] + (*this)[1][1] * v[1] + (*this)[1][2] * v[2];
  v1[2] = (*this)[2][0] * v[0] + (*this)[2][1] * v[1] + (*this)[2][2] * v[2];

  //  v1 = M*v ;
  a_P.set_x(v1[0]);
  a_P.set_y(v1[1]);
  a_P.set_w(v1[2]);

  return a_P;
}
/*!

  Divide an homography by a scalar.

  \param v : Value of the scalar.

  \code
  vpHomography aHb;
  // Initialize aHb
  vpHomography H = aHb / aHb[2][2];
  \endcode

*/
vpHomography vpHomography::operator/(const double &v) const
{
  vpHomography H;
  if (std::fabs(v) <= std::numeric_limits<double>::epsilon())
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /=(double v)");

  double vinv = 1 / v;

  for (unsigned int i = 0; i < 9; i++) {
    H.data[i] = this->data[i] * vinv;
  }

  return H;
}

//! Divide  all the element of the homography matrix by v : Hij = Hij / v
vpHomography &vpHomography::operator/=(double v)
{
  // if (x == 0)
  if (std::fabs(v) <= std::numeric_limits<double>::epsilon())
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /=(double v)");

  double vinv = 1 / v;

  for (unsigned int i = 0; i < 9; i++)
    data[i] *= vinv;

  return *this;
}

/*!
  Copy operator.
  Allow operation such as aHb = H

  \param H : Homography matrix to be copied.
*/
vpHomography &vpHomography::operator=(const vpHomography &H)
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      (*this)[i][j] = H[i][j];

  aMb = H.aMb;
  bP = H.bP;
  return *this;
}
/*!
  Copy operator.
  Allow operation such as aHb = H

  \param H : Matrix to be copied.
*/
vpHomography &vpHomography::operator=(const vpMatrix &H)
{
  if (H.getRows() != 3 || H.getCols() != 3)
    throw(vpException(vpException::dimensionError, "The matrix is not an homography"));

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      (*this)[i][j] = H[i][j];

  return *this;
}

/*!
  Read an homography in a file, verify if it is really an homogeneous
  matrix.

  \param f : the file. This file has to be written using save().

  \sa save()
*/
void vpHomography::load(std::ifstream &f)
{
  if (!f.fail()) {
    for (unsigned int i = 0; i < 3; i++)
      for (unsigned int j = 0; j < 3; j++) {
        f >> (*this)[i][j];
      }
  } else {
    throw(vpException(vpException::ioError, "Cannot read the homography from the input stream"));
  }
}

/*!
  \brief Compute aHb such that

  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]
*/
void vpHomography::build()
{
  vpColVector n(3);
  vpColVector atb(3);
  vpMatrix aRb(3, 3);
  for (unsigned int i = 0; i < 3; i++) {
    atb[i] = aMb[i][3];
    for (unsigned int j = 0; j < 3; j++)
      aRb[i][j] = aMb[i][j];
  }

  bP.getNormal(n);

  double d = bP.getD();
  vpMatrix aHb = aRb - atb * n.t() / d; // the d used in the equation is such as nX=d is the
                                        // plane equation. So if the plane is described by
                                        // Ax+By+Cz+D=0, d=-D

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      (*this)[i][j] = aHb[i][j];
}

/*!
  \brief Compute aHb such that

  \f[  ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
  { ^b{\bf n}^T}
  \f]
  //note d => -d verifier
*/
void vpHomography::build(vpHomography &aHb, const vpHomogeneousMatrix &aMb, const vpPlane &bP)
{
  vpColVector n(3);
  vpColVector atb(3);
  vpMatrix aRb(3, 3);
  for (unsigned int i = 0; i < 3; i++) {
    atb[i] = aMb[i][3];
    for (unsigned int j = 0; j < 3; j++)
      aRb[i][j] = aMb[i][j];
  }

  bP.getNormal(n);

  double d = bP.getD();
  vpMatrix aHb_ = aRb - atb * n.t() / d; // the d used in the equation is such as nX=d is the
                                         // plane equation. So if the plane is described by
                                         // Ax+By+Cz+D=0, d=-D

  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      aHb[i][j] = aHb_[i][j];
}

/*!
  Set the homography as identity transformation by setting the diagonal to 1
  and all other values to 0.
*/
void vpHomography::eye()
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      if (i == j)
        (*this)[i][j] = 1.0;
      else
        (*this)[i][j] = 0.0;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated You should rather use eye().

  Set the homography as identity transformation.
  \sa eye()
*/
void vpHomography::setIdentity() { eye(); }
#endif // #if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  Given \c iPa a point with coordinates \f$(u_a,v_a)\f$ expressed in pixel in
  image a, and the homography \c bHa that links image a and b, computes the
  coordinates of the point \f$(u_b,v_b)\f$ in the image b using the camera
  parameters matrix \f$\bf K\f$.

  Compute \f$^b{\bf p} = {\bf K} \; {^b}{\bf H}_a \; {\bf K}^{-1} {^a}{\bf
  p}\f$ with \f$^a{\bf p}=(u_a,v_a,1)\f$ and \f$^b{\bf p}=(u_b,v_b,1)\f$

  \return The coordinates in pixel of the point with coordinates
  \f$(u_b,v_b)\f$.
  */
vpImagePoint vpHomography::project(const vpCameraParameters &cam, const vpHomography &bHa, const vpImagePoint &iPa)
{
  double xa = iPa.get_u();
  double ya = iPa.get_v();
  vpMatrix H = cam.get_K() * bHa.convert() * cam.get_K_inverse();
  double z = xa * H[2][0] + ya * H[2][1] + H[2][2];
  double xb = (xa * H[0][0] + ya * H[0][1] + H[0][2]) / z;
  double yb = (xa * H[1][0] + ya * H[1][1] + H[1][2]) / z;

  vpImagePoint iPb(yb, xb);

  return iPb;
}

/*!
  Given \c Pa a point with normalized coordinates \f$(x_a,y_a,1)\f$ in the
  image plane a, and the homography \c bHa that links image a and b, computes
  the normalized coordinates of the point \f$(x_b,y_b,1)\f$ in the image plane
  b.

  Compute \f$^b{\bf p} = {^b}{\bf H}_a \; {^a}{\bf p}\f$ with \f$^a{\bf
  p}=(x_a,y_a,1)\f$ and \f$^b{\bf p}=(x_b,y_b,1)\f$

  \return The coordinates in meter of the point with coordinates
  \f$(x_b,y_b)\f$.
  */
vpPoint vpHomography::project(const vpHomography &bHa, const vpPoint &Pa)
{
  double xa = Pa.get_x();
  double ya = Pa.get_y();
  double z = xa * bHa[2][0] + ya * bHa[2][1] + bHa[2][2];
  double xb = (xa * bHa[0][0] + ya * bHa[0][1] + bHa[0][2]) / z;
  double yb = (xa * bHa[1][0] + ya * bHa[1][1] + bHa[1][2]) / z;

  vpPoint Pb;
  Pb.set_x(xb);
  Pb.set_y(yb);

  return Pb;
}

/*!
  From couples of matched points \f$^a{\bf p}=(x_a,y_a,1)\f$ in image a
  and \f$^b{\bf p}=(x_b,y_b,1)\f$ in image b with homogeneous coordinates,
  computes the homography matrix by resolving \f$^a{\bf p} = ^a{\bf H}_b\;
  ^b{\bf p}\f$ using a robust estimation scheme.

  This method is to compare to DLT() except that here a robust estimator is
  used to reject couples of points that are considered as outliers.

  At least 4 couples of points are needed.

  \param xb, yb : Coordinates vector of matched points in image b. These
  coordinates are expressed in meters. \param xa, ya : Coordinates vector of
  matched points in image a. These coordinates are expressed in meters. \param
  aHb : Estimated homography that relies the transformation from image a to
  image b. \param inliers : Vector that indicates if a matched point is an
  inlier (true) or an outlier (false). \param residual : Global residual
  computed as \f$r = \sqrt{1/n \sum_{inliers} {\| {^a{\bf p} - {\hat{^a{\bf
  H}_b}} {^b{\bf p}}} \|}^{2}}\f$ with \f$n\f$ the number of inliers. \param
  weights_threshold : Threshold applied on the weights updated during the
  robust estimation and used to consider if a point is an outlier or an
  inlier. Values should be in [0:1]. A couple of matched points that have a
  weight lower than this threshold is considered as an outlier. A value equal
  to zero indicates that all the points are inliers. \param niter : Number of
  iterations of the estimation process. \param normalization : When set to
  true, the coordinates of the points are normalized. The normalization
  carried out is the one preconized by Hartley.

  \sa DLT(), ransac()
 */
void vpHomography::robust(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                          const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers,
                          double &residual, double weights_threshold, unsigned int niter, bool normalization)
{
  unsigned int n = (unsigned int)xb.size();
  if (yb.size() != n || xa.size() != n || ya.size() != n)
    throw(vpException(vpException::dimensionError, "Bad dimension for robust homography estimation"));

  // 4 point are required
  if (n < 4)
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));

  try {
    std::vector<double> xan, yan, xbn, ybn;

    double xg1 = 0., yg1 = 0., coef1 = 0., xg2 = 0., yg2 = 0., coef2 = 0.;

    vpHomography aHbn;

    if (normalization) {
      vpHomography::HartleyNormalization(xb, yb, xbn, ybn, xg1, yg1, coef1);
      vpHomography::HartleyNormalization(xa, ya, xan, yan, xg2, yg2, coef2);
    } else {
      xbn = xb;
      ybn = yb;
      xan = xa;
      yan = ya;
    }

    unsigned int nbLinesA = 2;
    vpMatrix A(nbLinesA * n, 8);
    vpColVector X(8);
    vpColVector Y(nbLinesA * n);
    vpMatrix W(nbLinesA * n, nbLinesA * n); // Weight matrix

    vpColVector w(nbLinesA * n);

    // All the weights are set to 1 at the beginning to use a classical least
    // square scheme
    w = 1;
    // Update the square matrix associated to the weights
    for (unsigned int i = 0; i < nbLinesA * n; i++) {
      W[i][i] = w[i];
    }

    // build matrix A
    for (unsigned int i = 0; i < n; i++) {
      A[nbLinesA * i][0] = xbn[i];
      A[nbLinesA * i][1] = ybn[i];
      A[nbLinesA * i][2] = 1;
      A[nbLinesA * i][3] = 0;
      A[nbLinesA * i][4] = 0;
      A[nbLinesA * i][5] = 0;
      A[nbLinesA * i][6] = -xbn[i] * xan[i];
      A[nbLinesA * i][7] = -ybn[i] * xan[i];

      A[nbLinesA * i + 1][0] = 0;
      A[nbLinesA * i + 1][1] = 0;
      A[nbLinesA * i + 1][2] = 0;
      A[nbLinesA * i + 1][3] = xbn[i];
      A[nbLinesA * i + 1][4] = ybn[i];
      A[nbLinesA * i + 1][5] = 1;
      A[nbLinesA * i + 1][6] = -xbn[i] * yan[i];
      A[nbLinesA * i + 1][7] = -ybn[i] * yan[i];

      Y[nbLinesA * i] = xan[i];
      Y[nbLinesA * i + 1] = yan[i];
    }

    vpMatrix WA;
    vpMatrix WAp;
    unsigned int iter = 0;
    vpRobust r(nbLinesA * n); // M-Estimator

    while (iter < niter) {
      WA = W * A;

      X = WA.pseudoInverse(1e-26) * W * Y;
      vpColVector residu;
      residu = Y - A * X;

      // Compute the weights using the Tukey biweight M-Estimator
      r.setIteration(iter);
      r.MEstimator(vpRobust::TUKEY, residu, w);

      // Update the weights matrix
      for (unsigned int i = 0; i < n * nbLinesA; i++) {
        W[i][i] = w[i];
      }
      // Build the homography
      for (unsigned int i = 0; i < 8; i++)
        aHbn.data[i] = X[i];
      aHbn[2][2] = 1;
      {
        vpMatrix aHbnorm = aHbn.convert();
        aHbnorm /= aHbnorm[2][2];
      }

      iter++;
    }
    inliers.resize(n);
    unsigned int nbinliers = 0;
    for (unsigned int i = 0; i < n; i++) {
      if (w[i * 2] < weights_threshold && w[i * 2 + 1] < weights_threshold)
        inliers[i] = false;
      else {
        inliers[i] = true;
        nbinliers++;
      }
    }

    if (normalization) {
      // H after denormalization
      vpHomography::HartleyDenormalization(aHbn, aHb, xg1, yg1, coef1, xg2, yg2, coef2);
    } else {
      aHb = aHbn;
    }

    residual = 0;
    vpColVector a(3), b(3), c(3);
    for (unsigned int i = 0; i < n; i++) {
      if (inliers[i]) {
        a[0] = xa[i];
        a[1] = ya[i];
        a[2] = 1;
        b[0] = xb[i];
        b[1] = yb[i];
        b[2] = 1;

        c = aHb * b;
        c /= c[2];
        residual += (a - c).sumSquare();
      }
    }

    residual = sqrt(residual / nbinliers);
  } catch (...) {
    throw(vpException(vpException::fatalError, "Cannot estimate an homography"));
  }
}

/*!
  Project the current image point (in frame b) into the frame a using the
  homography aHb.

  \param ipb : Homography defining the relation between frame a and frame b.
  \return The projected image point in the frame a.
*/
vpImagePoint vpHomography::projection(const vpImagePoint &ipb)
{
  vpImagePoint ipa;
  double u = ipb.get_u();
  double v = ipb.get_v();

  double u_a = (*this)[0][0] * u + (*this)[0][1] * v + (*this)[0][2];
  double v_a = (*this)[1][0] * u + (*this)[1][1] * v + (*this)[1][2];
  double w_a = (*this)[2][0] * u + (*this)[2][1] * v + (*this)[2][2];

  if (std::fabs(w_a) > std::numeric_limits<double>::epsilon()) {
    ipa.set_u(u_a / w_a);
    ipa.set_v(v_a / w_a);
  }

  return ipa;
}

/*!
  Converts an homography to a matrix.
  \return The 3x3 matrix corresponding to the homography.
 */
vpMatrix vpHomography::convert() const
{
  vpMatrix M(3, 3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      M[i][j] = (*this)[i][j];

  return M;
}
