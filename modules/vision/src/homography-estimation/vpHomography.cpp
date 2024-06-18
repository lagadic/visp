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

BEGIN_VISP_NAMESPACE

vpHomography::vpHomography() : vpArray2D<double>(3, 3), m_aMb(), m_bP() { eye(); }

vpHomography::vpHomography(const vpHomography &H) : vpArray2D<double>(3, 3), m_aMb(), m_bP() { *this = H; }

vpHomography::vpHomography(const vpHomogeneousMatrix &aMb, const vpPlane &bP) : vpArray2D<double>(3, 3), m_aMb(), m_bP()
{
  build(aMb, bP);
}

vpHomography::vpHomography(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &p)
  : vpArray2D<double>(3, 3), m_aMb(), m_bP()
{
  build(tu, atb, p);
}

vpHomography::vpHomography(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &p)
  : vpArray2D<double>(3, 3), m_aMb(), m_bP()
{
  build(aRb, atb, p);
}

vpHomography::vpHomography(const vpPoseVector &arb, const vpPlane &p) : vpArray2D<double>(3, 3), m_aMb(), m_bP()
{
  build(arb, p);
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/**
 * \deprecated You should use build(const vpHomogeneousMatrix &, const vpPlane &) instead.
 */
void vpHomography::buildFrom(const vpHomogeneousMatrix &aMb, const vpPlane &bP)
{
  build(aMb, bP);
}

/**
 * \deprecated You should use build(const vpThetaUVector &, const vpTranslationVector &, const vpPlane &) instead.
 */
void vpHomography::buildFrom(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &p)
{
  build(tu, atb, p);
}

/**
 * \deprecated You should use build(const vpRotationMatrix &, const vpTranslationVector &, const vpPlane &) instead.
 */
void vpHomography::buildFrom(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &p)
{
  build(aRb, atb, p);
}

/**
 * \deprecated You should use build(const vpPoseVector &, const vpPlane &) instead.
 */
void vpHomography::buildFrom(const vpPoseVector &arb, const vpPlane &p)
{
  build(arb, p);
}
#endif

vpHomography &vpHomography::build(const vpHomogeneousMatrix &aMb, const vpPlane &bP)
{
  insert(aMb);
  insert(bP);
  build();
  return *this;
}

vpHomography &vpHomography::build(const vpThetaUVector &tu, const vpTranslationVector &atb, const vpPlane &p)
{
  insert(tu);
  insert(atb);
  insert(p);
  build();
  return *this;
}

vpHomography &vpHomography::build(const vpRotationMatrix &aRb, const vpTranslationVector &atb, const vpPlane &p)
{
  insert(aRb);
  insert(atb);
  insert(p);
  build();
  return *this;
}

vpHomography &vpHomography::build(const vpPoseVector &arb, const vpPlane &p)
{
  double tx = arb[0], ty = arb[1], tz = arb[2], tux = arb[3], tuy = arb[4], tuz = arb[5];
  m_aMb.build(tx, ty, tz, tux, tuy, tuz);
  insert(p);
  build();
  return *this;
}

/*********************************************************************/

void vpHomography::insert(const vpRotationMatrix &aRb) { m_aMb.insert(aRb); }

void vpHomography::insert(const vpHomogeneousMatrix &M) { m_aMb = M; }

void vpHomography::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix aRb(tu);
  m_aMb.insert(aRb);
}

void vpHomography::insert(const vpTranslationVector &atb) { m_aMb.insert(atb); }

void vpHomography::insert(const vpPlane &bP) { m_bP = bP; }

vpHomography vpHomography::inverse(double sv_threshold, unsigned int *rank) const
{
  vpMatrix M = (*this).convert();
  vpMatrix Minv;
  unsigned int r = M.pseudoInverse(Minv, sv_threshold);
  if (rank != nullptr) {
    *rank = r;
  }

  vpHomography H;
  unsigned int nbRows = H.getRows();
  unsigned int nbCols = H.getCols();
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      H[i][j] = Minv[i][j];
    }
  }
  return H;
}

void vpHomography::inverse(vpHomography &bHa) const { bHa = inverse(); }

void vpHomography::save(std::ofstream &f) const
{
  if (!f.fail()) {
    f << *this;
  }
  else {
    throw(vpException(vpException::ioError, "Cannot write the homography to the output stream"));
  }
}

vpHomography vpHomography::operator*(const vpHomography &H) const
{
  vpHomography Hp;
  const unsigned int nbCols = 3, nbRows = 3;
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      double s = 0.;
      for (unsigned int k = 0; k < nbCols; ++k) {
        s += (*this)[i][k] * H[k][j];
      }
      Hp[i][j] = s;
    }
  }
  return Hp;
}

vpColVector vpHomography::operator*(const vpColVector &b) const
{
  const unsigned int requiredSize = 3;
  if (b.size() != requiredSize) {
    throw(vpException(vpException::dimensionError, "Cannot multiply an homography by a vector of dimension %d",
                      b.size()));
  }

  vpColVector a(requiredSize);
  for (unsigned int i = 0; i < requiredSize; ++i) {
    a[i] = 0.;
    for (unsigned int j = 0; j < requiredSize; ++j) {
      a[i] += (*this)[i][j] * b[j];
    }
  }

  return a;
}

vpHomography vpHomography::operator*(const double &v) const
{
  const unsigned int nbData = 9; // cols x rows = 3 x 3
  vpHomography H;

  for (unsigned int i = 0; i < nbData; ++i) {
    H.data[i] = this->data[i] * v;
  }

  return H;
}

vpPoint vpHomography::operator*(const vpPoint &b_P) const
{
  vpPoint a_P;
  vpColVector v(3), v1(3);

  v[0] = b_P.get_x();
  v[1] = b_P.get_y();
  v[2] = b_P.get_w();

  v1[0] = ((*this)[0][0] * v[0]) + ((*this)[0][1] * v[1]) + ((*this)[0][2] * v[2]);
  v1[1] = ((*this)[1][0] * v[0]) + ((*this)[1][1] * v[1]) + ((*this)[1][2] * v[2]);
  v1[2] = ((*this)[2][0] * v[0]) + ((*this)[2][1] * v[1]) + ((*this)[2][2] * v[2]);

  //  v1 is equal to M v ;
  a_P.set_x(v1[0]);
  a_P.set_y(v1[1]);
  a_P.set_w(v1[2]);

  return a_P;
}

vpHomography vpHomography::operator/(const double &v) const
{
  vpHomography H;
  if (std::fabs(v) <= std::numeric_limits<double>::epsilon()) {
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /=(double v)");
  }

  double vinv = 1 / v;

  const unsigned int nbData = 9; // cols x rows = 3 x 3
  for (unsigned int i = 0; i < nbData; ++i) {
    H.data[i] = this->data[i] * vinv;
  }

  return H;
}

vpHomography &vpHomography::operator/=(double v)
{
  if (std::fabs(v) <= std::numeric_limits<double>::epsilon()) {
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /=(double v)");
  }

  double vinv = 1 / v;

  const unsigned int nbData = 9; // cols x rows = 3 x 3
  for (unsigned int i = 0; i < nbData; ++i) {
    data[i] *= vinv;
  }

  return *this;
}

vpHomography &vpHomography::operator=(const vpHomography &H)
{
  const unsigned int nbCols = 3, nbRows = 3;
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      (*this)[i][j] = H[i][j];
    }
  }

  m_aMb = H.m_aMb;
  m_bP = H.m_bP;
  return *this;
}

vpHomography &vpHomography::operator=(const vpMatrix &H)
{
  const unsigned int nbCols = 3, nbRows = 3;
  if ((H.getRows() != nbRows) || (H.getCols() != nbCols)) {
    throw(vpException(vpException::dimensionError, "The matrix is not an homography"));
  }

  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      (*this)[i][j] = H[i][j];
    }
  }

  return *this;
}

void vpHomography::load(std::ifstream &f)
{
  const unsigned int nbCols = 3, nbRows = 3;
  if (!f.fail()) {
    for (unsigned int i = 0; i < nbRows; ++i) {
      for (unsigned int j = 0; j < nbCols; ++j) {
        f >> (*this)[i][j];
      }
    }
  }
  else {
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
  const unsigned int nbCols = 3, nbRows = 3;
  vpColVector n(nbRows);
  vpColVector atb(nbRows);
  vpMatrix aRb(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    atb[i] = m_aMb[i][3];
    for (unsigned int j = 0; j < nbCols; ++j) {
      aRb[i][j] = m_aMb[i][j];
    }
  }

  m_bP.getNormal(n);

  double d = m_bP.getD();
  vpMatrix aHb = aRb - ((atb * n.t()) / d); // the d used in the equation is such as nX=d is the
                                        // plane equation. So if the plane is described by
                                        // Ax+By+Cz+D=0, d=-D

  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      (*this)[i][j] = aHb[i][j];
    }
  }
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
 * Compute \f$ ^a{\bf H}_b \f$ homography such that
 *
 * \f[ ^a{\bf H}_b = ^a{\bf R}_b + \frac{^a{\bf t}_b}{^bd}
 * { ^b{\bf n}^T}
 * \f]
 *
 * \param[out] aHb : Computed homography.
 * \param[in] aMb : Homogeneous transformation from frame a to frame b.
 * \param[in] bP : Plane at frame b.
 */
void vpHomography::build(vpHomography &aHb, const vpHomogeneousMatrix &aMb, const vpPlane &bP)
{
  const unsigned int nbCols = 3, nbRows = 3;
  vpColVector n(nbRows);
  vpColVector atb(nbRows);
  vpMatrix aRb(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    atb[i] = aMb[i][3];
    for (unsigned int j = 0; j < nbCols; ++j) {
      aRb[i][j] = aMb[i][j];
    }
  }

  bP.getNormal(n);

  double d = bP.getD();
  vpMatrix aHb_ = aRb - ((atb * n.t()) / d); // the d used in the equation is such as nX=d is the
                                         // plane equation. So if the plane is described by
                                         // Ax+By+Cz+D=0, d=-D

  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      aHb[i][j] = aHb_[i][j];
    }
  }
}
#endif

double vpHomography::det() const
{
  return ((((*this)[0][0] * (((*this)[1][1] * (*this)[2][2]) - ((*this)[1][2] * (*this)[2][1]))) -
           ((*this)[0][1] * (((*this)[1][0] * (*this)[2][2]) - ((*this)[1][2] * (*this)[2][0])))) +
          ((*this)[0][2] * (((*this)[1][0] * (*this)[2][1]) - ((*this)[1][1] * (*this)[2][0]))));
}

void vpHomography::eye()
{
  const unsigned int nbCols = 3, nbRows = 3;
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      if (i == j) {
        (*this)[i][j] = 1.0;
      }
      else {
        (*this)[i][j] = 0.0;
      }
    }
  }
}

vpImagePoint vpHomography::project(const vpCameraParameters &cam, const vpHomography &bHa, const vpImagePoint &iPa)
{
  double xa = iPa.get_u();
  double ya = iPa.get_v();
  vpHomography bGa = bHa.homography2collineation(cam);
  double z = (xa * bGa[2][0]) + (ya * bGa[2][1]) + bGa[2][2];
  double xb = ((xa * bGa[0][0]) + (ya * bGa[0][1]) + bGa[0][2]) / z;
  double yb = ((xa * bGa[1][0]) + (ya * bGa[1][1]) + bGa[1][2]) / z;

  vpImagePoint iPb(yb, xb);

  return iPb;
}

vpPoint vpHomography::project(const vpHomography &bHa, const vpPoint &Pa)
{
  double xa = Pa.get_x();
  double ya = Pa.get_y();
  double z = (xa * bHa[2][0]) + (ya * bHa[2][1]) + bHa[2][2];
  double xb = ((xa * bHa[0][0]) + (ya * bHa[0][1]) + bHa[0][2]) / z;
  double yb = ((xa * bHa[1][0]) + (ya * bHa[1][1]) + bHa[1][2]) / z;

  vpPoint Pb;
  Pb.set_x(xb);
  Pb.set_y(yb);

  return Pb;
}


void vpHomography::robust(const std::vector<double> &xb, const std::vector<double> &yb, const std::vector<double> &xa,
                          const std::vector<double> &ya, vpHomography &aHb, std::vector<bool> &inliers,
                          double &residual, double weights_threshold, unsigned int niter, bool normalization)
{
  unsigned int n = static_cast<unsigned int>(xb.size());
  if ((yb.size() != n) || (xa.size() != n) || (ya.size() != n)) {
    throw(vpException(vpException::dimensionError, "Bad dimension for robust homography estimation"));
  }

  // 4 point are required
  const unsigned int nbRequiredPoints = 4;
  if (n < nbRequiredPoints) {
    throw(vpException(vpException::fatalError, "There must be at least 4 matched points"));
  }

  try {
    std::vector<double> xan, yan, xbn, ybn;

    double xg1 = 0., yg1 = 0., coef1 = 0., xg2 = 0., yg2 = 0., coef2 = 0.;

    vpHomography aHbn;

    if (normalization) {
      vpHomography::hartleyNormalization(xb, yb, xbn, ybn, xg1, yg1, coef1);
      vpHomography::hartleyNormalization(xa, ya, xan, yan, xg2, yg2, coef2);
    }
    else {
      xbn = xb;
      ybn = yb;
      xan = xa;
      yan = ya;
    }

    unsigned int nbLinesA = 2;
    vpMatrix A(nbLinesA * n, 8);
    vpColVector X(8);
    vpColVector Y(nbLinesA * n);
    vpMatrix W(nbLinesA * n, nbLinesA * n, 0); // Weight matrix

    vpColVector w(nbLinesA * n);

    // All the weights are set to 1 at the beginning to use a classical least
    // square scheme
    w = 1;
    // Update the square matrix associated to the weights
    for (unsigned int i = 0; i < (nbLinesA * n); ++i) {
      W[i][i] = w[i];
    }

    // build matrix A
    for (unsigned int i = 0; i < n; ++i) {
      A[nbLinesA * i][0] = xbn[i];
      A[nbLinesA * i][1] = ybn[i];
      A[nbLinesA * i][2] = 1;
      A[nbLinesA * i][3] = 0;
      A[nbLinesA * i][4] = 0;
      A[nbLinesA * i][5] = 0;
      A[nbLinesA * i][6] = -xbn[i] * xan[i];
      A[nbLinesA * i][7] = -ybn[i] * xan[i];

      A[(nbLinesA * i) + 1][0] = 0;
      A[(nbLinesA * i) + 1][1] = 0;
      A[(nbLinesA * i) + 1][2] = 0;
      A[(nbLinesA * i) + 1][3] = xbn[i];
      A[(nbLinesA * i) + 1][4] = ybn[i];
      A[(nbLinesA * i) + 1][5] = 1;
      A[(nbLinesA * i) + 1][6] = -xbn[i] * yan[i];
      A[(nbLinesA * i) + 1][7] = -ybn[i] * yan[i];

      Y[nbLinesA * i] = xan[i];
      Y[(nbLinesA * i) + 1] = yan[i];
    }

    vpMatrix WA;
    vpMatrix WAp;
    unsigned int iter = 0;
    vpRobust r; // M-Estimator

    while (iter < niter) {
      WA = W * A;

      X = WA.pseudoInverse(1e-26) * W * Y;
      vpColVector residu;
      residu = Y - (A * X);

      // Compute the weights using the Tukey biweight M-Estimator
      r.MEstimator(vpRobust::TUKEY, residu, w);

      // Update the weights matrix
      for (unsigned int i = 0; i < (n * nbLinesA); ++i) {
        W[i][i] = w[i];
      }
      // Build the homography
      for (unsigned int i = 0; i < 8; ++i) {
        aHbn.data[i] = X[i];
      }
      aHbn[2][2] = 1;

      ++iter;
    }
    inliers.resize(n);
    unsigned int nbinliers = 0;
    for (unsigned int i = 0; i < n; ++i) {
      if ((w[i * 2] < weights_threshold) && (w[(i * 2) + 1] < weights_threshold)) {
        inliers[i] = false;
      }
      else {
        inliers[i] = true;
        ++nbinliers;
      }
    }

    if (normalization) {
      // H after denormalization
      vpHomography::hartleyDenormalization(aHbn, aHb, xg1, yg1, coef1, xg2, yg2, coef2);
    }
    else {
      aHb = aHbn;
    }

    residual = 0;
    vpColVector a(3), b(3), c(3);
    for (unsigned int i = 0; i < n; ++i) {
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
  }
  catch (...) {
    throw(vpException(vpException::fatalError, "Cannot estimate an homography"));
  }
}

vpImagePoint vpHomography::projection(const vpImagePoint &ipb)
{
  vpImagePoint ipa;
  double u = ipb.get_u();
  double v = ipb.get_v();

  double u_a = ((*this)[0][0] * u) + ((*this)[0][1] * v) + (*this)[0][2];
  double v_a = ((*this)[1][0] * u) + ((*this)[1][1] * v) + (*this)[1][2];
  double w_a = ((*this)[2][0] * u) + ((*this)[2][1] * v) + (*this)[2][2];

  if (std::fabs(w_a) > std::numeric_limits<double>::epsilon()) {
    ipa.set_u(u_a / w_a);
    ipa.set_v(v_a / w_a);
  }

  return ipa;
}

vpMatrix vpHomography::convert() const
{
  const unsigned int nbRows = 3, nbCols = 3;
  vpMatrix M(nbRows, nbCols);
  for (unsigned int i = 0; i < nbRows; ++i) {
    for (unsigned int j = 0; j < nbCols; ++j) {
      M[i][j] = (*this)[i][j];
    }
  }

  return M;
}

vpHomography vpHomography::collineation2homography(const vpCameraParameters &cam) const
{
  vpHomography H;
  double px = cam.get_px();
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double one_over_px = cam.get_px_inverse();
  double one_over_py = cam.get_py_inverse();
  double h00 = (*this)[0][0], h01 = (*this)[0][1], h02 = (*this)[0][2];
  double h10 = (*this)[1][0], h11 = (*this)[1][1], h12 = (*this)[1][2];
  double h20 = (*this)[2][0], h21 = (*this)[2][1], h22 = (*this)[2][2];

  double u0_one_over_px = u0 * one_over_px;
  double v0_one_over_py = v0 * one_over_py;

  double A = (h00 * one_over_px) - (h20 * u0_one_over_px);
  double B = (h01 * one_over_px) - (h21 * u0_one_over_px);
  double C = (h02 * one_over_px) - (h22 * u0_one_over_px);
  double D = (h10 * one_over_py) - (h20 * v0_one_over_py);
  double E = (h11 * one_over_py) - (h21 * v0_one_over_py);
  double F = (h12 * one_over_py) - (h22 * v0_one_over_py);

  H[0][0] = A * px;
  H[1][0] = D * px;
  H[2][0] = h20 * px;

  H[0][1] = B * py;
  H[1][1] = E * py;
  H[2][1] = h21 * py;

  H[0][2] = (A * u0) + (B * v0) + C;
  H[1][2] = (D * u0) + (E * v0) + F;
  H[2][2] = (h20 * u0) + (h21 * v0) + h22;

  return H;
}

vpHomography vpHomography::homography2collineation(const vpCameraParameters &cam) const
{
  vpHomography H;
  double px = cam.get_px();
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();
  double one_over_px = cam.get_px_inverse();
  double one_over_py = cam.get_py_inverse();
  double h00 = (*this)[0][0], h01 = (*this)[0][1], h02 = (*this)[0][2];
  double h10 = (*this)[1][0], h11 = (*this)[1][1], h12 = (*this)[1][2];
  double h20 = (*this)[2][0], h21 = (*this)[2][1], h22 = (*this)[2][2];

  double A = (h00 * px) + (u0 * h20);
  double B = (h01 * px) + (u0 * h21);
  double C = (h02 * px) + (u0 * h22);
  double D = (h10 * py) + (v0 * h20);
  double E = (h11 * py) + (v0 * h21);
  double F = (h12 * py) + (v0 * h22);

  H[0][0] = A * one_over_px;
  H[1][0] = D * one_over_px;
  H[2][0] = h20 * one_over_px;

  H[0][1] = B * one_over_py;
  H[1][1] = E * one_over_py;
  H[2][1] = h21 * one_over_py;

  double u0_one_over_px = u0 * one_over_px;
  double v0_one_over_py = v0 * one_over_py;

  H[0][2] = ((-A * u0_one_over_px) - (B * v0_one_over_py)) + C;
  H[1][2] = ((-D * u0_one_over_px) - (E * v0_one_over_py)) + F;
  H[2][2] = ((-h20 * u0_one_over_px) - (h21 * v0_one_over_py)) + h22;

  return H;
}

END_VISP_NAMESPACE
