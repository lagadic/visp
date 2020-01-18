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
 * Velocity twist transformation matrix.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <assert.h>
#include <sstream>

#include <visp3/core/vpException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

/*!
  \file vpVelocityTwistMatrix.cpp

  \brief Definition of the vpVelocityTwistMatrix. Class that consider the
  particular case of twist transformation matrix that allows to
  transform a velocity skew from one frame to an other.
*/

/*!
  Copy operator that allow to set a velocity twist matrix from an other one.

  \param V : Velocity twist matrix to copy.
*/
vpVelocityTwistMatrix &vpVelocityTwistMatrix::operator=(const vpVelocityTwistMatrix &V)
{
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      rowPtrs[i][j] = V.rowPtrs[i][j];
    }
  }

  return *this;
}

/*!
  Initialize a 6x6 velocity twist matrix as identity.
*/
void vpVelocityTwistMatrix::eye()
{
  for (unsigned int i = 0; i < 6; i++)
    for (unsigned int j = 0; j < 6; j++)
      if (i == j)
        (*this)[i][j] = 1.0;
      else
        (*this)[i][j] = 0.0;
}

/*!
  Initialize a velocity twist transformation matrix as identity.
*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix() : vpArray2D<double>(6, 6) { eye(); }

/*!
  Initialize a velocity twist transformation matrix from another velocity
  twist matrix.

  \param V : Velocity twist matrix used as initializer.
*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpVelocityTwistMatrix &V) : vpArray2D<double>(6, 6) { *this = V; }

/*!

  Initialize a velocity twist transformation matrix from an homogeneous matrix
  \f$M\f$ with \f[ {\bf M} = \left[\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right] \f]

  \param M : Homogeneous matrix \f$\bf M\f$ used to initialize the velocity
  twist transformation matrix. \param full : Boolean used to indicate which
  matrix should be filled.
  - When set to true, use the complete velocity skew transformation :
  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]
  - When set to false, use the block diagonal velocity skew transformation:
  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpHomogeneousMatrix &M, bool full) : vpArray2D<double>(6, 6)
{
  if (full)
    buildFrom(M);
  else
    buildFrom(M.getRotationMatrix());
}

/*!

  Initialize a velocity twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param t : Translation vector.
  \param thetau : \f$\theta u\f$ rotation vector used to initialize rotation
  vector \f$R\f$ .

*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau)
  : vpArray2D<double>(6, 6)
{
  buildFrom(t, thetau);
}

/*!

  Initialize a velocity twist transformation matrix from a rotation vector
  with \f$\theta u \f$ parametrization.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3}\\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param thetau : \f$\theta u\f$ rotation vector used to initialize rotation
  vector \f$R\f$ .

*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpThetaUVector &thetau) : vpArray2D<double>(6, 6)
{
  buildFrom(thetau);
}

/*!

  Initialize a velocity twist transformation matrix from a translation vector
  \e t and a rotation matrix \e R.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param t : Translation vector.
  \param R : Rotation matrix.

*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R)
  : vpArray2D<double>(6, 6)
{
  buildFrom(t, R);
}

/*!

  Initialize a velocity twist transformation matrix from a rotation matrix \e
  R.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param R : Rotation matrix.

*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const vpRotationMatrix &R) : vpArray2D<double>(6, 6) { buildFrom(R); }

/*!

  Initialize a velocity twist transformation matrix from a translation vector
  \f${\bf t}=(t_x, t_y, t_z)^T\f$ and a rotation vector with \f$\theta {\bf
  u}=(\theta u_x, \theta u_y, \theta u_z)^T \f$ parametrization.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param tx,ty,tz : Translation vector in meters.

  \param tux,tuy,tuz : \f$\theta {\bf u}\f$ rotation vector expressed in
  radians used to initialize \f$R\f$.
*/
vpVelocityTwistMatrix::vpVelocityTwistMatrix(const double tx, const double ty, const double tz, const double tux,
                                             const double tuy, const double tuz)
  : vpArray2D<double>(6, 6)
{
  vpTranslationVector t(tx, ty, tz);
  vpThetaUVector tu(tux, tuy, tuz);
  buildFrom(t, tu);
}

/*!

  Operator that allows to multiply a velocity twist transformation matrix by
  an other velocity twist transformation matrix.

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::operator*(const vpVelocityTwistMatrix &V) const
{
  vpVelocityTwistMatrix p;

  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      double s = 0;
      for (int k = 0; k < 6; k++)
        s += rowPtrs[i][k] * V.rowPtrs[k][j];
      p[i][j] = s;
    }
  }
  return p;
}

/*!
  Operator that allows to multiply a velocity twist transformation matrix by a
matrix.

  As shown in the example below, this operator can be used to compute the
corresponding camera velocity skew from the joint velocities knowing the robot
jacobian.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpSimulatorCamera.h>

int main()
{
  vpSimulatorCamera robot;

  vpColVector q_vel(6); // Joint velocity on the 6 joints
  // ... q_vel need here to be initialized

  vpColVector c_v(6); // Velocity in the camera frame: vx,vy,vz,wx,wy,wz

  vpVelocityTwistMatrix cVe;  // Velocity skew transformation from camera frame to end-effector
  robot.get_cVe(cVe);

  vpMatrix eJe;       // Robot jacobian
  robot.get_eJe(eJe);

  // Compute the velocity in the camera frame
  c_v = cVe * eJe * q_vel;

  return 0;
}
  \endcode

  \exception vpException::dimensionError If M is not a 6 rows dimension
matrix.
*/
vpMatrix vpVelocityTwistMatrix::operator*(const vpMatrix &M) const
{
  if (6 != M.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply a (6x6) velocity twist matrix by a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }

  vpMatrix p(6, M.getCols());
  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < M.getCols(); j++) {
      double s = 0;
      for (unsigned int k = 0; k < 6; k++)
        s += rowPtrs[i][k] * M[k][j];
      p[i][j] = s;
    }
  }
  return p;
}

/*!

  Operator that allows to multiply a twist transformation matrix by a
  6-dimension column vector.

  \param v : Velocity skew vector.

  \exception vpException::dimensionError If v is not a 6 dimension column
  vector.

*/
vpColVector vpVelocityTwistMatrix::operator*(const vpColVector &v) const
{
  vpColVector c(6);

  if (6 != v.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (6x6) velocity twist matrix by a "
                      "(%d) column vector",
                      v.getRows()));
  }

  c = 0.0;

  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      {
        c[i] += rowPtrs[i][j] * v[j];
      }
    }
  }

  return c;
}

/*!

  Build a velocity twist transformation block diagonal matrix from a rotation
  matrix R.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param R : Rotation matrix.

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::buildFrom(const vpRotationMatrix &R)
{
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      (*this)[i][j] = R[i][j];
      (*this)[i + 3][j + 3] = R[i][j];
      (*this)[i][j + 3] = 0;
    }
  }
  return (*this);
}

/*!

  Build a velocity twist transformation matrix from a translation vector
  \e t and a rotation matrix \e R.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param t : Translation vector.

  \param R : Rotation matrix.

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R)
{
  vpMatrix skewaR = t.skew(t) * R;

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      (*this)[i][j] = R[i][j];
      (*this)[i + 3][j + 3] = R[i][j];
      (*this)[i][j + 3] = skewaR[i][j];
    }
  }

  return (*this);
}

/*!

  Initialize a velocity twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param t : Translation vector.

  \param thetau : \f$\theta {\bf u}\f$ rotation vector used to create rotation
  matrix \f${\bf R}\f$.

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::buildFrom(const vpTranslationVector &t, const vpThetaUVector &thetau)
{
  buildFrom(t, vpRotationMatrix(thetau));
  return (*this);
}

/*!

  Initialize a velocity twist transformation matrix from a rotation vector
  with \f$\theta u \f$ parametrization.

  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

  \param thetau : \f$\theta {\bf u}\f$ rotation vector used to create rotation
  matrix \f${\bf R}\f$.

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::buildFrom(const vpThetaUVector &thetau)
{
  buildFrom(vpRotationMatrix(thetau));
  return (*this);
}

/*!

  Initialize a velocity twist transformation matrix from an homogeneous matrix
  \f$M\f$ with \f[ {\bf M} = \left[\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right] \f]

  \param M : Homogeneous matrix \f$M\f$ used to initialize the velocity twist
  transformation matrix.
  \param full : Boolean used to indicate which matrix should be filled.
  - When set to true, use the complete velocity skew transformation :
  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & [{\bf t}]_\times \; {\bf R}
  \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]
  - When set to false, use the block diagonal velocity skew transformation:
  \f[ {\bf V} = \left[\begin{array}{cc} {\bf R} & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & {\bf R} \end{array} \right] \f]

*/
vpVelocityTwistMatrix vpVelocityTwistMatrix::buildFrom(const vpHomogeneousMatrix &M, bool full)
{
  if (full)
    buildFrom(M.getTranslationVector(), M.getRotationMatrix());
  else
    buildFrom(M.getRotationMatrix());

  return (*this);
}

//! Invert the velocity twist matrix.
vpVelocityTwistMatrix vpVelocityTwistMatrix::inverse() const
{
  vpVelocityTwistMatrix Wi;
  vpRotationMatrix R;
  extract(R);
  vpTranslationVector T;
  extract(T);
  vpTranslationVector RtT;
  RtT = -(R.t() * T);

  Wi.buildFrom(RtT, R.t());

  return Wi;
}

//! Invert the velocity twist matrix.
void vpVelocityTwistMatrix::inverse(vpVelocityTwistMatrix &V) const { V = inverse(); }

//! Extract the rotation matrix from the velocity twist matrix.
void vpVelocityTwistMatrix::extract(vpRotationMatrix &R) const
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      R[i][j] = (*this)[i][j];
}

//! Extract the translation vector from the velocity twist matrix.
void vpVelocityTwistMatrix::extract(vpTranslationVector &tv) const
{
  vpRotationMatrix R;
  extract(R);
  vpMatrix skTR(3, 3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      skTR[i][j] = (*this)[i][j + 3];

  vpMatrix skT = skTR * R.t();
  tv[0] = skT[2][1];
  tv[1] = skT[0][2];
  tv[2] = skT[1][0];
}

/*!

  Pretty print a velocity twist matrix. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each matrix element.
  The actual width grows in order to accomodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the matrix.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all matrix elements

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int vpVelocityTwistMatrix::print(std::ostream &s, unsigned int length, char const *intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = getCols();

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // ossFixed <<std::fixed;
  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int i = 0; i < m; ++i) {
    for (unsigned int j = 0; j < n; ++j) {
      oss.str("");
      oss << (*this)[i][j];
      if (oss.str().find("e") != std::string::npos) {
        ossFixed.str("");
        ossFixed << (*this)[i][j];
        oss.str(ossFixed.str());
      }

      values[i * n + j] = oss.str();
      size_type thislen = values[i * n + j].size();
      size_type p = values[i * n + j].find('.');

      if (p == std::string::npos) {
        maxBefore = vpMath::maximum(maxBefore, thislen);
        // maxAfter remains the same
      } else {
        maxBefore = vpMath::maximum(maxBefore, p);
        maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
      }
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = (std::min)(maxAfter, totalLength - maxBefore);
  if (maxAfter == 1)
    maxAfter = 0;

  // the following line is useful for debugging
  // std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro)
    s << intro;
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; i++) {
    s << "  ";
    for (unsigned int j = 0; j < n; j++) {
      size_type p = values[i * n + j].find('.');
      s.setf(std::ios::right, std::ios::adjustfield);
      s.width((std::streamsize)maxBefore);
      s << values[i * n + j].substr(0, p).c_str();

      if (maxAfter > 0) {
        s.setf(std::ios::left, std::ios::adjustfield);
        if (p != std::string::npos) {
          s.width((std::streamsize)maxAfter);
          s << values[i * n + j].substr(p, maxAfter).c_str();
        } else {
          assert(maxAfter > 1);
          s.width((std::streamsize)maxAfter);
          s << ".0";
        }
      }

      s << ' ';
    }
    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return (int)(maxBefore + maxAfter);
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  \deprecated You should rather use eye().

  Set the velocity twist transformation matrix to identity.

*/
void vpVelocityTwistMatrix::setIdentity() { eye(); }

#endif // #if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
