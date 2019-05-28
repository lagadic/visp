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
 * Twist transformation matrix that allows to transform forces from one
 * frame to an other.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <assert.h>
#include <sstream>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpForceTwistMatrix.h>

/*!
  \file vpForceTwistMatrix.cpp

  \brief Definition of the vpForceTwistMatrix. Class that consider the
  particular case of twist transformation matrix that allows to
  transform a force/torque skew from one frame to an other.
*/

/*!
  Copy operator.

  \param M : Force/torque twist matrix to copy.
*/
vpForceTwistMatrix &vpForceTwistMatrix::operator=(const vpForceTwistMatrix &M)
{
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }

  return *this;
}

/*!
  Initialize the force/torque 6 by 6 twist matrix to identity.
*/
void vpForceTwistMatrix::eye()
{
  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      if (i == j)
        (*this)[i][j] = 1.0;
      else
        (*this)[i][j] = 0.0;
    }
  }
}

/*!
  Initialize a force/torque twist transformation matrix to identity.
*/
vpForceTwistMatrix::vpForceTwistMatrix() : vpArray2D<double>(6, 6) { eye(); }

/*!

  Initialize a force/torque twist transformation matrix from another
  force/torque twist matrix.

  \param F : Force/torque twist matrix used as initializer.
*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpForceTwistMatrix &F) : vpArray2D<double>(6, 6) { *this = F; }

/*!

  Initialize a force/torque twist transformation matrix from an homogeneous
  matrix \f$M\f$ with \f[ {\bf M} = \left[\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right] \f]

  \param M : Homogeneous matrix \f$\bf M\f$ used to initialize the twist
  transformation matrix.
  \param full : Boolean used to indicate which matrix should be filled.
  - When set to true, use the complete force/torque skew transformation:
  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]
  - When set to false, use the block diagonal velocity skew transformation:
  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}} & {\bf R}
  \end{array}
  \right]
  \f]

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpHomogeneousMatrix &M, bool full) : vpArray2D<double>(6, 6)
{
  if (full)
    buildFrom(M);
  else
    buildFrom(M.getRotationMatrix());
}

/*!

  Initialize a force/torque twist transformation matrix from a translation
  vector \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param t : Translation vector.

  \param thetau : \f$\theta u\f$ rotation vector used to initialize \f$R\f$.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau)
  : vpArray2D<double>(6, 6)
{
  buildFrom(t, thetau);
}

/*!

  Initialize a force/torque block diagonal twist transformation matrix from a
  rotation vector with \f$\theta u \f$ parametrization.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param thetau : \f$\theta u\f$ rotation vector used to initialize \f$R\f$.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpThetaUVector &thetau) : vpArray2D<double>(6, 6) { buildFrom(thetau); }

/*!

  Initialize a force/torque twist transformation matrix from a translation
  vector \e t and a rotation matrix \e R.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param t : Translation vector.

  \param R : Rotation matrix.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R)
  : vpArray2D<double>(6, 6)
{
  buildFrom(t, R);
}

/*!

  Initialize a force/torque block diagonal twist transformation matrix from a
  rotation matrix \e R.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param R : Rotation matrix.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpRotationMatrix &R) : vpArray2D<double>(6, 6) { buildFrom(R); }

/*!

  Initialize a force/torque twist transformation matrix from a translation
  vector \f${\bf t}=(t_x, t_y, t_z)^T\f$ and a rotation vector with \f$\theta
  {\bf u}=(\theta u_x, \theta u_y, \theta u_z)^T \f$ parametrization.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param tx,ty,tz : Translation vector in meters.

  \param tux,tuy,tuz : \f$\theta {\bf u}\f$ rotation vector expressed in
  radians used to initialize \f$R\f$.
*/
vpForceTwistMatrix::vpForceTwistMatrix(const double tx, const double ty, const double tz, const double tux,
                                       const double tuy, const double tuz)
  : vpArray2D<double>(6, 6)
{
  vpTranslationVector T(tx, ty, tz);
  vpThetaUVector tu(tux, tuy, tuz);
  buildFrom(T, tu);
}

/*!

  Operator that allows to multiply a force/torque twist transformation matrix
by an other force/torque skew transformation matrix.

\code
#include <visp3/core/vpForceTwistMatrix.h>

int main()
{
  vpForceTwistMatrix aFb, bFc;
  // ... initialize the force/torque twist transformations aFb and bFc
  // Compute the force/torque transformation from frame a to c
  vpForceTwistMatrix aFc = aFb * bFc;
}
\endcode

*/
vpForceTwistMatrix vpForceTwistMatrix::operator*(const vpForceTwistMatrix &F) const
{
  vpForceTwistMatrix Fout;

  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      double s = 0;
      for (unsigned int k = 0; k < 6; k++)
        s += rowPtrs[i][k] * F.rowPtrs[k][j];
      Fout[i][j] = s;
    }
  }
  return Fout;
}

/*!
  Operator that allows to multiply a force/torque skew transformation matrix
  by a matrix.

  \exception vpException::dimensionError : If \f$\bf M\f$ is not a 6 rows
  dimension matrix.
*/
vpMatrix vpForceTwistMatrix::operator*(const vpMatrix &M) const
{

  if (6 != M.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (6x6) force/torque twist matrix by a (%dx%d) matrix", M.getRows(), M.getCols()));
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

  Operator that allows to multiply a force/torque skew transformation matrix
by a column vector.

  \param H : Force/torque skew vector \f${\bf H} = [f_x, f_y, f_z, \tau_x,
\tau_y, \tau_z] \f$.

  For example, this operator can be used to convert a force/torque skew from
sensor frame into the probe frame :

  \f[{^p}{\bf H}_{p} = {^p}{\bf F}_s \; {^s}{\bf H}_s\f]

  The example below shows how to handle that transformation.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpForceTwistMatrix.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
  vpColVector sH = robot.getForceTorque(sH); // Get the force/torque measures
#endif

  // Set the transformation from sensor frame to the probe frame
  vpHomogeneousMatrix pMs;
  pMs[2][3] = -0.262; // tz only

  // Set the force/torque twist transformation
  vpForceTwistMatrix pFs(pMs); // Twist transformation matrix from probe to sensor frame

  // Compute the resulting force/torque in the probe frame
  vpColVector pH(6); // Force/torque in the probe frame
  pH = pFs * sH;

  return 0;
}
  \endcode

  \exception vpException::dimensionError If \f$ \bf H \f$is not a 6
  dimension vector.

*/
vpColVector vpForceTwistMatrix::operator*(const vpColVector &H) const
{
  vpColVector Hout(6);

  if (6 != H.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (6x6) force/torque twist matrix by "
                      "a %d dimension column vector",
                      H.getRows()));
  }

  Hout = 0.0;

  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      Hout[i] += rowPtrs[i][j] * H[j];
    }
  }

  return Hout;
}

/*!

  Build a force/torque twist transformation matrix from a translation vector
  \e t and a rotation matrix \e R.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param t : Translation vector.

  \param R : Rotation matrix.

*/
vpForceTwistMatrix vpForceTwistMatrix::buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R)
{
  vpMatrix skewaR = t.skew(t) * R;

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      (*this)[i][j] = R[i][j];
      (*this)[i + 3][j + 3] = R[i][j];
      (*this)[i + 3][j] = skewaR[i][j];
    }
  }
  return (*this);
}

/*!

  Build a block diagonal force/torque twist transformation matrix from a
  rotation matrix \e R.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param R : Rotation matrix.

*/
vpForceTwistMatrix vpForceTwistMatrix::buildFrom(const vpRotationMatrix &R)
{
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      (*this)[i][j] = R[i][j];
      (*this)[i + 3][j + 3] = R[i][j];
      (*this)[i + 3][j] = 0;
    }
  }
  return (*this);
}

/*!

  Initialize a force/torque twist transformation matrix from a translation
  vector \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param tv : Translation vector.

  \param thetau : \f$\theta {\bf u}\f$ rotation vector used to initialise
  \f$\bf R \f$.

*/
vpForceTwistMatrix vpForceTwistMatrix::buildFrom(const vpTranslationVector &tv, const vpThetaUVector &thetau)
{
  buildFrom(tv, vpRotationMatrix(thetau));
  return (*this);
}

/*!

  Initialize a force/torque block diagonal twist transformation matrix from a
  rotation vector with \f$\theta u \f$ parametrization.

  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}}  & {\bf R}
  \end{array}
  \right]
  \f]

  \param thetau : \f$\theta {\bf u}\f$ rotation vector used to initialise
  \f$\bf R \f$.

*/
vpForceTwistMatrix vpForceTwistMatrix::buildFrom(const vpThetaUVector &thetau)
{
  buildFrom(vpRotationMatrix(thetau));
  return (*this);
}

/*!

  Initialize a force/torque twist transformation matrix from an homogeneous
  matrix \f$M\f$ with \f[ {\bf M} = \left[\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right] \f]

  \param M : Homogeneous matrix \f$M\f$ used to initialize the velocity twist
  transformation matrix.
  \param full : Boolean used to indicate which matrix should be filled.
  - When set to true, use the complete force/torque skew transformation:
  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {[{\bf t}]}_{\times} \; {\bf R}  & {\bf R}
  \end{array}
  \right]
  \f]
  - When set to false, use the block diagonal velocity skew transformation:
  \f[
  {\bf F} = \left[
  \begin{array}{cc}
  {\bf R} & {\bf 0}_{3 \times 3} \\
  {{\bf 0}_{3 \times 3}} & {\bf R}
  \end{array}
  \right]
  \f]
*/
vpForceTwistMatrix vpForceTwistMatrix::buildFrom(const vpHomogeneousMatrix &M, bool full)
{
  if (full)
    buildFrom(M.getTranslationVector(), M.getRotationMatrix());
  else
    buildFrom(M.getRotationMatrix());

  return (*this);
}

/*!

  Pretty print a force/torque twist matrix. The data are tabulated.
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

  \sa std::ostream &operator <<(ostream &s,const vpMatrix &m)
*/
int vpForceTwistMatrix::print(std::ostream &s, unsigned int length, char const *intro) const
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

  Set the twist transformation matrix to identity.
  \sa eye()
*/
void vpForceTwistMatrix::setIdentity() { eye(); }

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
