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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 *
*****************************************************************************/

/*!
  \file vpPoseVector.cpp
  \brief  Pose vector.

*/

#include <assert.h>
#include <sstream>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpPoseVector.h>

BEGIN_VISP_NAMESPACE

/*!

  Default constructor that construct a 6 dimension pose vector \f$ [\bf t,
  \theta \bf u]^\top\f$ where \f$ \theta \bf u\f$ is a rotation vector
  \f$[\theta u_x, \theta u_y, \theta u_z]^\top\f$ and \f$ \bf t \f$ is a
  translation vector \f$[t_x, t_y, t_z]^\top\f$.

  The pose vector is initialized to zero.

*/
vpPoseVector::vpPoseVector() : vpArray2D<double>(6, 1) { }

/*!

  Construct a 6 dimension pose vector \f$ [\bf{t}, \theta
  \bf{u}]^\top\f$ from 3 translations and 3 \f$ \theta \bf{u}\f$
  angles.

  Translations are expressed in meters, while rotations in radians.

  \param tx,ty,tz : Translations \f$[t_x, t_y, t_z]^\top\f$
  respectively along the x, y and z axis (in meters).

  \param tux,tuy,tuz : Rotations \f$[\theta u_x, \theta u_y, \theta
  u_z]^\top\f$ respectively around the x, y and z axis (in radians).

*/
vpPoseVector::vpPoseVector(double tx, double ty, double tz, double tux, double tuy, double tuz)
  : vpArray2D<double>(6, 1)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  (*this)[index_0] = tx;
  (*this)[index_1] = ty;
  (*this)[index_2] = tz;

  (*this)[index_3] = tux;
  (*this)[index_4] = tuy;
  (*this)[index_5] = tuz;
}

/*!

  Construct a 6 dimension pose vector \f$ [\bf t, \theta \bf
  u]^\top\f$ from a translation vector \f$ \bf tv \f$ and a \f$\theta
  \bf u\f$ vector.

  \param tv : Translation vector \f$ \bf t \f$.
  \param tu : \f$\theta \bf u\f$ rotation  vector.

*/
vpPoseVector::vpPoseVector(const vpTranslationVector &tv, const vpThetaUVector &tu) : vpArray2D<double>(6, 1)
{
  build(tv, tu);
}

/*!

  Construct a 6 dimension pose vector \f$ [\bf t, \theta \bf
  u]^\top\f$ from a translation vector \f$ \bf t \f$ and a rotation
  matrix \f$ \bf R \f$.

  \param tv : Translation vector \f$ \bf t \f$.

  \param R : Rotation matrix \f$ \bf R \f$ from which \f$\theta \bf
  u\f$ vector is extracted to initialise the pose vector.

*/
vpPoseVector::vpPoseVector(const vpTranslationVector &tv, const vpRotationMatrix &R) : vpArray2D<double>(6, 1)
{
  build(tv, R);
}

/*!

  Construct a 6 dimension pose vector \f$ [\bf t, \theta \bf
  u]^\top\f$ from an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation
  \f$ \bf t \f$ and \f$\theta \bf u \f$ vectors are extracted to
  initialize the pose vector.

*/
vpPoseVector::vpPoseVector(const vpHomogeneousMatrix &M) : vpArray2D<double>(6, 1) { build(M); }

/*!

  Set the 6 dimension pose vector \f$ [\bf{t}, \theta
  \bf{u}]^\top\f$ from 3 translations and 3 \f$ \theta \bf{u}\f$
  angles.

  Translations are expressed in meters, while rotations in radians.

  \param tx,ty,tz : Translations \f$[t_x, t_y, t_z]^\top\f$
  respectively along the x, y and z axis (in meters).

  \param tux,tuy,tuz : Rotations \f$[\theta u_x, \theta u_y, \theta
  u_z]^\top\f$ respectively around the x, y and z axis (in radians).

*/
void vpPoseVector::set(double tx, double ty, double tz, double tux, double tuy, double tuz)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  (*this)[index_0] = tx;
  (*this)[index_1] = ty;
  (*this)[index_2] = tz;

  (*this)[index_3] = tux;
  (*this)[index_4] = tuy;
  (*this)[index_5] = tuz;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const double &, const double &, const double &, const double &, const double &, const double &) instead.
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from 3 translations and 3 \f$ \theta \bf{u}\f$ angles.

  Translations are expressed in meters, while rotations in radians.

  \param tx,ty,tz : Translations \f$[t_x, t_y, t_z]^\top\f$
  respectively along the x, y and z axis (in meters).

  \param tux,tuy,tuz : Rotations \f$[\theta u_x, \theta u_y, \theta
  u_z]^\top\f$ respectively around the x, y and z axis (in radians).

  \return The build pose vector.

  \sa set()
*/
vpPoseVector vpPoseVector::buildFrom(double tx, double ty, double tz, double tux, double tuy, double tuz)
{
  build(tx, ty, tz, tux, tuy, tuz);
  return *this;
}

/*!
  \deprecated You should use build(const vpHomogeneousMatrix &) instead.
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$ from
  an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation \f$
  \bf t \f$ and \f$\theta \bf u \f$ vectors are extracted to initialize
  the pose vector.

  \return The build pose vector.

*/
vpPoseVector vpPoseVector::buildFrom(const vpHomogeneousMatrix &M)
{
  build(M);
  return *this;
}

/*!
  \deprecated You should use build(const vpTranslationVector &, const vpThetaUVector &) instead.
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a \f$\theta \bf u\f$
  vector.

  \param tv : Translation vector \f$ \bf t \f$.
  \param tu : \f$\theta \bf u\f$ rotation  vector.

  \return The build pose vector.
*/
vpPoseVector vpPoseVector::buildFrom(const vpTranslationVector &tv, const vpThetaUVector &tu)
{
  build(tv, tu);
  return *this;
}

/*!
  \deprecated You should use build(const vpTranslationVector &, const vpRotationMatrix &) instead.
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a rotation matrix \f$
  \bf R \f$.

  \param tv : Translation vector \f$ \bf t \f$.

  \param R : Rotation matrix \f$ \bf R \f$ from which \f$\theta \bf
  u\f$ vector is extracted to initialise the pose vector.

  \return The build pose vector.
*/
vpPoseVector vpPoseVector::buildFrom(const vpTranslationVector &tv, const vpRotationMatrix &R)
{
  build(tv, R);
  return *this;
}
#endif

/*!
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from 3 translations and 3 \f$ \theta \bf{u}\f$ angles.

  Translations are expressed in meters, while rotations in radians.

  \param tx,ty,tz : Translations \f$[t_x, t_y, t_z]^\top\f$
  respectively along the x, y and z axis (in meters).

  \param tux,tuy,tuz : Rotations \f$[\theta u_x, \theta u_y, \theta
  u_z]^\top\f$ respectively around the x, y and z axis (in radians).

  \return The build pose vector.

  \sa set()
*/
vpPoseVector &vpPoseVector::build(const double &tx, const double &ty, const double &tz, const double &tux, const double &tuy, const double &tuz)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  (*this)[index_0] = tx;
  (*this)[index_1] = ty;
  (*this)[index_2] = tz;

  (*this)[index_3] = tux;
  (*this)[index_4] = tuy;
  (*this)[index_5] = tuz;
  return *this;
}

/*!
  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$ from
  an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation \f$
  \bf t \f$ and \f$\theta \bf u \f$ vectors are extracted to initialize
  the pose vector.

  \return The build pose vector.

*/
vpPoseVector &vpPoseVector::build(const vpHomogeneousMatrix &M)
{
  vpRotationMatrix R;
  M.extract(R);
  vpTranslationVector tv;
  M.extract(tv);
  build(tv, R);
  return *this;
}

/*!

  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a \f$\theta \bf u\f$
  vector.

  \param tv : Translation vector \f$ \bf t \f$.
  \param tu : \f$\theta \bf u\f$ rotation  vector.

  \return The build pose vector.
*/
vpPoseVector &vpPoseVector::build(const vpTranslationVector &tv, const vpThetaUVector &tu)
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    (*this)[i] = tv[i];
    (*this)[i + 3] = tu[i];
  }
  return *this;
}

/*!

  Build a 6 dimension pose vector \f$ [\bf t, \theta \bf u]^\top\f$
  from a translation vector \f$ \bf t \f$ and a rotation matrix \f$
  \bf R \f$.

  \param tv : Translation vector \f$ \bf t \f$.

  \param R : Rotation matrix \f$ \bf R \f$ from which \f$\theta \bf
  u\f$ vector is extracted to initialise the pose vector.

  \return The build pose vector.
*/
vpPoseVector &vpPoseVector::build(const vpTranslationVector &tv, const vpRotationMatrix &R)
{
  vpThetaUVector tu;
  tu.build(R);

  build(tv, tu);
  return *this;
}

/*!
  Extract the translation vector from the homogeneous matrix.
*/
void vpPoseVector::extract(vpTranslationVector &tv) const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  tv[index_0] = (*this)[index_0];
  tv[index_1] = (*this)[index_1];
  tv[index_2] = (*this)[index_2];
}

/*!
  Extract the rotation as a \f$\theta \bf u\f$ vector.
*/
void vpPoseVector::extract(vpThetaUVector &tu) const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  tu[index_0] = (*this)[index_3];
  tu[index_1] = (*this)[index_4];
  tu[index_2] = (*this)[index_5];
}
/*!
  Extract the rotation as a quaternion vector.
*/
void vpPoseVector::extract(vpQuaternionVector &q) const
{
  vpRotationMatrix R((*this)[3], (*this)[4], (*this)[5]);
  q.build(R);
}
/*!
  Extract the rotation as a rotation matrix.
*/
void vpPoseVector::extract(vpRotationMatrix &R) const { R.build((*this)[3], (*this)[4], (*this)[5]); }
/*!
  Return the translation vector that corresponds to the translation part of
  the pose vector.
 */
vpTranslationVector vpPoseVector::getTranslationVector() const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  vpTranslationVector tr((*this)[index_0], (*this)[index_1], (*this)[index_2]);
  return tr;
}

/*!
  Return the rotation matrix that corresponds to the rotation part of the
  pose vector.
 */
vpRotationMatrix vpPoseVector::getRotationMatrix() const
{
  vpRotationMatrix R((*this)[3], (*this)[4], (*this)[5]);
  return R;
}

/*!
  Return the \f$\theta {\bf u}\f$ vector that corresponds to the rotation part
  of the pose vector.
 */
vpThetaUVector vpPoseVector::getThetaUVector() const
{
  vpThetaUVector tu((*this)[3], (*this)[4], (*this)[5]);
  return tu;
}

/*!

  Prints to the standard stream the pose vector.

  \warning Values concerning the \f$ \theta {\bf u}\f$ rotation are
  converted in degrees.

  The following code
  \code
  // Create a pose vector
  vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
  r.print();
  \endcode
  produces the output:

  \code
  1 2 3 180 -180 0
  \endcode

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
void vpPoseVector::print() const
{
  const unsigned int nparam = 6;
  const unsigned int nparam_t = 3;
  for (unsigned int i = 0; i < nparam; ++i) {
    if (i < nparam_t) {
      std::cout << (*this)[i] << " ";
    }
    else {
      std::cout << vpMath::deg((*this)[i]) << " ";
    }
  }
  std::cout << std::endl;
}

/*!

  Save the pose vector in the output file stream.

  \param f : Output file stream. Should be open before entering in this
  method.

  \exception vpException::ioError : If the output stream is not open.

  \sa load()
*/
void vpPoseVector::save(std::ofstream &f) const
{
  if (!f.fail()) {
    f << *this;
  }
  else {
    throw(vpException(vpException::ioError, "Cannot save the pose vector: ofstream not opened"));
  }
}

/*!
  Read a pose vector from an input file stream.

  \param f : The input file stream..Should be open before entering in
  this method.

  \exception vpException::ioError : If the input file stream is not open.

  \sa save()
*/
void vpPoseVector::load(std::ifstream &f)
{
  if (!f.fail()) {
    const unsigned int nparam = 6;
    for (unsigned int i = 0; i < nparam; ++i) {
      f >> (*this)[i];
    }
  }
  else {
    throw(vpException(vpException::ioError, "Cannot read pose vector: ifstream not opened"));
  }
}

/*
  Transpose the pose vector. The resulting vector becomes a row vector.

*/
vpRowVector vpPoseVector::t() const
{
  vpRowVector v(rowNum);
  memcpy(v.data, data, rowNum * sizeof(double));
  return v;
}

/*!

  Pretty print a pose vector. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each vector element.
  The actual width grows in order to accommodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the vector.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all vector elements.

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int vpPoseVector::print(std::ostream &s, unsigned int length, char const *intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = 1;

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // --comment: ossFixed less less std fixed
  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int i = 0; i < m; ++i) {
    oss.str("");
    oss << (*this)[i];
    if (oss.str().find("e") != std::string::npos) {
      ossFixed.str("");
      ossFixed << (*this)[i];
      oss.str(ossFixed.str());
    }

    values[i] = oss.str();
    size_type thislen = values[i].size();
    size_type p = values[i].find('.');

    if (p == std::string::npos) {
      maxBefore = vpMath::maximum(maxBefore, thislen);
      // maxAfter remains the same
    }
    else {
      maxBefore = vpMath::maximum(maxBefore, p);
      maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = std::min<size_type>(maxAfter, totalLength - maxBefore);
  if (maxAfter == 1) {
    maxAfter = 0;
  }

  // the following line is useful for debugging
  // std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro) {
    s << intro;
  }
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; ++i) {
    s << "  ";
    size_type p = values[i].find('.');
    s.setf(std::ios::right, std::ios::adjustfield);
    s.width(static_cast<std::streamsize>(maxBefore));
    s << values[i].substr(0, p).c_str();

    if (maxAfter > 0) {
      s.setf(std::ios::left, std::ios::adjustfield);
      if (p != std::string::npos) {
        s.width(static_cast<std::streamsize>(maxAfter));
        s << values[i].substr(p, maxAfter).c_str();
      }
      else {
        assert(maxAfter > 1);
        s.width(static_cast<std::streamsize>(maxAfter));
        s << ".0";
      }
    }

    s << ' ';

    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return static_cast<int>(maxBefore + maxAfter);
}

/*!
 * Converts the vpPoseVector to a 6-dim std::vector.
 * \return The corresponding std::vector<double>.
 */
std::vector<double> vpPoseVector::toStdVector() const
{
  std::vector<double> v(this->size());

  unsigned int this_size = this->size();
  for (unsigned int i = 0; i < this_size; ++i) {
    v[i] = data[i];
  }
  return v;
}

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <visp3/core/vpJsonParsing.h>
const std::string vpPoseVector::jsonTypeName = "vpPoseVector";
void vpPoseVector::convert_to_json(nlohmann::json &j) const
{
  const vpArray2D<double> *asArray = (vpArray2D<double>*) this;
  to_json(j, *asArray);
  j["type"] = vpPoseVector::jsonTypeName;
}
void vpPoseVector::parse_json(const nlohmann::json &j)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpArray2D<double> *asArray = (vpArray2D<double>*) this;
  if (j.is_object() && j.contains("type")) { // Specific conversions
    const bool converted = convertFromTypeAndBuildFrom<vpPoseVector, vpHomogeneousMatrix>(j, *this);
    if (!converted) {
      from_json(j, *asArray);
    }
  }
  else { // Generic 2D array conversion
    from_json(j, *asArray);
  }

  if ((getCols() != 1) && (getRows() != 6)) {
    throw vpException(vpException::badValue, "From JSON, tried to read something that is not a 6D pose vector");
  }
}
#endif
END_VISP_NAMESPACE
