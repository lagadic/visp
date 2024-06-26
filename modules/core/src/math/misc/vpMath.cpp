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
 * Simple mathematical function not available in the C math library (math.h).
 *
*****************************************************************************/

/*!
  \file vpMath.cpp
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
*/

#include <cmath>
#include <functional>
#include <numeric>
#include <stdint.h>
#include <cassert>
#include <ctype.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#if defined(VISP_HAVE_FUNC__FINITE)
#include <float.h>
#endif

BEGIN_VISP_NAMESPACE
#if !(defined(VISP_HAVE_FUNC_ISNAN) || defined(VISP_HAVE_FUNC_STD_ISNAN)) ||                                           \
    !(defined(VISP_HAVE_FUNC_ISINF) || defined(VISP_HAVE_FUNC_STD_ISINF)) ||                                           \
    !(defined(VISP_HAVE_FUNC_ISFINITE) || defined(VISP_HAVE_FUNC_STD_ISFINITE) || defined(VISP_HAVE_FUNC__FINITE))
#if defined _MSC_VER || defined __BORLANDC__
typedef __int64 int64;
typedef unsigned __int64 uint64;
#else
typedef int64_t int64;
typedef uint64_t uint64;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef union Vp64suf
{
  //  int64 i; //Unused variable, should be harmless to comment it
  uint64 u;
  double f;
} Vp64suf;

typedef union Vp32suf
{
  // int i; //Unused variable, should be harmless to comment it
  unsigned u;
  float f;
} Vp32suf;
#endif
#endif

const double vpMath::ang_min_sinc = 1.0e-8;
const double vpMath::ang_min_mc = 2.5e-4;

/*!
  Check whether a double number is not a number (NaN) or not.
  \param value : Double number to check.
  \return true if value is Not A Number (as defined by IEEE754 standard) and false otherwise.
 */
bool vpMath::isNaN(double value)
{
#if defined(VISP_HAVE_FUNC_STD_ISNAN)
  return std::isnan(value);
#elif defined(VISP_HAVE_FUNC_ISNAN)
  return isnan(value);
#elif defined(VISP_HAVE_FUNC__ISNAN)
  return (_isnan(value) != 0);
#else
  // Taken from OpenCV source code CvIsNan()
  Vp64suf ieee754;
  ieee754.f = value;
  return (((unsigned)(ieee754.u >> 32) & 0x7fffffff) + ((unsigned)ieee754.u != 0) > 0x7ff00000) != 0;
#endif
}

/*!
  Check whether a float number is not a number (NaN) or not.
  \param value : Float number to check.
  \return true if value is Not A Number (as defined by IEEE754 standard) and false otherwise.
 */
bool vpMath::isNaN(float value)
{
#if defined(VISP_HAVE_FUNC_STD_ISNAN)
  return std::isnan(value);
#elif defined(VISP_HAVE_FUNC_ISNAN)
  return isnan(value);
#elif defined(VISP_HAVE_FUNC__ISNAN)
  return (_isnan(value) != 0);
#else
  // Taken from OpenCV source code CvIsNan()
  Vp32suf ieee754;
  ieee754.f = value;
  return ((unsigned)ieee754.u & 0x7fffffff) > 0x7f800000;
#endif
}

/*!
  Returns whether a double is an infinity value (either positive infinity or
  negative infinity).
  \param value : Double number to check.
  \return true if value is a plus or minus infinity (as defined by IEEE754 standard)
  and false otherwise.
 */
bool vpMath::isInf(double value)
{
#if defined(VISP_HAVE_FUNC_STD_ISINF)
  return std::isinf(value);
#elif defined(VISP_HAVE_FUNC_ISINF)
  return isinf(value);
#else
  // Taken from OpenCV source code CvIsInf()
  Vp64suf ieee754;
  ieee754.f = value;
  return ((unsigned)(ieee754.u >> 32) & 0x7fffffff) == 0x7ff00000 && (unsigned)ieee754.u == 0;
#endif
}

/*!
  Returns whether a float is an infinity value (either positive infinity or
  negative infinity).
  \param value : Float number to check.
  \return true if value is a plus or minus infinity (as defined by IEEE754 standard)
  and false otherwise.
 */
bool vpMath::isInf(float value)
{
#if defined(VISP_HAVE_FUNC_STD_ISINF)
  return std::isinf(value);
#elif defined(VISP_HAVE_FUNC_ISINF)
  return isinf(value);
#else
  // Taken from OpenCV source code CvIsInf()
  Vp32suf ieee754;
  ieee754.f = value;
  return ((unsigned)ieee754.u & 0x7fffffff) == 0x7f800000;
#endif
}

/*!
  Returns whether a double is a finite value (neither infinite nor NaN).
  \param value : Double number to check.
  \return true if value is neither infinite nor NaN (as defined by IEEE754 standard)
  and false otherwise.
 */
bool vpMath::isFinite(double value)
{
#if defined(VISP_HAVE_FUNC_STD_ISFINITE)
  return std::isfinite(value);
#elif defined(VISP_HAVE_FUNC_ISFINITE)
  return isfinite(value);
#elif defined(VISP_HAVE_FUNC__FINITE)
  return _finite(value);
#else
  return !vpMath::isInf(value) && !vpMath::isNaN(value);
#endif
}

/*!
  Returns whether a float is a finite value (neither infinite nor NaN).
  \param value : Float number to check.
  \return true if value is neither infinite nor NaN (as defined by IEEE754 standard)
  and false otherwise.
 */
bool vpMath::isFinite(float value)
{
#if defined(VISP_HAVE_FUNC_STD_ISFINITE)
  return std::isfinite(value);
#elif defined(VISP_HAVE_FUNC_ISFINITE)
  return isfinite(value);
#elif defined(VISP_HAVE_FUNC__FINITE)
  return _finitef(value);
#else
  return !vpMath::isInf(value) && !vpMath::isNaN(value);
#endif
}

/*!
  Returns whether a string is a number.
  \param[in] str : String to check.
  \return true if string is number and false otherwise.
 */
bool vpMath::isNumber(const std::string &str)
{
  size_t str_size = str.size();
  for (size_t i = 0; i < str_size; ++i) {
    if (isdigit(str[i]) == false) {
      return false;
    }
  }
  return true;
}

/*!
  Compute \f$ (1-cos(x))/x^2 \f$

  \param cosx : Value of cos(x).
  \param x : Value of x.

  \return \f$ (1-cosx)/x^2 \f$
*/
double vpMath::mcosc(double cosx, double x)
{
  if (fabs(x) < ang_min_mc) {
    return 0.5;
  }
  else {
    return ((1.0 - cosx) / x / x);
  }
}

/*!
  Compute \f$ (1-sinc(x))/x^2 \f$ with \f$ sinc(x) = sinx / x \f$.

  \param sinx : value of sin(x).
  \param x  : Value of x.

  \return \f$ (1-sinc(x))/x^2 \f$
*/
double vpMath::msinc(double sinx, double x)
{
  if (fabs(x) < ang_min_mc) {
    return (1. / 6.0);
  }
  else {
    return ((1.0 - (sinx / x)) / x / x);
  }
}

/*!
  Compute sinus cardinal \f$ \frac{sin(x)}{x} \f$.

  \param x : Value of x.

  \return Sinus cardinal.
*/
double vpMath::sinc(double x)
{
  if (fabs(x) < ang_min_sinc) {
    return 1.0;
  }
  else {
    return sin(x) / x;
  }
}
/*!
  Compute sinus cardinal \f$ \frac{sin(x)}{x}\f$.

  \param sinx : Value of sin(x).
  \param x : Value of x.

  \return Sinus cardinal.
*/
double vpMath::sinc(double sinx, double x)
{
  if (fabs(x) < ang_min_sinc) {
    return 1.0;
  }
  else {
    return (sinx / x);
  }
}

/*!
  Compute the mean value for a vector of double.

  \param v : Vector of double values.

  \return The mean value.
*/
double vpMath::getMean(const std::vector<double> &v)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  size_t size = v.size();

  double sum = std::accumulate(v.begin(), v.end(), 0.0);

  return sum / (static_cast<double>(size));
}

/*!
  Compute the median value for a vector of double.

  \param v : Vector of double values.

  \return The median value.
*/
double vpMath::getMedian(const std::vector<double> &v)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  std::vector<double> v_copy = v;
  size_t size = v_copy.size();

  size_t n = size / 2;
  std::nth_element(v_copy.begin(), v_copy.begin() + n, v_copy.end());
  double val_n = v_copy[n];

  if ((size % 2) == 1) {
    return val_n;
  }
  else {
    std::nth_element(v_copy.begin(), v_copy.begin() + (n - 1), v_copy.end());
    return 0.5 * (val_n + v_copy[n - 1]);
  }
}

/*!
  Compute the standard deviation value for a vector of double.

  \param v : Vector of double values.
  \param useBesselCorrection : If true, the Bessel correction is used
  (normalize by N-1).

  \return The standard deviation value.
*/
double vpMath::getStdev(const std::vector<double> &v, bool useBesselCorrection)
{
  if (v.empty()) {
    throw vpException(vpException::notInitialized, "Empty vector !");
  }

  double mean = getMean(v);

  std::vector<double> diff(v.size());
#if VISP_CXX_STANDARD > VISP_CXX_STANDARD_98
  std::transform(v.begin(), v.end(), diff.begin(), std::bind(std::minus<double>(), std::placeholders::_1, mean));
#else
  std::transform(v.begin(), v.end(), diff.begin(), std::bind2nd(std::minus<double>(), mean));
#endif

  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double divisor = static_cast<double> (v.size());
  if (useBesselCorrection && (v.size() > 1)) {
    divisor = divisor - 1;
  }

  return std::sqrt(sq_sum / divisor);
}

/*!
  Compute the line equation using least-squares fitting that minimizes the cost function:
  \f[
    \mathbf{E} = \sum_{i=1}^{n}\left ( ax_i + by_i - c \right )^2
  \f]

  \param imPts : Image points (size  >= 3).
  \param a : a coefficient.
  \param b : b coefficient.
  \param c : c coefficient.

  \return The mean distance error (point-to-line distance) between the points and the fitted line.
*/
double vpMath::lineFitting(const std::vector<vpImagePoint> &imPts, double &a, double &b, double &c)
{
  if (imPts.size() < 3) {
    throw vpException(vpException::dimensionError, "Number of image points must be greater or equal to 3.");
  }

  double x_mean = 0, y_mean = 0;
  size_t imPts_size = imPts.size();
  for (size_t i = 0; i < imPts_size; ++i) {
    const vpImagePoint &imPt = imPts[i];
    x_mean += imPt.get_u();
    y_mean += imPt.get_v();
  }
  x_mean /= imPts.size();
  y_mean /= imPts.size();

  vpMatrix AtA(2, 2, 0.0);
  imPts_size = imPts.size();
  for (size_t i = 0; i < imPts_size; ++i) {
    const vpImagePoint &imPt = imPts[i];
    AtA[0][0] += (imPt.get_u() - x_mean) * (imPt.get_u() - x_mean);
    AtA[0][1] += (imPt.get_u() - x_mean) * (imPt.get_v() - y_mean);
    AtA[1][1] += (imPt.get_v() - y_mean) * (imPt.get_v() - y_mean);
  }
  AtA[1][0] = AtA[0][1];

  vpColVector eigenvalues;
  vpMatrix eigenvectors;
  AtA.eigenValues(eigenvalues, eigenvectors);

  a = eigenvectors[0][0];
  b = eigenvectors[1][0];
  c = (a * x_mean) + (b * y_mean);

  double error = 0;
  imPts_size = imPts.size();
  for (size_t i = 0; i < imPts_size; ++i) {
    double x0 = imPts[i].get_u();
    double y0 = imPts[i].get_v();

    error += std::fabs((a * x0) + ((b * y0) - c));
  }

  return error / imPts.size();
}

/*!
  Compute the modified modulo:
    - modulo(11, 10) == 1 == 11 % 10
    - modulo(-1, 10) == 9

  \param a : The dividend.
  \param n : The divisor.

  \return The modified modulo of a mod n.
*/
int vpMath::modulo(int a, int n) { return ((a % n) + n) % n; }

/*!
  Compute from a given longitude, latitude and a sphere radius the homogeneous transformation
  from the NED frame to the ECEF frame:

  \f[
  \begin{bmatrix}
    X_{\text{ecef}} \\
    Y_{\text{ecef}} \\
    Z_{\text{ecef}}
  \end{bmatrix}
  =
  \begin{bmatrix}
    -\sin \varphi \cos \lambda & -\sin \lambda & -\cos \varphi \cos \lambda \\
    -\sin \varphi \sin \lambda & \cos \lambda & -\cos \varphi \sin \lambda \\
    \cos \varphi & 0 & -\sin \varphi
  \end{bmatrix}
  +
  \begin{bmatrix}
    \text{r} \cos \varphi \cos \lambda \\
    \text{r} \cos \varphi \sin \lambda \\
    \text{r} \sin \varphi
  \end{bmatrix}
  \f]

  \image html vpMath_ECEF_NED_Longitude_Latitude_relationships.png

  See also:
    - [Local north, east, down (NED) coordinates](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates)
    - [Cartesian and ellipsoidal coordinates](https://gssc.esa.int/navipedia/index.php/Cartesian_and_ellipsoidal_coordinates)

  \param lonDeg : The longitude in degree or angle \f$\lambda\f$ in previous equation.
  \param latDeg : The latitude in degree or angle \f$\varphi\f$ in previous equation.
  \param radius : The sphere radius \f$r\f$ in meter.

  \return The homogeneous transformation from NED to ECEF frame.

  \sa enu2ecef(), getLocalTangentPlaneTransformations()
*/
vpHomogeneousMatrix vpMath::ned2ecef(double lonDeg, double latDeg, double radius)
{
  double lon = vpMath::rad(lonDeg);
  double lat = vpMath::rad(latDeg);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  vpHomogeneousMatrix ecef_M_ned;
  ecef_M_ned[index_0][index_0] = -sin(lat) * cos(lon);
  ecef_M_ned[index_0][index_1] = -sin(lon);
  ecef_M_ned[index_0][index_2] = -cos(lat) * cos(lon);
  ecef_M_ned[index_0][index_3] = radius * cos(lon) * cos(lat);
  ecef_M_ned[index_1][index_0] = -sin(lat) * sin(lon);
  ecef_M_ned[index_1][index_1] = cos(lon);
  ecef_M_ned[index_1][index_2] = -cos(lat) * sin(lon);
  ecef_M_ned[index_1][index_3] = radius * sin(lon) * cos(lat);
  ecef_M_ned[index_2][index_0] = cos(lat);
  ecef_M_ned[index_2][index_1] = 0;
  ecef_M_ned[index_2][index_2] = -sin(lat);
  ecef_M_ned[index_2][index_3] = radius * sin(lat);

  return ecef_M_ned;
}

/*!
  Compute from a given longitude, latitude and a sphere radius the homogeneous transformation
  from the ENU frame to the ECEF frame:

  \f[
    \begin{bmatrix}
      X_{\text{ecef}} \\
      Y_{\text{ecef}} \\
      Z_{\text{ecef}}
    \end{bmatrix}
    =
    \begin{bmatrix}
      -\sin \lambda & -\sin \varphi \cos \lambda & \cos \varphi \cos \lambda \\
      \cos \lambda & -\sin \varphi \sin \lambda & \cos \varphi \sin \lambda \\
      0 & \cos \varphi & \sin \varphi
    \end{bmatrix}
    +
    \begin{bmatrix}
      \text{r} \cos \varphi \cos \lambda \\
      \text{r} \cos \varphi \sin \lambda \\
      \text{r} \sin \varphi
    \end{bmatrix}
  \f]

  \image html vpMath_ECEF_ENU_Longitude_Latitude_relationships.png

  \sa
    - [From ENU to ECEF](https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ENU_to_ECEF)
    - [Transformations between ECEF and ENU coordinates](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates)
    - [Cartesian and ellipsoidal coordinates](https://gssc.esa.int/navipedia/index.php/Cartesian_and_ellipsoidal_coordinates)

  \param lonDeg : The longitude in degree or angle \f$\lambda\f$ in previous equation.
  \param latDeg : The latitude in degree or angle \f$\varphi\f$ in previous equation.
  \param radius : The sphere radius \f$r\f$ in meter.

  \return The homogeneous transformation from ENU to ECEF frame.

  \sa ned2ecef(), getLocalTangentPlaneTransformations()
*/
vpHomogeneousMatrix vpMath::enu2ecef(double lonDeg, double latDeg, double radius)
{
  double lon = vpMath::rad(lonDeg);
  double lat = vpMath::rad(latDeg);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  vpHomogeneousMatrix ecef_M_enu;
  ecef_M_enu[index_0][index_0] = -sin(lon);
  ecef_M_enu[index_0][index_1] = -sin(lat) * cos(lon);
  ecef_M_enu[index_0][index_2] = cos(lat) * cos(lon);
  ecef_M_enu[index_0][index_3] = radius * cos(lon) * cos(lat);
  ecef_M_enu[index_1][index_0] = cos(lon);
  ecef_M_enu[index_1][index_1] = -sin(lat) * sin(lon);
  ecef_M_enu[index_1][index_2] = cos(lat) * sin(lon);
  ecef_M_enu[index_1][index_3] = radius * sin(lon) * cos(lat);
  ecef_M_enu[index_2][index_0] = 0;
  ecef_M_enu[index_2][index_1] = cos(lat);
  ecef_M_enu[index_2][index_2] = sin(lat);
  ecef_M_enu[index_2][index_3] = radius * sin(lat);

  return ecef_M_enu;
}

/*!
  Compute the vector of longitude / latitude (in degree) couples for \e maxPoints regularly spaced on a sphere,
  using the following paper:
    - "How to generate equidistributed points on the surface of a sphere", Markus Deserno
    - https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf

  Following image illustrates the camera poses regularly spaced on a sphere:

  \image html vpMath_regular_points_on_sphere.png

  \param maxPoints : The number of point coordinates to be sampled on a sphere.

  \return The vector of longitude / latitude (in degree) pairs for the \e maxPoints on a sphare.
*/
std::vector<std::pair<double, double> > vpMath::computeRegularPointsOnSphere(unsigned int maxPoints)
{
  assert(maxPoints > 0);

  double a = (4.0 * M_PI) / maxPoints;
  double d = sqrt(a);
  int m_theta = static_cast<int>(round(M_PI / d));
  double d_theta = M_PI / m_theta;
  double d_phi = a / d_theta;

  std::vector<std::pair<double, double> > lonlat_vec;
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  lonlat_vec.reserve(static_cast<unsigned int>(std::sqrt(maxPoints)));
#else
  lonlat_vec.reserve(static_cast<unsigned int>(std::sqrt(static_cast<double>(maxPoints))));
#endif

  for (int m = 0; m < m_theta; ++m) {
    double theta = (M_PI * (m + 0.5)) / m_theta;
    int m_phi = static_cast<int>(round((2.0 * M_PI * sin(theta)) / d_phi));

    for (int n = 0; n < m_phi; ++n) {
      double phi = (2.0 * M_PI * n) / m_phi;
      double lon = phi;
      double lat = M_PI_2 - theta;
      lonlat_vec.push_back(std::make_pair(deg(lon), deg(lat)));
    }
  }

  return lonlat_vec;
}

/*!
  Compute transformations from the local tangent plane (e.g. NED, ECU, ...) to the ECEF frame.

  \sa
    - [Local tangent plane coordinates](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates)

  Following image illustrates the camera poses sampled using longitude / latitude coordinates:

  \image html vpMath_lon_lat.png

  \param lonlatVec : Vector of longitude/latitude coordinates in degree.
  \param radius : Sphere radius in meter.
  \param toECEF : Pointer to the function computing from a longitude / latitude coordinates in degree
  and a radius the corresponding transformation from the local frame (e.g. NED or ENU) to the ECEF frame.

  \return The vector of ecef_M_local homogeneous transformations.

  \sa enu2ecef(), ned2ecef()
*/
std::vector<vpHomogeneousMatrix> vpMath::getLocalTangentPlaneTransformations(const std::vector<std::pair<double, double> > &lonlatVec, double radius,
                                                                             vpHomogeneousMatrix(*toECEF)(double lonDeg_, double latDeg_, double radius_))
{
  std::vector<vpHomogeneousMatrix> ecef_M_local_vec;
  ecef_M_local_vec.reserve(lonlatVec.size());
  size_t lonlatVec_size = lonlatVec.size();
  for (size_t i = 0; i < lonlatVec_size; ++i) {
    double lonDeg = lonlatVec[i].first;
    double latDeg = lonlatVec[i].second;

    vpHomogeneousMatrix ecef_M_local = toECEF(lonDeg, latDeg, radius);
    ecef_M_local_vec.push_back(ecef_M_local);
  }
  return ecef_M_local_vec;
}

/*!
  Compute the transformation such that the camera located at \e from position looks toward \e to position.

  \image html vpMath_look-at.png

  Right-handed coordinate system for OpenGL (figure from https://learnopengl.com/Getting-started/Coordinate-Systems):

  \image html vpMath_coordinate_systems_right_handed.png

  See also:
    - https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function
    - https://github.com/g-truc/glm/blob/6ad79aae3eb5bf809c30bf1168171e9e55857e45/glm/ext/matrix_transform.inl#L98-L119
    - https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml

  \param from : Current camera position as a 3-dim vector with 3D coordinates (X,Y,Z) in meter..
  \param to : Where the camera must point toward as a 3-dim vector with 3D coordinates (X,Y,Z) in meter.
  \param tmp : Arbitrary up-vector as a 3-dim vector with coordinates along (X,Y,Z) in meter.

  \return The homogeneous transformation from the camera frame to the OpenGL frame.
*/
vpHomogeneousMatrix vpMath::lookAt(const vpColVector &from, const vpColVector &to, vpColVector tmp)
{
  assert(from.size() == 3);
  assert(to.size() == 3);
  assert(tmp.size() == 3);
  vpColVector forward = (from - to).normalize();
  vpColVector right = vpColVector::crossProd(tmp.normalize(), forward).normalize();
  vpColVector up = vpColVector::crossProd(forward, right).normalize();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  vpHomogeneousMatrix wMc;
  wMc[index_0][index_0] = right[index_0];
  wMc[index_0][index_1] = up[index_0];
  wMc[index_0][index_2] = forward[index_0];
  wMc[index_0][index_3] = from[index_0];
  wMc[index_1][index_0] = right[index_1];
  wMc[index_1][index_1] = up[index_1];
  wMc[index_1][index_2] = forward[index_1];
  wMc[index_1][index_3] = from[index_1];
  wMc[index_2][index_0] = right[index_2];
  wMc[index_2][index_1] = up[index_2];
  wMc[index_2][index_2] = forward[index_2];
  wMc[index_2][index_3] = from[index_2];

  return wMc;
}

/*!
 * Convert angles of a rotation vector into degrees.
 *
 * \param r : Rotation vector with angles in radians.
 * \return Corresponding column vector with angles converted in degrees.
 */
vpColVector vpMath::deg(const vpRotationVector &r)
{
  if (r.size() == 4) {
    throw(vpException(vpException::fatalError, "Cannot convert angles of a quaternion vector in degrees!"));
  }
  vpColVector r_deg(r.size());
  unsigned int r_size = r.size();
  for (unsigned int i = 0; i < r_size; ++i) {
    r_deg[i] = vpMath::deg(r[i]);
  }
  return r_deg;
}

/*!
 * Convert angles of a column vector from radians to degrees.
 *
 * \param r : Column vector with angles in radians.
 * \return Corresponding column vector with angles converted in degrees.
 */
vpColVector vpMath::deg(const vpColVector &r)
{
  vpColVector r_deg(r.size());
  unsigned int r_size = r.size();
  for (unsigned int i = 0; i < r_size; ++i) {
    r_deg[i] = vpMath::deg(r[i]);
  }
  return r_deg;
}

/*!
 * Convert angles of a column vector from degrees to radians.
 *
 * \param r : Column vector with angles in degrees.
 * \return Corresponding column vector with angles converted in radians.
 */
vpColVector vpMath::rad(const vpColVector &r)
{
  vpColVector r_rad(r.size());
  unsigned int r_size = r.size();
  for (unsigned int i = 0; i < r_size; ++i) {
    r_rad[i] = vpMath::rad(r[i]);
  }
  return r_rad;
}

/*!
 * Convert from ENU (East-North-Up) to NED (North-East-Down) frame.
 * \param enu_M : HomogeneousMatrix expressed in ENU frame.
 * \return Converted homogeneous matrix in NED frame.
 */
vpHomogeneousMatrix vpMath::enu2ned(const vpHomogeneousMatrix &enu_M)
{
  vpHomogeneousMatrix ned_M_enu;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  ned_M_enu[index_0][index_0] = 0;
  ned_M_enu[index_0][index_1] = 1;
  ned_M_enu[index_1][index_0] = 1;
  ned_M_enu[index_1][index_1] = 0;
  ned_M_enu[index_2][index_2] = -1;

  vpHomogeneousMatrix ned_M = ned_M_enu * enu_M;
  return ned_M;
}
END_VISP_NAMESPACE
