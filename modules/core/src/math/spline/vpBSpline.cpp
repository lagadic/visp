/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * This class implements the B-Spline
 */

#include <visp3/core/vpBSpline.h>
#include <visp3/core/vpDebug.h>

BEGIN_VISP_NAMESPACE
/*!
  Basic constructor.

  The degree \f$ p \f$ of the B-Spline basis functions is set to 3 to compute cubic B-Spline.
*/
vpBSpline::vpBSpline()
  : controlPoints(), knots(), p(3), // By default : p=3 for clubic spline
  crossingPoints()
{ }

/*!
  Find the knot interval in which the parameter \f$ l_u \f$ lies. Indeed \f$ l_u \in [u_i, u_{i+1}[ \f$.

  Example : The knot vector is the following \f$ U = \{0,  0 , 1 , 2 ,3 , 3\}
  \f$ with \f$ p \f$ is equal to 1.
    - For \f$ l_u \f$ equal to 0.5 the method will return 1.
    - For \f$ l_u \f$ equal to 2.5 the method will return 3.
    - For \f$ l_u \f$ equal to 3 the method will return 3.

  \param l_u : The knot whose knot interval is seeked.
  \param l_p : Degree of the B-Spline basis functions.
  \param l_knots : The knot vector.

  \return the number of the knot interval in which \f$ l_u \f$ lies.
*/
unsigned int vpBSpline::findSpan(double l_u, unsigned int l_p, const std::vector<double> &l_knots)
{
  unsigned int m = static_cast<unsigned int>(l_knots.size()) - 1;

  if (l_u > l_knots.back()) {
    // vpTRACE("l_u higher than the maximum value in the knot vector  :
    // %lf",l_u);
    return (static_cast<unsigned int>(m - l_p - 1));
  }

  // if (l_u == l_knots.back())
  if (std::fabs(l_u - l_knots.back()) <=
      std::fabs(vpMath::maximum(l_u, l_knots.back())) * std::numeric_limits<double>::epsilon())
    return (static_cast<unsigned int>(m - l_p - 1));

  double low = l_p;
  double high = m - l_p;
  double middle = (low + high) / 2.0;

  while (l_u < l_knots[static_cast<unsigned int>(middle)] || l_u >= l_knots[static_cast<unsigned int>(middle) + 1]) {
    if (l_u < l_knots[static_cast<unsigned int>(vpMath::round(middle))])
      high = middle;
    else
      low = middle;
    middle = (low + high) / 2.0;
  }

  return static_cast<unsigned int>(middle);
}

/*!
  Find the knot interval in which the parameter \f$ u \f$ lies. Indeed \f$ u \in [u_i, u_{i+1}[ \f$.

  Example : The knot vector is the following \f$ U = \{0,  0 , 1 , 2 ,3 , 3\}
  \f$ with \f$ p \f$ is equal to 1.
    - For \f$ u \f$ equal to 0.5 the method will return 1.
    - For \f$ u \f$ equal to 2.5 the method will return 3.
    - For \f$ u \f$ equal to 3 the method will return 3.

  \param u : The knot whose knot interval is seeked.

  \return the number of the knot interval in which \f$ u \f$ lies.
*/
unsigned int vpBSpline::findSpan(double u) const { return findSpan(u, p, knots); }

/*!
  Compute the nonvanishing basis functions at \f$ l_u \f$ which is in the \f$ l_i \f$ th knot interval.
  All the basis functions are stored in an array such as :

  N = \f$ N_{l_i,0}(l_u) \f$, \f$ N_{l_i-1,1}(l_u) \f$, \f$ N_{l_i,1}(l_u)
  \f$, ... , \f$ N_{l_i-k,k}(l_u) \f$, ..., \f$ N_{l_i,k}(l_u) \f$, ... , \f$
  N_{l_i-p,p}(l_u) \f$, ... , \f$ N_{l_i,p}(l_u) \f$

  \param l_u : A real number which is between the extremities of the knot vector.
  \param l_i : The number of the knot interval in which \f$ l_u \f$ lies.
  \param l_p : Degree of the B-Spline basis functions.
  \param l_knots : The knot vector.

  \return An array containing the nonvanishing basis functions at \f$ l_u \f$.
  The size of the array is \f$ l_p +1 \f$.
*/
vpBasisFunction *vpBSpline::computeBasisFuns(double l_u, unsigned int l_i, unsigned int l_p,
                                             const  std::vector<double> &l_knots)
{
  vpBasisFunction *N = new vpBasisFunction[l_p + 1];

  N[0].value = 1.0;

  double *left = new double[l_p + 1];
  double *right = new double[l_p + 1];
  double temp = 0.0;

  for (unsigned int j = 1; j <= l_p; j++) {
    left[j] = l_u - l_knots[l_i + 1 - j];
    right[j] = l_knots[l_i + j] - l_u;
    double saved = 0.0;

    for (unsigned int r = 0; r < j; r++) {
      temp = N[r].value / (right[r + 1] + left[j - r]);
      N[r].value = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    N[j].value = saved;
  }
  for (unsigned int j = 0; j < l_p + 1; j++) {
    N[j].i = l_i - l_p + j;
    N[j].p = l_p;
    N[j].u = l_u;
    N[j].k = 0;
  }

  delete[] left;
  delete[] right;

  return N;
}

/*!
  Compute the nonvanishing basis functions at \f$ u \f$. All the basis
  functions are stored in an array such as :

  N = \f$ N_{i,0}(u) \f$, \f$ N_{i-1,1}(u) \f$, \f$ N_{i,1}(u) \f$, ... , \f$
  N_{i-k,k}(u) \f$, ..., \f$ N_{i,k}(u) \f$, ... , \f$ N_{i-p,p}(u) \f$, ... ,
  \f$ N_{i,p}(u) \f$

  where i the number of the knot interval in which \f$ u \f$ lies.

  \param u : A real number which is between the extremities of the knot vector

  \return An array containing the nonvanishing basis functions at \f$ u \f$.
  The size of the array is \f$ p +1 \f$.
*/
vpBasisFunction *vpBSpline::computeBasisFuns(double u) const
{
  unsigned int i = findSpan(u);
  return computeBasisFuns(u, i, p, knots);
}

/*!
  Compute the nonzero basis functions and their derivatives until the \f$
  l_der \f$ th derivative. All the functions are computed at l_u.

  \warning The value of \f$ l_der \f$ must be under or equal \f$ l_p \f$.

  The result is given as an array of size l_der+1 x l_p+1. The kth line
  corresponds to the kth basis functions derivatives.

  The formula to compute the kth derivative at \f$ u \f$ is :

  \f[ N_{i,p}^{(k)}(u) =p \left( \frac{N_{i,p-1}^{(k-1)}}{u_{i+p}-u_i} -
  \frac{N_{i+1,p-1}^{(k-1)}}{u_{i+p+1}-u_{i+1}} \right) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies and \f$
  p \f$ is the degree of the B-Spline basis function.

  \param l_u : A real number which is between the extremities of the knot vector.
  \param l_i : The number of the knot interval in which \f$ l_u \f$ lies.
  \param l_p : Degree of the B-Spline basis functions.
  \param l_der : The last derivative to be computed.
  \param l_knots : The knot vector.

  \return the basis functions and their derivatives as an array of size
  l_der+1 x l_p+1. The kth line corresponds to the kth basis functions
  derivatives.

  Example : return[0] is the list of the 0th derivatives ie the basis
  functions. return[k] is the list of the kth derivatives.
*/
vpBasisFunction **vpBSpline::computeDersBasisFuns(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                                  const std::vector<double> &l_knots)
{
  vpBasisFunction **N;
  N = new vpBasisFunction *[l_der + 1];
  for (unsigned int j = 0; j <= l_der; j++)
    N[j] = new vpBasisFunction[l_p + 1];

  vpMatrix a(2, l_p + 1);
  vpMatrix ndu(l_p + 1, l_p + 1);
  ndu[0][0] = 1.0;

  double *left = new double[l_p + 1];
  double *right = new double[l_p + 1];
  double temp = 0.0;

  for (unsigned int j = 1; j <= l_p; j++) {
    left[j] = l_u - l_knots[l_i + 1 - j];
    right[j] = l_knots[l_i + j] - l_u;
    double saved = 0.0;

    for (unsigned int r = 0; r < j; r++) {
      ndu[j][r] = right[r + 1] + left[j - r];
      temp = ndu[r][j - 1] / ndu[j][r];
      ndu[r][j] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    ndu[j][j] = saved;
  }

  for (unsigned int j = 0; j <= l_p; j++) {
    N[0][j].value = ndu[j][l_p];
    N[0][j].i = l_i - l_p + j;
    N[0][j].p = l_p;
    N[0][j].u = l_u;
    N[0][j].k = 0;
  }

  if (l_der > l_p) {
    vpTRACE("l_der must be under or equal to l_p");
    l_der = l_p;
  }

  double d;
  int rk;
  unsigned int pk;
  unsigned int j1, j2;

  for (unsigned int r = 0; r <= l_p; r++) {
    unsigned int s1 = 0;
    unsigned int s2 = 1;
    a[0][0] = 1.0;
    for (unsigned int k = 1; k <= l_der; k++) {
      d = 0.0;
      rk = static_cast<int>(r - k);
      pk = l_p - k;
      if (r >= k) {
        a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
        d = a[s2][0] * ndu[static_cast<unsigned int>(rk)][pk];
      }

      if (rk >= -1)
        j1 = 1;
      else
        j1 = static_cast<unsigned int>(-rk);

      if (r - 1 <= pk)
        j2 = k - 1;
      else
        j2 = l_p - r;

      for (unsigned int j = j1; j <= j2; j++) {
        a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][static_cast<unsigned int>(rk) + j];
        d += a[s2][j] * ndu[static_cast<unsigned int>(rk) + j][pk];
      }

      if (r <= pk) {
        a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
        d += a[s2][k] * ndu[r][pk];
      }
      N[k][r].value = d;
      N[k][r].i = l_i - l_p + r;
      N[k][r].p = l_p;
      N[k][r].u = l_u;
      N[k][r].k = k;

      s1 = (s1 + 1) % 2;
      s2 = (s2 + 1) % 2;
    }
  }

  double r = l_p;
  for (unsigned int k = 1; k <= l_der; k++) {
    for (unsigned int j = 0; j <= l_p; j++)
      N[k][j].value *= r;
    r *= (l_p - k);
  }

  delete[] left;
  delete[] right;

  return N;
}

/*!
  Compute the nonzero basis functions and their derivatives until the \f$ der
  \f$ th derivative. All the functions are computed at u.

  \warning The value of \f$ der \f$ must be under or equal \f$ p \f$.

  The result is given as an array of size der+1 x p+1. The kth line
  corresponds to the kth basis functions derivatives.

  The formula to compute the kth derivative at \f$ u \f$ is :

  \f[ N_{i,p}^{(k)}(u) =p \left( \frac{N_{i,p-1}^{(k-1)}}{u_{i+p}-u_i} -
  \frac{N_{i+1,p-1}^{(k-1)}}{u_{i+p+1}-u_{i+1}} \right) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies and \f$
  p \f$ is the degree of the B-Spline basis function.

  \param u : A real number which is between the extremities of the knot vector
  \param der : The last derivative to be computed.

  \return The basis functions and their derivatives as an array of size der+1
  x p+1. The kth line corresponds to the kth basis functions derivatives.

  Example : return[0] is the list of the 0th derivatives ie the basis
  functions. return[k] is the list of the kth derivatives.
*/
vpBasisFunction **vpBSpline::computeDersBasisFuns(double u, unsigned int der) const
{
  unsigned int i = findSpan(u);
  return computeDersBasisFuns(u, i, p, der, knots);
}

/*!
  Compute the coordinates of a point \f$ C(u) = \sum_{i=0}^n (N_{i,p}(u)P_i)
  \f$ corresponding to the knot \f$ u \f$.

  \param l_u : A real number which is between the extremities of the knot vector.
  \param l_i : The number of the knot interval in which \f$ l_u \f$ lies.
  \param l_p : Degree of the B-Spline basis functions.
  \param l_knots : The knot vector.
  \param l_controlPoints : the list of control points.

  \return The coordinates of a point corresponding to the knot \f$ u \f$.
*/
vpImagePoint vpBSpline::computeCurvePoint(double l_u, unsigned int l_i, unsigned int l_p, const std::vector<double> &l_knots,
                                          const std::vector<vpImagePoint> &l_controlPoints)
{
  vpBasisFunction *N = computeBasisFuns(l_u, l_i, l_p, l_knots);
  vpImagePoint pt;

  double ic = 0;
  double jc = 0;
  for (unsigned int j = 0; j <= l_p; j++) {
    ic = ic + N[j].value * (l_controlPoints[l_i - l_p + j]).get_i();
    jc = jc + N[j].value * (l_controlPoints[l_i - l_p + j]).get_j();
  }

  pt.set_i(ic);
  pt.set_j(jc);

  delete[] N;

  return pt;
}

/*!
  Compute the coordinates of a point \f$ C(u) = \sum_{i=0}^n (N_{i,p}(u)P_i)
  \f$ corresponding to the knot \f$ u \f$.

  \param u : A real number which is between the extremities of the knot vector.

  \return The coordinates of a point corresponding to the knot \f$ u \f$.
*/
vpImagePoint vpBSpline::computeCurvePoint(double u) const
{
  vpBasisFunction *N = computeBasisFuns(u);
  vpImagePoint pt;

  double ic = 0;
  double jc = 0;
  for (unsigned int j = 0; j <= p; j++) {
    ic = ic + N[j].value * (controlPoints[N[0].i + j]).get_i();
    jc = jc + N[j].value * (controlPoints[N[0].i + j]).get_j();
  }

  pt.set_i(ic);
  pt.set_j(jc);

  delete[] N;

  return pt;
}

/*!
  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der} \f$.

  The formula used is the following :

  \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)P_i) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies and \f$
  p \f$ is the degree of the B-Spline basis function.

  \param l_u : A real number which is between the extremities of the knot vector.
  \param l_i : The number of the knot interval in which \f$ l_u \f$ lies
  \param l_p : Degree of the B-Spline basis functions.
  \param l_der : The last derivative to be computed.
  \param l_knots : The knot vector.
  \param l_controlPoints : The list of control points.

  \return An array of size l_der+1 containing the coordinates \f$ C^{(k)}(u)
  \f$ for \f$ k = 0, ... , l_der \f$. The kth derivative is in the kth cell of the array.
*/
vpImagePoint *vpBSpline::computeCurveDers(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                          const std::vector<double> &l_knots, const std::vector<vpImagePoint> &l_controlPoints)
{
  vpImagePoint *derivate = new vpImagePoint[l_der + 1];
  vpBasisFunction **N;
  N = computeDersBasisFuns(l_u, l_i, l_p, l_der, l_knots);

  unsigned int du;
  if (l_p < l_der) {
    vpTRACE("l_der must be under or equal to l_p");
    du = l_p;
  }
  else
    du = l_der;

  for (unsigned int k = 0; k <= du; k++) {
    derivate[k].set_ij(0.0, 0.0);
    for (unsigned int j = 0; j <= l_p; j++) {
      derivate[k].set_i(derivate[k].get_i() + N[k][j].value * (l_controlPoints[l_i - l_p + j]).get_i());
      derivate[k].set_j(derivate[k].get_j() + N[k][j].value * (l_controlPoints[l_i - l_p + j]).get_j());
    }
  }

  for (unsigned int j = 0; j <= l_der; j++)
    delete[] N[j];
  delete[] N;

  return derivate;
}

/*!
  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , der \f$.

  The formula used is the following :

  \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)P_i) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies and \f$
  p \f$ is the degree of the B-Spline basis function.

  \param u : A real number which is between the extremities of the knot vector
  \param der : The last derivative to be computed.

  \return An array of size der+1 containing the coordinates \f$ C^{(k)}(u) \f$
  for \f$ k = 0, ... , der \f$. The kth derivative is in the kth cell of the array.
*/
vpImagePoint *vpBSpline::computeCurveDers(double u, unsigned int der) const
{
  vpImagePoint *derivate = new vpImagePoint[der + 1];
  vpBasisFunction **N;
  N = computeDersBasisFuns(u, der);

  unsigned int du;
  if (p < der) {
    vpTRACE("der must be under or equal to p");
    du = p;
  }
  else
    du = der;

  for (unsigned int k = 0; k <= du; k++) {
    derivate[k].set_ij(0.0, 0.0);
    for (unsigned int j = 0; j <= p; j++) {
      derivate[k].set_i(derivate[k].get_i() + N[k][j].value * (controlPoints[N[0][0].i - p + j]).get_i());
      derivate[k].set_j(derivate[k].get_j() + N[k][j].value * (controlPoints[N[0][0].i - p + j]).get_j());
    }
  }

  for (unsigned int j = 0; j <= der; j++)
    delete[] N[j];
  delete[] N;

  return derivate;
}
END_VISP_NAMESPACE
