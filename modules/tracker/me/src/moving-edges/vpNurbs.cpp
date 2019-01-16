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
 * This class implements the Non Uniform Rational B-Spline (NURBS)
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <visp3/core/vpColVector.h>
#include <visp3/me/vpNurbs.h>
/*
  Compute the distance d = |Pw1-Pw2|
*/
inline double distance(const vpImagePoint &iP1, const double w1, const vpImagePoint &iP2, const double w2)
{
  double distancei = iP1.get_i() - iP2.get_i();
  double distancej = iP1.get_j() - iP2.get_j();
  double distancew = w1 - w2;
  return sqrt(vpMath::sqr(distancei) + vpMath::sqr(distancej) + vpMath::sqr(distancew));
}

/*!
  Basic constructor.

  The degree \f$ p \f$ of the NURBS basis functions is set to 3 to
  compute cubic NURBS.
*/
vpNurbs::vpNurbs() : weights() { p = 3; }

/*!
  Copy constructor.
*/
vpNurbs::vpNurbs(const vpNurbs &nurbs) : vpBSpline(nurbs), weights(nurbs.weights) {}

/*!
  Basic destructor
*/
vpNurbs::~vpNurbs() {}

/*!
  Compute the coordinates of a point \f$ C(u) = \frac{\sum_{i=0}^n
  (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)} \f$ corresponding to the
  knot \f$ u \f$.

  \param l_u : A real number which is between the extrimities of the knot
  vector \param l_i : the number of the knot interval in which \f$ l_u \f$
  lies \param l_p : Degree of the NURBS basis functions. \param l_knots : The
  knot vector \param l_controlPoints : the list of control points. \param
  l_weights : the list of weights.

  return the coordinates of a point corresponding to the knot \f$ u \f$.
*/
vpImagePoint vpNurbs::computeCurvePoint(double l_u, unsigned int l_i, unsigned int l_p, std::vector<double> &l_knots,
                                        std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights)
{
  vpBasisFunction *N = NULL;
  N = computeBasisFuns(l_u, l_i, l_p, l_knots);
  vpImagePoint pt;

  double ic = 0;
  double jc = 0;
  double wc = 0;
  for (unsigned int j = 0; j <= l_p; j++) {
    ic = ic + N[j].value * (l_controlPoints[l_i - l_p + j]).get_i() * l_weights[l_i - l_p + j];
    jc = jc + N[j].value * (l_controlPoints[l_i - l_p + j]).get_j() * l_weights[l_i - l_p + j];
    wc = wc + N[j].value * l_weights[l_i - l_p + j];
  }

  pt.set_i(ic / wc);
  pt.set_j(jc / wc);

  if (N != NULL)
    delete[] N;

  return pt;
}

/*!
  Compute the coordinates of a point \f$ C(u) = \frac{\sum_{i=0}^n
  (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)} \f$ corresponding to the
  knot \f$ u \f$.

  \param u : A real number which is between the extrimities of the knot vector

  return the coordinates of a point corresponding to the knot \f$ u \f$.
*/
vpImagePoint vpNurbs::computeCurvePoint(double u)
{
  vpBasisFunction *N = NULL;
  N = computeBasisFuns(u);
  vpImagePoint pt;

  double ic = 0;
  double jc = 0;
  double wc = 0;
  for (unsigned int j = 0; j <= p; j++) {
    ic = ic + N[j].value * (controlPoints[N[0].i + j]).get_i() * weights[N[0].i + j]; // N[0].i = findSpan(u)-p
    jc = jc + N[j].value * (controlPoints[N[0].i + j]).get_j() * weights[N[0].i + j];
    wc = wc + N[j].value * weights[N[0].i + j];
  }

  pt.set_i(ic / wc);
  pt.set_j(jc / wc);

  if (N != NULL)
    delete[] N;

  return pt;
}

/*!
  This function is used in the computeCurveDersPoint method.

  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
  \f$.

  The formula used is the following :

  \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)Pw_i) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies, \f$ p
  \f$ is the degree of the NURBS basis function and \f$ Pw_i = (P_i w_i) \f$
  contains the control points and the associatede weights.

  \param l_u : A real number which is between the extrimities of the knot
  vector \param l_i : the number of the knot interval in which \f$ l_u \f$
  lies \param l_p : Degree of the NURBS basis functions. \param l_der : The
  last derivative to be computed. \param l_knots : The knot vector \param
  l_controlPoints : the list of control points. \param l_weights : the list of
  weights.

  \return a matrix of size (l_der+1)x3 containing the coordinates \f$
  C^{(k)}(u) \f$ for \f$ k = 0, ... , l_{der} \f$. The kth derivative is in
  the kth line of the matrix. For each lines the first and the second column
  coresponds to the coordinates (i,j) of the point and the third column
  corresponds to the associated weight.
*/
vpMatrix vpNurbs::computeCurveDers(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                   std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                   std::vector<double> &l_weights)
{
  vpMatrix derivate(l_der + 1, 3);
  vpBasisFunction **N = NULL;
  N = computeDersBasisFuns(l_u, l_i, l_p, l_der, l_knots);

  for (unsigned int k = 0; k <= l_der; k++) {
    derivate[k][0] = 0.0;
    derivate[k][1] = 0.0;
    derivate[k][2] = 0.0;

    for (unsigned int j = 0; j <= l_p; j++) {
      derivate[k][0] = derivate[k][0] + N[k][j].value * (l_controlPoints[l_i - l_p + j]).get_i();
      derivate[k][1] = derivate[k][1] + N[k][j].value * (l_controlPoints[l_i - l_p + j]).get_j();
      derivate[k][2] = derivate[k][2] + N[k][j].value * (l_weights[l_i - l_p + j]);
    }
  }

  if (N != NULL) {
    for (unsigned int i = 0; i <= l_der; i++)
      delete[] N[i];
    delete[] N;
  }
  return derivate;
}

/*!
  This function is used in the computeCurveDersPoint method.

  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , der \f$.

  The formula used is the following :

  \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)Pw_i) \f]

  where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies, \f$ p
  \f$ is the degree of the NURBS basis function and \f$ Pw_i = (P_i w_i) \f$
  contains the control points and the associatede weights.

  \param u : A real number which is between the extrimities of the knot vector
  \param der : The last derivative to be computed.

  \return a matrix of size (l_der+1)x3 containing the coordinates \f$
  C^{(k)}(u) \f$ for \f$ k = 0, ... , der \f$. The kth derivative is in the
  kth line of the matrix. For each lines the first and the second column
  coresponds to the coordinates (i,j) of the point and the third column
  corresponds to the associated weight.
*/
vpMatrix vpNurbs::computeCurveDers(double u, unsigned int der)
{
  vpMatrix derivate(der + 1, 3);
  vpBasisFunction **N = NULL;
  N = computeDersBasisFuns(u, der);

  for (unsigned int k = 0; k <= der; k++) {
    derivate[k][0] = 0.0;
    derivate[k][1] = 0.0;
    derivate[k][2] = 0.0;
    for (unsigned int j = 0; j <= p; j++) {
      derivate[k][0] = derivate[k][0] + N[k][j].value * (controlPoints[N[0][0].i - p + j]).get_i();
      derivate[k][1] = derivate[k][1] + N[k][j].value * (controlPoints[N[0][0].i - p + j]).get_j();
      derivate[k][2] = derivate[k][2] + N[k][j].value * (weights[N[0][0].i - p + j]);
    }
  }

  if (N != NULL)
    delete[] N;

  return derivate;
}

/*!
  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
  \f$.

  To see how the derivatives are computed refers to the Nurbs book.

  \param l_u : A real number which is between the extrimities of the knot
  vector \param l_i : the number of the knot interval in which \f$ l_u \f$
  lies \param l_p : Degree of the NURBS basis functions. \param l_der : The
  last derivative to be computed. \param l_knots : The knot vector \param
  l_controlPoints : the list of control points. \param l_weights : the list of
  weights.

  \return an array of size l_der+1 containing the coordinates \f$ C^{(k)}(u)
  \f$ for \f$ k = 0, ... , l_{der} \f$. The kth derivative is in the kth cell
  of the array.
*/
vpImagePoint *vpNurbs::computeCurveDersPoint(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                             std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                             std::vector<double> &l_weights)
{
  std::vector<vpImagePoint> A;
  vpImagePoint pt;
  for (unsigned int j = 0; j < l_controlPoints.size(); j++) {
    pt = l_controlPoints[j];
    pt.set_i(pt.get_i() * l_weights[j]);
    pt.set_j(pt.get_j() * l_weights[j]);
    A.push_back(pt);
  }

  vpMatrix Awders = computeCurveDers(l_u, l_i, l_p, l_der, l_knots, A, l_weights);

  vpImagePoint *CK = new vpImagePoint[l_der + 1];

  for (unsigned int k = 0; k <= l_der; k++) {
    double ic = Awders[k][0];
    double jc = Awders[k][1];
    for (unsigned int j = 1; j <= k; j++) {
      double tmpComb = static_cast<double>(vpMath::comb(k, j));
      ic = ic - tmpComb * Awders[k][2] * (CK[k - j].get_i());
      jc = jc - tmpComb * Awders[j][2] * (CK[k - j].get_j());
    }
    CK[k].set_ij(ic / Awders[0][2], jc / Awders[0][2]);
  }
  return CK;
}

/*!
  Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
  \f$.

  To see how the derivatives are computed refers to the Nurbs book.

  \param u : A real number which is between the extrimities of the knot vector
  \param der : The last derivative to be computed.

  \return an array of size l_der+1 containing the coordinates \f$ C^{(k)}(u)
  \f$ for \f$ k = 0, ... , der \f$. The kth derivative is in the kth cell of
  the array.
*/
vpImagePoint *vpNurbs::computeCurveDersPoint(double u, unsigned int der)
{
  unsigned int i = findSpan(u);
  return computeCurveDersPoint(u, i, p, der, knots, controlPoints, weights);
}

/*!
  Insert \f$ l_r \f$ times a knot in the \f$ l_k \f$ th interval of the knot
  vector. The inserted knot \f$ l_u \f$ has multiplicity \f$ l_s \f$.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param l_u : A real number which is between the extrimities of the knot
  vector and which has to be inserted. \param l_k : The number of the knot
  interval in which \f$ l_u \f$ lies. \param l_s : Multiplicity of \f$ l_u \f$
  \param l_r : Number of times \f$ l_u \f$ has to be inserted.
  \param l_p : Degree of the NURBS basis functions.
  \param l_knots : The knot vector
  \param l_controlPoints : the list of control points.
  \param l_weights : the list of weights.
*/
void vpNurbs::curveKnotIns(double l_u, unsigned int l_k, unsigned int l_s, unsigned int l_r, unsigned int l_p,
                           std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                           std::vector<double> &l_weights)
{
  vpMatrix Rw(l_p + 1, 3);
  std::vector<vpImagePoint>::iterator it1;
  std::vector<double>::iterator it2;
  vpImagePoint pt;
  double w = 0;

  for (unsigned int j = 0; j <= l_p - l_s; j++) {
    Rw[j][0] = (l_controlPoints[l_k - l_p + j]).get_i() * l_weights[l_k - l_p + j];
    Rw[j][1] = (l_controlPoints[l_k - l_p + j]).get_j() * l_weights[l_k - l_p + j];
    Rw[j][2] = l_weights[l_k - l_p + j];
  }

  it1 = l_controlPoints.begin();
  l_controlPoints.insert(it1 + (int)l_k - (int)l_s, l_r, pt);
  it2 = l_weights.begin();
  l_weights.insert(it2 + (int)l_k - (int)l_s, l_r, w);

  unsigned int L = 0;
  double alpha;
  for (unsigned int j = 1; j <= l_r; j++) {
    L = l_k - l_p + j;

    for (unsigned int i = 0; i <= l_p - j - l_s; i++) {
      alpha = (l_u - l_knots[L + i]) / (l_knots[i + l_k + 1] - l_knots[L + i]);
      Rw[i][0] = alpha * Rw[i + 1][0] + (1.0 - alpha) * Rw[i][0];
      Rw[i][1] = alpha * Rw[i + 1][1] + (1.0 - alpha) * Rw[i][1];
      Rw[i][2] = alpha * Rw[i + 1][2] + (1.0 - alpha) * Rw[i][2];
    }

    pt.set_ij(Rw[0][0] / Rw[0][2], Rw[0][1] / Rw[0][2]);
    l_controlPoints[L] = pt;
    l_weights[L] = Rw[0][2];

    pt.set_ij(Rw[l_p - j - l_s][0] / Rw[l_p - j - l_s][2], Rw[l_p - j - l_s][1] / Rw[l_p - j - l_s][2]);
    l_controlPoints[l_k + l_r - j - l_s] = pt;
    l_weights[l_k + l_r - j - l_s] = Rw[l_p - j - l_s][2];
  }

  for (unsigned int j = L + 1; j < l_k - l_s; j++) {
    pt.set_ij(Rw[j - L][0] / Rw[j - L][2], Rw[j - L][1] / Rw[j - L][2]);
    l_controlPoints[j] = pt;
    l_weights[j] = Rw[j - L][2];
  }

  it2 = l_knots.begin();
  l_knots.insert(it2 + (int)l_k, l_r, l_u);
}

/*!
  Insert \f$ r \f$ times a knot in the \f$ k \f$ th interval of the knot
  vector. The inserted knot \f$ u \f$ has multiplicity \f$ s \f$.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param u : A real number which is between the extrimities of the knot vector
  and which has to be inserted. \param s : Multiplicity of \f$ l_u \f$ \param
  r : Number of times \f$ l_u \f$ has to be inserted.
*/
void vpNurbs::curveKnotIns(double u, unsigned int s, unsigned int r)
{
  unsigned int i = findSpan(u);
  curveKnotIns(u, i, s, r, p, knots, controlPoints, weights);
}

/*!
  Insert \f$ l_r \f$ knots in the knot vector.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param l_x : Several real numbers which are between the extrimities of the
  knot vector and which have to be inserted. \param l_r : Number of knot in
  the array \f$ l_x \f$. \param l_p : Degree of the NURBS basis functions.
  \param l_knots : The knot vector
  \param l_controlPoints : the list of control points.
  \param l_weights : the list of weights.
*/
void vpNurbs::refineKnotVectCurve(double *l_x, unsigned int l_r, unsigned int l_p, std::vector<double> &l_knots,
                                  std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights)
{
  unsigned int a = findSpan(l_x[0], l_p, l_knots);
  unsigned int b = findSpan(l_x[l_r], l_p, l_knots);
  b++;

  unsigned int n = (unsigned int)l_controlPoints.size();
  unsigned int m = (unsigned int)l_knots.size();

  for (unsigned int j = 0; j < n; j++) {
    l_controlPoints[j].set_ij(l_controlPoints[j].get_i() * l_weights[j], l_controlPoints[j].get_j() * l_weights[j]);
  }

  std::vector<double> l_knots_tmp(l_knots);
  std::vector<vpImagePoint> l_controlPoints_tmp(l_controlPoints);
  std::vector<double> l_weights_tmp(l_weights);

  vpImagePoint pt;
  double w = 0;

  for (unsigned int j = 0; j <= l_r; j++) {
    l_controlPoints.push_back(pt);
    l_weights.push_back(w);
    l_knots.push_back(w);
  }

  for (unsigned int j = b + l_p; j <= m - 1; j++)
    l_knots[j + l_r + 1] = l_knots_tmp[j];

  for (unsigned int j = b - 1; j <= n - 1; j++) {
    l_controlPoints[j + l_r + 1] = l_controlPoints_tmp[j];
    l_weights[j + l_r + 1] = l_weights_tmp[j];
  }

  unsigned int i = b + l_p - 1;
  unsigned int k = b + l_p + l_r;

  {
    unsigned int j = l_r + 1;
    do {
      j--;
      while (l_x[j] <= l_knots[i] && i > a) {
        l_controlPoints[k - l_p - 1] = l_controlPoints_tmp[i - l_p - 1];
        l_weights[k - l_p - 1] = l_weights_tmp[i - l_p - 1];
        l_knots[k] = l_knots_tmp[i];
        k--;
        i--;
      }

      l_controlPoints[k - l_p - 1] = l_controlPoints[k - l_p];
      l_weights[k - l_p - 1] = l_weights[k - l_p];

      for (unsigned int l = 1; l <= l_p; l++) {
        unsigned int ind = k - l_p + l;
        double alpha = l_knots[k + l] - l_x[j];
        // if (vpMath::abs(alpha) == 0.0)
        if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon()) {
          l_controlPoints[ind - 1] = l_controlPoints[ind];
          l_weights[ind - 1] = l_weights[ind];
        } else {
          alpha = alpha / (l_knots[k + l] - l_knots_tmp[i - l_p + l]);
          l_controlPoints[ind - 1].set_i(alpha * l_controlPoints[ind - 1].get_i() +
                                         (1.0 - alpha) * l_controlPoints[ind].get_i());
          l_controlPoints[ind - 1].set_j(alpha * l_controlPoints[ind - 1].get_j() +
                                         (1.0 - alpha) * l_controlPoints[ind].get_j());
          l_weights[ind - 1] = alpha * l_weights[ind - 1] + (1.0 - alpha) * l_weights[ind];
        }
      }
      l_knots[k] = l_x[j];
      k--;
    } while (j != 0);
  }

  for (unsigned int j = 0; j < n; j++) {
    l_controlPoints[j].set_ij(l_controlPoints[j].get_i() / l_weights[j], l_controlPoints[j].get_j() / l_weights[j]);
  }
}

/*!
  Insert \f$ r \f$ knots in the knot vector.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param x : Several real numbers which are between the extrimities of the
  knot vector and which have to be inserted. \param r : Number of knot in the
  array \f$ l_x \f$.
*/
void vpNurbs::refineKnotVectCurve(double *x, unsigned int r)
{
  refineKnotVectCurve(x, r, p, knots, controlPoints, weights);
}

/*!
  Remove \f$ l_num \f$ times the knot \f$ l_u \f$ from the knot vector. The
  removed knot \f$ l_u \f$ is the \f$ l_r \f$ th vector in the knot vector.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param l_u : A real number which is between the extrimities of the knot
  vector and which has to be removed. \param l_r : Index of \f$ l_u \f$ in the
  knot vector. \param l_num : Number of times \f$ l_u \f$ has to be removed.
  \param l_TOL : A parameter which has to be computed.
  \param l_s : Multiplicity of \f$ l_u \f$.
  \param l_p : Degree of the NURBS basis functions.
  \param l_knots : The knot vector
  \param l_controlPoints : the list of control points.
  \param l_weights : the list of weights.

  \return The number of time that l_u was removed.

  \f$ l_{TOL} = \frac{dw_{min}}{1+|P|_{max}} \f$

  where \f$ w_{min} \f$ is the minimal weight on the original curve, \f$
  |P|_{max} \f$ is the maximum distance of any point on the original curve
  from the origin and \f$ d \f$ is the desired bound on deviation.
*/
unsigned int vpNurbs::removeCurveKnot(double l_u, unsigned int l_r, unsigned int l_num, double l_TOL, unsigned int l_s,
                                      unsigned int l_p, std::vector<double> &l_knots,
                                      std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights)
{
  unsigned int n = (unsigned int)l_controlPoints.size();
  unsigned int m = n + l_p + 1;

  for (unsigned int j = 0; j < n; j++) {
    l_controlPoints[j].set_ij(l_controlPoints[j].get_i() * l_weights[j], l_controlPoints[j].get_j() * l_weights[j]);
  }

  unsigned int ord = l_p + 1;
  double fout = (2 * l_r - l_s - l_p) / 2.;
  unsigned int last = l_r - l_s;
  unsigned int first = l_r - l_p;
  unsigned int tblSize = 2 * l_p + 1;
  vpImagePoint *tempP = new vpImagePoint[tblSize];
  double *tempW = new double[tblSize];
  vpImagePoint pt;
  unsigned int t = 0;
  double alfi = 0;
  double alfj = 0;
  unsigned int i, j;

  for (t = 0; t < l_num; t++) {
    unsigned int off = first - 1;
    tempP[0] = l_controlPoints[off];
    tempW[0] = l_weights[off];
    tempP[last + 1 - off] = l_controlPoints[last + 1];
    tempW[last + 1 - off] = l_weights[last + 1];
    i = first;
    j = last;
    unsigned int ii = 1;
    unsigned int jj = last - off;
    int remflag = 0;
    while (j - i > t) {
      alfi = (l_u - l_knots[i]) / (l_knots[i + ord + t] - l_knots[i]);
      alfj = (l_u - l_knots[j - t]) / (l_knots[j + ord] - l_knots[j - t]);
      pt.set_i((l_controlPoints[i].get_i() - (1.0 - alfi) * tempP[ii - 1].get_i()) / alfi);
      tempP[ii].set_i((l_controlPoints[i].get_i() - (1.0 - alfi) * tempP[ii - 1].get_i()) / alfi);
      tempP[ii].set_j((l_controlPoints[i].get_j() - (1.0 - alfi) * tempP[ii - 1].get_j()) / alfi);
      tempW[ii] = ((l_weights[i] - (1.0 - alfi) * tempW[ii - 1]) / alfi);
      tempP[jj].set_i((l_controlPoints[j].get_i() - alfj * tempP[jj + 1].get_i()) / (1.0 - alfj));
      tempP[jj].set_j((l_controlPoints[j].get_j() - alfj * tempP[jj + 1].get_j()) / (1.0 - alfj));
      tempW[jj] = ((l_weights[j] - alfj * tempW[jj + 1]) / (1.0 - alfj));
      i++;
      j--;
      ii++;
      jj--;
    }

    if (j - i < t) {
      double distancei = tempP[ii - 1].get_i() - tempP[jj + 1].get_i();
      double distancej = tempP[ii - 1].get_j() - tempP[jj + 1].get_j();
      double distancew = tempW[ii - 1] - tempW[jj + 1];
      double distance = sqrt(vpMath::sqr(distancei) + vpMath::sqr(distancej) + vpMath::sqr(distancew));
      if (distance <= l_TOL)
        remflag = 1;
    } else {
      alfi = (l_u - l_knots[i]) / (l_knots[i + ord + t] - l_knots[i]);
      double distancei =
          l_controlPoints[i].get_i() - (alfi * tempP[ii + t + 1].get_i() + (1.0 - alfi) * tempP[ii - 1].get_i());
      double distancej =
          l_controlPoints[i].get_j() - (alfi * tempP[ii + t + 1].get_j() + (1.0 - alfi) * tempP[ii - 1].get_j());
      double distancew = l_weights[i] - (alfi * tempW[ii + t + 1] + (1.0 - alfi) * tempW[ii - 1]);
      double distance = sqrt(vpMath::sqr(distancei) + vpMath::sqr(distancej) + vpMath::sqr(distancew));
      if (distance <= l_TOL)
        remflag = 1;
    }
    if (remflag == 0)
      break;
    else {
      i = first;
      j = last;
      while (j - i > t) {
        l_controlPoints[i].set_i(tempP[i - off].get_i());
        l_controlPoints[i].set_j(tempP[i - off].get_j());
        l_weights[i] = tempW[i - off];
        l_controlPoints[j].set_i(tempP[j - off].get_i());
        l_controlPoints[j].set_j(tempP[j - off].get_j());
        l_weights[j] = tempW[j - off];
        i++;
        j--;
      }
    }
    first--;
    last++;
  }
  if (t == 0) {
    delete[] tempP;
    delete[] tempW;
    return t;
  }
  for (unsigned int k = l_r + 1; k <= m; k++)
    l_knots[k - t] = l_knots[k];
  j = (unsigned int)fout;
  i = j;
  for (unsigned int k = 1; k < t; k++) {
    if (k % 2)
      i++;
    else
      j--;
  }
  for (unsigned int k = i + 1; k <= n; k++) {
    l_controlPoints[j].set_i(l_controlPoints[k].get_i());
    l_controlPoints[j].set_j(l_controlPoints[k].get_j());
    l_weights[j] = l_weights[k];
    j++;
  }
  for (unsigned int k = 0; k < t; k++) {
    l_knots.erase(l_knots.end() - 1);
    l_controlPoints.erase(l_controlPoints.end() - 1);
  }

  for (unsigned int k = 0; k < l_controlPoints.size(); k++)
    l_controlPoints[k].set_ij(l_controlPoints[k].get_i() / l_weights[k], l_controlPoints[k].get_j() / l_weights[k]);

  delete[] tempP;
  delete[] tempW;
  return t;
}

/*!
  Remove \f$ num \f$ times the knot \f$ u \f$ from the knot vector. The
  removed knot \f$ u \f$ is the \f$ r \f$ th vector in the knot vector.

  Of course the knot vector changes. But The list of control points and the
  list of the associated weights change too.

  \param u : A real number which is between the extrimities of the knot vector
  and which has to be removed. \param r : Index of \f$ l_u \f$ in the knot
  vector. \param num : Number of times \f$ l_u \f$ has to be removed. \param
  TOL : A parameter which has to be computed.

  \return The number of time that l_u was removed.

  \f$ TOL = \frac{dw_{min}}{1+|P|_{max}} \f$

  where \f$ w_{min} \f$ is the minimal weight on the original curve, \f$
  |P|_{max} \f$ is the maximum distance of any point on the original curve
  from the origin and \f$ d \f$ is the desired bound on deviation.
*/
unsigned int vpNurbs::removeCurveKnot(double u, unsigned int r, unsigned int num, double TOL)
{
  return removeCurveKnot(u, r, num, TOL, 0, p, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve passing through a set of data
  points.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated. \param l_p : Degree of the NURBS basis functions. This value
  need to be > 0. \param l_knots : The knot vector \param l_controlPoints :
  the list of control points. \param l_weights : the list of weights.
*/
void vpNurbs::globalCurveInterp(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights)
{
  if (l_p == 0) {
    // vpERROR_TRACE("Bad degree of the NURBS basis functions");
    throw(vpException(vpException::badValue, "Bad degree of the NURBS basis functions"));
  }

  l_knots.clear();
  l_controlPoints.clear();
  l_weights.clear();
  unsigned int n = (unsigned int)l_crossingPoints.size() - 1;
  unsigned int m = n + l_p + 1;

  double d = 0;
  for (unsigned int k = 1; k <= n; k++)
    d = d + distance(l_crossingPoints[k], 1, l_crossingPoints[k - 1], 1);

  // Compute ubar
  std::vector<double> ubar;
  ubar.push_back(0.0);
  for (unsigned int k = 1; k < n; k++) {
    ubar.push_back(ubar[k - 1] + distance(l_crossingPoints[k], 1, l_crossingPoints[k - 1], 1) / d);
  }
  ubar.push_back(1.0);

  // Compute the knot vector
  for (unsigned int k = 0; k <= l_p; k++)
    l_knots.push_back(0.0);

  double sum = 0;
  for (unsigned int k = 1; k <= l_p; k++)
    sum = sum + ubar[k];

  // Centripetal method
  for (unsigned int k = 1; k <= n - l_p; k++) {
    l_knots.push_back(sum / l_p);
    sum = sum - ubar[k - 1] + ubar[l_p + k - 1];
  }

  for (unsigned int k = m - l_p; k <= m; k++)
    l_knots.push_back(1.0);

  vpMatrix A(n + 1, n + 1);
  vpBasisFunction *N;

  for (unsigned int i = 0; i <= n; i++) {
    unsigned int span = findSpan(ubar[i], l_p, l_knots);
    N = computeBasisFuns(ubar[i], span, l_p, l_knots);
    for (unsigned int k = 0; k <= l_p; k++)
      A[i][span - l_p + k] = N[k].value;
    delete[] N;
  }
  // vpMatrix Ainv = A.inverseByLU();
  vpMatrix Ainv;
  A.pseudoInverse(Ainv);
  vpColVector Qi(n + 1);
  vpColVector Qj(n + 1);
  vpColVector Qw(n + 1);
  for (unsigned int k = 0; k <= n; k++) {
    Qi[k] = l_crossingPoints[k].get_i();
    Qj[k] = l_crossingPoints[k].get_j();
  }
  Qw = 1;
  vpColVector Pi = Ainv * Qi;
  vpColVector Pj = Ainv * Qj;
  vpColVector Pw = Ainv * Qw;

  vpImagePoint pt;
  for (unsigned int k = 0; k <= n; k++) {
    pt.set_ij(Pi[k], Pj[k]);
    l_controlPoints.push_back(pt);
    l_weights.push_back(Pw[k]);
  }
}

/*!
  Method which enables to compute a NURBS curve passing through a set of data
  points.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated.
*/
void vpNurbs::globalCurveInterp(vpList<vpMeSite> &l_crossingPoints)
{
  std::vector<vpImagePoint> v_crossingPoints;
  l_crossingPoints.front();
  vpMeSite s = l_crossingPoints.value();
  vpImagePoint pt(s.ifloat, s.jfloat);
  vpImagePoint pt_1 = pt;
  v_crossingPoints.push_back(pt);
  l_crossingPoints.next();
  while (!l_crossingPoints.outside()) {
    s = l_crossingPoints.value();
    pt.set_ij(s.ifloat, s.jfloat);
    if (vpImagePoint::distance(pt_1, pt) >= 10) {
      v_crossingPoints.push_back(pt);
      pt_1 = pt;
    }
    l_crossingPoints.next();
  }
  globalCurveInterp(v_crossingPoints, p, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve passing through a set of data
  points.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated.
*/
void vpNurbs::globalCurveInterp(const std::list<vpImagePoint> &l_crossingPoints)
{
  std::vector<vpImagePoint> v_crossingPoints;
  for (std::list<vpImagePoint>::const_iterator it = l_crossingPoints.begin(); it != l_crossingPoints.end(); ++it) {
    v_crossingPoints.push_back(*it);
  }
  globalCurveInterp(v_crossingPoints, p, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve passing through a set of data
  points.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated.
*/
void vpNurbs::globalCurveInterp(const std::list<vpMeSite> &l_crossingPoints)
{
  std::vector<vpImagePoint> v_crossingPoints;
  vpMeSite s = l_crossingPoints.front();
  vpImagePoint pt(s.ifloat, s.jfloat);
  vpImagePoint pt_1 = pt;
  v_crossingPoints.push_back(pt);
  std::list<vpMeSite>::const_iterator it = l_crossingPoints.begin();
  ++it;
  for (; it != l_crossingPoints.end(); ++it) {
    vpImagePoint pt_tmp(it->ifloat, it->jfloat);
    if (vpImagePoint::distance(pt_1, pt_tmp) >= 10) {
      v_crossingPoints.push_back(pt_tmp);
      pt_1 = pt_tmp;
    }
  }
  globalCurveInterp(v_crossingPoints, p, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve passing through a set of data
  points.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.
*/
void vpNurbs::globalCurveInterp() { globalCurveInterp(crossingPoints, p, knots, controlPoints, weights); }

/*!
  Method which enables to compute a NURBS curve approximating a set of data
  points.

  The data points are approximated thanks to a least square method.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated. \param l_p : Degree of the NURBS basis functions. \param l_n :
  The desired number of control points. l_n must be under or equal to the
  number of data points. \param l_knots : The knot vector. \param
  l_controlPoints : the list of control points. \param l_weights : the list of
  weights.
*/
void vpNurbs::globalCurveApprox(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p, unsigned int l_n,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights)
{
  l_knots.clear();
  l_controlPoints.clear();
  l_weights.clear();
  unsigned int m = (unsigned int)l_crossingPoints.size() - 1;

  double d = 0;
  for (unsigned int k = 1; k <= m; k++)
    d = d + distance(l_crossingPoints[k], 1, l_crossingPoints[k - 1], 1);

  // Compute ubar
  std::vector<double> ubar;
  ubar.push_back(0.0);
  for (unsigned int k = 1; k < m; k++)
    ubar.push_back(ubar[k - 1] + distance(l_crossingPoints[k], 1, l_crossingPoints[k - 1], 1) / d);
  ubar.push_back(1.0);

  // Compute the knot vector
  for (unsigned int k = 0; k <= l_p; k++)
    l_knots.push_back(0.0);

  d = (double)(m + 1) / (double)(l_n - l_p + 1);

  for (unsigned int j = 1; j <= l_n - l_p; j++) {
    double i = floor(j * d);
    double alpha = j * d - i;
    l_knots.push_back((1.0 - alpha) * ubar[(unsigned int)i - 1] + alpha * ubar[(unsigned int)i]);
  }

  for (unsigned int k = 0; k <= l_p; k++)
    l_knots.push_back(1.0);

  // Compute Rk
  std::vector<vpImagePoint> Rk;
  vpBasisFunction *N;
  for (unsigned int k = 1; k <= m - 1; k++) {
    unsigned int span = findSpan(ubar[k], l_p, l_knots);
    if (span == l_p && span == l_n) {
      N = computeBasisFuns(ubar[k], span, l_p, l_knots);
      vpImagePoint pt(l_crossingPoints[k].get_i() - N[0].value * l_crossingPoints[0].get_i() -
                          N[l_p].value * l_crossingPoints[m].get_i(),
                      l_crossingPoints[k].get_j() - N[0].value * l_crossingPoints[0].get_j() -
                          N[l_p].value * l_crossingPoints[m].get_j());
      Rk.push_back(pt);
      delete[] N;
    } else if (span == l_p) {
      N = computeBasisFuns(ubar[k], span, l_p, l_knots);
      vpImagePoint pt(l_crossingPoints[k].get_i() - N[0].value * l_crossingPoints[0].get_i(),
                      l_crossingPoints[k].get_j() - N[0].value * l_crossingPoints[0].get_j());
      Rk.push_back(pt);
      delete[] N;
    } else if (span == l_n) {
      N = computeBasisFuns(ubar[k], span, l_p, l_knots);
      vpImagePoint pt(l_crossingPoints[k].get_i() - N[l_p].value * l_crossingPoints[m].get_i(),
                      l_crossingPoints[k].get_j() - N[l_p].value * l_crossingPoints[m].get_j());
      Rk.push_back(pt);
      delete[] N;
    } else {
      Rk.push_back(l_crossingPoints[k]);
    }
  }

  vpMatrix A(m - 1, l_n - 1);
  // Compute A
  for (unsigned int i = 1; i <= m - 1; i++) {
    unsigned int span = findSpan(ubar[i], l_p, l_knots);
    N = computeBasisFuns(ubar[i], span, l_p, l_knots);
    for (unsigned int k = 0; k <= l_p; k++) {
      if (N[k].i > 0 && N[k].i < l_n)
        A[i - 1][N[k].i - 1] = N[k].value;
    }
    delete[] N;
  }

  vpColVector Ri(l_n - 1);
  vpColVector Rj(l_n - 1);
  vpColVector Rw(l_n - 1);
  for (unsigned int i = 0; i < l_n - 1; i++) {
    double sum = 0;
    for (unsigned int k = 0; k < m - 1; k++)
      sum = sum + A[k][i] * Rk[k].get_i();
    Ri[i] = sum;
    sum = 0;
    for (unsigned int k = 0; k < m - 1; k++)
      sum = sum + A[k][i] * Rk[k].get_j();
    Rj[i] = sum;
    sum = 0;
    for (unsigned int k = 0; k < m - 1; k++)
      sum = sum + A[k][i]; // The crossing points weigths are equal to 1.
    Rw[i] = sum;
  }

  vpMatrix AtA = A.AtA();
  vpMatrix AtAinv;
  AtA.pseudoInverse(AtAinv);

  vpColVector Pi = AtAinv * Ri;
  vpColVector Pj = AtAinv * Rj;
  vpColVector Pw = AtAinv * Rw;

  vpImagePoint pt;
  l_controlPoints.push_back(l_crossingPoints[0]);
  l_weights.push_back(1.0);
  for (unsigned int k = 0; k < l_n - 1; k++) {
    pt.set_ij(Pi[k], Pj[k]);
    l_controlPoints.push_back(pt);
    l_weights.push_back(Pw[k]);
  }
  l_controlPoints.push_back(l_crossingPoints[m]);
  l_weights.push_back(1.0);
}

/*!

  Method which enables to compute a NURBS curve approximating a set of
  data points.

  The data points are approximated thanks to a least square method.

  The result of the method is composed by a knot vector, a set of
  control points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated.

  \param n : The desired number of control points. This parameter \e n
  must be under or equal to the number of data points.
*/
void vpNurbs::globalCurveApprox(vpList<vpMeSite> &l_crossingPoints, unsigned int n)
{
  std::vector<vpImagePoint> v_crossingPoints;
  l_crossingPoints.front();
  while (!l_crossingPoints.outside()) {
    vpMeSite s = l_crossingPoints.value();
    vpImagePoint pt(s.ifloat, s.jfloat);
    v_crossingPoints.push_back(pt);
    l_crossingPoints.next();
  }
  globalCurveApprox(v_crossingPoints, p, n, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve approximating a set of data
  points.

  The data points are approximated thanks to a least square method.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated. \param n : The desired number of control points. The parameter
  \e n must be under or equal to the number of data points.
*/
void vpNurbs::globalCurveApprox(const std::list<vpImagePoint> &l_crossingPoints, unsigned int n)
{
  std::vector<vpImagePoint> v_crossingPoints;
  for (std::list<vpImagePoint>::const_iterator it = l_crossingPoints.begin(); it != l_crossingPoints.end(); ++it) {
    v_crossingPoints.push_back(*it);
  }
  globalCurveApprox(v_crossingPoints, p, n, knots, controlPoints, weights);
}

/*!

  Method which enables to compute a NURBS curve approximating a set of
  data points.

  The data points are approximated thanks to a least square method.

  The result of the method is composed by a knot vector, a set of
  control points and a set of associated weights.

  \param l_crossingPoints : The list of data points which have to be
  interpolated.

  \param n : The desired number of control points. This parameter \e n
  must be under or equal to the number of data points.
*/
void vpNurbs::globalCurveApprox(const std::list<vpMeSite> &l_crossingPoints, unsigned int n)
{
  std::vector<vpImagePoint> v_crossingPoints;
  for (std::list<vpMeSite>::const_iterator it = l_crossingPoints.begin(); it != l_crossingPoints.end(); ++it) {
    vpImagePoint pt(it->ifloat, it->jfloat);
    v_crossingPoints.push_back(pt);
  }
  globalCurveApprox(v_crossingPoints, p, n, knots, controlPoints, weights);
}

/*!
  Method which enables to compute a NURBS curve approximating a set of data
  points.

  The data points are approximated thanks to a least square method.

  The result of the method is composed by a knot vector, a set of control
  points and a set of associated weights.
*/
void vpNurbs::globalCurveApprox(unsigned int n)
{
  globalCurveApprox(crossingPoints, p, n, knots, controlPoints, weights);
}
