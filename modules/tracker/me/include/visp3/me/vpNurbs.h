/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * This class implements the Non Uniform Rational B-Spline (NURBS)
 */

#ifndef vpNurbs_H
#define vpNurbs_H

/*!
 * \file vpNurbs.h
 * \brief Class that provides tools to compute and manipulate a Non Uniform
 * Rational B-Spline curve.
 */

#include <visp3/core/vpBSpline.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpList.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMeSite.h>

#include <list>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpNurbs
 * \ingroup module_me
 *
 * \brief Class that provides tools to compute and manipulate a Non Uniform
 * Rational B-Spline curve.
 *
 * The different parameters are :
 *
 * - The knot vector \f$ U = {u_0, ... , u_m} \f$ where the knots \f$ u_i, i =
 * 0, ...,m \f$ are real number such as \f$ u_i < u_{i+1} i = 0, ...,m \f$. To
 * define a curve, the knot vector is such as : \f$ U = {a , ... , a, u_{p+1} ,
 * ... , u_{m-p-1} , b , ... , b} \f$ where \f$ a \f$ and \f$ b \f$ are real
 * numbers and p is the degree of the B-Spline basis functions.
 *
 * - The B-Spline basis functions \f$ N_{i,p} \f$ defined as :
 * \f[ N_{i,0}(u) = \left\{\begin{array}{cc}
 * 1 & \mbox{if } u_i \leq u_{i+1} \\ 0 & else
 * \end{array}\right.\f]
 *
 * \f[ N_{i,p}(u) =
 * \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u)\f]
 *
 * where \f$ i = 0 , ... , m-1 \f$ and p is the degree of the B-Spline basis
 * functions.
 *
 * - The control points \f$ {P_i} \f$ which are defined by the coordinates \f$
 * (i,j) \f$ of a point in an image.
 *
 * - The weight \f$ {w_i} \f$ associated to each control points. The weights
 * value is upper than 0.
 *
 * It is possible to compute the coordinates of a point corresponding to the
 * knots \f$ u \f$ (\f$ u \in [u_0,u_m]\f$) thanks to the formula : \f[ C(u) =
 * \frac{\sum_{i=0}^n (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)}\f]
 *
 * You can find much more information about the B-Splines and the
 * implementation of all the methods in the Nurbs Book.
*/
class VISP_EXPORT vpNurbs : public vpBSpline
{
protected:
  //! Vector which contains the weights associated to each control Points
  std::vector<double> weights;

protected:
  /*!
   * This function is used in the computeCurveDersPoint method.
   *
   * Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
   * \f$.
   *
   * The formula used is the following :
   *
   * \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)Pw_i) \f]
   *
   * where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies, \f$ p
   * \f$ is the degree of the NURBS basis function and \f$ Pw_i = (P_i w_i) \f$
   * contains the control points and the associated weights.
   *
   * \param l_u : A real number which is between the extremities of the knot
   * vector.
   * \param l_i : the number of the knot interval in which \f$ l_u \f$
   * lies.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_der : The last derivative to be computed.
   * \param l_knots : The knot vector.
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   *
   * \return a matrix of size (l_der+1)x3 containing the coordinates \f$
   * C^{(k)}(u) \f$ for \f$ k = 0, ... , l_{der} \f$. The kth derivative is in
   * the kth line of the matrix. For each lines the first and the second column
   * corresponds to the coordinates (i,j) of the point and the third column
   * corresponds to the associated weight.
   */
  static vpMatrix computeCurveDers(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                   std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                   std::vector<double> &l_weights);

  /*!
   * This function is used in the computeCurveDersPoint method.
   *
   * Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , der \f$.
   *
   * The formula used is the following :
   *
   * \f[ C^{(k)}(u) = \sum_{i=0}^n (N_{i,p}^{(k)}(u)Pw_i) \f]
   *
   * where \f$ i \f$ is the knot interval number in which \f$ u \f$ lies, \f$ p
   * \f$ is the degree of the NURBS basis function and \f$ Pw_i = (P_i w_i) \f$
   * contains the control points and the associated weights.
   *
   * \param u : A real number which is between the extremities of the knot vector
   * \param der : The last derivative to be computed.
   *
   * \return a matrix of size (l_der+1)x3 containing the coordinates \f$
   * C^{(k)}(u) \f$ for \f$ k = 0, ... , der \f$. The kth derivative is in the
   * kth line of the matrix. For each lines the first and the second column
   * corresponds to the coordinates (i,j) of the point and the third column
   * corresponds to the associated weight.
   */
  vpMatrix computeCurveDers(double u, unsigned int der);

public:
  /*!
   * Basic constructor.
   *
   * The degree \f$ p \f$ of the NURBS basis functions is set to 3 to
   * compute cubic NURBS.
   */
  vpNurbs();

  /*!
   * Copy constructor.
   */
  vpNurbs(const vpNurbs &nurbs);


  /*!
   * Gets all the weights relative to the control points.
   *
   * \param list [out] : A std::list containing weights relative to the
   *  control points.
   */
  inline void get_weights(std::list<double> &list) const
  {
    list.clear();
    for (unsigned int i = 0; i < weights.size(); i++)
      list.push_back(*(&(weights[0]) + i));
  }

  /*!
   * Sets all the knots.
   *
   * \param list : A std::list containing the value of the knots.
   */
  inline void set_weights(const std::list<double> &list)
  {
    weights.clear();
    for (std::list<double>::const_iterator it = list.begin(); it != list.end(); ++it) {
      weights.push_back(*it);
    }
  }

  /*!
   * Compute the coordinates of a point \f$ C(u) = \frac{\sum_{i=0}^n
   * (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)} \f$ corresponding to the
   * knot \f$ u \f$.
   *
   * \param l_u : A real number which is between the extremities of the knot
   * vector.
   * \param l_i : the number of the knot interval in which \f$ l_u \f$
   * lies.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_knots : The knot vector.
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   *
   * \return The coordinates of a point corresponding to the knot \f$ u \f$.
   */
  static vpImagePoint computeCurvePoint(double l_u, unsigned int l_i, unsigned int l_p, std::vector<double> &l_knots,
                                        std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);

  /*!
   * Compute the coordinates of a point \f$ C(u) = \frac{\sum_{i=0}^n
   * (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)} \f$ corresponding to the
   * knot \f$ u \f$.
   *
   * \param u : A real number which is between the extremities of the knot vector
   *
   * return the coordinates of a point corresponding to the knot \f$ u \f$.
   */
  vpImagePoint computeCurvePoint(double u);

  /*!
   * Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
   * \f$.
   *
   * To see how the derivatives are computed refers to the Nurbs book.
   *
   * \param l_u : A real number which is between the extremities of the knot
   * vector.
   * \param l_i : the number of the knot interval in which \f$ l_u \f$
   * lies.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_der : The last derivative to be computed.
   * \param l_knots : The knot vector.
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   *
   * \return an array of size l_der+1 containing the coordinates \f$ C^{(k)}(u)
   * \f$ for \f$ k = 0, ... , l_{der} \f$. The kth derivative is in the kth cell
   * of the array.
   */
  static vpImagePoint *computeCurveDersPoint(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                             std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                             std::vector<double> &l_weights);

  /*!
   * Compute the kth derivatives of \f$ C(u) \f$ for \f$ k = 0, ... , l_{der}
   * \f$.
   *
   * To see how the derivatives are computed refers to the Nurbs book.
   *
   * \param u : A real number which is between the extremities of the knot vector
   * \param der : The last derivative to be computed.
   *
   * \return an array of size l_der+1 containing the coordinates \f$ C^{(k)}(u)
   * \f$ for \f$ k = 0, ... , der \f$. The kth derivative is in the kth cell of
   * the array.
   */
  vpImagePoint *computeCurveDersPoint(double u, unsigned int der);

  /*!
   * Insert \f$ l_r \f$ times a knot in the \f$ l_k \f$ th interval of the knot
   * vector. The inserted knot \f$ l_u \f$ has multiplicity \f$ l_s \f$.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param l_u : A real number which is between the extremities of the knot
   * vector and which has to be inserted.
   * \param l_k : The number of the knot interval in which \f$ l_u \f$ lies.
   * \param l_s : Multiplicity of \f$ l_u \f$
   * \param l_r : Number of times \f$ l_u \f$ has to be inserted.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_knots : The knot vector
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   */
  static void curveKnotIns(double l_u, unsigned int l_k, unsigned int l_s, unsigned int l_r, unsigned int l_p,
                           std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                           std::vector<double> &l_weights);

  /*!
   * Insert \f$ r \f$ times a knot in the \f$ k \f$ th interval of the knot
   * vector. The inserted knot \f$ u \f$ has multiplicity \f$ s \f$.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param u : A real number which is between the extremities of the knot vector
   * and which has to be inserted.
   * \param s : Multiplicity of \f$ l_u \f$.
   * \param r : Number of times \f$ l_u \f$ has to be inserted.
   */
  void curveKnotIns(double u, unsigned int s = 0, unsigned int r = 1);

  /*!
   * Insert \f$ l_r \f$ knots in the knot vector.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param l_x : Several real numbers which are between the extremities of the
   * knot vector and which have to be inserted.
   * \param l_r : Number of knot in the array \f$ l_x \f$.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_knots : The knot vector
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   */
  static void refineKnotVectCurve(double *l_x, unsigned int l_r, unsigned int l_p, std::vector<double> &l_knots,
                                  std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);

  /*!
   * Insert \f$ r \f$ knots in the knot vector.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param x : Several real numbers which are between the extremities of the
   * knot vector and which have to be inserted. \param r : Number of knot in the
   * array \f$ l_x \f$.
   */
  void refineKnotVectCurve(double *x, unsigned int r);

  /*!
   * Remove \f$ l_num \f$ times the knot \f$ l_u \f$ from the knot vector. The
   * removed knot \f$ l_u \f$ is the \f$ l_r \f$ th vector in the knot vector.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param l_u : A real number which is between the extremities of the knot
   * vector and which has to be removed.
   * \param l_r : Index of \f$ l_u \f$ in the knot vector.
   * \param l_num : Number of times \f$ l_u \f$ has to be removed.
   * \param l_TOL : A parameter which has to be computed.
   * \param l_s : Multiplicity of \f$ l_u \f$.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_knots : The knot vector
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   *
   * \return The number of time that l_u was removed.
   *
   * \f$ l_{TOL} = \frac{dw_{min}}{1+|P|_{max}} \f$
   *
   * where \f$ w_{min} \f$ is the minimal weight on the original curve, \f$
   * |P|_{max} \f$ is the maximum distance of any point on the original curve
   * from the origin and \f$ d \f$ is the desired bound on deviation.
   */
  static unsigned int removeCurveKnot(double l_u, unsigned int l_r, unsigned int l_num, double l_TOL, unsigned int l_s,
                                      unsigned int l_p, std::vector<double> &l_knots,
                                      std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);

  /*!
   * Remove \f$ num \f$ times the knot \f$ u \f$ from the knot vector. The
   * removed knot \f$ u \f$ is the \f$ r \f$ th vector in the knot vector.
   *
   * Of course the knot vector changes. But The list of control points and the
   * list of the associated weights change too.
   *
   * \param l_u : A real number which is between the extremities of the knot vector
   * and which has to be removed.
   * \param l_r : Index of \f$ l_u \f$ in the knot vector.
   * \param l_num : Number of times \f$ l_u \f$ has to be removed.
   * \param l_TOL : A parameter which has to be computed.
   *
   * \return The number of time that l_u was removed.
   *
   * \f$ TOL = \frac{dw_{min}}{1+|P|_{max}} \f$
   *
   * where \f$ w_{min} \f$ is the minimal weight on the original curve, \f$
   * |P|_{max} \f$ is the maximum distance of any point on the original curve
   * from the origin and \f$ d \f$ is the desired bound on deviation.
   */
  unsigned int removeCurveKnot(double l_u, unsigned int l_r, unsigned int l_num, double l_TOL);

  /*!
   * Method which enables to compute a NURBS curve passing through a set of data
   * points.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be interpolated.
   * \param l_p : Degree of the NURBS basis functions. This value need to be > 0.
   * \param l_knots : The knot vector.
   * \param l_controlPoints : The list of control points.
   * \param l_weights : the list of weights.
   */
  static void globalCurveInterp(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights);

  /*!
   * Method which enables to compute a NURBS curve passing through a set of data
   * points.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
   */
  void globalCurveInterp(vpList<vpMeSite> &l_crossingPoints);

  /*!
   * Method which enables to compute a NURBS curve passing through a set of data
   * points.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
   */
  void globalCurveInterp(const std::list<vpImagePoint> &l_crossingPoints);

  /*!
   * Method which enables to compute a NURBS curve passing through a set of data
   * points.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
  */
  void globalCurveInterp(const std::list<vpMeSite> &l_crossingPoints);

  /*!
   * Method which enables to compute a NURBS curve passing through a set of data
   * points.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   */
  void globalCurveInterp();

  /*!
   * Method which enables to compute a NURBS curve approximating a set of data
   * points.
   *
   * The data points are approximated thanks to a least square method.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be interpolated.
   * \param l_p : Degree of the NURBS basis functions.
   * \param l_n : The desired number of control points. l_n must be under or equal to the
   * number of data points.
   * \param l_knots : The knot vector.
   * \param l_controlPoints : the list of control points.
   * \param l_weights : the list of weights.
   */
  static void globalCurveApprox(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p, unsigned int l_n,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights);
  /*!
   * Method which enables to compute a NURBS curve approximating a set of
   * data points.
   *
   * The data points are approximated thanks to a least square method.
   *
   * The result of the method is composed by a knot vector, a set of
   * control points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
   *
   * \param n : The desired number of control points. This parameter \e n
   * must be under or equal to the number of data points.
   */
  void globalCurveApprox(vpList<vpMeSite> &l_crossingPoints, unsigned int n);

  /*!
   * Method which enables to compute a NURBS curve approximating a set of data
   * points.
   *
   * The data points are approximated thanks to a least square method.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
   * \param n : The desired number of control points. The parameter
   * \e n must be under or equal to the number of data points.
  */
  void globalCurveApprox(const std::list<vpImagePoint> &l_crossingPoints, unsigned int n);

  /*!
   * Method which enables to compute a NURBS curve approximating a set of
   * data points.
   *
   * The data points are approximated thanks to a least square method.
   *
   * The result of the method is composed by a knot vector, a set of
   * control points and a set of associated weights.
   *
   * \param l_crossingPoints : The list of data points which have to be
   * interpolated.
   *
   * \param n : The desired number of control points. This parameter \e n
   * must be under or equal to the number of data points.
   */
  void globalCurveApprox(const std::list<vpMeSite> &l_crossingPoints, unsigned int n);

  /*!
   * Method which enables to compute a NURBS curve approximating a set of data
   * points.
   *
   * The data points are approximated thanks to a least square method.
   *
   * The result of the method is composed by a knot vector, a set of control
   * points and a set of associated weights.
   */
  void globalCurveApprox(unsigned int n);
};
END_VISP_NAMESPACE
#endif
