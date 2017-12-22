/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef vpNurbs_H
#define vpNurbs_H

/*!
  \file vpNurbs.h
  \brief Class that provides tools to compute and manipulate a Non Uniform
  Rational B-Spline curve.
*/

#include <visp3/core/vpBSpline.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpList.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/me/vpMeSite.h>

#include <list>

/*!
  \class vpNurbs
  \ingroup module_me

  \brief Class that provides tools to compute and manipulate a Non Uniform
  Rational B-Spline curve.

  The different parameters are :

  - The knot vector \f$ U = {u_0, ... , u_m} \f$ where the knots \f$ u_i, i =
  0, ...,m \f$ are real number such as \f$ u_i < u_{i+1} i = 0, ...,m \f$. To
  define a curve, the knot vector is such as : \f$ U = {a , ... , a, u_{p+1} ,
  ... , u_{m-p-1} , b , ... , b} \f$ where \f$ a \f$ and \f$ b \f$ are real
  numbers and p is the degree of the B-Spline basis functions.

  - The B-Spline basis functions \f$ N_{i,p} \f$ defined as :
  \f[ N_{i,0}(u) = \left\{\begin{array}{cc}
  1 & \mbox{if } u_i \leq u_{i+1} \\ 0 & else
  \end{array}\right.\f]

  \f[ N_{i,p}(u) =
  \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u)\f]

  where \f$ i = 0 , ... , m-1 \f$ and p is the degree of the B-Spline basis
  functions.

  - The control points \f$ {P_i} \f$ which are defined by the coordinates \f$
  (i,j) \f$ of a point in an image.

  - The weight \f$ {w_i} \f$ associated to each control points.The wheights
  value is upper than 0.

  It is possible to compute the coordinates of a point corresponding to the
  knots \f$ u \f$ (\f$ u \in [u_0,u_m]\f$) thanks to the formula : \f[ C(u) =
  \frac{\sum_{i=0}^n (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)}\f]

  You can find much more information about the B-Splines and the
  implementation of all the methods in the Nurbs Book.
*/

class VISP_EXPORT vpNurbs : public vpBSpline
{
protected:
  std::vector<double> weights; // Vector which contains the weights associated
                               // to each control Points

protected:
  static vpMatrix computeCurveDers(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                   std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                   std::vector<double> &l_weights);
  vpMatrix computeCurveDers(double u, unsigned int der);

public:
  vpNurbs();
  vpNurbs(const vpNurbs &nurbs);
  virtual ~vpNurbs();

  /*!
      Gets all the weights relative to the control points.

      \param list [out] : A std::list containing weights relative to the
     control points.
    */
  inline void get_weights(std::list<double> &list) const
  {
    list.clear();
    for (unsigned int i = 0; i < weights.size(); i++)
      list.push_back(*(&(weights[0]) + i));
  }

  /*!
      Sets all the knots.

      \param list : A std::list containing the value of the knots.
    */
  inline void set_weights(const std::list<double> &list)
  {
    weights.clear();
    for (std::list<double>::const_iterator it = list.begin(); it != list.end(); ++it) {
      weights.push_back(*it);
    }
  }

  static vpImagePoint computeCurvePoint(double l_u, unsigned int l_i, unsigned int l_p, std::vector<double> &l_knots,
                                        std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
  vpImagePoint computeCurvePoint(double u);

  static vpImagePoint *computeCurveDersPoint(double l_u, unsigned int l_i, unsigned int l_p, unsigned int l_der,
                                             std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                             std::vector<double> &l_weights);
  vpImagePoint *computeCurveDersPoint(double u, unsigned int der);

  static void curveKnotIns(double l_u, unsigned int l_k, unsigned int l_s, unsigned int l_r, unsigned int l_p,
                           std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                           std::vector<double> &l_weights);
  void curveKnotIns(double u, unsigned int s = 0, unsigned int r = 1);

  static void refineKnotVectCurve(double *l_x, unsigned int l_r, unsigned int l_p, std::vector<double> &l_knots,
                                  std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
  void refineKnotVectCurve(double *x, unsigned int r);

  static unsigned int removeCurveKnot(double l_u, unsigned int l_r, unsigned int l_num, double l_TOL, unsigned int l_s,
                                      unsigned int l_p, std::vector<double> &l_knots,
                                      std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
  unsigned int removeCurveKnot(double l_u, unsigned int l_r, unsigned int l_num, double l_TOL);

  static void globalCurveInterp(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights);
  void globalCurveInterp(vpList<vpMeSite> &l_crossingPoints);
  void globalCurveInterp(const std::list<vpImagePoint> &l_crossingPoints);
  void globalCurveInterp(const std::list<vpMeSite> &l_crossingPoints);
  void globalCurveInterp();

  static void globalCurveApprox(std::vector<vpImagePoint> &l_crossingPoints, unsigned int l_p, unsigned int l_n,
                                std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints,
                                std::vector<double> &l_weights);
  void globalCurveApprox(vpList<vpMeSite> &l_crossingPoints, unsigned int n);
  void globalCurveApprox(const std::list<vpImagePoint> &l_crossingPoints, unsigned int n);
  void globalCurveApprox(const std::list<vpMeSite> &l_crossingPoints, unsigned int n);
  void globalCurveApprox(unsigned int n);
};

#endif
