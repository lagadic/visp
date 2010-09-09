/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  \brief Class that provides tools to compute and manipulate a Non Uniform Rational B-Spline curve.
*/

#include <visp/vpConfig.h>
#include <visp/vpImagePoint.h>
#include <visp/vpList.h>
#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpMeSite.h>
#include <visp/vpBSpline.h>


/*!
  \class vpNurbs
  \ingroup MathTools

  \brief Class that provides tools to compute and manipulate a Non Uniform Rational B-Spline curve.
  
  The different parameters are :

  - The knot vector \f$ U = {u_0, ... , u_m} \f$ where the knots \f$ u_i, i = 0, ...,m \f$ are real number such as \f$ u_i < u_{i+1} i = 0, ...,m \f$.
    To define a curve, the knot vector is such as : \f$ U = {a , ... , a, u_{p+1} , ... , u_{m-p-1} , b , ... , b} \f$ where \f$ a \f$ and \f$ b \f$ are real numbers and p is the degree of the B-Spline basis functions.

  - The B-Spline basis functions \f$ N_{i,p} \f$ defined as :
  \f[ N_{i,0}(u) = \left\{\begin{array}{cc}
  1 & \mbox{si } u_i \leq u_{i+1} \\ 0 & sinon
  \end{array}\right.\f]

  \f[ N_{i,p}(u) = \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u)\f]
  
  where \f$ i = 0 , ... , m-1 \f$ and p is the degree of the B-Spline basis functions.

  - The control points \f$ {P_i} \f$ which are defined by the coordinates \f$ (i,j) \f$ of a point in an image.
  
  - The weight \f$ {w_i} \f$ associated to each control points.The wheights value is upper than 0.

  It is possible to compute the coordinates of a point corresponding to the knots \f$ u \f$ (\f$ u \in [u_0,u_m]\f$) thanks to the formula :
  \f[ C(u) = \frac{\sum_{i=0}^n (N_{i,p}(u)w_iP_i)}{\sum_{i=0}^n (N_{i,p}(u)w_i)}\f]

  You can find much more information about the B-Splines and the implementation of all the methods in the Nurbs Book. 
*/

class VISP_EXPORT vpNurbs : public vpBSpline 
{
  protected:
    std::vector<double> weights;  //Vector which contains the weights associated to each control Points

  public:

    vpNurbs();
    vpNurbs(const vpNurbs &nurbs);
    virtual ~vpNurbs();
    
    /*!
      Gets all the weights relative to the control points.
  
      \return list : A vpList containing weights relative to the control points.
    */
    inline vpList<double> get_weights() const {
      vpList<double> list;
      for (unsigned int i = 0; i < weights.size(); i++) list.addRight(*(&(weights[0])+i));
      return list; }
      
    /*!
      Sets all the knots.
      
      \param list : A vpList containing the value of the knots.
    */
    inline void set_weights(vpList<double> &list) {
      weights.clear();
      list.front();
      for (int i = 0; i < list.nbElements(); i++) 
      {
        weights.push_back(list.value());
        list.next();
      }
    }

    static vpImagePoint computeCurvePoint(double l_u, int l_i, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    vpImagePoint computeCurvePoint(double u);

    static vpImagePoint* computeCurveDersPoint(double l_u, int l_i, int l_p, int l_der, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    vpImagePoint* computeCurveDersPoint(double u, int der);

    static void curveKnotIns(double l_u, int l_k, int l_s, int l_r, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    void curveKnotIns(double u, int s = 0, int r = 1);

    static void refineKnotVectCurve(double* l_x, int l_r, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    void refineKnotVectCurve(double* x, int r);

    static int removeCurveKnot(double l_u, int l_r, int l_num, double l_TOL, int l_s, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    int removeCurveKnot(double l_u, int l_r, int l_num, double l_TOL);

    static void globalCurveInterp(std::vector<vpImagePoint> &l_crossingPoints, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    void globalCurveInterp(vpList<vpImagePoint>& l_crossingPoints);
    void globalCurveInterp(vpList<vpMeSite>& l_crossingPoints);
    void globalCurveInterp();

    static void globalCurveApprox(std::vector<vpImagePoint> &l_crossingPoints, int l_p, int l_n, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    void globalCurveApprox(vpList<vpImagePoint>& l_crossingPoints, int n);
    void globalCurveApprox(vpList<vpMeSite>& l_crossingPoints, int n);
    void globalCurveApprox(int n);
    
  protected:
    static vpMatrix computeCurveDers(double l_u, int l_i, int l_p, int l_der, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints, std::vector<double> &l_weights);
    vpMatrix computeCurveDers(double u, int der);
};

#endif
