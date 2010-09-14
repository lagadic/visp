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
 * See the file LICENSE.txt at the root directory of this source
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
 * This class implements the B-Spline
 *
 * Authors:
 * Nicolas Melchior
 * 
 *****************************************************************************/

#ifndef vpBSpline_H
#define vpBSpline_H

/*!
  \file vpBSpline.h
  \brief Class that provides tools to compute and manipulate a B-Spline curve.
*/

#include <vector>

#include <visp/vpConfig.h>
#include <visp/vpImagePoint.h>
#include <visp/vpList.h>
#include <visp/vpMatrix.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// For template instantiation with Visual Studio
#if defined(VISP_BUILD_SHARED_LIBS) && defined(VISP_USE_MSVC)
template class VISP_EXPORT std::allocator<vpImagePoint>;
template class VISP_EXPORT std::vector<vpImagePoint>;
template class VISP_EXPORT std::allocator<double>;
template class VISP_EXPORT std::vector<double>;
#endif


/*!
  Structure that defines a B-Spline basis function \f$ N_{i,p}^k(u) \f$.
  
  - i is the number of the knot interval in which the basis function is computed.
  - p is the degree of the B-Spline basis function.
  - u is the "point" point of the curve where the basis function is computed.
  - k indicates which kth derivative is computed.
  - value is the numerical value of \f$ N_{i,p}^k(u) \f$.
*/
typedef struct vpBasisFunction{
  int i;
  int p;
  double u;
  int k;
  double value;
} vpBasisFunction;
#endif

/*!
  \class vpBSpline
  \ingroup MathTools


  \brief Class that provides tools to compute and manipulate a B-Spline curve.

  The different parameters are :

  - The knot vector \f$ U = {u_0, ... , u_m} \f$ where the knots \f$ u_i, i = 0, ...,m \f$ are real number such as \f$ u_i < u_{i+1}, i = 0, ...,m \f$.
    To define a curve, the knot vector is such as : \f$ U = {a , ... , a, u_{p+1} , ... , u_{m-p-1} , b , ... , b} \f$ where \f$ a \f$ and \f$ b \f$ are real numbers and p is the degree of the B-Spline basis functions.

  - The B-Spline basis functions \f$ N_{i,p} \f$ defined as :
  \f[ N_{i,0}(u) = \left\{\begin{array}{cc}
  1 & \mbox{if } u_i \leq u \leq u_{i+1} \\ 0 & else
  \end{array}\right.\f]

  \f[ N_{i,p}(u) = \frac{u-u_i}{u_{i+p}-u_i}N_{i,p-1}(u)+\frac{u_{i+p+1}-u}{u_{i+p+1}-u_{i+1}}N_{i+1,p-1}(u)\f]
  
  where \f$ i = 0 , ... , m-1 \f$ and p is the degree of the B-Spline basis functions.

  - The control points \f$ {P_i} \f$ which are defined by the coordinates \f$ (i,j) \f$ of a point in an image.

  It is possible to compute the coordinates of a point corresponding to the knots \f$ u \f$ (\f$ u \in [u_0,u_m]\f$) thanks to the formula :
  \f[ C(u) = \sum_{i=0}^n (N_{i,p}(u)P_i)\f]

  You can find much more information about the B-Splines and the implementation of all the methods in the Nurbs Book. 
*/

class VISP_EXPORT vpBSpline
{
  public/*protected*/:
    //!Vector wich contains the control points 
    std::vector<vpImagePoint> controlPoints;
    //! Vector which contain the knots \f$ {u0, ..., um} \f$
    std::vector<double> knots;
    //! Degree of the B-Spline basis functions.
    int p;
    //! Vector wich contains the points used during the interpolation method.
    std::vector<vpImagePoint> crossingPoints;  

  public:

    vpBSpline();
    vpBSpline(const vpBSpline &bspline);
    virtual ~vpBSpline();
    
	/*!
	  Gets the degree of the B-Spline.
	  
	  \return the degree of the B-Spline.
	*/
	inline int get_p() const {return p;}
	
	/*!
	  Gets all the control points.
	  
	  \return list : A vpList containing the coordinates of the control points
	*/
	inline vpList<vpImagePoint> get_controlPoints() const {
	  vpList<vpImagePoint> list;
	  for (unsigned int i = 0; i < controlPoints.size(); i++) list.addRight(*(&(controlPoints[0])+i));
	  return list; }

	/*!
	  Gets all the knots.
	  
	  \return list : A vpList containing the value of the knots.
	*/
	inline vpList<double> get_knots() const {
	  vpList<double> list;
	  for (unsigned int i = 0; i < knots.size(); i++) list.addRight(*(&(knots[0])+i));
	  return list; }

	/*!
	  Gets all the crossing points (used in the interpolation method)
	  
	  \return list : A vpList containing the coordinates of the crossing points
	*/
	inline vpList<vpImagePoint> get_crossingPoints() const {
	  vpList<vpImagePoint> list;
	  for (unsigned int i = 0; i < crossingPoints.size(); i++) list.addRight(*(&(crossingPoints[0])+i));
	  return list; }
	  
	  
	/*!
	  Sets the degree of the B-Spline.
	  
	  \param p : the degree of the B-Spline.
	*/
	inline void set_p(int p) {this->p = p;}
	  
	/*!
	  Sets all the control points.
	  
	  \param list : A vpList containing the coordinates of the control points
	*/
	inline void set_controlPoints(vpList<vpImagePoint> &list) {
	  controlPoints.clear();
	  list.front();
	  for (int i = 0; i < list.nbElements(); i++) 
	  {
	    controlPoints.push_back(list.value());
		list.next();
	  }
	}

	/*!
	  Sets all the knots.
	  
	  \param list : A vpList containing the value of the knots.
	*/
	inline void set_knots(vpList<double> &list) {
	  knots.clear();
	  list.front();
	  for (int i = 0; i < list.nbElements(); i++) 
	  {
	    knots.push_back(list.value());
		list.next();
	  }
	}

	/*!
	  Sets all the crossing points (used in the interpolation method)
	  
	  \param list : A vpList containing the coordinates of the crossing points
	*/
	inline void set_crossingPoints(vpList<vpImagePoint> &list) {
	  crossingPoints.clear();
	  list.front();
	  for (int i = 0; i < list.nbElements(); i++) 
	  {
	    crossingPoints.push_back(list.value());
		list.next();
	  }
	}

    static int findSpan(double l_u, int l_p, std::vector<double> &l_knots);
    int findSpan(double u);

    static vpBasisFunction* computeBasisFuns(double l_u, int l_i, int l_p, std::vector<double> &l_knots);
    vpBasisFunction* computeBasisFuns(double u);

    static vpBasisFunction** computeDersBasisFuns(double l_u, int l_i, int l_p, int l_der, std::vector<double> &l_knots);
    vpBasisFunction** computeDersBasisFuns(double u, int der);

    static vpImagePoint computeCurvePoint(double l_u, int l_i, int l_p, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints);
    vpImagePoint computeCurvePoint(double u);

    static vpImagePoint* computeCurveDers(double l_u, int l_i, int l_p, int l_der, std::vector<double> &l_knots, std::vector<vpImagePoint> &l_controlPoints);
    vpImagePoint* computeCurveDers(double u, int der);
};

#endif
