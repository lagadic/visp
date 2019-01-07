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
 * Alpha moment descriptor for in-plane orientation.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpMomentAlpha.h
  \brief Alpha moment descriptor for in-plane orientation.
*/

#ifndef _vpMomentAlpha_h_
#define _vpMomentAlpha_h_

#include <visp3/core/vpMoment.h>

/*!
  \class vpMomentAlpha

  \ingroup group_core_moments

  \brief This class defines the orientation of the object inside the plane
  parallel to the object.

  In general the value of the moment is computed in \f$ [-\pi/2 ; \pi/2] \f$
  interval by the formula \f$ \alpha = \frac{1}{2}
  \mathrm{atan2}(2\mu_{11}, \mu_{20}-\mu_{02}) \f$.

  To obtain a \f$ [-\pi ; \pi] \f$ precision for non symetric object, you
  have to specify a reference information. This reference information is an
  alpha computed using the previous formula in \f$ [-\pi/2 ; \pi/2] \f$.
  Obtaining this precision comes from third-order centered moments and this
  reference information.

  Therefore there are two modes for vpMomentAlpha and one constructor per
  mode:
  - Reference mode using the empty constructor vpMomentAlpha():
  The vpMomentAlpha doesn't need any additionnal information, it will compute
  its values from available moments in \f$ [-\pi/2 ; \pi/2] \f$.
  - Relative mode using non-empty constructor
  vpMomentAlpha(std::vector<double>&, double): The vpMomentAlpha is computed in
  \f$ [-\pi ; \pi] \f$ from the available moments and the reference
  information. By knowing the reference, it may distinguish in-plane rotations
  of \f$ \alpha \f$ from rotations of \f$ \alpha + \pi \f$.

  The following code demonstrates a calculation of a reference alpha and then
  uses this alpha to estimate the orientation of the same object after
  performing a 180 degrees rotation. Therefore the first and second alpha should
  have opposite values.

  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAlpha.h>

//generic function for printing
void print (double i) { std::cout << i << "\t";}

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // Vector that contains the vertices of the contour polygon
  p.set_x(1); p.set_y(1);     // Coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2);     // Coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(-3); p.set_y(0);    // Coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(-3); p.set_y(-1);   // Coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject objRef(3);                      // Reference object. Must be of order 3 because we will
                                                 // need the 3rd order centered moments

  objRef.setType(vpMomentObject::DENSE_POLYGON); // Object is the inner part of a polygon
  objRef.fromVector(vec_p);                      // Init the dense object with the polygon

  vpMomentDatabase dbRef;                        // Reference database
  vpMomentGravityCenter gRef;                    // Declaration of gravity center
  vpMomentCentered mcRef;                        // Centered moments
  vpMomentAlpha alphaRef;                        // Declare alpha as reference

  gRef.linkTo(dbRef);                            // Add gravity center to database
  mcRef.linkTo(dbRef);                           // Add centered moments
  alphaRef.linkTo(dbRef);                        // Add alpha depending on centered moments

  dbRef.updateAll(objRef);                       // All of the moments must be updated, not just alpha

  gRef.compute();                                // Compute the moment
  mcRef.compute();                               // Compute centered moments AFTER gravity center
  alphaRef.compute();                            // Compute alpha AFTER centered moments.

  // The order of values in the vector must be as follows: mu30 mu21 mu12 mu03
  std::vector<double> mu3ref = {mcRef.get(3,0), mcRef.get(2,1), mcRef.get(1,2), mcRef.get(0,3)};

  std::cout << "--- Reference object ---" << std::endl;
  std::cout << "alphaRef=" << vpMath::deg(alphaRef.get()) << " deg" << std::endl << "mu3="; // print reference alpha
  std::for_each (mu3ref.begin(), mu3ref.end(), print);
  std::cout << std::endl;

  ////////////CURRENT VALUES (same object rotated 180deg - must be
  ////////////entered in reverse order)////////////////
  vec_p.clear();

  p.set_x(-3); p.set_y(1);                       // Coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-3); p.set_y(0);                       // Coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(-2);                       // Coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(1); p.set_y(-1);                       // Coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);

  vpMomentObject obj(3);                         // Second object. Order 3 is also required because of the Alpha
                                                 // will compare third-order centered moments to given reference.

  obj.setType(vpMomentObject::DENSE_POLYGON);    // Object is the inner part of a polygon
  obj.fromVector(vec_p);                         // Init the dense object with the polygon

  vpMomentDatabase db;                           // Database
  vpMomentGravityCenter g;                       // Declaration of gravity center
  vpMomentCentered mc;                           // mc containts centered moments
  vpMomentAlpha alpha(mu3ref, alphaRef.get());   // Declare alpha as relative to a reference

  g.linkTo(db);                                  // Add gravity center to database
  mc.linkTo(db);                                 // Add centered moments
  alpha.linkTo(db);                              // Add alpha depending on centered moments

  db.updateAll(obj);                             // All of the moments must be updated

  g.compute();                                   // Compute the moment
  mc.compute();                                  // Compute centered moments AFTER gravity center
  alpha.compute();                               // Compute alpha AFTER centered moments.

  std::cout << "--- current object ---" << std::endl;
  std::cout << "alpha=" << vpMath::deg(alpha.get()) << " deg" << std::endl;

  return 0;
}
  \endcode
This program outputs:
\code
--- Reference object ---
alphaRef=25.3019 deg
mu3=1.80552	0.921882	0.385828	0.122449
--- current object ---
alpha=-25.3019 deg
\endcode

  There is also testMomentAlpha.cpp example that shows how to compute alpha in the range \f$ [-\pi ; \pi] \f$
  using arrow images as input. The code is given below:
  \include testMomentAlpha.cpp

  From the first image we compute the 3rd order centered moments and the value of the reference alpha
  that is than used to compute the alpha moment in the range \f$ [-\pi ; \pi] \f$. Running this example you will get:
  \code
alpha expected 0 computed -0.128108 deg
alpha expected 45 computed 44.8881 deg
alpha expected 90 computed 89.8719 deg
alpha expected 135 computed 134.888 deg
alpha expected 180 computed 179.872 deg
alpha expected -135 computed -135.112 deg
alpha expected -90 computed -90.1281 deg
alpha expected -45 computed -45.1119 deg
  \endcode

  Shortcuts for quickly getting those references exist in vpMomentCommon.

  This moment depends on vpMomentCentered.
*/
class VISP_EXPORT vpMomentAlpha : public vpMoment
{
private:
  bool m_isRef;
  bool m_symmetric;
  std::vector<double> m_mu3Ref;
  double m_alphaRef;
  double m_symmetricThreshold;

public:
  vpMomentAlpha();
  vpMomentAlpha(const std::vector<double> &mu3_ref, double alpha_ref, double threshold = 1e-6);
  virtual ~vpMomentAlpha(){};

  void compute();
  /*!
     Retrieve the orientation of the object as a single double value.
   */
  double get() const { return values[0]; }
  /*!
     Moment name.
   */
  const char *name() const { return "vpMomentAlpha"; }

  /*!
     Returns true if the alpha moment was constructed as a reference with values in \f$ [-\pi/2 ; \pi/2] \f$, false otherwise.
   */
  inline bool is_ref() const
  {
    if (m_isRef)
      return true;
    else
      return false;
  }

  /*!
     Returns true if the alpha moment is computed on a symmetric object along its two axis.
     Symmetry is computed using 3rd order centered moments \f$\mu_{30},\mu_{21},\mu_{12},\mu_{03}\f$.
   */
  inline bool is_symmetric() const
  {
    if (m_symmetric)
      return true;
    else
      return false;
  }

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentAlpha &v);
  void printDependencies(std::ostream &os) const;
};

#endif
