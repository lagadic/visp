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
 * 2D Gravity Center moment descriptor (usually described by the pair Xg,Yg)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentGravityCenter.h
  \brief 2D Gravity Center moment descriptor (usually described by the pair
  Xg,Yg).
*/
#ifndef __MOMENTGRAVITYCENTER_H__
#define __MOMENTGRAVITYCENTER_H__

#include <visp3/core/vpMoment.h>
#include <visp3/core/vpMomentDatabase.h>
class vpMomentObject;

/*!
  \class vpMomentGravityCenter

  \ingroup group_core_moments

  \brief Class describing 2D gravity center moment.

  This moment can be computed from scratch (no need to compute any different
  moments before computing this). It gives access to both coordinates of the
  gravity center \f$x_g\f$ and \f$y_g\f$.

  These coordinates are defined as follows: \f$x_g = \frac{m_{01}}{m_{00}}
  \f$,\f$y_g = \frac{m_{10}}{m_{00}} \f$
\code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <iostream>

int main()
{
  // Define the contour of an object by a 5 clockwise vertices on a plane
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices of the contour polygon

  p.set_x(-0.2); p.set_y(0.1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(+0.3); p.set_y(0.1); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(+0.2); p.set_y(-0.1); // coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(-0.15); // coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-0.2); p.set_y(0.1); // close the contour (vertex 5 = vertex 1)
  vec_p.push_back(p);

  vpMomentObject obj(1); // Create an image moment object with 1 as
                         // maximum order (because only m00,m01,m10
                         // are needed to compute the gravity center primitive.
  obj.setType(vpMomentObject::DENSE_POLYGON); // The object is defined by a countour polygon
  obj.fromVector(vec_p); // Init the dense object with the polygon

  vpMomentGravityCenter g; // declaration of gravity center
  g.update(obj); // specify the object
  g.compute(); // compute the moment

  std::cout << "Xg=" << g.getXg() << std::endl; // access to Xg
  std::cout << "Yg=" << g.getYg() << std::endl; // access to Yg

  std::cout << g << std:: endl; // print gravity center

  return 0;
}
\endcode

This example produces the following results:
\code
Xg=0.0166667
Yg=-0.00833333
Xg=0.0166667, Yg=-0.00833333
\endcode
*/

class VISP_EXPORT vpMomentGravityCenter : public vpMoment
{
public:
  vpMomentGravityCenter();
  virtual ~vpMomentGravityCenter(){};

  /** @name Inherited functionalities from vpMomentGravityCenter */
  //@{
  void compute();
  const std::vector<double> &get() const;
  /*!
    Shortcut function to retrieve \f$x_g\f$.
    \return The first gravity center coordinate.
   */
  double getXg() const { return values[0]; }
  /*!
    Shortcut function to retrieve \f$y_g\f$.
    \return The second gravity center coordinate.
   */
  double getYg() const { return values[1]; }
  /*!
    The class's string name.
   */
  const char *name() const { return "vpMomentGravityCenter"; }
  void printDependencies(std::ostream &os) const;
  //@}
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentGravityCenter &v);
};

#endif
