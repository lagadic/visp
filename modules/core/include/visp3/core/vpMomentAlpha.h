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

#ifndef __MOMENTALPHA_H__
#define __MOMENTALPHA_H__

#include <visp3/core/vpMoment.h>

/*!
  \class vpMomentAlpha

  \ingroup group_core_moments

  \brief This class defines the orientation of the object inside the plane
parallel to the object.

  In general the value of the moment is computed in \f$ [-\pi/2 .. \pi/2] \f$
interval by the formula \f$ \alpha = \frac{1}{2}
arctan(\frac{2\mu_{11}}{\mu_{20}-\mu_{02}}) \f$.

  To obtain a \f$ [-\pi .. \pi] \f$ precision for non symetric object, you
have to specify a reference information. This reference information is an
alpha computed using the previous formula in \f$ [-\pi/2 .. \pi/2] \f$.
Obtaining this precision comes from third-order centered moments and this
reference information.

  Therefore there are two modes for vpMomentAlpha and one constructor per
mode:
  - Reference mode (using the empty constructor vpMomentAlpha()):
  The vpMomentAlpha doesn't need any additionnal information, it will compute
its values from available moments in \f$ [-\pi/2 .. \pi/2] \f$.
  - Relative mode (using non-empty constructor
vpMomentAlpha(std::vector<double>&, double)): The vpMomentAlpha is computed in
\f$ [-\pi .. \pi] \f$ from the available moments and the reference
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
#include <iostream>
#include <vector>
#include <algorithm>

//generic function for printing
void print (double i) { std::cout << i << "\t";}

int main()
{
  vpPoint p;
  // vector that contains the vertices of the contour polygon
  std::vector<vpPoint> vec_p;

  // coordinates in meters in the image plane (vertex 1)
  p.set_x(1); p.set_y(1);
  vec_p.push_back(p);
  // coordinates in meters in the image plane (vertex 2)
  p.set_x(2); p.set_y(2);
  vec_p.push_back(p);
  // coordinates in meters in the image plane (vertex 3)
  p.set_x(-3); p.set_y(0);
  vec_p.push_back(p);
  // coordinates in meters in the image plane (vertex 4)
  p.set_x(-3); p.set_y(-1);
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject objRef(3); // Reference object. Must be of order 3
                            // because we will need the 3rd order
                            // centered moments
  // object is the inner part of a polygon
  objRef.setType(vpMomentObject::DENSE_POLYGON);
  objRef.fromVector(vec_p); // Init the dense object with the polygon

  vpMomentDatabase dbRef; //reference database
  vpMomentGravityCenter gRef; // declaration of gravity center
  vpMomentCentered mcRef; //  centered moments
  vpMomentAlpha alphaRef; //declare alpha as reference

  gRef.linkTo(dbRef); //add gravity center to database
  mcRef.linkTo(dbRef); //add centered moments
  alphaRef.linkTo(dbRef); //add alpha depending on centered moments

  dbRef.updateAll(objRef); // All of the moments must be updated, not just alpha

  gRef.compute(); // compute the moment
  mcRef.compute(); //compute centered moments AFTER gravity center
  alphaRef.compute(); //compute alpha AFTER centered moments.

  //the order of values in the vector must be as follows:
  //mu30 mu21 mu12 mu03
  std::vector<double> mu3ref(4);
  mu3ref[0] = mcRef.get(3,0);
  mu3ref[1] = mcRef.get(2,1);
  mu3ref[2] = mcRef.get(1,2);
  mu3ref[3] = mcRef.get(0,3);


  std::cout << "--- Reference object ---" << std::endl;
  std::cout << "alphaRef=" << alphaRef << std::endl << "mu3="; // print reference alpha
  std::for_each (mu3ref.begin(), mu3ref.end(),print);
  std::cout << std::endl;

  ////////////CURRENT VALUES (same object rotated 180deg - must be
  ////////////entered in reverse order)////////////////
  vec_p.clear();

  p.set_x(-3); p.set_y(1); // coordinates in meters in the image plane (vertex 4)
  vec_p.push_back(p);
  p.set_x(-3); p.set_y(0); // coordinates in meters in the image plane (vertex 3)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(-2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  p.set_x(1); p.set_y(-1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);

  vpMomentObject obj(3); // second object. Order 3 is also required
                         // because of the Alpha will compare
                         // third-order centered moments to given reference.

  obj.setType(vpMomentObject::DENSE_POLYGON); // object is the inner part of a polygon
  obj.fromVector(vec_p); // Init the dense object with the polygon

  vpMomentDatabase db; // database
  vpMomentGravityCenter g; // declaration of gravity center
  vpMomentCentered mc; // mc containts centered moments
  vpMomentAlpha alpha(mu3ref,alphaRef.get()); //declare alpha as relative to a reference

  g.linkTo(db); //add gravity center to database
  mc.linkTo(db); //add centered moments
  alpha.linkTo(db); //add alpha depending on centered moments

  db.updateAll(obj); // All of the moments must be updated

  g.compute(); // compute the moment
  mc.compute(); //compute centered moments AFTER gravity center
  alpha.compute(); //compute alpha AFTER centered moments.

  std::cout << "--- current object ---" << std::endl;
  std::cout << "alpha=" << alpha.get() << std::endl;

  return 0;
}
  \endcode
This program outputs:
\code
--- Reference object ---
alphaRef=0.441601
mu3=1.80552	0.921882	0.385828	0.122449
--- current object ---
alpha=-0.441601

\endcode

Shortcuts for quickly getting those references exist in vpMomentCommon.

This moment depends on vpMomentCentered.
*/
class VISP_EXPORT vpMomentAlpha : public vpMoment
{
private:
  bool isRef;
  bool symmetric;
  std::vector<double> ref;
  double alphaRef;

public:
  vpMomentAlpha();
  vpMomentAlpha(const std::vector<double> &ref, double alphaRef);
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

  inline bool is_ref() const
  {
    if (isRef)
      return true;
    else
      return false;
  }

  inline bool is_symmetric() const
  {
    if (symmetric)
      return true;
    else
      return false;
  }

  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentAlpha &v);
  void printDependencies(std::ostream &os) const;
};

#endif
