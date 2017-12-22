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
 * 2D normalized surface moment descriptor (usually described as An)
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentAreaNormalized.h
  \brief 2D normalized surface moment descriptor (usually described as An).
*/
#ifndef __MOMENTSURFACENORMALIZED_H__
#define __MOMENTSURFACENORMALIZED_H__

#include <visp3/core/vpMoment.h>

class vpMomentObject;
class vpMomentCentered;

/*!
  \class vpMomentAreaNormalized

  \ingroup group_core_moments

  \brief Class handling the normalized surface moment that is invariant in
scale and used to estimate depth.

  This moment depends on vpMomentCentered.

  The idea behind vpMomentAreaNormalized is described in \cite Tahri05z.

  During a visual servoing process, a vpMomentAreaNormalized will converge
towards the desired depth when the current surface will converge to the
destination surface. It is defined as follows: \f$ a_n=Z^*
\sqrt{\frac{a^*}{a}} \f$ where \e a is the current surface and \e a* the
destination surface. Consequently, the vpMomentAreaNormalized needs to have
information about the desired depth \e Z* and the desired surface \e a*.

  \warning About the order of the object.
  The surface (refered to as \e a in the above paragraph) depends of the
nature of the object.
  - In case of a continuous object (when vpMomentObject::getType() is
vpMomentObject::DENSE_FULL_OBJECT or vpMomentObject::DENSE_POLYGON)
\f$a=m_{00}\f$.
  - In case of a discrete object (when vpMomentObject::getType() is
vpMomentObject::DISCRETE) \f$a=\mu_{20}+\mu_{02}\f$.

  Therefore, a vpMomentObject has to be of minimum order 2 in order to compute
a vpMomentAreaNormalized moment in the discrete case and of minimum order 0 in
continous cases.

  This example shows a computation in the discrete case.
  \code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAreaNormalized.h>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains object points

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane of point 1
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane of point 2
  vec_p.push_back(p);

  //////////////////////////////REFERENCE VALUES////////////////////////////////
  vpMomentObject obj(2); // Object where all the moment defined with
                         // i+j <= 2 will be computed below. Order is
                         // 2 because in discrete mode, the surface
                         // computation is a=mu02+mu02


  obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
  obj.fromVector(vec_p); // initialize the object with the points coordinates

  vpMomentDatabase db; //reference database
  vpMomentGravityCenter g; // declaration of gravity center
  vpMomentCentered mc; //  centered moments
  vpMomentAreaNormalized an(2,1); //declare normalized surface with
                                //destination depth of 1 meter and
                                //destination surface of 2 m2

  g.linkTo(db); //add gravity center to database
  mc.linkTo(db); //add centered moments
  an.linkTo(db); //add alpha depending on centered moments

  db.updateAll(obj); // All of the moments must be updated, not just an

  g.compute(); // compute the moment
  mc.compute(); //compute centered moments AFTER gravity center
  an.compute(); //compute alpha AFTER centered moments.

  std::cout << an << std::endl;

  return 0;
}
  \endcode
    This code produces the following output:
  \code
An:1.41421
  \endcode
*/
class VISP_EXPORT vpMomentAreaNormalized : public vpMoment
{
private:
  double desiredSurface;
  double desiredDepth;

public:
  vpMomentAreaNormalized(double desiredSurface, double desiredDepth);
  virtual ~vpMomentAreaNormalized(){};
  void compute();
  /*!
  Retrieves the desired depth \e Z* as specified in the constructor.
  */
  double getDesiredDepth() const { return desiredDepth; }
  /*!
  Retrieves the desired surface \e a* as specified in the constructor.
  */
  double getDesiredSurface() const { return desiredSurface; }

  /*!
  Moment name.
  */
  const char *name() const { return "vpMomentAreaNormalized"; }
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentAreaNormalized &v);
  void printDependencies(std::ostream &os) const;
};

#endif
