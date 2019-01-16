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
 * Base for 2D moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpMoment.cpp
  \brief Base class for all 2D moments.
*/

#include <cstring>
#include <visp3/core/vpMoment.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentObject.h>
/*!
  Default constructor
*/
vpMoment::vpMoment() : object(NULL), moments(NULL), values() {}

/*!
  Links the moment to a database of moment primitives.
  If the moment depends on other moments, these moments must be linked to the
same database. \attention Two moments of the same class cannot be stored in
the same database
\code
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p;

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p); p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)

  vpMomentObject obj(2);
  obj.setType(vpMomentObject::DISCRETE); // Discrete mode.
  obj.fromVector(vec_p); // Init the dense object with the polygon

  vpMomentDatabase db;
  vpMomentGravityCenter G; // declaration of gravity center
  vpMomentCentered mc; // mc containts centered moments

  G.linkTo(db); //add gravity center to database
  mc.linkTo(db); //centered moments depend on gravity, add them to the
                 //database to grant access

  G.update(obj); // specify the object for gravity center
  mc.update(obj); // and for centered moments

  G.compute(); // compute the moment
  mc.compute(); //compute centered moments AFTER gravity center

  return 0;
}

  \endcode

  \param data_base : database of moment primitives.
*/
void vpMoment::linkTo(vpMomentDatabase &data_base)
{
  if (strlen(name()) >= 255) {
    throw(vpException(vpException::memoryAllocationError, "Not enough memory to intialize the moment name"));
  }

  std::strcpy(_name, name());
  this->moments = &data_base;

  data_base.add(*this, _name);
}

/*!
  Updates the moment with the current object. This does not compute any
  values. \param moment_object : object descriptor of the current camera
  vision.
*/
void vpMoment::update(vpMomentObject &moment_object) { this->object = &moment_object; }

/*!
  Prints the moment contents to a stream
  \param os : a std::stream.
  \param m : a moment instance.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMoment &m)
{
  for (std::vector<double>::const_iterator i = m.values.begin(); i != m.values.end(); ++i)
    os << *i << ",";

  return os;
}

/*!
Prints values of all dependent moments required to calculate a specific
vpMoment. Not made pure to maintain compatibility Recommended : Types
inheriting from vpMoment should implement this function
*/
void vpMoment::printDependencies(std::ostream &os) const
{
  os << " WARNING : Falling back to base class version of "
        "printDependencies(). To prevent that, this has to be implemented in "
        "the derived classes!"
     << std::endl;
}
