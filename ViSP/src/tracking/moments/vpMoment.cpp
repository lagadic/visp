/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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


#include <visp/vpMomentObject.h>
#include <visp/vpMoment.h>
#include <visp/vpMomentDatabase.h>
#include <cstring>
/*!
  Default constructor
*/
vpMoment::vpMoment(): object(NULL),moments(NULL) {
}


/*!
  Links the moment to a database of moment primitives.
  If the moment depends on other moments, these moments must be linked to the same database.
  \attention Two moments of the same class cannot be stored in the same database
  \code
#include <visp/vpMomentObject.h>
#include <visp/vpPoint.h>
#include <visp/vpMomentGravityCenter.h>
#include <visp/vpMomentDatabase.h>
#include <visp/vpMomentCentered.h>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p;

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)

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

  \param moments : database of moment primitives.
*/
void vpMoment::linkTo(vpMomentDatabase& moments){
    std::strcpy(_name,name());
    this->moments=&moments;


    moments.add(*this,_name);
}


/*!
  Updates the moment with the current object. This does not compute any values.
  \param object : object descriptor of the current camera vision.
*/
void vpMoment::update(vpMomentObject& object){
    this->object=&object;
}

/*!
  Prints the moment contents to a stream
  \param os : a std::stream.
  \param m : a moment instance.
*/
std::ostream & operator<<(std::ostream & os, const vpMoment& m){
  for(std::vector<double>::const_iterator i = m.values.begin();i!=m.values.end();i++)
    os << *i << ",";

  return os;
}

