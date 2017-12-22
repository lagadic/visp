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
 * Pseudo-database used to handle dependencies between moments
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/
/*!
  \file vpMomentDatabase.h
  \brief Pseudo-database used to handle dependencies between moments.
*/
#ifndef __MOMENTDATABASE_H__
#define __MOMENTDATABASE_H__

#include <visp3/core/vpImage.h>

#include <cstring>
#include <iostream>
#include <map>

class vpMoment;
class vpMomentObject;

/*!
  \class vpMomentDatabase

  \ingroup group_core_moments

  \brief This class allows to register all vpMoments so they can access each
other according to their dependencies.

  Sometimes, a moment needs to have access to other moment's values to be
computed. For example vpMomentCentered needs additionnal information about the
gravity center vpMomentGravityCenter in order to compute the moment's value
from a vpMomentObject. This gravity center should be stored in a
vpMomentDatabase where it can be accessed.

  All moments in a database can access each other freely at any time. They can
also verify if a moment is present in the database or not. Here is a example
of a dependency between two moments using a vpMomentDatabase:

\code
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <iostream>

int main()
{
  vpPoint p;
  std::vector<vpPoint> vec_p; // vector that contains the vertices of the contour polygon

  p.set_x(1); p.set_y(1); // coordinates in meters in the image plane (vertex 1)
  vec_p.push_back(p);
  p.set_x(2); p.set_y(2); // coordinates in meters in the image plane (vertex 2)
  vec_p.push_back(p);
  vpMomentObject obj(1); // Create an image moment object with 1 as
       // maximum order (sufficient for gravity center)
  obj.setType(vpMomentObject::DISCRETE); // The object is defined by
           // two discrete points
  obj.fromVector(vec_p); // Init the dense object with the polygon

  vpMomentDatabase db;
  vpMomentGravityCenter g; // declaration of gravity center
  vpMomentCentered mc; // mc containts centered moments

  g.linkTo(db); //add gravity center to database
  mc.linkTo(db); //centered moments depend on gravity, add them to the
     //database to grant access

  db.updateAll(obj); // All of the moments must be updated, not just mc

  //There is no global compute method since the order of compute calls
  //depends on the implementation
  g.compute(); // compute the moment
  mc.compute(); //compute centered moments AFTER gravity center

  std::cout << "Gravity center: " << g << std:: endl; // print gravity center moment
  std::cout << "Centered moments: " << mc << std:: endl; // print centered moment

  return 0;
}
  \endcode

  The following code outputs:
  \code
Gravity center:
Xg=1.5, Yg=1.5
Centered moments:
2	0
0	x
  \endcode

  A moment is identified in the database by it's vpMoment::name method.
Consequently, a database can contain at most one moment of each type. Often it
is useful to update all moments with the same object. Shortcuts
(vpMomentDatabase::updateAll) are provided for that matter.
*/
class VISP_EXPORT vpMomentDatabase
{
private:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct cmp_str {
    bool operator()(char const *a, char const *b) const { return std::strcmp(a, b) < 0; }
  };
#endif
  std::map<const char *, vpMoment *, cmp_str> moments;
  void add(vpMoment &moment, const char *name);

public:
  vpMomentDatabase() : moments() {}
  virtual ~vpMomentDatabase() {}

  /** @name Inherited functionalities from vpMomentDatabase */
  //@{
  const vpMoment &get(const char *type, bool &found) const;
  /*!
    Get the first element in the database.
    May be useful in case an unnamed object is present but is the only element
    in the database. \return the first element in the database.
    */
  vpMoment &get_first() { return *(moments.begin()->second); }

  virtual void updateAll(vpMomentObject &object);
  //@}

  friend class vpMoment;
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentDatabase &v);
};

#endif
