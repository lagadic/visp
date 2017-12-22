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
 * Base for 2D moment descriptor
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

/*!
  \file vpMoment.h
  \brief Base class for all 2D moments.
*/

#ifndef __MOMENT_H__
#define __MOMENT_H__

#include <iostream>
#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>

class vpMomentDatabase;
class vpMomentObject;

/*!
  \class vpMoment

  \ingroup group_core_moments

  \brief This class defines shared methods/attributes for 2D moments.

  All moments or combination of moments in the moments module are based on
  this class. A moment uses a vpMomentObject object to access all useful
  information. Moment values are obtained by a 4-step process common for all
  moment types:
  - Declaration.
  \code
  vpMoment moment;
  \endcode
  - Update with object.
  \code
  moment.update(object);
  \endcode
  - Compute the moment value
  \code
  moment.compute();
  \endcode
  - Access the values:
  \code
  std::vector<double> values = moment.get();
  \endcode

  A moment may also be linked to a vpMomentDatabase. Moments linked to a
  database are able to access each others values. Some moments can be computed
  only if they are linked to a a database containing their dependencies.
  Linking to a database is done using the vpMoment::linkTo(...) method.

  There are no constraints about format of the array returned by
  vpMoment::get(); any implementation is fine.

  Each moment must have a string name by implementing the char*
  vpMoment::name() method which allows to identify the moment in the database.
  Each moment must also implement a compute method describing how to obtain
  its values from the object.

  \attention Order of moment computation DOES matter: when you compute a
  moment using vpMoment::compute(), all moment dependencies must be computed.
  We recall that implemented moments are:
  - vpMomentAlpha
  - vpMomentArea
  - vpMomentAreaNormalized
  - vpMomentBasic
  - vpMomentCentered
  - vpMomentCInvariant
  - vpMomentGravityCenter
  - vpMomentGravityCenterNormalized

*/
class VISP_EXPORT vpMoment
{
private:
  vpMomentObject *object;
  vpMomentDatabase *moments;
  char _name[255];

protected:
  std::vector<double> values;
  /*!
     Returns the linked moment database.
     \return the moment database
   */
  inline vpMomentDatabase &getMoments() const { return *moments; }

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpMoment(const vpMoment &)
  //    : object(NULL), moments(NULL), values()
  //  {
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!");
  //  }
  //  vpMoment &operator=(const vpMoment &){
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  vpMoment();

  /*!
     Virtual destructor.
   */
  virtual ~vpMoment() {}

  /** @name Inherited functionalities from vpMoment */
  //@{
  virtual void compute() = 0;
  inline const vpMomentObject &getObject() const { return *object; }
  /*!
     Returns all values computed by the moment.
     \return vector of values
   */
  const std::vector<double> &get() const { return values; }
  void linkTo(vpMomentDatabase &moments);
  virtual const char *name() const = 0;
  virtual void printDependencies(std::ostream &os) const;
  void update(vpMomentObject &object);
  //@}
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMoment &m);
};
#endif
