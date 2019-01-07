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
 * Pseudo-database used to handle dependencies between moments
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <iostream>
#include <typeinfo>
#include <visp3/core/vpMoment.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentObject.h>

/*!
        Adds a moment to the database.
        \param moment : moment to add
        \param : name of the moment's class

        \attention You cannot add two moments with the same name. The rules
   for insersion are the same as those of std::map.
*/
void vpMomentDatabase::add(vpMoment &moment, const char *name)
{
  moments.insert(std::pair<const char *, vpMoment *>((const char *)name, &moment));
}

/*!
  Retrieves a moment from the database.
  \param type : Name of the moment's class.
  \param found : true if the moment's type exists in the database, false
  otherwise. \return Moment corresponding to \e type.
*/
const vpMoment &vpMomentDatabase::get(const char *type, bool &found) const
{
  std::map<const char *, vpMoment *, vpMomentDatabase::cmp_str>::const_iterator it = moments.find(type);

  found = (it != moments.end());
  return *(it->second);
}

/*!
        Updates the moment object for all moments in the database
  \param object : Moment object for which all the moments in the database
  should be updated.

    Sometimes, it might be useful to update the whole database when computing
  only one moment when this moment depends on other moments. The example
  provided in the header of this class gives an example that shows how to
  compute gravity center moment and the centered moment using a mass update.
*/
void vpMomentDatabase::updateAll(vpMomentObject &object)
{
  std::map<const char *, vpMoment *, vpMomentDatabase::cmp_str>::const_iterator itr;
  for (itr = moments.begin(); itr != moments.end(); ++itr) {
    (*itr).second->update(object);
  }
}

/*!
        Outputs all the moments values in the database to a stream.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpMomentDatabase &m)
{
  std::map<const char *, vpMoment *, vpMomentDatabase::cmp_str>::const_iterator itr;
  os << "{";

  for (itr = m.moments.begin(); itr != m.moments.end(); ++itr) {
    os << (*itr).first << ": [" << *((*itr).second) << "],";
  }
  os << "}";

  return os;
}
