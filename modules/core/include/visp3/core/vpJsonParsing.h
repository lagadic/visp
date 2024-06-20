/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * JSON parsing helpers.
 */

#ifndef VP_JSON_PARSING_H
#define VP_JSON_PARSING_H

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>

BEGIN_VISP_NAMESPACE
/*!
  Parse the flag values defined in a JSON object.
  if the flags are defined as an int, then this is int is directly returned.
  If it is defined as a combination of options (defined from an enumeration E) then the logical or of theses enum values is returned.
  Beware that invalid values may be defined in the JSON object: the int value may be invalid, or the parsing of enum values may fail.

  \param j: the JSON object to parse

  \return an int, corresponding to the combination of boolean flags

*/
template<typename E>
int flagsFromJSON(const nlohmann::json &j)
{
  int flags = 0;
  if (j.is_array()) {
    flags = 0;
    for (const auto &v : j) {
      E value = v.get<E>(); // If a value is incorrect, this will default to the first value of the enum
      flags |= value;
    }
  }
  else if (j.is_number_integer()) {
    flags = j.get<int>();
  }
  return flags;
}

/*!
  Serialize flag values as a json array.
  \param flags the value to serialize
  \param options the possible values that can be contained in flags. A flag i is set if flags & options[i] != 0.

  \return a json object (an array) that contains the different flags of the variable flags.

*/
template<typename E>
nlohmann::json flagsToJSON(const int flags, const std::vector<E> &options)
{
  nlohmann::json j = nlohmann::json::array();
  for (const E option : options) {
    if (flags & option) {
      j.push_back(option);
    }
  }
  return j;
}

template<typename T>
bool convertFromTypeAndBuildFrom(const nlohmann::json &, T &)
{
  return false;
}

template<typename T, typename O, typename... Os>
bool convertFromTypeAndBuildFrom(const nlohmann::json &j, T &t)
{
  if (j["type"] == O::jsonTypeName) {
    O other;
    from_json(j, other);
    t.build(other);
    return true;
  }
  else {
    return convertFromTypeAndBuildFrom<T, Os...>(j, t);
  }
}
END_VISP_NAMESPACE
#endif
#endif
