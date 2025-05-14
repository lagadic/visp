/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 */

#include <visp3/core/vpConfig.h>
#include <visp3/rbt/vpRBTrackingResult.h>

#include <iostream>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

#ifdef VISP_HAVE_NLOHMANN_JSON
BEGIN_VISP_NAMESPACE
void vpRBTrackingResult::saveToFile(const std::string &path) const
{
  nlohmann::json j = *this;
  std::ofstream out(path);
  if (!out.good()) {
    throw vpException(vpException::ioError, "Path %s could not be opened to write tracking results", path.c_str());
  }
  out << j.dump(2);
  out.close();
}

vpRBTrackingResult vpRBTrackingResult::readFromJsonFile(const std::string &path)
{
  vpRBTrackingResult result;
  std::ifstream input(path);
  if (!input.good()) {
    throw vpException(vpException::ioError, "Path %s could not be opened to read tracking results", path.c_str());
  }
  nlohmann::json j = nlohmann::json::parse(input);
  result = j;
  return result;
}
END_VISP_NAMESPACE
#endif
