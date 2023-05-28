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
 * Test vpJsonArgumentParser
 *
 *****************************************************************************/

 /*!
   \file testJsonArgumentParser.cpp

   Test parsing command line argument and json files with vpJsonArgumentParser
 */

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

json loadJson(const std::string &path)
{
  std::ifstream json_file(path);
  if (!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file");
  }
  json j = json::parse(json_file);
  json_file.close();
  return j;
}

void saveJson(const json &j, const std::string &path)
{
  std::ofstream json_file(path);
  if (!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file to write modifications");
  }
  json_file << j.dump();
  json_file.close();
}

SCENARIO("Parsing arguments from JSON file", "[json]")
{
  // setup test dir
  // Get the user login name

  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath() + vpIoTools::path("/") + "visp_test_json_argument_parsing");
  const std::string jsonPath = tmp_dir + "/" + "arguments.json";

  const auto modifyJson = [&jsonPath](std::function<void(json &)> modify) -> void {
    json j = loadJson(jsonPath);
    modify(j);
    saveJson(j, jsonPath);
  };

}
int main(int argc, char *argv [])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main()
{
  return EXIT_SUCCESS;
}

#endif
