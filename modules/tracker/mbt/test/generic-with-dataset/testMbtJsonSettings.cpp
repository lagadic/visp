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
 * Test vpMbGenericTracker JSON parse / save.
 *
 *****************************************************************************/

/*!
  \file testMbtJsonSettings.cpp

  Test test saving and parsing JSON configuration for vpMbGenericTracker
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
using json = nlohmann::json;
bool test_json(const std::string& dir, const std::string& test_name, const std::function<vpMbGenericTracker()> setup, const std::function<void(json&)> modifyJson, const std::function<bool(const vpMbGenericTracker&, const vpMbGenericTracker&)> compare) {
  const std::string json_path = dir + "/" + test_name + ".json";
  vpMbGenericTracker t1 = setup();
  t1.saveConfigFile(json_path);

  std::ifstream json_file(json_path);
  if(!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file");
  }
  json j = json::parse(json_file);
  json_file.close();
  modifyJson(j);
  std::ofstream json_modif_file(json_path);
  if(!json_modif_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file for writing modifications");
  }
  json_modif_file << j.dump();

  vpMbGenericTracker t2;
  t2.loadConfigFile(json_path);

  return compare(t1, t2);
}

#endif


int main()
{
#if defined(VISP_HAVE_NLOHMANN_JSON)

#if defined(_WIN32)
  std::string tmp_dir = "C:/temp/";
#else
  std::string tmp_dir = "/tmp/";
#endif

  // setup test dir
  // Get the user login name
  std::string username;
  vpIoTools::getUserName(username);

  tmp_dir += username + "/visp_test_json_parsing_mbt/";
  vpIoTools::remove(tmp_dir);
  std::cout << "Create: " << tmp_dir << std::endl;
  vpIoTools::makeDirectory(tmp_dir);


  const auto base_constructor = []() -> vpMbGenericTracker {
    const std::vector<std::string> names = {"C1", "C2"};
    const std::vector<int> featureTypes = {
      vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER,
      vpMbGenericTracker::DEPTH_DENSE_TRACKER | vpMbGenericTracker::DEPTH_NORMAL_TRACKER
    };
    vpMbGenericTracker tracker(names, featureTypes);
    return tracker;
  };
  const auto base_compare = [](const vpMbGenericTracker& t1, const vpMbGenericTracker& t2) -> bool {
    const bool same_cameras = t1.getCameraNames() == t2.getCameraNames();
    if(!same_cameras) {
      return false;
    }
    const bool same_features = t1.getCameraTrackerTypes() == t2.getCameraTrackerTypes();
    if(!same_features) {
      return false;
    }
    return true;

  };
  const auto no_modif = [](json& j) -> void {

  };
  //! First test, check that a basic tracker can be saved and loaded
  if(!test_json(tmp_dir, "basic_test", base_constructor, no_modif, base_compare)) {
    std::cerr << "Basic JSON parsing test failed: not same camera names or camera features" << std::endl;
    return EXIT_FAILURE;
  }

  //! Test on version field: should be present and the correct version
  try {
    test_json(tmp_dir, "throw_version_inexistant", base_constructor, [](json& j) -> void { j.erase("version"); }, base_compare);
    return EXIT_FAILURE;
  } catch(vpException& e) {
    if(e.getCode() != vpException::badValue) {
      return EXIT_FAILURE;
    }
  } catch(...) {
    std::cerr << "Unexpected exception was caught" << std::endl;
    return EXIT_FAILURE;
  }
  
  try {
    test_json(tmp_dir, "throw_wrong_version", base_constructor, [](json& j) -> void { j["version"] = "0.1"; }, base_compare);
    return EXIT_FAILURE;
  } catch(vpException& e) {
    if(e.getCode() != vpException::badValue) {
      return EXIT_FAILURE;
    }
  } catch(...) {
    std::cerr << "Unexpected exception was caught" << std::endl;
    return EXIT_FAILURE;
  }
  
#else
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
#endif

  return EXIT_SUCCESS;
}
