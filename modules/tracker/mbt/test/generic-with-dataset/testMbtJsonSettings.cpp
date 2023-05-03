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

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

vpMbGenericTracker baseTrackerConstructor() {
  const std::vector<std::string> names = {"C1", "C2"};
  const std::vector<int> featureTypes = {
    vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER,
    vpMbGenericTracker::DEPTH_DENSE_TRACKER | vpMbGenericTracker::DEPTH_NORMAL_TRACKER
  };
  vpCameraParameters cam1;
  cam1.initPersProjWithoutDistortion(300, 300, 200, 200);
  vpCameraParameters cam2;
  cam2.initPersProjWithoutDistortion(500, 400, 250, 250);
  
  vpMbGenericTracker t = vpMbGenericTracker(names, featureTypes);

  std::map<std::string, vpCameraParameters> cams;
  cams[names[0]] = cam1;
  cams[names[1]] = cam2;
  t.setCameraParameters(cams);
  return t;
}

void compareNamesAndTypes(const vpMbGenericTracker& t1, const vpMbGenericTracker& t2) {
  REQUIRE( t1.getCameraNames() == t2.getCameraNames() );
  REQUIRE( t1.getCameraTrackerTypes() == t2.getCameraTrackerTypes() );
}

void compareCameraParameters(const vpMbGenericTracker& t1, const vpMbGenericTracker& t2) {
  std::map<std::string, vpCameraParameters> c1, c2;
  vpCameraParameters c;
  t1.getCameraParameters(c1);
  t2.getCameraParameters(c2);
  REQUIRE( c1 == c2 );
}

json loadJson(const std::string& path) {
  std::ifstream json_file(path);
  if(!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file");
  }
  json j = json::parse(json_file);
  json_file.close();
  return j;
}


void saveJson(const json& j, const std::string& path) {
  std::ofstream json_file(path);
  if(!json_file.good()) {
    throw vpException(vpException::ioError, "Could not open JSON settings file to write modifications");
  }
  json_file << j.dump();
  json_file.close();
}

void test_json(const std::string& dir, const std::string& test_name, const std::function<vpMbGenericTracker()> setup, const std::function<void(json&)> modifyJson, const std::function<void(const vpMbGenericTracker&, const vpMbGenericTracker&)> compare) {
  const std::string json_path = dir + "/" + test_name + ".json";
  vpMbGenericTracker t1 = setup();
  t1.saveConfigFile(json_path);

  json j = loadJson(json_path);
  modifyJson(j);
  saveJson(j, json_path);
  

  vpMbGenericTracker t2;
  t2.loadConfigFile(json_path);

  compare(t1, t2);
}


//   // // Reference camera name not found
//   // try {
//   //   test_json(tmp_dir, "reference_cam_not_found", baseTrackerConstructor, [](json& j) -> void { j["referenceCameraName"] = "C3"; }, compareNamesAndTypes);
//   //   return EXIT_FAILURE;
//   // } catch(vpException& e) {
//   //   if(e.getCode() != vpException::badValue) {
//   //     return EXIT_FAILURE;
//   //   }
//   // } catch(...) {
//   //   std::cerr << "Unexpected exception was caught" << std::endl;
//   //   return EXIT_FAILURE;
//   // }

//   // // Camera to ref transformations test
  
//   // if(!test_json(tmp_dir, "camTref_removed_for_ref", baseTrackerConstructor, [](json& j) -> void { j["trackers"][j["referenceCameraName"].get<std::string>()].erase("camTref"); }, compareNamesAndTypes)) {
//   //   std::cerr << "Removing the camera transformations should be allowed for the reference camera" << std::endl;
//   //   return EXIT_FAILURE;
//   // }

SCENARIO("MBT JSON Parsing", "[json]") {
  // setup test dir
  // Get the user login name
#if defined(_WIN32)
  std::string tmp_dir = "C:/temp/";
#else
  std::string tmp_dir = "/tmp/";
#endif
  std::string username;
  vpIoTools::getUserName(username);

  tmp_dir += username + "/visp_test_json_parsing_mbt/";
  vpIoTools::remove(tmp_dir);
  std::cout << "Create: " << tmp_dir << std::endl;
  vpIoTools::makeDirectory(tmp_dir);



  GIVEN("A generic tracker with two cameras, one with edge and KLT features, the other with depth features") {
    vpMbGenericTracker t1 = baseTrackerConstructor();
    WHEN("Saving to a JSON settings file") {
      const std::string jsonPath = tmp_dir + "/" + "tracker_save.json";

      const auto modifyJson = [&jsonPath](std::function<void(json&)> modify) -> void {
        json j = loadJson(jsonPath);
        modify(j);
        saveJson(j, jsonPath);
      };

      REQUIRE_NOTHROW(t1.saveConfigFile(jsonPath));
      THEN("Reloading this tracker has the same basic properties") {
        vpMbGenericTracker t2;
        REQUIRE_NOTHROW(t2.loadConfigFile(jsonPath));
        compareNamesAndTypes(t1, t2);
        compareCameraParameters(t1, t2);
      }
      WHEN("Modifying JSON file") {
        THEN("Removing version from file generates an error on load") {
          modifyJson([](json& j) -> void {
            j.erase("version");
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Using an unsupported version generates an error on load") {
          modifyJson([](json& j) -> void {
            j["version"] = "0.0.0";
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Using an undefined reference camera generates an error") {
          modifyJson([](json& j) -> void {
            j["referenceCameraName"] = "C3";
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Not defining a transformation matrix for the reference camera is valid") {
          modifyJson([&t1](json& j) -> void {
            j["trackers"][t1.getReferenceCameraName()].erase("camTref");
          });
          REQUIRE_NOTHROW(t1.loadConfigFile(jsonPath));
        }

        THEN("Not defining a transformation from a non-reference camera to the reference camera generates an error") {
          modifyJson([&t1](json& j) -> void {
            std::string otherCamName = t1.getReferenceCameraName() == "C1" ? "C2" : "C1";
            j["trackers"][otherCamName].erase("camTref");
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }
      }
    }
  }


  
}
int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else 

int main() {
  return EXIT_SUCCESS;
}

#endif