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
 * Test vpMbGenericTracker JSON parse / save.
 */

/*!
  \file testMbtJsonSettings.cpp

  Test test saving and parsing JSON configuration for vpMbGenericTracker
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON) && defined(VISP_HAVE_CATCH2)
#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

vpMbGenericTracker baseTrackerConstructor()
{
  const std::vector<std::string> names = { "C1", "C2" };
  std::vector<int> featureTypes;
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
  featureTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
#else
  featureTypes.push_back(vpMbGenericTracker::EDGE_TRACKER);
#endif
  featureTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER | vpMbGenericTracker::DEPTH_NORMAL_TRACKER);
  vpCameraParameters cam1;
  cam1.initPersProjWithoutDistortion(300, 300, 200, 200);
  vpCameraParameters cam2;
  cam2.initPersProjWithoutDistortion(500, 400, 250, 250);

  vpMbGenericTracker t = vpMbGenericTracker(names, featureTypes);

  std::map<std::string, vpCameraParameters> cams;
  cams[names[0]] = cam1;
  cams[names[1]] = cam2;
  t.setCameraParameters(cams);

  t.setLod(false);

  t.setAngleAppear(vpMath::rad(60));
  t.setAngleDisappear(vpMath::rad(90));
  vpMe me;
  me.setSampleStep(2.0);
  me.setMaskSize(7);
  me.setMaskNumber(160);
  me.setRange(8);
  me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
  me.setThreshold(20);
  me.setMu1(0.2);
  me.setMu2(0.3);
  me.setSampleStep(4);
  t.setMovingEdge(me);
  return t;
}

template<typename T, typename C>
void checkProperties(const T &t1, const T &t2, C fn, const std::string &message)
{
  THEN(message)
  {
    REQUIRE((t1.*fn)() == (t2.*fn)());
  }
}

template<typename T, typename C, typename... Fns>
void checkProperties(const T &t1, const T &t2, C fn, const std::string &message, Fns... fns)
{
  checkProperties(t1, t2, fn, message);
  checkProperties(t1, t2, fns...);
}

void compareNamesAndTypes(const vpMbGenericTracker &t1, const vpMbGenericTracker &t2)
{
  REQUIRE(t1.getCameraNames() == t2.getCameraNames());
  REQUIRE(t1.getCameraTrackerTypes() == t2.getCameraTrackerTypes());
}

void compareCameraParameters(const vpMbGenericTracker &t1, const vpMbGenericTracker &t2)
{
  std::map<std::string, vpCameraParameters> c1, c2;
  t1.getCameraParameters(c1);
  t2.getCameraParameters(c2);
  REQUIRE(c1 == c2);
}

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

SCENARIO("MBT JSON Serialization", "[json]")
{
  // setup test dir
  // Get the user login name

  std::string tmp_dir = vpIoTools::makeTempDirectory(vpIoTools::getTempPath() + vpIoTools::path("/") + "visp_test_json_parsing_mbt");

  GIVEN("A generic tracker with two cameras, one with edge and KLT features, the other with depth features")
  {
    vpMbGenericTracker t1 = baseTrackerConstructor();
    WHEN("Saving to a JSON settings file")
    {
      const std::string jsonPath = tmp_dir + "/" + "tracker_save.json";

      const auto modifyJson = [&jsonPath](std::function<void(json &)> modify) -> void {
        json j = loadJson(jsonPath);
        modify(j);
        saveJson(j, jsonPath);
        };

      REQUIRE_NOTHROW(t1.saveConfigFile(jsonPath));
      THEN("Reloading this tracker has the same basic properties")
      {
        vpMbGenericTracker t2;
        REQUIRE_NOTHROW(t2.loadConfigFile(jsonPath));
        compareNamesAndTypes(t1, t2);
        compareCameraParameters(t1, t2);
      }

      THEN("Reloading this tracker has the same basic properties")
      {
        vpMbGenericTracker t2;
        REQUIRE_NOTHROW(t2.loadConfigFile(jsonPath));
        checkProperties(t1, t2,
          &vpMbGenericTracker::getAngleAppear, "Angle appear should be the same",
          &vpMbGenericTracker::getAngleDisappear, "Angle appear should be the same"
        );
      }

      THEN("Reloaded edge tracker parameters should be the same")
      {
        std::map<std::string, vpMe> oldvpMe, newvpMe;
        t1.getMovingEdge(oldvpMe);
        vpMbGenericTracker t2;
        t2.loadConfigFile(jsonPath);
        t2.getMovingEdge(newvpMe);
        for (const auto &it : oldvpMe) {
          vpMe o = it.second;
          vpMe n;
          REQUIRE_NOTHROW(n = newvpMe[it.first]);
          checkProperties(o, n,
            &vpMe::getAngleStep, "Angle step should be equal",
            &vpMe::getMaskNumber, "Mask number should be equal",
            &vpMe::getMaskSign, "Mask sign should be equal",
            &vpMe::getMinSampleStep, "Min sample step should be equal",
            &vpMe::getSampleStep, "Min sample step should be equal",

            &vpMe::getMu1, "Mu 1 should be equal",
            &vpMe::getMu2, "Mu 2 should be equal",
            &vpMe::getNbTotalSample, "Nb total sample should be equal",
            &vpMe::getPointsToTrack, "Number of points to track should be equal",
            &vpMe::getRange, "Range should be equal",
            &vpMe::getStrip, "Strip should be equal"
          );
        }
      }

#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV)
      THEN("Reloaded KLT tracker parameters should be the same")
      {
        std::map<std::string, vpKltOpencv> oldvpKlt, newvpKlt;
        t1.getKltOpencv(oldvpKlt);
        vpMbGenericTracker t2;
        t2.loadConfigFile(jsonPath);
        t2.getKltOpencv(newvpKlt);
        for (const auto &it : oldvpKlt) {
          vpKltOpencv o = it.second;
          vpKltOpencv n;
          REQUIRE_NOTHROW(n = newvpKlt[it.first]);
          checkProperties(o, n,
            &vpKltOpencv::getBlockSize, "Block size should be equal",
            &vpKltOpencv::getHarrisFreeParameter, "Harris parameter should be equal",
            &vpKltOpencv::getMaxFeatures, "Max number of features should be equal",
            &vpKltOpencv::getMinDistance, "Minimum distance should be equal",
            &vpKltOpencv::getPyramidLevels, "Pyramid levels should be equal",
            &vpKltOpencv::getQuality, "Quality should be equal",
            &vpKltOpencv::getWindowSize, "Window size should be equal"
          );
        }
      }
#endif

      THEN("Clipping properties should be the same")
      {
        vpMbGenericTracker t2 = baseTrackerConstructor();
        t2.setNearClippingDistance(0.1);
        t2.setFarClippingDistance(2.0);
        t2.setClipping(vpPolygon3D::ALL_CLIPPING);
        t2.loadConfigFile(jsonPath);
        std::map<std::string, unsigned int> oldFlags, newFlags;
        t1.getClipping(oldFlags);
        t2.getClipping(newFlags);
        for (const auto &it : oldFlags) {
          unsigned int o = it.second;
          unsigned int n;
          REQUIRE_NOTHROW(n = newFlags[it.first]);
          THEN("Clipping flags for camera " + it.first + " should be the same")
          {
            REQUIRE(o == n);
          }
        }
        checkProperties(t1, t2,
          &vpMbGenericTracker::getNearClippingDistance, "Near clipping distance should be the same",
          &vpMbGenericTracker::getFarClippingDistance, "Far clipping distance should be the same"
        );
      }

      THEN("VVS properties should be the same")
      {
        vpMbGenericTracker t2 = baseTrackerConstructor();
        t2.setMaxIter(4096);
        t2.setLambda(5.0);
        t2.setInitialMu(5.0);

        t2.loadConfigFile(jsonPath);

        checkProperties(t1, t2,
          &vpMbGenericTracker::getMaxIter, "VVS m iterations be the same",
          &vpMbGenericTracker::getLambda, "VVS lambda should be the same",
          &vpMbGenericTracker::getInitialMu, "VVS initial mu be the same"
        );
      }

      WHEN("Modifying JSON file/Using a custom JSON file")
      {
        THEN("Removing version from file generates an error on load")
        {
          modifyJson([](json &j) -> void {
            j.erase("version");
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Using an unsupported version generates an error on load")
        {
          modifyJson([](json &j) -> void {
            j["version"] = "0.0.0";
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Using an undefined reference camera generates an error")
        {
          modifyJson([](json &j) -> void {
            j["referenceCameraName"] = "C3";
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("Not defining a transformation matrix for the reference camera is valid")
        {
          modifyJson([&t1](json &j) -> void {
            j["trackers"][t1.getReferenceCameraName()].erase("camTref");
          });
          REQUIRE_NOTHROW(t1.loadConfigFile(jsonPath));
        }

        THEN("Not defining a transformation from a non-reference camera to the reference camera generates an error")
        {
          modifyJson([&t1](json &j) -> void {
            std::string otherCamName = t1.getReferenceCameraName() == "C1" ? "C2" : "C1";
            j["trackers"][otherCamName].erase("camTref");
          });
          REQUIRE_THROWS(t1.loadConfigFile(jsonPath));
        }

        THEN("The full clipping config is optional")
        {
          vpMbGenericTracker t2 = baseTrackerConstructor();
          const double clipping_near = 0.21;
          const double clipping_far = 5.2;
          const int clipping = vpPolygon3D::LEFT_CLIPPING;
          t2.setNearClippingDistance(clipping_near);
          t2.setFarClippingDistance(clipping_far);
          t2.setClipping(clipping);
          modifyJson([&t1](json &j) -> void {
            for (const auto &c : t1.getCameraNames()) {
              j["trackers"][c].erase("clipping");
            }
          });
          REQUIRE_NOTHROW(t2.loadConfigFile(jsonPath, false));
          REQUIRE(t2.getClipping() == clipping);
          REQUIRE(t2.getNearClippingDistance() == clipping_near);
          REQUIRE(t2.getFarClippingDistance() == clipping_far);
        }

        THEN("Each clipping param is optional on its own")
        {
          vpMbGenericTracker t2 = baseTrackerConstructor();
          const double clipping_near = 0.21;
          const double clipping_far = 5.2;
          const int clipping = vpPolygon3D::LEFT_CLIPPING;
          t2.setNearClippingDistance(clipping_near);
          t2.setFarClippingDistance(clipping_far);
          t2.setClipping(clipping);
          THEN("Near clipping is optional")
          {
            modifyJson([&t1](json &j) -> void {
              for (const auto &c : t1.getCameraNames()) {
                j["trackers"][c]["clipping"].erase("near");
              }
            });
            t2.loadConfigFile(jsonPath);
            REQUIRE(t2.getNearClippingDistance() == clipping_near);
            REQUIRE(t2.getFarClippingDistance() == t1.getFarClippingDistance());
            REQUIRE(t2.getClipping() == t1.getClipping());
          }
          THEN("Far clipping is optional")
          {
            modifyJson([&t1](json &j) -> void {
              for (const auto &c : t1.getCameraNames()) {
                j["trackers"][c]["clipping"].erase("far");
              }
            });
            t2.loadConfigFile(jsonPath);
            REQUIRE(t2.getNearClippingDistance() == t1.getNearClippingDistance());
            REQUIRE(t2.getFarClippingDistance() == clipping_far);
            REQUIRE(t2.getClipping() == t1.getClipping());
          }
          THEN("Clipping flags are optional")
          {
            modifyJson([&t1](json &j) -> void {
              for (const auto &c : t1.getCameraNames()) {
                j["trackers"][c]["clipping"].erase("flags");
              }
            });
            t2.loadConfigFile(jsonPath);
            REQUIRE(t2.getNearClippingDistance() == t1.getNearClippingDistance());
            REQUIRE(t2.getFarClippingDistance() == t1.getFarClippingDistance());
            REQUIRE(t2.getClipping() & clipping);
          }
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

int main()
{
  return EXIT_SUCCESS;
}

#endif
