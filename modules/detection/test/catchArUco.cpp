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
 *
 * Description:
 * Test ArUco detection.
 */
/*!
  \example catchArUco.cpp

  \brief Test ArUco detection.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_APRILTAG)

#include <catch_amalgamated.hpp>

#include <iostream>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorAprilTag.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x040700)
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
#endif

#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif


static bool g_debug_print = false;

bool opt_no_display = false; // If true, disable display or tests requiring display

TEST_CASE("ArUco detection test", "[aruco_detection_test]")
{
  std::map<vpDetectorAprilTag::vpAprilTagFamily, int> apriltagMap = {
    {vpDetectorAprilTag::TAG_ARUCO_4x4_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO_4x4_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO_4x4_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO_4x4_1000, 1000},

    {vpDetectorAprilTag::TAG_ARUCO_5x5_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO_5x5_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO_5x5_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO_5x5_1000, 1000},

    {vpDetectorAprilTag::TAG_ARUCO_6x6_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO_6x6_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO_6x6_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO_6x6_1000, 1000},

    {vpDetectorAprilTag::TAG_ARUCO_7x7_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO_7x7_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO_7x7_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO_7x7_1000, 1000},

    {vpDetectorAprilTag::TAG_ARUCO_MIP_36h12, 250}
  };

  const int nb_tests = 50;
  SECTION("Constructor")
  {
    for (const auto &kv : apriltagMap) {
      vpDetectorAprilTag detector(kv.first);

      for (int id = 0; id < kv.second; id += kv.second/nb_tests) {
        vpImage<unsigned char> tag_img;
        detector.getTagImage(tag_img, id);

        vpImage<unsigned char> tag_img_big(tag_img.getHeight()*20, tag_img.getWidth()*20);
        vpImageTools::resize(tag_img, tag_img_big, vpImageTools::INTERPOLATION_NEAREST);

        bool detect = detector.detect(tag_img_big);
        CHECK(detect == true);
        if (detect) {
          bool found_id = false;
          // Parse the message
          for (size_t i = 0; i < detector.getNbObjects(); i++) {
            std::string message = detector.getMessage(i);
            std::size_t tag_id_pos = message.find("id: ");

            if (tag_id_pos != std::string::npos) {
              int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
              if (g_debug_print) {
                WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
              }
              if (tag_id == id) {
                found_id = true;
              }
            }
          }
          CHECK(found_id == true);

          found_id = false;
          // Use directly the getter
          std::vector<int> tagsId = detector.getTagsId();
          for (auto tag_id : tagsId) {
            if (g_debug_print) {
              WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            }
            if (tag_id == id) {
              found_id = true;
              break;
            }
          }
          CHECK(found_id == true);
        }
      }
    }
  }

  SECTION("Copy constructor")
  {
    for (const auto &kv : apriltagMap) {
      vpDetectorAprilTag detector(kv.first);

      for (int id = 0; id < kv.second; id += kv.second/nb_tests) {
        vpImage<unsigned char> tag_img;
        detector.getTagImage(tag_img, id);

        vpImage<unsigned char> tag_img_big(tag_img.getHeight()*20, tag_img.getWidth()*20);
        vpImageTools::resize(tag_img, tag_img_big, vpImageTools::INTERPOLATION_NEAREST);

        bool detect = detector.detect(tag_img_big);
        CHECK(detect == true);
        if (detect) {
          std::vector<int> tagsId = detector.getTagsId();
          std::vector<float> tagsDecisionMargin = detector.getTagsDecisionMargin();
          std::vector<int> tagsHammingDistance = detector.getTagsHammingDistance();
          CHECK(tagsId.size() == tagsDecisionMargin.size());
          CHECK(tagsId.size() == tagsHammingDistance.size());
          CHECK(tagsId.size() == detector.getNbObjects());

          // Use directly the getter
          bool found_id = false;
          for (auto tag_id : tagsId) {
            if (g_debug_print) {
              WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            }
            if (tag_id == id) {
              found_id = true;
              break;
            }
          }
          CHECK(found_id == true);
        }
      }
    }
  }

  SECTION("Getter/Setter")
  {
    for (const auto &kv : apriltagMap) {
      vpDetectorAprilTag detector(kv.first);
      int hamming_dist_ref = 0;
      float decision_margin = 50.0f;
      detector.setAprilTagHammingDistanceThreshold(hamming_dist_ref);
      detector.setAprilTagDecisionMarginThreshold(decision_margin);
      CHECK(detector.getAprilTagHammingDistanceThreshold() == hamming_dist_ref);
      CHECK(detector.getAprilTagDecisionMarginThreshold() == decision_margin);

      for (int id = 0; id < kv.second; id += kv.second/nb_tests) {
        vpImage<unsigned char> tag_img;
        detector.getTagImage(tag_img, id);

        vpImage<unsigned char> tag_img_big(tag_img.getHeight()*20, tag_img.getWidth()*20);
        vpImageTools::resize(tag_img, tag_img_big, vpImageTools::INTERPOLATION_NEAREST);

        bool detect = detector.detect(tag_img_big);
        CHECK(detect == true);
        if (detect) {
          std::vector<int> tagsId = detector.getTagsId();
          std::vector<float> tagsDecisionMargin = detector.getTagsDecisionMargin();
          std::vector<int> tagsHammingDistance = detector.getTagsHammingDistance();
          CHECK(tagsId.size() == tagsDecisionMargin.size());
          CHECK(tagsId.size() == tagsHammingDistance.size());
          CHECK(tagsId.size() == detector.getNbObjects());

          // Use directly the getter
          bool found_id = false;
          for (auto tag_id : tagsId) {
            if (g_debug_print) {
              WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            }
            if (tag_id == id) {
              found_id = true;
              break;
            }
          }
          CHECK(found_id == true);
        }
      }
    }
  }

#ifdef VISP_HAVE_NLOHMANN_JSON
  SECTION("From_to_JSON")
  {
    for (const auto &kv : apriltagMap) {
      vpDetectorAprilTag detector(kv.first);
      int hamming_dist_ref = 0;
      float decision_margin = 50.0f;
      detector.setAprilTagHammingDistanceThreshold(hamming_dist_ref);
      detector.setAprilTagDecisionMarginThreshold(decision_margin);

      nlohmann::json origSettings;
      to_json(origSettings, detector);

      vpDetectorAprilTag detectorCpy;
      from_json(origSettings, detectorCpy);

      CHECK(detectorCpy.getAprilTagHammingDistanceThreshold() == hamming_dist_ref);
      CHECK(detectorCpy.getAprilTagDecisionMarginThreshold() == decision_margin);

      for (int id = 0; id < kv.second; id += kv.second/nb_tests) {
        vpImage<unsigned char> tag_img;
        detector.getTagImage(tag_img, id);

        vpImage<unsigned char> tag_img_big(tag_img.getHeight()*20, tag_img.getWidth()*20);
        vpImageTools::resize(tag_img, tag_img_big, vpImageTools::INTERPOLATION_NEAREST);

        bool detect = detector.detect(tag_img_big);
        bool detectCpy = detectorCpy.detect(tag_img_big);
        CHECK(detect == true);
        CHECK(detectCpy == true);
        if (detect) {
          std::vector<int> tagsId = detector.getTagsId();
          std::vector<float> tagsDecisionMargin = detector.getTagsDecisionMargin();
          std::vector<int> tagsHammingDistance = detector.getTagsHammingDistance();
          CHECK(tagsId.size() == tagsDecisionMargin.size());
          CHECK(tagsId.size() == tagsHammingDistance.size());
          CHECK(tagsId.size() == detector.getNbObjects());

          std::vector<int> tagsIdCpy = detectorCpy.getTagsId();
          CHECK(tagsId.size() == tagsIdCpy.size());

          // Use directly the getter
          bool found_id = false;
          for (auto tag_id : tagsId) {
            if (g_debug_print) {
              WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            }
            if (tag_id == id) {
              found_id = true;
              break;
            }
          }
          CHECK(found_id == true);

          // Use directly the getter
          found_id = false;
          for (auto tag_id : tagsIdCpy) {
            if (g_debug_print) {
              WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            }
            if (tag_id == id) {
              found_id = true;
              break;
            }
          }
          CHECK(found_id == true);
        }
      }
    }
  }
#endif
}

#if (VISP_HAVE_OPENCV_VERSION >= 0x040800)
TEST_CASE("ArUco pose computation test", "[aruco_detection_test]")
{
  std::map<vpDetectorAprilTag::vpAprilTagFamily, aruco::PredefinedDictionaryType> apriltagMap = {
    {vpDetectorAprilTag::TAG_ARUCO_4x4_50, aruco::PredefinedDictionaryType::DICT_4X4_50},
    {vpDetectorAprilTag::TAG_ARUCO_5x5_50, aruco::PredefinedDictionaryType::DICT_5X5_50},
    {vpDetectorAprilTag::TAG_ARUCO_6x6_50, aruco::PredefinedDictionaryType::DICT_6X6_50},
    {vpDetectorAprilTag::TAG_ARUCO_7x7_50, aruco::PredefinedDictionaryType::DICT_7X7_50},
    {vpDetectorAprilTag::TAG_ARUCO_MIP_36h12, aruco::PredefinedDictionaryType::DICT_ARUCO_MIP_36h12}
  };

  const float markerLength = 0.05f;

  // ViSP
  vpCameraParameters cam(600, 600, 100/2, 100/2);
  std::vector<vpHomogeneousMatrix> cMo_vec;

  // OpenCV
  aruco::DetectorParameters detectorParams;
  Matx33d camMatrix = Matx33d::eye();
  camMatrix(0, 0) = cam.get_px(); camMatrix(0, 2) = cam.get_u0();
  camMatrix(1, 1) = cam.get_py(); camMatrix(1, 2) = cam.get_v0();
  Matx41d distCoeffs = Matx41d::zeros();
  Mat image;

  Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength/2.f, markerLength/2.f, 0);
  objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength/2.f, markerLength/2.f, 0);
  objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength/2.f, -markerLength/2.f, 0);
  objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

  for (const auto &kv : apriltagMap) {
     // ViSP
    vpDetectorAprilTag detector(kv.first);

    // OpenCV
    aruco::ArucoDetector cv_detector(aruco::getPredefinedDictionary(kv.second), detectorParams);
    std::vector<int> ids;
    std::vector<std::vector<Point2f> > corners, rejected;

    const int tag_id_ref = 3;
    vpImage<unsigned char> tag_img;
    detector.getTagImage(tag_img, tag_id_ref);

    vpImage<unsigned char> tag_img_big(tag_img.getHeight()*20, tag_img.getWidth()*20);
    vpImageTools::resize(tag_img, tag_img_big, vpImageTools::INTERPOLATION_NEAREST);
    vpImageConvert::convert(tag_img_big, image);

    bool detect = detector.detect(tag_img_big, markerLength, cam, cMo_vec);
    CHECK(detect == true);
    if (detect) {
      std::vector<int> tagsId = detector.getTagsId();
      std::vector<float> tagsDecisionMargin = detector.getTagsDecisionMargin();
      std::vector<int> tagsHammingDistance = detector.getTagsHammingDistance();
      CHECK(tagsId.size() == tagsDecisionMargin.size());
      CHECK(tagsId.size() == tagsHammingDistance.size());
      CHECK(tagsId.size() == detector.getNbObjects());
      CHECK(tagsId.size() == cMo_vec.size());

      vpHomogeneousMatrix cMo;
      // Use directly the getter
      bool found_id = false;
      for (size_t idx = 0; idx < tagsId.size(); idx++) {
        int tag_id = tagsId[idx];
        if (g_debug_print) {
          WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << tag_id_ref);
        }
        if (tag_id == tag_id_ref) {
          cMo = cMo_vec[idx];
          found_id = true;
          break;
        }
      }

      CHECK(found_id == true);
      if (found_id) {
        // OpenCV detect markers and estimate pose
        cv_detector.detectMarkers(image, corners, ids, rejected);
        CHECK(!ids.empty());

        size_t idx_tag_id_ref = 0;
        bool found_tag_id_ref = false;
        for (size_t i = 0; i < ids.size(); i++) {
          if (ids[i] == tag_id_ref) {
            found_tag_id_ref = true;
            idx_tag_id_ref = i;
          }
        }

        CHECK(found_tag_id_ref);
        if (found_tag_id_ref) {
          Matx31d rvec, tvec;
          solvePnP(objPoints, corners.at(idx_tag_id_ref), camMatrix, distCoeffs, rvec, tvec);
          vpThetaUVector cro = cMo.getThetaUVector();
          if (g_debug_print) {
            WARN("rvec: " << rvec(0) << " " << rvec(1) << " " << rvec(2));
            WARN("tvec: " << tvec(0) << " " << tvec(1) << " " << tvec(2));

            WARN("cro: " << cro[0] << " " << cro[1] << " " << cro[2]);
            WARN("cto: " << cMo[0][3] << " " << cMo[1][3] << " " << cMo[2][3]);
          }

          CHECK_THAT(tvec(0), Catch::Matchers::WithinAbs(cMo[0][3], 1e-2));
          CHECK_THAT(tvec(1), Catch::Matchers::WithinAbs(cMo[1][3], 1e-2));
          CHECK_THAT(tvec(2), Catch::Matchers::WithinAbs(cMo[2][3], 1e-2));

          double rvec_magn = std::sqrt(rvec(0)*rvec(0) + rvec(1)*rvec(1) + rvec(2)*rvec(2));
          double cro_magn = std::sqrt(cro[0]*cro[0] + cro[1]*cro[1] + cro[2]*cro[2]);
          CHECK_THAT(rvec_magn, Catch::Matchers::WithinAbs(cro_magn, 1e-2));
        }
      }
    }
  }
}
#endif

int main(int argc, const char *argv[])
{
  Catch::Session session;

  using namespace Catch::Clara;
  auto cli = session.cli()
    | Catch::Clara::Opt(g_debug_print)["--debug-print"]("Force the printing of some debug information.")
    | Catch::Clara::Opt(opt_no_display)["--no-display"]("Disable display");

  session.cli(cli);
  int returnCode = session.applyCommandLine(argc, argv);
  if (returnCode != 0) { // Indicates a command line error
    return returnCode;
  }

  int numFailed = session.run();
  return numFailed;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
