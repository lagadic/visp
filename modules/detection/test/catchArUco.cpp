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
#include <visp3/detection/vpDetectorAprilTag.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

TEST_CASE("ArUco detection test", "[aruco_detection_test]")
{
  std::map<vpDetectorAprilTag::vpAprilTagFamily, int> apriltagMap = {
    {vpDetectorAprilTag::TAG_ARUCO4x4_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO4x4_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO4x4_250, 250},
    // {vpDetectorAprilTag::TAG_ARUCO4x4_1000, 1000}, // there are some deconding issues, probably due to too few bits

    {vpDetectorAprilTag::TAG_ARUCO5x5_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO5x5_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO5x5_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO5x5_1000, 1000},

    {vpDetectorAprilTag::TAG_ARUCO6x6_50, 50},
    {vpDetectorAprilTag::TAG_ARUCO6x6_100, 100},
    {vpDetectorAprilTag::TAG_ARUCO6x6_250, 250},
    {vpDetectorAprilTag::TAG_ARUCO6x6_1000, 1000}
  };

  const int nb_tests = 50;
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
        for (size_t i = 0; i < detector.getNbObjects(); i++) {
          std::string message = detector.getMessage(i);
          std::size_t tag_id_pos = message.find("id: ");

          if (tag_id_pos != std::string::npos) {
            int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
            INFO("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id);
            // WARN("tag_dict=" << kv.first << " ; tag_id=" << tag_id << " ; tag_ref=" << id); // for printing
            if (tag_id == id) {
              found_id = true;
            }
          }
        }

        CHECK(found_id == true);
      }
    }
  }
}

int main(int argc, const char *argv[])
{
  Catch::Session session;

  session.applyCommandLine(argc, argv);
  int numFailed = session.run();

  return numFailed;
}

#else
int main() { return EXIT_SUCCESS; }
#endif
