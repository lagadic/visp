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
 * Test face detection.
 *
 *****************************************************************************/
/*!
  \example testFaceDetection.cpp

  \brief Test face detection.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <iostream>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/detection/vpDetectorCascade.h>
#include <visp3/io/vpImageIo.h>

static bool g_save_results = false;
static std::string g_image_filename = "";
static std::string g_haar_filename = "";
static bool g_no_ground_truth = false;
static unsigned int g_min_size = 30;

namespace
{
struct Detection {
  std::vector<vpImagePoint> corners;

  Detection(const std::vector<vpImagePoint> &c) : corners(c) {}

  bool operator==(const Detection &b) const
  {
    if (corners.size() != b.corners.size())
      return false;

    for (size_t i = 0; i < corners.size(); i++) {
      const vpImagePoint& ref = corners[i];
      bool find = false;
      for (size_t j = 0; j < b.corners.size() && !find; j++) {
        const vpImagePoint& cur = b.corners[j];
        // Allow 1 pixel of difference
        if (vpMath::equal(ref.get_u(), cur.get_u(), 1.01) &&
            vpMath::equal(ref.get_v(), cur.get_v(), 1.01)) {
          find = true;
        }
      }
      if (!find) {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const Detection &b) const { return !(*this == b); }
};

std::ostream &operator<<(std::ostream &os, Detection &d)
{
  for (size_t i = 0; i < d.corners.size(); i++)
    os << d.corners[i] << std::endl;

  return os;
}
}

TEST_CASE("Face detection", "[face_detection]") {
  std::string filename = !g_image_filename.empty() ? g_image_filename :
                          vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                                   "faces/1280px-Solvay_conference_1927.png");
  std::cout << "Image: " << filename << std::endl;
  REQUIRE(vpIoTools::checkFilename(filename));
  vpImage<unsigned char> I;
  vpImageIo::read(I, filename);

  vpDetectorCascade face_detector;
  filename = !g_haar_filename.empty() ? g_haar_filename :
              vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                       "faces/haar_face_0.xml");
  std::cout << "Haar: " << filename << std::endl;
  REQUIRE(vpIoTools::checkFilename(filename));
  face_detector.setCascadeClassifierFile(filename);
  std::cout << "Minimum object size: " << g_min_size << std::endl;
  face_detector.setDetectionMinSize(g_min_size, g_min_size);

  vpChrono chrono;
  chrono.start();
  face_detector.detect(I);
  chrono.stop();

  std::cout << "Detected " << face_detector.getNbObjects() << " faces in " << chrono.getDurationMs() << " ms" << std::endl;

  vpImage<vpRGBa> results;
  vpImageConvert::convert(I, results);

  std::vector<Detection> ground_truth_detections;
  if (!g_no_ground_truth) {
    filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                         "faces/ground_truth_detection.txt");
    REQUIRE(vpIoTools::checkFilename(filename));
    std::ifstream file(filename.c_str());
    if (file.is_open()) {
      std::string id;
      double v1 = 0.0, v2 = 0.0, v3 = 0.0, v4 = 0.0;
      double u1 = 0.0, u2 = 0.0, u3 = 0.0, u4 = 0.0;
      while (file >> id >> v1 >> u1 >> v2 >> u2 >> v3 >> u3 >> v4 >> u4) {
        std::vector<vpImagePoint> corners(4);
        corners[0].set_ij(v1, u1);
        corners[1].set_ij(v2, u2);
        corners[2].set_ij(v3, u3);
        corners[3].set_ij(v4, u4);
        ground_truth_detections.push_back(Detection(corners));
      }
    }

    SECTION("Check number of detected faces")
    {
      CHECK(ground_truth_detections.size() == face_detector.getNbObjects());
    }
  }

  for (size_t i = 0; i < face_detector.getNbObjects(); i++) {
    vpRect bb = face_detector.getBBox(i);
    vpImageDraw::drawRectangle(results, bb, vpColor::red, false, 3);

    if (!g_no_ground_truth) {
      Detection current_detection(face_detector.getPolygon()[i]);
      std::cout << "current_detection:\n" << current_detection << std::endl;
      bool find_detection = std::find(ground_truth_detections.begin(), ground_truth_detections.end(),
                                      current_detection) != ground_truth_detections.end();
      CHECK(find_detection);
    }
  }

  if (g_save_results) {
    vpImageIo::write(results, "test_faces_detection.png");
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Build a new parser on top of Catch's
  using namespace Catch::clara;
  auto cli = session.cli()            // Get Catch's composite command line parser
    | Opt(g_save_results)             // bind variable to a new option, with a hint string
    ["--save_results"]                // the option names it will respond to
    ("Save detected faces image.")    // description string for the help output
    | Opt(g_image_filename, "g_image_filename")
    ["--input"]
    ("Input image.")
    | Opt(g_haar_filename, "g_haar_filename")
    ["--haar"]
    ("Input Haar cascade filename.")
    | Opt(g_no_ground_truth)
    ["--no_ground_truth"]
    ("Do not run check against ground truth.")
    | Opt(g_min_size, "g_min_size")
    ["--min_size"]
    ("Minimum size of the object in the image.");

  // Now pass the new composite back to Catch so it uses that
  session.cli(cli);

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main()
{
  return 0;
}
#endif
