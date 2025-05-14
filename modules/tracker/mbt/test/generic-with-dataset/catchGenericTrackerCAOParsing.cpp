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
 * Test MBT CAO parsing.
 */

/*!
  \example catchGenericTrackerCAOParsing.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030400)

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <catch_amalgamated.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

VP_ATTRIBUTE_NO_DESTROY static std::string ipath = vpIoTools::getViSPImagesDataPath();

TEST_CASE("vpMbGenericTracker load CAO model Linux line ending", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename = vpIoTools::createFilePath(ipath, "mbt-cao/cylinder_cao_model_linux_line_ending.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceCylinder *> cylinders;
  tracker.getLcylinder(cylinders);
  CHECK(cylinders.size() == 1);

  std::list<vpMbtDistanceCircle *> circles;
  tracker.getLcircle(circles);
  CHECK(circles.size() == 1);
}

TEST_CASE("vpMbGenericTracker load CAO model Windows line ending", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/cylinder_cao_model_windows_line_ending.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceCylinder *> cylinders;
  tracker.getLcylinder(cylinders);
  CHECK(cylinders.size() == 1);

  std::list<vpMbtDistanceCircle *> circles;
  tracker.getLcircle(circles);
  CHECK(circles.size() == 1);
}

#if (VISP_HAVE_DATASET_VERSION >= 0x030700)
static const double margin = 1e-9;

static void printFacesInfo(vpMbHiddenFaces<vpMbtPolygon> &faces)
{
  std::cout << "Number of faces: " << faces.size() << std::endl;

  for (unsigned int i = 0; i < faces.size(); i++) {
    std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
    std::cout << "face " << i << " with index: " << poly[i]->getIndex()
      << (poly[i]->getName().empty() ? "" : (" with name: " + poly[i]->getName()))
      << " is " << (poly[i]->isVisible() ? "visible" : "not visible")
      << " and has " << poly[i]->getNbPoint() << " points"
      << " and LOD is " << (poly[i]->useLod ? "enabled" : "disabled") << std::endl;

    for (unsigned int j = 0; j < poly[i]->getNbPoint(); j++) {
      vpPoint P = poly[i]->getPoint(j);
      std::cout << " P obj " << j << ": " << P.get_oX() << " " << P.get_oY() << " " << P.get_oZ() << std::endl;
    }
  }
}

TEST_CASE("vpMbGenericTracker load CAO model [lines]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/line_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceLine *> lines;
  tracker.getLline(lines);
  REQUIRE(lines.size() == 2);

  int idx = 0;
  std::vector<std::string> line_names = { "line_0", "line_1" };
  for (auto line : lines) {
    CHECK_THAT(line->p1->get_oX(), Catch::Matchers::WithinAbs(idx*3 + 0, margin));
    CHECK_THAT(line->p1->get_oY(), Catch::Matchers::WithinAbs(idx*3 + 1, margin));
    CHECK_THAT(line->p1->get_oZ(), Catch::Matchers::WithinAbs(idx*3 + 2, margin));
    CHECK(line->getName() == line_names[idx]);
    ++idx;
  }
}

TEST_CASE("vpMbGenericTracker load CAO model [face from lines]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/face_from_lines_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceLine *> lines;
  tracker.getLline(lines);
  CHECK(lines.size() == 3);

  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
  CHECK(faces.size() == 1);

  for (unsigned int i = 0; i < faces.size(); i++) {
    const std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
    CHECK(poly[i]->getNbPoint() == 4);
    CHECK(poly[i]->getName() == "face_from_3D_lines");

    for (unsigned int j = 0; j < poly[i]->getNbPoint(); j++) {
      vpPoint P = poly[i]->getPoint(j);
      if (j == 3) {
        CHECK_THAT(P.get_oX(), Catch::Matchers::WithinAbs(0, margin));
        CHECK_THAT(P.get_oY(), Catch::Matchers::WithinAbs(1, margin));
        CHECK_THAT(P.get_oZ(), Catch::Matchers::WithinAbs(2, margin));
      }
      else {
        CHECK_THAT(P.get_oX(), Catch::Matchers::WithinAbs(j*3 + 0, margin));
        CHECK_THAT(P.get_oY(), Catch::Matchers::WithinAbs(j*3 + 1, margin));
        CHECK_THAT(P.get_oZ(), Catch::Matchers::WithinAbs(j*3 + 2, margin));
      }
    }
  }
}

TEST_CASE("vpMbGenericTracker load CAO model [face from points]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/face_from_points_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceLine *> lines;
  tracker.getLline(lines);
  CHECK(lines.size() == 3);

  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
  CHECK(faces.size() == 1);

  for (unsigned int i = 0; i < faces.size(); i++) {
    const std::vector<vpMbtPolygon *> &poly = faces.getPolygon();
    CHECK(poly[i]->getNbPoint() == 3);
    CHECK(poly[i]->getName() == "face_from_3D_points");

    for (unsigned int j = 0; j < poly[i]->getNbPoint(); j++) {
      vpPoint P = poly[i]->getPoint(j);
      CHECK_THAT(P.get_oX(), Catch::Matchers::WithinAbs(j*3 + 0, margin));
      CHECK_THAT(P.get_oY(), Catch::Matchers::WithinAbs(j*3 + 1, margin));
      CHECK_THAT(P.get_oZ(), Catch::Matchers::WithinAbs(j*3 + 2, margin));
    }
  }
}

TEST_CASE("vpMbGenericTracker load CAO model [cylinder]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/cylinder_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceCylinder *> cylinders;
  tracker.getLcylinder(cylinders);
  CHECK(cylinders.size() == 1);

  vpMbtDistanceCylinder *cylinder = cylinders.front();
  CHECK(cylinder->getName() == "cylinder");
  CHECK_THAT(cylinder->radius, Catch::Matchers::WithinAbs(0.5, margin));

  CHECK_THAT(cylinder->p1->get_oX(), Catch::Matchers::WithinAbs(0, margin));
  CHECK_THAT(cylinder->p1->get_oY(), Catch::Matchers::WithinAbs(1, margin));
  CHECK_THAT(cylinder->p1->get_oZ(), Catch::Matchers::WithinAbs(2, margin));

  CHECK_THAT(cylinder->p2->get_oX(), Catch::Matchers::WithinAbs(3, margin));
  CHECK_THAT(cylinder->p2->get_oY(), Catch::Matchers::WithinAbs(4, margin));
  CHECK_THAT(cylinder->p2->get_oZ(), Catch::Matchers::WithinAbs(5, margin));

  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
  CHECK(faces.size() == 5);
  printFacesInfo(faces);
}

TEST_CASE("vpMbGenericTracker load CAO model [circle]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/circle_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  std::list<vpMbtDistanceCircle *> circles;
  tracker.getLcircle(circles);
  CHECK(circles.size() == 1);

  vpMbtDistanceCircle *circle = circles.front();
  CHECK(circle->getName() == "circle");
  CHECK_THAT(circle->radius, Catch::Matchers::WithinAbs(0.5, margin));

  CHECK_THAT(circle->p1->get_oX(), Catch::Matchers::WithinAbs(0, margin));
  CHECK_THAT(circle->p1->get_oY(), Catch::Matchers::WithinAbs(1, margin));
  CHECK_THAT(circle->p1->get_oZ(), Catch::Matchers::WithinAbs(2, margin));

  CHECK_THAT(circle->p2->get_oX(), Catch::Matchers::WithinAbs(3, margin));
  CHECK_THAT(circle->p2->get_oY(), Catch::Matchers::WithinAbs(4, margin));
  CHECK_THAT(circle->p2->get_oZ(), Catch::Matchers::WithinAbs(5, margin));

  CHECK_THAT(circle->p3->get_oX(), Catch::Matchers::WithinAbs(6, margin));
  CHECK_THAT(circle->p3->get_oY(), Catch::Matchers::WithinAbs(7, margin));
  CHECK_THAT(circle->p3->get_oZ(), Catch::Matchers::WithinAbs(8, margin));

  vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
  CHECK(faces.size() == 1);
  printFacesInfo(faces);
}

TEST_CASE("vpMbGenericTracker load CAO model [hierarchical]", "[vpMbGenericTracker CAO parsing]")
{
  const std::string cao_filename =
    vpIoTools::createFilePath(ipath, "mbt-cao/hierarchical_model.cao");
  vpMbGenericTracker tracker;
  const bool verbose = true;
  REQUIRE_NOTHROW(tracker.loadModel(cao_filename, verbose));

  // Lines
  {
    std::list<vpMbtDistanceLine *> lines;
    tracker.getLline(lines);
    REQUIRE(lines.size() == 2);
    std::vector<std::string> line_names = { "line_0", "line_1" };

    int idx = 0;
    for (auto line : lines) {
      CHECK_THAT(line->p1->get_oX(), Catch::Matchers::WithinAbs(idx*3 + 0, margin));
      CHECK_THAT(line->p1->get_oY(), Catch::Matchers::WithinAbs(idx*3 + 1, margin));
      CHECK_THAT(line->p1->get_oZ(), Catch::Matchers::WithinAbs(idx*3 + 2, margin));
      CHECK(line->getName() == line_names[idx]);
      ++idx;
    }
  }

  // Face from points
  {
    vpMbHiddenFaces<vpMbtPolygon> &faces = tracker.getFaces();
    // vpMbtDistanceLine --> 2 faces
    // vpMbtDistanceCylinder --> 1 + 4 faces
    // vpMbtDistanceCircle --> 1 face
    CHECK(faces.size() == 8);
    printFacesInfo(faces);
  }

  // Cylinder
  {
    std::list<vpMbtDistanceCylinder *> cylinders;
    tracker.getLcylinder(cylinders);
    CHECK(cylinders.size() == 1);

    vpMbtDistanceCylinder *cylinder = cylinders.front();
    CHECK(cylinder->getName() == "cylinder");
    CHECK_THAT(cylinder->radius, Catch::Matchers::WithinAbs(0.5, margin));

    CHECK_THAT(cylinder->p1->get_oX(), Catch::Matchers::WithinAbs(0, margin));
    CHECK_THAT(cylinder->p1->get_oY(), Catch::Matchers::WithinAbs(1, margin));
    CHECK_THAT(cylinder->p1->get_oZ(), Catch::Matchers::WithinAbs(2, margin));

    CHECK_THAT(cylinder->p2->get_oX(), Catch::Matchers::WithinAbs(3, margin));
    CHECK_THAT(cylinder->p2->get_oY(), Catch::Matchers::WithinAbs(4, margin));
    CHECK_THAT(cylinder->p2->get_oZ(), Catch::Matchers::WithinAbs(5, margin));
  }

  // Circle
  {
    std::list<vpMbtDistanceCircle *> circles;
    tracker.getLcircle(circles);
    CHECK(circles.size() == 1);

    vpMbtDistanceCircle *circle = circles.front();
    CHECK(circle->getName() == "circle");
    CHECK_THAT(circle->radius, Catch::Matchers::WithinAbs(0.5, margin));

    CHECK_THAT(circle->p1->get_oX(), Catch::Matchers::WithinAbs(0, margin));
    CHECK_THAT(circle->p1->get_oY(), Catch::Matchers::WithinAbs(1, margin));
    CHECK_THAT(circle->p1->get_oZ(), Catch::Matchers::WithinAbs(2, margin));

    CHECK_THAT(circle->p2->get_oX(), Catch::Matchers::WithinAbs(3, margin));
    CHECK_THAT(circle->p2->get_oY(), Catch::Matchers::WithinAbs(4, margin));
    CHECK_THAT(circle->p2->get_oZ(), Catch::Matchers::WithinAbs(5, margin));

    CHECK_THAT(circle->p3->get_oX(), Catch::Matchers::WithinAbs(6, margin));
    CHECK_THAT(circle->p3->get_oY(), Catch::Matchers::WithinAbs(7, margin));
    CHECK_THAT(circle->p3->get_oZ(), Catch::Matchers::WithinAbs(8, margin));
  }
}

#endif

int main(int argc, char *argv[])
{
  Catch::Session session;
  session.applyCommandLine(argc, argv);
  int numFailed = session.run();
  return numFailed;
}

#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
