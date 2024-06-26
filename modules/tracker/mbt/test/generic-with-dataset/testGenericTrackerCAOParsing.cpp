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
 * Test MBT CAO parsing.
 */

/*!
  \example testGenericTrackerCAOParsing.cpp
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && (VISP_HAVE_DATASET_VERSION >= 0x030400)

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static std::string ipath = vpIoTools::getViSPImagesDataPath();

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

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
#include <iostream>

int main() { return EXIT_SUCCESS; }
#endif
