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
 * Test video i/o.
 */

/*!
  \file testVideo.cpp
  \brief Test vpVideoReader and vpVideoWriter classes.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpVideoWriter.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

static long first_frame = 100;
static int frame_step = 2;
static unsigned int nframes = 3;
static long last_frame = first_frame + static_cast<long>(frame_step) * (nframes - 1);
static std::string tmp;
static std::string videoname_grey;
static std::string videoname_color;

template <class Type>
bool test_createSequence(vpImage<Type> &I, const std::string &videoname, unsigned int first_frame, int frame_step,
                         unsigned int nframes)
{
  try {
    vpVideoWriter writer;
    writer.setFileName(videoname);
    writer.setFirstFrameIndex(static_cast<int>(first_frame));
    writer.setFrameStep(frame_step);
    writer.open(I);

    for (unsigned int i = 0; i < nframes; i++) {
      writer.saveFrame(I);
      std::cout << "Frame saved in: " << writer.getFrameName() << std::endl;
    }
    return true;
  }
  catch (...) {
    return false;
  }
}

template <class Type>
bool test_readSequence(vpImage<Type> &I, const std::string &videoname, long first_frame, int frame_step, int step,
                       long last_frame)
{
  vpVideoReader reader;
  reader.setFileName(videoname);
  reader.setFrameStep(step);
  reader.open(I);

  long frame = reader.getFirstFrameIndex();
  std::cout << "First frame: " << frame << std::endl;
  if (frame != first_frame) {
    std::cout << "Wrong first frame" << std::endl;
    return false;
  }
  frame = reader.getLastFrameIndex();
  std::cout << "Last frame: " << frame << std::endl;
  if (frame != last_frame) {
    std::cout << "Wrong last frame" << std::endl;
    return false;
  }

  long cpt = 0;
  while (!reader.end()) {
    reader.acquire(I);
    long index = reader.getFrameIndex();
    std::cout << "Read frame with index " << index << " from: " << reader.getFrameName() << std::endl;
    if (index != first_frame + cpt * frame_step) {
      std::cout << "Read wrong frame index" << std::endl;
      return false;
    }
    cpt++;
  }
  return true;
}

TEST_CASE("Test saving sequence of uchar images with step 2", "[grey]")
{
  std::cout << "** Create sequence of uchar images with step " << frame_step << std::endl;
  vpImage<unsigned char> I(2, 4, 0);
  CHECK(test_createSequence(I, videoname_grey, first_frame, frame_step, nframes));
}

TEST_CASE("Test reading a sequence of grey images", "[grey]")
{
  SECTION("Read sequence of uchar images with step 1")
  {
    int step = 1;
    vpImage<unsigned char> I;
    CHECK(test_readSequence(I, videoname_grey, first_frame, frame_step, step, last_frame));
  }

  SECTION("Read sequence of uchar images with step 2")
  {
    int step = frame_step;
    vpImage<unsigned char> I;
    CHECK(test_readSequence(I, videoname_grey, first_frame, frame_step, step, last_frame));
  }
}

TEST_CASE("Test saving sequence of color images with step 2", "[color]")
{
  std::cout << "** Create sequence of color images with step " << frame_step << std::endl;
  vpImage<vpRGBa> I(2, 4);
  CHECK(test_createSequence(I, videoname_color, first_frame, frame_step, nframes));
}

TEST_CASE("Test reading a sequence of color images", "[color]")
{
  SECTION("Read sequence of color images with step 1")
  {
    int step = 1;
    vpImage<vpRGBa> I;
    CHECK(test_readSequence(I, videoname_color, first_frame, frame_step, step, last_frame));
  }

  SECTION("Read sequence of color images with step 2")
  {
    int step = frame_step;
    vpImage<vpRGBa> I;
    CHECK(test_readSequence(I, videoname_color, first_frame, frame_step, step, last_frame));
  }
}

int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  std::string tmp = vpIoTools::makeTempDirectory("./");

  std::string username = vpIoTools::getUserName();

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(tmp) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(tmp);
    }
    catch (...) {
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << tmp << std::endl;
    }
  }

  std::cout << "** Create temp directory: " << tmp << std::endl;

  videoname_grey = tmp + std::string("/I%d.pgm");
  videoname_color = tmp + std::string("/I%d.ppm");

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
